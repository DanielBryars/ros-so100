#!/usr/bin/env python3
"""MuJoCo backend for the SO100 pipeline (docs/ros2-pipeline.md, decision 6).

Presents the same topics as the real arm will:

    subscribes  /joint_command               sensor_msgs/JointState  position targets, rad
    publishes   /joint_states                sensor_msgs/JointState  every frame (90 Hz)
                /wrist_cam/image_rect        sensor_msgs/Image rgb8  30 Hz
                /overhead_cam/image_rect     (+ camera_info for each)
                /sim/target_truth            geometry_msgs/PointStamped, latched
    service     /sim/reset                   std_srvs/Trigger, re-randomises the block

Physics runs against the wall clock. With vr:=true the loop is paced by the headset and the
scene is rendered there too; otherwise a 90 Hz sleep paces it.

Needs mujoco (and pyopenxr for VR), which live in ~/venvs/so100 rather than the system
Python, so start it with that interpreter (sim.launch.py does):

    ~/venvs/so100/bin/python -m so100_sim.sim_node --ros-args -p vr:=true
"""
import math
import os
import sys
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_srvs.srv import Trigger

JOINT_NAMES = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']
CAMERAS = ['wrist_cam', 'overhead_cam']
DEFAULT_VR_TOOLS = os.path.expanduser('~/git/ros-so100/tools/vr')


class SimNode(Node):
    def __init__(self):
        super().__init__('so100_sim')
        self.use_vr = self.declare_parameter('vr', False).value
        self.scene_name = self.declare_parameter('scene', '').value
        self.floor = self.declare_parameter('floor', 'black').value
        self.rate_hz = self.declare_parameter('rate_hz', 90.0).value
        self.image_every = self.declare_parameter('image_every', 3).value
        self.width = self.declare_parameter('image_width', 640).value
        self.height = self.declare_parameter('image_height', 480).value
        self.block_pos_range = self.declare_parameter('block_pos_range', 0.02).value
        self.mirror = self.declare_parameter('mirror', True).value
        vr_tools = self.declare_parameter(
            'vr_tools_dir', os.environ.get('SO100_VR_TOOLS', DEFAULT_VR_TOOLS)).value

        # Scene loading and the VR renderer are shared with the stand-alone scripts in tools/vr.
        sys.path.insert(0, vr_tools)
        import mujoco
        import vr_common
        self.mujoco, self.vr_common = mujoco, vr_common

        self.model, self.data = vr_common.load_scene(
            self.scene_name or vr_common.DEFAULT_SCENE, floor=self.floor)
        m = self.model
        jids = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in JOINT_NAMES]
        self.qadr = np.array([m.jnt_qposadr[j] for j in jids])
        self.vadr = np.array([m.jnt_dofadr[j] for j in jids])
        self.act = {n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in JOINT_NAMES}
        self.duplo_body = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'duplo')
        duplo_joint = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, 'duplo_joint')
        self.duplo_qadr = m.jnt_qposadr[duplo_joint]
        self.duplo_vadr = m.jnt_dofadr[duplo_joint]
        self.qpos0 = self.data.qpos.copy()   # settled state from load_scene
        self.data.ctrl[[self.act[n] for n in JOINT_NAMES]] = self.data.qpos[self.qadr]

        self.lock = threading.Lock()
        self.pending = {}            # actuator id -> target, written by the subscriber thread
        self.reset_requested = False
        self.commands_seen = 0

        reliable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        best_effort = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(JointState, '/joint_states', reliable)
        self.truth_pub = self.create_publisher(PointStamped, '/sim/target_truth', latched)
        self.image_pubs = {c: self.create_publisher(Image, f'/{c}/image_rect', best_effort)
                           for c in CAMERAS}
        self.info_pubs = {c: self.create_publisher(CameraInfo, f'/{c}/camera_info', best_effort)
                          for c in CAMERAS}
        self.create_subscription(JointState, '/joint_command', self.on_command, reliable)
        self.create_service(Trigger, '/sim/reset', self.on_reset)

        self.cam_ids = {c: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, c) for c in CAMERAS}
        self.cam_renderer = None     # created on the render thread (owns a GL context)

    # ---- callbacks (executor thread) ----

    def on_command(self, msg):
        with self.lock:
            for name, pos in zip(msg.name, msg.position):
                if name in self.act and not math.isnan(pos):
                    self.pending[self.act[name]] = pos
            self.commands_seen += 1

    def on_reset(self, request, response):
        with self.lock:
            self.reset_requested = True
        response.success = True
        response.message = 'block will be re-randomised on the next frame'
        return response

    # ---- main loop (render thread) ----

    def reset_block(self):
        """Put the block back at its scene position plus a random XY offset and yaw."""
        a, v = self.duplo_qadr, self.duplo_vadr
        self.data.qpos[a:a + 7] = self.qpos0[a:a + 7]
        self.data.qvel[v:v + 6] = 0.0
        r = self.block_pos_range
        self.data.qpos[a:a + 2] += np.random.uniform(-r, r, size=2)
        yaw = np.random.uniform(-math.pi, math.pi)
        self.data.qpos[a + 3:a + 7] = [math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]
        self.mujoco.mj_forward(self.model, self.data)
        self.publish_truth()

    def publish_truth(self):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base'   # the robot base body sits at the MuJoCo world origin
        p = self.data.xpos[self.duplo_body]
        msg.point.x, msg.point.y, msg.point.z = float(p[0]), float(p[1]), float(p[2])
        self.truth_pub.publish(msg)
        self.get_logger().info('Block at x=%.3f y=%.3f' % (p[0], p[1]))

    def apply_inputs(self):
        with self.lock:
            pending, self.pending = self.pending, {}
            reset, self.reset_requested = self.reset_requested, False
        for aid, pos in pending.items():
            lo, hi = self.model.actuator_ctrlrange[aid]
            self.data.ctrl[aid] = min(max(pos, lo), hi)
        if reset:
            self.reset_block()

    def publish_state(self, stamp):
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = JOINT_NAMES
        msg.position = self.data.qpos[self.qadr].tolist()
        msg.velocity = self.data.qvel[self.vadr].tolist()
        self.state_pub.publish(msg)

    def camera_info(self, cam, stamp):
        fy = (self.height / 2) / math.tan(math.radians(self.model.cam_fovy[self.cam_ids[cam]]) / 2)
        cx, cy = self.width / 2, self.height / 2
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = cam
        info.width, info.height = self.width, self.height
        info.distortion_model = 'plumb_bob'
        info.d = [0.0] * 5
        info.k = [fy, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fy, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def publish_images(self, stamp):
        if self.cam_renderer is None:
            self.cam_renderer = self.mujoco.Renderer(self.model, self.height, self.width)
        for cam in CAMERAS:
            self.cam_renderer.update_scene(self.data, camera=self.cam_ids[cam])
            rgb = self.cam_renderer.render()
            img = Image()
            img.header.stamp = stamp
            img.header.frame_id = cam
            img.height, img.width = self.height, self.width
            img.encoding = 'rgb8'
            img.step = self.width * 3
            img.data = rgb.tobytes()
            self.image_pubs[cam].publish(img)
            self.info_pubs[cam].publish(self.camera_info(cam, stamp))

    def run(self):
        mujoco, model, data = self.mujoco, self.model, self.data
        vr = None
        if self.use_vr:
            vr = self.vr_common.VRRenderer(model, data, mirror=self.mirror)
            vr.init_all()
            self.get_logger().info(
                'VR up. X/A recentre, Y/B recentre in 5 s, thumbstick click = robot at controller.')
        self.publish_truth()
        max_steps = int(0.05 / model.opt.timestep)
        period = 1.0 / self.rate_hz
        debt, last = 0.0, time.perf_counter()
        frame, t_report, n_report = 0, last, 0
        try:
            while rclpy.ok():
                self.apply_inputs()
                now = time.perf_counter()
                debt += now - last
                last = now
                steps = min(int(debt / model.opt.timestep), max_steps)
                debt = min(debt - steps * model.opt.timestep, model.opt.timestep)
                for _ in range(steps):
                    mujoco.mj_step(model, data)

                stamp = self.get_clock().now().to_msg()
                self.publish_state(stamp)
                if frame % self.image_every == 0:
                    self.publish_images(stamp)
                frame += 1

                if vr is not None:
                    if not vr.render_frame():   # blocks in xrWaitFrame at the headset rate
                        break
                    vr.check_keyboard()
                else:
                    time.sleep(max(0.0, period - (time.perf_counter() - now)))

                n_report += 1
                if now - t_report > 5.0:
                    with self.lock:
                        cmds, self.commands_seen = self.commands_seen, 0
                    state = vr.session_state.name if vr is not None else 'no VR'
                    self.get_logger().info('%.1f loop/s | %.1f cmd/s | %s' % (
                        n_report / (now - t_report), cmds / (now - t_report), state))
                    t_report, n_report = now, 0
        finally:
            if vr is not None:
                vr.cleanup()


def main(args=None):
    rclpy.init(args=args)
    node = SimNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def spin():
        try:
            executor.spin()
        except Exception:
            if rclpy.ok():   # anything raised after shutdown is just the context going away
                raise

    spinner = threading.Thread(target=spin, daemon=True)
    spinner.start()
    try:
        node.run()   # GL and OpenXR stay on the main thread
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
