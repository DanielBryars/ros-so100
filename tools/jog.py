#!/usr/bin/env python
"""Jog the arms from the keyboard, one joint at a time, and show what the model expects.

    source /opt/ros/lyrical/setup.bash
    ~/venvs/so100/bin/python tools/jog.py

Needs dual_arm.launch.py running with drive:=true. Starting it switches torque on for both
arms (they hold where they are). Quitting leaves them holding.

Keys:
    1-6        pick the joint          left/right  jog it by one step
    [ ]        smaller / larger step   a           follower, leader or both
    z          go to the zero pose     h           go to the raised home pose
    q          quit (arms keep holding)

The table at the bottom gives, for each arm's measured joint angles, the height above the
desk that the model predicts for each joint axis and for the jaw tip, to compare with a
ruler. The desk is 2.4 mm below the model's base frame (the underside of the base plate).
Joint axis heights depend only on link lengths, which the SO-100 and SO-101 share; the jaw
tip is the SO-101 model's and may differ on an SO-100.
"""
import curses
import math
import os
import time

import mujoco
import numpy as np
import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray
from std_srvs.srv import SetBool

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENE = os.path.join(REPO, 'scenes', 'so101_with_wrist_cam.xml')
NAMES = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']
ARMS = ['follower', 'leader']
STEPS_DEG = [0.5, 1.0, 2.0, 5.0, 10.0]
HOME = [0.0, -0.3, 0.3, 0.0, 0.0]
DESK_Z = -0.0024                                    # desk surface in the base frame, m
JAW_TIP = np.array([-0.0102, -0.0002, -0.1045])     # fixed jaw tip in the gripper body frame
POINTS = [('shoulder lift axis', 'upper_arm'), ('elbow axis', 'lower_arm'),
          ('wrist flex axis', 'wrist'), ('wrist roll body', 'gripper')]


class Model:
    def __init__(self):
        self.m = mujoco.MjModel.from_xml_path(SCENE)
        self.d = mujoco.MjData(self.m)
        self.adr = [self.m.jnt_qposadr[mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, n)]
                    for n in NAMES]
        self.body = {b: mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, b)
                     for _, b in POINTS}

    def heights(self, q):
        """[(label, height above the desk in mm, distance forward of the base frame in mm)]"""
        for a, v in zip(self.adr, q):
            self.d.qpos[a] = v
        mujoco.mj_forward(self.m, self.d)
        rows = [(label, self.d.xpos[self.body[b]]) for label, b in POINTS]
        g = self.body['gripper']
        rows.append(('jaw tip', self.d.xpos[g] + self.d.xmat[g].reshape(3, 3) @ JAW_TIP))
        return [(label, (p[2] - DESK_Z) * 1000, math.hypot(p[0], p[1]) * 1000) for label, p in rows]


class Arm:
    def __init__(self, node, name):
        self.name = name
        self.q = None
        self.counts = None
        self.target = None
        node.create_subscription(JointState, f'/{name}/joint_states', self.on_state, 1)
        node.create_subscription(Int32MultiArray, f'/{name}/joint_counts',
                                 lambda m: setattr(self, 'counts', list(m.data)), 1)
        self.pub = node.create_publisher(JointState, f'/{name}/joint_command', 1)
        self.enable = node.create_client(SetBool, f'/{name}/enable')

    def on_state(self, m):
        q = dict(zip(m.name, m.position))
        if all(math.isfinite(q.get(n, math.nan)) for n in NAMES):
            self.q = [q[n] for n in NAMES]


def run(screen):
    curses.curs_set(0)
    screen.nodelay(True)
    rclpy.init()
    node = rclpy.create_node('jog')
    arms = [Arm(node, n) for n in ARMS]
    model = Model()
    end = time.time() + 5.0
    while any(a.q is None for a in arms) and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    arms = [a for a in arms if a.q is not None]
    if not arms:
        raise RuntimeError('no joint states: is dual_arm.launch.py running?')
    for a in arms:
        fut = a.enable.call_async(SetBool.Request(data=True))
        rclpy.spin_until_future_complete(node, fut, timeout_sec=3.0)
        a.target = list(a.q)

    joint, step, who = 0, 2, 0            # who: index into arms, or len(arms) for all
    message = ''
    while True:
        rclpy.spin_once(node, timeout_sec=1 / 30)
        active = arms if who == len(arms) else [arms[who]]
        key = screen.getch()
        if key == ord('q'):
            break
        if ord('1') <= key <= ord('6'):
            joint = key - ord('1')
        elif key == ord('['):
            step = max(0, step - 1)
        elif key == ord(']'):
            step = min(len(STEPS_DEG) - 1, step + 1)
        elif key == ord('a'):
            who = (who + 1) % (len(arms) + 1)
        elif key in (curses.KEY_LEFT, curses.KEY_RIGHT):
            delta = math.radians(STEPS_DEG[step]) * (1 if key == curses.KEY_RIGHT else -1)
            for a in active:
                a.target[joint] += delta
        elif key in (ord('z'), ord('h')):
            for a in active:
                a.target[:5] = [0.0] * 5 if key == ord('z') else list(HOME)
            message = 'going to the %s pose' % ('zero' if key == ord('z') else 'home')
        for a in arms:                     # stream targets: the driver stops if they stop
            msg = JointState(name=NAMES, position=[float(v) for v in a.target])
            msg.header.stamp = node.get_clock().now().to_msg()
            a.pub.publish(msg)

        screen.erase()
        names = ' + '.join(a.name for a in active)
        screen.addstr(0, 0, f'Jogging: {names}   step {STEPS_DEG[step]:g} deg   (keys: 1-6 joint, '
                            'left/right jog, [ ] step, a arm, z zero, h home, q quit)')
        screen.addstr(1, 0, message)
        screen.addstr(3, 0, '%-16s' % '' + ''.join('| %-34s' % a.name for a in arms))
        screen.addstr(4, 0, '%-16s' % 'joint' + ''.join('| target   measured (deg)   counts  ' for _ in arms))
        for i, n in enumerate(NAMES):
            line = '%s %d %-12s' % ('>' if i == joint else ' ', i + 1, n)
            for a in arms:
                c = a.counts[i] if a.counts else -1
                line += '| %+7.1f  %+7.1f          %5d   ' % (
                    math.degrees(a.target[i]), math.degrees(a.q[i]), c)
            screen.addstr(5 + i, 0, line, curses.A_BOLD if i == joint else 0)
        screen.addstr(12, 0, 'Model, at the measured angles: height above desk / distance from base frame origin (mm)')
        tables = [model.heights(a.q) for a in arms]
        for r in range(len(tables[0])):
            line = '  %-14s' % tables[0][r][0]
            for t in tables:
                line += '| %7.1f  / %7.1f                ' % (t[r][1], t[r][2])
            screen.addstr(13 + r, 0, line)
        screen.refresh()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    curses.wrapper(run)
