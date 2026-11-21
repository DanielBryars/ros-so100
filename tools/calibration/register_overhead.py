#!/usr/bin/env python
"""Find where the overhead camera is relative to the arm's base, with no calibration target.

The arm is the target. For each captured view we know the joint angles, so the MuJoCo model
says exactly what the arm's printed (green) parts should look like from any camera pose. The
script searches for the camera pose whose rendered silhouettes best match the green pixels in
the real images.

The idea is from roboreg, "marker-free camera-to-robot registration" using the robot's mesh
as the calibration target, by Martin Huber (GitHub mhubii) and Lucas-Raphael Mueller
(haydnspass), built at King's College London: https://github.com/lbr-stack/roboreg, presented
as "ROS 2 roboreg: marker-free hand-eye calibration" at ROSCon UK 2026 (Edinburgh). This file
is an independent, much simpler take on it (colour-threshold silhouettes and a derivative-free
search instead of learned segmentation and differentiable rendering); none of roboreg's code
is used.

    # 1. capture views: run this, then move the unpowered follower through varied poses
    #    (reach left/right/forward, fold up, turn the wrist), leaving it still at each one.
    #    Keep hands out of the picture when it is still.
    python register_overhead.py capture --dir ~/overhead_views --count 8
    #    A window shows the live image with the detected arm outlined and the progress.
    # 2. solve
    python register_overhead.py solve --dir ~/overhead_views          # add --show to watch

If a ChArUco board is lying flat on the table in the views, `solve` takes the table plane from
it: its normal is the base's z axis (the camera's tilt) and its distance is the camera's
height. Both are far better than silhouettes can give. The silhouettes then only have to find
the rotation about z and the x, y position.

    # 3. check, independently of the silhouettes: touch the jaw tip to the board's corners
    python register_overhead.py touch --square-mm 28.56

`solve` writes base->overhead_cam as a YAML (default
ros2_ws/src/so100_telemetry/config/overhead_cam_extrinsics.yaml) and overlay images
(<view>_overlay.png) next to the views for checking by eye.
Needs the camera's intrinsics (config/overhead_cam.yaml), a display for MuJoCo's renderer,
and for `capture` a sourced ROS environment. Run with ~/venvs/so100/bin/python.
"""
import argparse
import glob
import json
import os
import sys
import time

import cv2
import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
CONFIG = os.path.join(REPO, 'ros2_ws', 'src', 'so100_telemetry', 'config')
SCENE = os.path.join(REPO, 'scenes', 'so101_with_wrist_cam.xml')
JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']
CV_TO_MJ = np.diag([1.0, -1.0, -1.0])   # OpenCV camera axes (z forward, y down) -> MuJoCo's


def capture(args):
    """Save a view each time the arm has been moved to a new pose and has come to rest."""
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, JointState
    rclpy.init()
    node = rclpy.create_node('register_overhead_capture')
    got = {'js': []}          # js: (time, joint dict) for the last couple of seconds

    def on_image(m):
        got['img'] = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, 3).copy()

    def on_joints(m):
        now = time.time()
        got['js'].append((now, dict(zip(m.name, m.position))))
        while got['js'][0][0] < now - args.still_seconds:
            got['js'].pop(0)

    node.create_subscription(Image, args.image_topic, on_image, qos_profile_sensor_data)
    node.create_subscription(JointState, args.joint_topic, on_joints, 10)
    os.makedirs(args.dir, exist_ok=True)
    n0 = len(glob.glob(os.path.join(args.dir, 'view_*.json')))
    saved, last, flash_until = 0, None, 0.0
    window = None if args.no_window else 'overhead registration: capture'
    print(f'Move the arm to a new pose and leave it still for {args.still_seconds:.0f} s; '
          f'{args.count} views wanted. Ctrl+C to stop early.', flush=True)
    start = time.time()
    try:
        while saved < args.count and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            js = got['js']
            if window and 'img' in got and js:
                bgr = cv2.cvtColor(got['img'], cv2.COLOR_RGB2BGR)
                cv2.drawContours(bgr, cv2.findContours(green_mask(bgr).astype(np.uint8), cv2.RETR_EXTERNAL,
                                                       cv2.CHAIN_APPROX_SIMPLE)[0], -1, (0, 255, 255), 1)
                moving = max(abs(a[k] - js[-1][1][k]) for _, a in js for k in js[-1][1]) >= 0.01
                is_new = last is None or max(abs(last[k] - js[-1][1][k]) for k in last) > args.min_change
                state = 'moving' if moving else ('hold still...' if is_new else 'move to a new pose')
                if time.time() < flash_until:
                    cv2.rectangle(bgr, (0, 0), (bgr.shape[1] - 1, bgr.shape[0] - 1), (0, 255, 0), 12)
                    state = 'saved'
                cv2.putText(bgr, f'views {saved}/{args.count}   {state}', (10, 26),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                cv2.imshow(window, bgr)
                if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
                    break
            if 'img' not in got or len(js) < 5 or js[-1][0] - js[0][0] < 0.8 * args.still_seconds:
                continue
            if time.time() - start < args.still_seconds:
                continue
            q = js[-1][1]
            still = max(abs(a[k] - q[k]) for _, a in js for k in q) < 0.01
            new = last is None or max(abs(last[k] - q[k]) for k in q) > args.min_change
            if still and new:
                stem = os.path.join(args.dir, f'view_{n0 + saved:02d}')
                cv2.imwrite(stem + '.png', cv2.cvtColor(got['img'], cv2.COLOR_RGB2BGR))
                with open(stem + '.json', 'w') as f:
                    json.dump(q, f)
                saved, last, flash_until = saved + 1, q, time.time() + 0.7
                print(f'\a{saved}/{args.count} saved {os.path.basename(stem)}', flush=True)
    except KeyboardInterrupt:
        pass
    print(f'{saved} views saved in {args.dir}')


def green_mask(bgr):
    """The arm's printed parts: saturated green."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (35, 90, 60), (85, 255, 255))
    return cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)) > 0


def load_intrinsics(path):
    import yaml
    with open(path) as f:
        y = yaml.safe_load(f)
    K = np.array(y['camera_matrix']['data'], float).reshape(3, 3)
    dist = np.array(y['distortion_coefficients']['data'], float)
    return K, dist, (y['image_width'], y['image_height'])


class ArmRenderer:
    """Renders the silhouette of the model's green parts from a camera pose given as
    (rotation vector, translation) of the base frame in OpenCV camera coordinates."""

    def __init__(self, K, size):
        import mujoco
        self.mj = mujoco
        self.m = mujoco.MjModel.from_xml_path(SCENE)
        self.d = mujoco.MjData(self.m)
        w, h = size
        self.cam = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_CAMERA, 'overhead_cam')
        # Pinhole camera with the real K (1 length unit per pixel; cx, cy as offsets from centre).
        self.m.cam_resolution[self.cam] = [w, h]
        self.m.cam_sensorsize[self.cam] = [w, h]
        self.m.cam_intrinsic[self.cam] = [K[0, 0], K[1, 1], K[0, 2] - w / 2, K[1, 2] - h / 2]
        self.renderer = mujoco.Renderer(self.m, h, w)
        self.renderer.enable_segmentation_rendering()
        self.opt = mujoco.MjvOption()
        self.opt.geomgroup[:] = [0, 0, 1, 0, 0, 0]   # visual meshes only: no floor, bowl, block
        green = np.array([0.45, 0.80, 0.20])
        self.green = np.array([self.m.geom_matid[g] >= 0 and
                               np.allclose(self.m.mat_rgba[self.m.geom_matid[g]][:3], green, atol=0.02)
                               for g in range(self.m.ngeom)])
        self.qadr = [self.m.jnt_qposadr[mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, j)]
                     for j in JOINTS]

    def mask(self, joints, rvec, tvec):
        mj = self.mj
        self.d.qpos[self.qadr] = [joints[j] for j in JOINTS]
        R = cv2.Rodrigues(np.asarray(rvec, float))[0]          # base -> camera (OpenCV)
        cam_in_base = R.T @ CV_TO_MJ                           # camera axes (MuJoCo) in base
        quat = np.zeros(4)
        mj.mju_mat2Quat(quat, cam_in_base.ravel())
        self.m.cam_pos[self.cam] = -R.T @ np.asarray(tvec, float)
        self.m.cam_quat[self.cam] = quat
        mj.mj_forward(self.m, self.d)
        self.renderer.update_scene(self.d, camera=self.cam, scene_option=self.opt)
        seg = self.renderer.render()                           # (h, w, 2): object id, type
        ids = seg[:, :, 0]
        is_geom = seg[:, :, 1] == mj.mjtObj.mjOBJ_GEOM
        return is_geom & self.green[np.clip(ids, 0, self.m.ngeom - 1)]


def chamfer(a, b, cap=40.0):
    """Symmetric mean distance (px) between two silhouettes, each pixel's distance capped."""
    if not a.any() or not b.any():
        return cap
    # Not `~a`: when the argument is a temporary, NumPy may invert it in place.
    da = cv2.distanceTransform(np.logical_not(a).astype(np.uint8), cv2.DIST_L2, 3)
    db = cv2.distanceTransform(np.logical_not(b).astype(np.uint8), cv2.DIST_L2, 3)
    return 0.5 * (np.minimum(da[b], cap).mean() + np.minimum(db[a], cap).mean())


def initial_guess(height=0.65):
    """Camera straight above the workspace, image-down = base +x, image-right = base +y."""
    R = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
    cam_pos = np.array([0.19, 0.0, height])
    return cv2.Rodrigues(R)[0].ravel(), -R @ cam_pos


def table_normal(views, K, board_json, square_mm):
    """The table plane in camera coordinates, from a ChArUco board lying flat on the table in
    any of the views: (unit normal pointing at the camera, distance from the camera to the
    board's surface, number of views used); normal is None if no view shows the board."""
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from calibrate_intrinsics import load_board
    board = load_board(board_json, square_mm)
    detector = cv2.aruco.CharucoDetector(board)
    normals, dists = [], []
    for _, img, _, _ in views:
        corners, ids, _, _ = detector.detectBoard(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
        if ids is None or len(ids) < 20:
            continue
        obj, pts = board.matchImagePoints(corners, ids)
        ok, rvec, tvec = cv2.solvePnP(obj, pts, K, None)
        n = cv2.Rodrigues(rvec)[0][:, 2]
        normals.append(-n if n[2] > 0 else n)
        dists.append(abs(float(n @ tvec.ravel())))
    if not normals:
        return None, 0.0, 0
    n = np.mean(normals, axis=0)
    return n / np.linalg.norm(n), float(np.mean(dists)), len(normals)


def rotation_between(a, b):
    """Smallest rotation taking unit vector a to unit vector b."""
    axis = np.cross(a, b)
    s, c = np.linalg.norm(axis), float(np.dot(a, b))
    if s < 1e-12:
        return np.eye(3)
    return cv2.Rodrigues(axis / s * np.arctan2(s, c))[0]


def solve(args):
    from scipy.optimize import minimize
    K, dist, size = load_intrinsics(args.intrinsics)
    if np.abs(dist).max() > 1e-6:
        print('note: lens distortion is ignored here (the overhead calibration has none)')
    views = []
    for jf in sorted(glob.glob(os.path.join(args.dir, '*.json'))):
        img = cv2.imread(jf[:-5] + '.png')
        if img is None:
            continue
        with open(jf) as f:
            views.append((jf[:-5], img, green_mask(img), json.load(f)))
    if not views:
        sys.exit('no views in ' + args.dir)
    print(len(views), 'views')
    arm = ArmRenderer(K, size)

    evals = [0]

    def overlay(img, real, model):
        over = img.copy()
        over[model & np.logical_not(real)] = (0, 0, 255)      # red: model says arm, image does not
        over[real & np.logical_not(model)] = (255, 0, 0)      # blue: image says arm, model does not
        edge = cv2.morphologyEx(model.astype(np.uint8), cv2.MORPH_GRADIENT, np.ones((3, 3), np.uint8)) > 0
        over[edge] = (0, 255, 255)                            # yellow: outline of the model
        return over

    def cost(x):
        masks = [arm.mask(q, x[:3], x[3:]) for _, _, _, q in views]
        value = float(np.mean([chamfer(real, m) for (_, _, real, _), m in zip(views, masks)]))
        evals[0] += 1
        if args.show and evals[0] % 5 == 1:
            tiles = [cv2.resize(overlay(img, real, m), None, fx=0.5, fy=0.5)
                     for (_, img, real, _), m in zip(views, masks)]
            cols = min(4, len(tiles))
            tiles += [np.zeros_like(tiles[0])] * (-len(tiles) % cols)
            grid = np.vstack([np.hstack(tiles[r:r + cols]) for r in range(0, len(tiles), cols)])
            cv2.putText(grid, f'step {evals[0]}   mismatch {value:.2f} px', (10, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.imshow('overhead registration: solve', grid)
            cv2.waitKey(1)
        return value

    r0, t0 = initial_guess()
    normal, dist_board, n_seen = (None, 0.0, 0) if args.no_board else table_normal(
        views, K, args.board, args.square_mm)
    if normal is not None:
        # The base stands on the table, so the board gives three of the six unknowns directly
        # and far better than silhouettes can: the table's normal is the base's z axis (the
        # camera's tilt), and the camera's distance from the table is its height. Height from
        # silhouettes alone is poor: a threshold that makes the arm look 1 px wider moves the
        # camera about 2 cm. Left for the silhouettes: the rotation about z, and x and y.
        R0 = cv2.Rodrigues(r0)[0]
        R_n = rotation_between(R0[:, 2], normal) @ R0
        cam_z = dist_board + args.board_thickness + args.table_z
        print(f'table plane from the ChArUco board in {n_seen} views: camera is {dist_board * 1e3:.0f} mm '
              f'above the board, {cam_z * 1e3:.0f} mm above the base frame; solving yaw, x and y')

        def full(p):
            R = R_n @ cv2.Rodrigues(np.array([0.0, 0.0, p[0]]))[0]
            return np.concatenate([cv2.Rodrigues(R)[0].ravel(), -R @ np.array([p[1], p[2], cam_z])])
        cam0 = -cv2.Rodrigues(r0)[0].T @ t0
        p = np.array([0.0, cam0[0], cam0[1]])
    else:
        print('no ChArUco board found lying on the table; solving all six parameters')
        full = lambda p: p
        p = np.concatenate([r0, t0])
    print(f'initial guess: {cost(full(p)):.2f} px')
    # Coarse then fine; Nelder-Mead copes with the rendered cost not being smooth.
    for scale in (0.05, 0.01, 0.002):
        simplex = np.vstack([p] + [p + scale * e for e in np.eye(len(p))])
        res = minimize(lambda v: cost(full(v)), p, method='Nelder-Mead',
                       options={'initial_simplex': simplex, 'xatol': 1e-5, 'fatol': 1e-4, 'maxiter': 1500})
        p = res.x
        print(f'  step {scale}: {res.fun:.2f} px after {res.nit} iterations', flush=True)
    x = full(p)
    rvec, tvec = x[:3], x[3:]

    ious = []
    for stem, img, real, q in views:
        model = arm.mask(q, rvec, tvec)
        ious.append((real & model).sum() / max((real | model).sum(), 1))
        cv2.imwrite(stem + '_overlay.png', overlay(img, real, model))
    R = cv2.Rodrigues(rvec)[0]
    cam_pos = -R.T @ tvec
    print(f'silhouette mismatch {cost(x):.2f} px; overlap (IoU) per view: '
          + ' '.join(f'{v:.2f}' for v in ious))
    print('camera position in base frame (m):', np.round(cam_pos, 4))
    print('camera optical axis in base frame:', np.round(R.T @ [0, 0, 1], 4))
    quat = np.zeros(4)
    import mujoco
    mujoco.mju_mat2Quat(quat, R.T.ravel())      # camera (OpenCV optical frame) in base, w x y z
    out = args.output or os.path.join(CONFIG, 'overhead_cam_extrinsics.yaml')
    with open(out, 'w') as f:
        f.write('# base -> overhead_cam optical frame (z forward, x right, y down), from\n'
                f'# tools/calibration/register_overhead.py on {time.strftime("%Y-%m-%d")}, {len(views)} views,\n'
                f'# silhouette mismatch {cost(x):.2f} px.\n'
                'parent_frame: base\nchild_frame: overhead_cam\n'
                f'translation: [{cam_pos[0]:.5f}, {cam_pos[1]:.5f}, {cam_pos[2]:.5f}]\n'
                f'rotation_xyzw: [{quat[1]:.6f}, {quat[2]:.6f}, {quat[3]:.6f}, {quat[0]:.6f}]\n')
    print('Wrote', out)
    if args.show:
        print('Press a key in the window to close it.')
        cv2.waitKey(0)
    return rvec, tvec


JAW_TIP = np.array([-0.0102, -0.0002, -0.1045])   # end of the fixed jaw, in the gripper body frame


def jaw_tip_in_base(joints):
    """Where the model puts the tip of the fixed jaw for these joint angles (base frame, m)."""
    import mujoco
    if not hasattr(jaw_tip_in_base, 'm'):
        jaw_tip_in_base.m = mujoco.MjModel.from_xml_path(SCENE)
        jaw_tip_in_base.d = mujoco.MjData(jaw_tip_in_base.m)
    m, d = jaw_tip_in_base.m, jaw_tip_in_base.d
    for j in JOINTS:
        d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)]] = joints[j]
    mujoco.mj_forward(m, d)
    b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'gripper')
    return d.xpos[b] + d.xmat[b].reshape(3, 3) @ JAW_TIP


def touch(args):
    """Independent check: touch the tip of the fixed jaw to the four outer corners of the
    board's chequer pattern. The arm model says where the tip is; the camera says where the
    corner is. Compares the two using the extrinsics file, and also solves the camera pose
    from the touches alone."""
    import rclpy
    import yaml
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, JointState
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from calibrate_intrinsics import load_board
    K, _, _ = load_intrinsics(args.intrinsics)
    board = load_board(args.board, args.square_mm)
    detector = cv2.aruco.CharucoDetector(board)
    nx, ny = board.getChessboardSize()
    sq = board.getSquareLength()
    outer = np.array([[0, 0, 0], [nx * sq, 0, 0], [nx * sq, ny * sq, 0], [0, ny * sq, 0]], float)

    rclpy.init()
    node = rclpy.create_node('register_overhead_touch')
    got = {'js': []}

    def on_image(m):
        got['img'] = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, 3).copy()

    def on_joints(m):
        now = time.time()
        got['js'].append((now, dict(zip(m.name, m.position))))
        while got['js'][0][0] < now - args.still_seconds:
            got['js'].pop(0)

    node.create_subscription(Image, args.image_topic, on_image, qos_profile_sensor_data)
    node.create_subscription(JointState, args.joint_topic, on_joints, 10)
    touches, last, flash_until, board_pose = [], None, 0.0, None
    start = time.time()
    print(f'Touch the tip of the fixed jaw to each outer corner of the chequer pattern and hold '
          f'it still for {args.still_seconds:.0f} s. {args.count} touches wanted; Q stops early.', flush=True)
    while len(touches) < args.count and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
        js = got['js']
        if 'img' not in got or len(js) < 5:
            continue
        bgr = cv2.cvtColor(got['img'], cv2.COLOR_RGB2BGR)
        corners, ids, _, _ = detector.detectBoard(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY))
        if ids is not None and len(ids) >= 8:
            obj, pts = board.matchImagePoints(corners, ids)
            ok, rvec, tvec = cv2.solvePnP(obj, pts, K, None)
            if ok:
                board_pose = (rvec, tvec)
        if board_pose is not None:
            px = cv2.projectPoints(outer, *board_pose, K, None)[0].reshape(-1, 2)
            for k, (u, v) in enumerate(px):
                cv2.circle(bgr, (int(u), int(v)), 9, (0, 255, 255), 2)
                cv2.putText(bgr, str(k + 1), (int(u) + 10, int(v) - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        q = js[-1][1]
        if last is None:
            last = q          # the pose the arm starts in is not a touch
        still = (js[-1][0] - js[0][0] > 0.8 * args.still_seconds
                 and max(abs(a[k] - q[k]) for _, a in js for k in q) < 0.01)
        new = max(abs(last[k] - q[k]) for k in q) > 0.1
        state = 'board not found' if board_pose is None else (
            'moving' if not still else ('hold still...' if new else 'go to the next corner'))
        if still and new and board_pose is not None and time.time() - start > args.still_seconds:
            corners_cam = (cv2.Rodrigues(board_pose[0])[0] @ outer.T).T + board_pose[1].ravel()
            touches.append({'joints': q, 'corners_cam': corners_cam.tolist()})
            last, flash_until = q, time.time() + 0.7
            print(f'\a{len(touches)}/{args.count} recorded', flush=True)
        if time.time() < flash_until:
            cv2.rectangle(bgr, (0, 0), (bgr.shape[1] - 1, bgr.shape[0] - 1), (0, 255, 0), 12)
            state = 'recorded'
        cv2.putText(bgr, f'touches {len(touches)}/{args.count}   {state}', (10, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.imshow('overhead registration: touch check', bgr)
        if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
            break
    cv2.destroyAllWindows()
    if args.save:
        with open(args.save, 'w') as f:
            json.dump(touches, f)
    if len(touches) < 3:
        sys.exit('need at least 3 touches')

    with open(args.extrinsics) as f:
        e = yaml.safe_load(f)
    from scipy.spatial.transform import Rotation
    R_bc, t_bc = Rotation.from_quat(e['rotation_xyzw']).as_matrix(), np.array(e['translation'])
    tips, cams = [], []
    print('\ntouch  corner  model tip (base, mm)      camera says corner is at (base, mm)   difference (mm)')
    for n, t in enumerate(touches):
        tip = jaw_tip_in_base(t['joints'])
        corners_base = (R_bc @ np.array(t['corners_cam']).T).T + t_bc
        k = int(np.argmin(np.linalg.norm(corners_base[:, :2] - tip[:2], axis=1)))
        diff = (tip - corners_base[k]) * 1e3
        tips.append(tip)
        cams.append(np.array(t['corners_cam'])[k])
        print(f'{n + 1:>5}  {k + 1:>6}  {np.round(tip * 1e3, 1)!s:<25} {np.round(corners_base[k] * 1e3, 1)!s:<37} '
              f'{np.round(diff, 1)}  |xy| {np.linalg.norm(diff[:2]):.1f}')
    tips, cams = np.array(tips), np.array(cams)
    # Camera pose from the touches alone: rigid fit of camera-frame corners onto model tips.
    ct, cc = tips.mean(0), cams.mean(0)
    U, _, Vt = np.linalg.svd((cams - cc).T @ (tips - ct))
    D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
    R_fit = Vt.T @ D @ U.T
    t_fit = ct - R_fit @ cc
    res = np.linalg.norm((R_fit @ cams.T).T + t_fit - tips, axis=1) * 1e3
    angle = np.degrees(np.linalg.norm(Rotation.from_matrix(R_fit.T @ R_bc).as_rotvec()))
    print(f'\ncamera pose from the touches alone: position {np.round(t_fit, 4)} m, '
          f'residual {res.mean():.1f} mm (max {res.max():.1f})')
    print(f'extrinsics file:                    position {np.round(t_bc, 4)} m; '
          f'differs by {np.linalg.norm(t_fit - t_bc) * 1e3:.0f} mm and {angle:.1f} deg')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    c = sub.add_parser('capture', help='save overhead images with the joint angles')
    c.add_argument('--dir', required=True)
    c.add_argument('--image-topic', default='/overhead_cam/image_raw')
    c.add_argument('--joint-topic', default='/follower/joint_states')
    c.add_argument('--count', type=int, default=8, help='views to capture')
    c.add_argument('--no-window', action='store_true')
    c.add_argument('--still-seconds', type=float, default=2.0)
    c.add_argument('--min-change', type=float, default=0.25,
                   help='a pose is new when some joint differs by this many radians')
    s = sub.add_parser('solve', help='find the camera pose from the captured views')
    s.add_argument('--dir', required=True)
    s.add_argument('--intrinsics', default=os.path.join(CONFIG, 'overhead_cam.yaml'))
    s.add_argument('--output')
    s.add_argument('--board', default=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'charuco_board.json'),
                   help='ChArUco board that may be lying flat on the table in the views')
    s.add_argument('--square-mm', type=float, help='measured square size of the printed board')
    s.add_argument('--board-thickness', type=float, default=0.0,
                   help='height of the board\'s printed surface above the table, m (e.g. its card backing)')
    s.add_argument('--table-z', type=float, default=-0.0024,
                   help='height of the table in the base frame, m (the model\'s base plate bottom)')
    s.add_argument('--no-board', action='store_true', help='ignore the board; solve all six parameters')
    s.add_argument('--show', action='store_true', help='show the fit improving in a window')
    t = sub.add_parser('touch', help='check the result by touching the jaw tip to the board corners')
    t.add_argument('--extrinsics', default=os.path.join(CONFIG, 'overhead_cam_extrinsics.yaml'))
    t.add_argument('--intrinsics', default=os.path.join(CONFIG, 'overhead_cam.yaml'))
    t.add_argument('--board', default=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'charuco_board.json'))
    t.add_argument('--square-mm', type=float, help='measured square size of the printed board')
    t.add_argument('--image-topic', default='/overhead_cam/image_raw')
    t.add_argument('--joint-topic', default='/follower/joint_states')
    t.add_argument('--count', type=int, default=4)
    t.add_argument('--still-seconds', type=float, default=2.0)
    t.add_argument('--save', help='also save the raw touches to this JSON file')
    args = ap.parse_args()
    if hasattr(args, 'dir'):
        args.dir = os.path.expanduser(args.dir)
    {'capture': capture, 'solve': solve, 'touch': touch}[args.cmd](args)


if __name__ == '__main__':
    main()
