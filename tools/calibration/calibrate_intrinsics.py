#!/usr/bin/env python
"""Camera intrinsics from a ChArUco board, live from a ROS image topic.

    python calibrate_intrinsics.py --camera wrist_cam
    python calibrate_intrinsics.py --camera overhead_cam --min-views 40

Hold the printed board (charuco_board.png next to this script, 6x9 squares of 30 mm)
in front of the camera and move it around: near and far, tilted, and into every corner of
the image. A view is kept automatically when the board is sharp enough, well detected and
different from the views already kept. The window shows which parts of the image are covered.
Keys: C = calibrate now, U = drop the last view, Q/Esc = quit without saving.

Writes a ROS camera_info YAML (default ros2_ws/src/so100_telemetry/config/<camera>.yaml).
Run with ~/venvs/so100/bin/python after sourcing ROS (needs OpenCV >= 4.7 and rclpy).
"""
import argparse
import json
import os
import sys

import cv2
import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DEFAULT_BOARD = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'charuco_board.json')


def load_board(path):
    with open(path) as f:
        spec = json.load(f)
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, 'DICT_' + spec['dict']))
    board = cv2.aruco.CharucoBoard((spec['squares_x'], spec['squares_y']),
                                   spec['square_mm'] / 1000.0, spec['marker_mm'] / 1000.0,
                                   dictionary)
    return board


class Collector:
    """Keeps a diverse set of board views and solves for K and distortion."""

    def __init__(self, board, min_corners=12, min_shift_px=25.0):
        self.board = board
        self.detector = cv2.aruco.CharucoDetector(board)
        self.min_corners = min_corners
        self.min_shift_px = min_shift_px
        self.views = []          # list of (corners Nx1x2, ids Nx1)
        self.size = None         # (width, height)

    def detect(self, gray):
        corners, ids, _, _ = self.detector.detectBoard(gray)
        if ids is None or len(ids) < self.min_corners:
            return None
        return corners, ids

    def is_new(self, corners, ids):
        """True if this view differs from every kept one (mean shift of the shared corners)."""
        cur = {int(i): c for i, c in zip(ids.ravel(), corners.reshape(-1, 2))}
        for kc, ki in self.views:
            shared = [(cur[int(i)], c) for i, c in zip(ki.ravel(), kc.reshape(-1, 2)) if int(i) in cur]
            if len(shared) >= 6:
                shift = np.mean([np.linalg.norm(a - b) for a, b in shared])
                if shift < self.min_shift_px:
                    return False
        return True

    def offer(self, gray):
        """Detect the board; keep the view if it adds something. Returns (detection, kept)."""
        self.size = (gray.shape[1], gray.shape[0])
        det = self.detect(gray)
        if det is None:
            return None, False
        kept = self.is_new(*det)
        if kept:
            self.views.append(det)
        return det, kept

    def coverage(self, grid=(8, 6)):
        """Fraction of image cells (grid columns x rows) that contain a kept corner."""
        if not self.views or self.size is None:
            return 0.0, np.zeros(grid[::-1], bool)
        cells = np.zeros(grid[::-1], bool)
        for corners, _ in self.views:
            p = corners.reshape(-1, 2)
            cx = np.clip((p[:, 0] / self.size[0] * grid[0]).astype(int), 0, grid[0] - 1)
            cy = np.clip((p[:, 1] / self.size[1] * grid[1]).astype(int), 0, grid[1] - 1)
            cells[cy, cx] = True
        return float(cells.mean()), cells

    def calibrate(self):
        obj_pts, img_pts = [], []
        for corners, ids in self.views:
            o, i = self.board.matchImagePoints(corners, ids)
            if o is not None and len(o) >= 6:
                obj_pts.append(o)
                img_pts.append(i)
        rms, K, dist, _, _ = cv2.calibrateCamera(obj_pts, img_pts, self.size, None, None)
        return rms, K, dist.ravel()[:5]


def write_camera_info(path, name, size, K, dist):
    """ROS camera_info YAML, as read by camera_info_manager (plumb_bob, 5 coefficients)."""
    w, h = size
    newK, _ = cv2.getOptimalNewCameraMatrix(K, dist, (w, h), 0)
    P = np.hstack([newK, np.zeros((3, 1))])
    fmt = lambda a: '[' + ', '.join('%.6f' % v for v in np.asarray(a).ravel()) + ']'
    text = (f"image_width: {w}\nimage_height: {h}\ncamera_name: {name}\n"
            f"camera_matrix:\n  rows: 3\n  cols: 3\n  data: {fmt(K)}\n"
            f"distortion_model: plumb_bob\n"
            f"distortion_coefficients:\n  rows: 1\n  cols: 5\n  data: {fmt(dist)}\n"
            f"rectification_matrix:\n  rows: 3\n  cols: 3\n  data: {fmt(np.eye(3))}\n"
            f"projection_matrix:\n  rows: 3\n  cols: 4\n  data: {fmt(P)}\n")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w') as f:
        f.write(text)


def draw(frame_bgr, collector, det, kept, min_views):
    out = frame_bgr.copy()
    frac, cells = collector.coverage()
    h, w = out.shape[:2]
    gy, gx = cells.shape
    for r in range(gy):
        for c in range(gx):
            if not cells[r, c]:   # shade the parts of the image no corner has reached yet
                x0, y0 = c * w // gx, r * h // gy
                roi = out[y0:(r + 1) * h // gy, x0:(c + 1) * w // gx]
                roi[:] = (roi * 0.6 + np.array([0, 0, 90])).astype(np.uint8)
    if det is not None:
        cv2.aruco.drawDetectedCornersCharuco(out, det[0], det[1], (0, 255, 0) if kept else (0, 200, 255))
    cv2.putText(out, f"views {len(collector.views)}/{min_views}  coverage {frac * 100:.0f}%",
                (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--camera', required=True, help='camera namespace, e.g. wrist_cam')
    ap.add_argument('--topic', help='image topic (default /<camera>/image_raw)')
    ap.add_argument('--board', default=DEFAULT_BOARD, help='ChArUco board JSON')
    ap.add_argument('--min-views', type=int, default=30)
    ap.add_argument('--output', help='camera_info YAML to write')
    args = ap.parse_args()
    topic = args.topic or f'/{args.camera}/image_raw'
    output = args.output or os.path.join(
        REPO, 'ros2_ws', 'src', 'so100_telemetry', 'config', args.camera + '.yaml')

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    collector = Collector(load_board(args.board))
    latest = {}

    def on_image(msg):
        if msg.encoding not in ('rgb8', 'bgr8'):
            raise RuntimeError('unsupported encoding ' + msg.encoding)
        a = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step // 3, 3)[:, :msg.width]
        latest['bgr'] = a[:, :, ::-1].copy() if msg.encoding == 'rgb8' else a.copy()

    rclpy.init()
    node = Node('calibrate_intrinsics')
    node.create_subscription(Image, topic, on_image, qos_profile_sensor_data)
    print(f'Waiting for images on {topic} ...')
    done = False
    try:
        while rclpy.ok() and not done:
            rclpy.spin_once(node, timeout_sec=0.05)
            bgr = latest.pop('bgr', None)
            if bgr is None:
                continue
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            sharp = cv2.Laplacian(gray, cv2.CV_64F).var() > 30.0   # skip motion-blurred frames
            det, kept = collector.offer(gray) if sharp else (collector.detect(gray), False)
            cv2.imshow('intrinsics: ' + args.camera, draw(bgr, collector, det, kept, args.min_views))
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                print('Quit without saving.')
                return
            if key == ord('u') and collector.views:
                collector.views.pop()
            frac, _ = collector.coverage()
            done = key == ord('c') or (len(collector.views) >= args.min_views and frac >= 0.9)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        cv2.destroyAllWindows()

    if len(collector.views) < 8:
        sys.exit(f'Only {len(collector.views)} views; need at least 8.')
    rms, K, dist = collector.calibrate()
    write_camera_info(output, args.camera, collector.size, K, dist)
    print(f'{len(collector.views)} views, RMS reprojection error {rms:.3f} px (aim for < 0.5)')
    print(f'fx {K[0, 0]:.1f}  fy {K[1, 1]:.1f}  cx {K[0, 2]:.1f}  cy {K[1, 2]:.1f}')
    print('distortion', np.round(dist, 4))
    print('Wrote', output)


if __name__ == '__main__':
    main()
