"""USB camera -> image_raw (rgb8) + camera_info, capturing in MJPEG.

Replaces v4l2_camera for the bench cameras: v4l2_camera 0.8 cannot read MJPEG, and in raw
YUYV the wrist cameras deliver a few percent of frames torn (bottom rows from another frame).
In MJPEG the same cameras gave no bad frames.

    ros2 run so100_telemetry camera_node --ros-args -r __ns:=/wrist_cam -p device:=/dev/video0

Parameters: device, width, height, fps, fourcc, frame_id, camera_info_file (a ROS
camera_info YAML as written by tools/calibration/calibrate_intrinsics.py; optional).
If the camera unplugs, the node keeps retrying once a second.
"""
import os
import time

import cv2
import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


def load_camera_info(path, width, height):
    info = CameraInfo(width=width, height=height)
    if not path or not os.path.exists(path):
        return info, False
    with open(path) as f:
        y = yaml.safe_load(f)
    if (y['image_width'], y['image_height']) != (width, height):
        raise ValueError(f"{path} is for {y['image_width']}x{y['image_height']}, "
                         f"camera runs at {width}x{height}")
    info.distortion_model = y.get('distortion_model', 'plumb_bob')
    info.d = [float(v) for v in y['distortion_coefficients']['data']]
    info.k = [float(v) for v in y['camera_matrix']['data']]
    info.r = [float(v) for v in y['rectification_matrix']['data']]
    info.p = [float(v) for v in y['projection_matrix']['data']]
    return info, True


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera')
        p = self.declare_parameter
        self.device = p('device', '/dev/video0').value
        self.width = p('width', 640).value
        self.height = p('height', 480).value
        self.fps = p('fps', 30).value
        self.fourcc = p('fourcc', 'MJPG').value
        self.frame_id = p('frame_id', 'camera').value
        info_file = p('camera_info_file', '').value

        self.info, calibrated = load_camera_info(info_file, self.width, self.height)
        self.info.header.frame_id = self.frame_id
        self.get_logger().info(
            f'{self.device}: ' + (f'calibration from {info_file}' if calibrated
                                  else 'no calibration file, camera_info has no K'))
        self.image_pub = self.create_publisher(Image, 'image_raw', 1)
        self.info_pub = self.create_publisher(CameraInfo, 'camera_info', 1)
        self.cap = None

    def open(self):
        cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not cap.isOpened():
            return False
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        got = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        fourcc = int(cap.get(cv2.CAP_PROP_FOURCC)).to_bytes(4, 'little').decode(errors='replace')
        if got != (self.width, self.height):
            cap.release()
            raise RuntimeError(f'{self.device} gave {got}, wanted {(self.width, self.height)}')
        self.get_logger().info(f'{self.device}: streaming {got[0]}x{got[1]} {fourcc}')
        self.cap = cap
        return True

    def run(self):
        while rclpy.ok():
            if self.cap is None and not self.open():
                self.get_logger().warning(f'cannot open {self.device}, retrying',
                                          throttle_duration_sec=10.0)
                time.sleep(1.0)
                continue
            ok, bgr = self.cap.read()
            stamp = self.get_clock().now().to_msg()
            if not ok:
                self.get_logger().warning(f'{self.device}: read failed, reopening',
                                          throttle_duration_sec=10.0)
                self.cap.release()
                self.cap = None
                time.sleep(1.0)
                continue
            msg = Image(height=self.height, width=self.width, encoding='rgb8',
                        is_bigendian=0, step=self.width * 3)
            msg.header.stamp = stamp
            msg.header.frame_id = self.frame_id
            msg.data = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB).tobytes()
            self.info.header.stamp = stamp
            self.image_pub.publish(msg)
            self.info_pub.publish(self.info)


def main():
    rclpy.init()
    node = CameraNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        if node.cap is not None:
            node.cap.release()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
