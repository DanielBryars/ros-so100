#!/usr/bin/env python3
"""Small, slow copies of the camera images for viewing over Wi-Fi.

A raw 640x480 rgb8 stream is 27.6 MB/s per camera, far too much for the laptop's wireless
link. This node republishes each camera at half size and a few frames per second on
/preview/<camera> (about 1 MB/s each), which is plenty for a second RViz screen. Nothing in
the pipeline reads these topics.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class ImagePreview(Node):
    def __init__(self):
        super().__init__('image_preview')
        topics = self.declare_parameter(
            'topics', ['/wrist_cam/image_raw', '/overhead_cam/image_raw',
                       '/leader/wrist_cam/image_raw']).value
        self.rate_hz = self.declare_parameter('rate_hz', 5.0).value
        self.step = self.declare_parameter('downscale', 2).value
        self.latest = {}
        self.pubs = {}
        for topic in topics:
            name = topic.strip('/').rsplit('/', 1)[0].replace('/', '_')   # leader/wrist_cam -> leader_wrist_cam
            self.pubs[topic] = self.create_publisher(Image, '/preview/' + name, qos_profile_sensor_data)
            self.create_subscription(Image, topic, lambda m, t=topic: self.latest.__setitem__(t, m),
                                     qos_profile_sensor_data)
        self.create_timer(1.0 / self.rate_hz, self.publish)

    def publish(self):
        latest, self.latest = self.latest, {}
        for topic, msg in latest.items():
            if msg.encoding not in ('rgb8', 'bgr8'):
                continue
            a = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step // 3, 3)
            small = np.ascontiguousarray(a[::self.step, :msg.width:self.step])
            out = Image()
            out.header = msg.header
            out.height, out.width = small.shape[0], small.shape[1]
            out.encoding = msg.encoding
            out.step = small.shape[1] * 3
            out.data = small.tobytes()
            self.pubs[topic].publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ImagePreview()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
