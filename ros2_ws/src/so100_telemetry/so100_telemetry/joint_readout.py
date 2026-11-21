#!/usr/bin/env python3
"""A table of both arms' joints as an image, for an RViz Image display: raw encoder counts
(as read from the servos) next to the calibrated angle in radians and degrees.

Subscribes to /<arm>/joint_counts and /<arm>/joint_states for each arm, publishes
/joint_readout (rgb8, a few frames per second, small enough for Wi-Fi).
"""
import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Int32MultiArray


class JointReadout(Node):
    def __init__(self):
        super().__init__('joint_readout')
        self.arms = self.declare_parameter('arms', ['leader', 'follower']).value
        rate_hz = self.declare_parameter('rate_hz', 4.0).value
        self.states, self.counts = {}, {}
        for arm in self.arms:
            self.create_subscription(JointState, f'/{arm}/joint_states',
                                     lambda m, a=arm: self.states.__setitem__(a, m), 1)
            self.create_subscription(Int32MultiArray, f'/{arm}/joint_counts',
                                     lambda m, a=arm: self.counts.__setitem__(a, m), 1)
        self.pub = self.create_publisher(Image, '/joint_readout', qos_profile_sensor_data)
        self.create_timer(1.0 / rate_hz, self.publish)

    def publish(self):
        col, row = 215, 22
        img = np.full((row * 9, 120 + col * len(self.arms), 3), 30, np.uint8)

        def text(s, x, y, colour=(230, 230, 230)):
            cv2.putText(img, s, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, colour, 1, cv2.LINE_AA)

        names = []
        for n, arm in enumerate(self.arms):
            x = 120 + col * n
            text(arm, x, row, (120, 220, 255))
            text('counts    rad     deg', x, 2 * row, (160, 160, 160))
            state, counts = self.states.get(arm), self.counts.get(arm)
            if state is None:
                text('no data', x, 3 * row, (255, 120, 120))
                continue
            names = names or list(state.name)
            for i, q in enumerate(state.position):
                c = counts.data[i] if counts is not None and i < len(counts.data) else -1
                if math.isfinite(q):
                    text('%5d  %+6.3f  %+6.1f' % (c, q, math.degrees(q)), x, (3 + i) * row)
                else:
                    text('no answer', x, (3 + i) * row, (255, 120, 120))
        for i, name in enumerate(names):
            text(name, 6, (3 + i) * row, (160, 160, 160))
        msg = Image(height=img.shape[0], width=img.shape[1], encoding='rgb8', is_bigendian=0,
                    step=img.shape[1] * 3, data=img.tobytes())
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointReadout()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
