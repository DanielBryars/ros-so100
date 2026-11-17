#!/usr/bin/env python3
"""Forward the leader arm's joint states to /joint_command (teleoperation).

The leader driver keeps publishing /leader/joint_states (so RViz still shows it); this node
is the single /joint_command publisher while teleoperating. Stop it before enabling a policy.
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class TeleopRelay(Node):
    def __init__(self):
        super().__init__('teleop_relay')
        source = self.declare_parameter('source', '/leader/joint_states').value
        self.pub = self.create_publisher(JointState, '/joint_command', 1)
        self.sub = self.create_subscription(JointState, source, self.on_state, 1)
        self.get_logger().info('Relaying %s -> /joint_command' % source)

    def on_state(self, msg):
        if any(math.isnan(p) for p in msg.position):
            return  # a servo did not answer; let the backend hold the last command
        out = JointState()
        out.header = msg.header
        out.name = msg.name
        out.position = msg.position
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
