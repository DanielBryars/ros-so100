#!/usr/bin/env python3
"""A minimal ROS2 node that reads SO100 servo positions and publishes them.

It publishes sensor_msgs/JointState on the /joint_states topic. That is the
standard topic/message pair for joint telemetry in ROS2, and it is exactly what
RViz + robot_state_publisher consume later for visualization.

Read-only: this node never enables torque or commands the motors. You can move
the arm by hand and watch the numbers change.
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# The Feetech SDK is vendored as a subpackage so this works without pip.
from .scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS

# STS/SMS register: Present_Position lives at address 56 and is 2 bytes.
ADDR_PRESENT_POSITION = 56

# Joint names follow the common SO100 convention, bus IDs 1..6 in order.
DEFAULT_JOINT_NAMES = [
    'shoulder_pan',   # ID 1
    'shoulder_lift',  # ID 2
    'elbow_flex',     # ID 3
    'wrist_flex',     # ID 4
    'wrist_roll',     # ID 5
    'gripper',        # ID 6
]


def counts_to_radians(counts):
    """Map a raw 12-bit encoder count (0..4095) to radians.

    4096 counts = one full revolution = 2*pi rad. We center on 2048 so that
    mid-travel reads ~0 rad. This is an UNCALIBRATED mapping: it has no per-joint
    home offset or direction sign yet. That calibration comes with the URDF step.
    """
    return (counts - 2048) / 4096.0 * 2.0 * math.pi


class JointStatePublisher(Node):
    def __init__(self):
        # Every ROS2 node has a name; this is how it appears in `ros2 node list`.
        super().__init__('so100_joint_state_publisher')

        # --- ROS parameters: configurable at launch without editing code. ---
        # Override like: ros2 run ... --ros-args -p rate_hz:=20.0 -p port:=/dev/ttyACM1
        self.port_name = self.declare_parameter('port', '/dev/ttyACM0').value
        self.baud = self.declare_parameter('baud', 1000000).value
        self.rate_hz = self.declare_parameter('rate_hz', 10.0).value
        self.ids = self.declare_parameter('ids', [1, 2, 3, 4, 5, 6]).value
        self.joint_names = self.declare_parameter('joint_names', DEFAULT_JOINT_NAMES).value

        # --- Open the serial bus to the servos. ---
        self.port = PortHandler(self.port_name)
        self.packet = PacketHandler(0)  # 0 = STS/SMS little-endian framing
        if not self.port.openPort():
            raise RuntimeError('Failed to open serial port %s' % self.port_name)
        if not self.port.setBaudRate(self.baud):
            raise RuntimeError('Failed to set baud %d' % self.baud)
        self.get_logger().info(
            'Opened %s @ %d bps, reading IDs %s' % (self.port_name, self.baud, self.ids))

        # --- Create the publisher: topic name, message type, queue depth. ---
        self.pub = self.create_publisher(JointState, '/joint_states', 10)

        # --- Create a timer that fires the callback at rate_hz. ---
        self.timer = self.create_timer(1.0 / self.rate_hz, self.publish_once)
        self._warned = False

    def publish_once(self):
        """Read every servo and publish one JointState message."""
        positions = []
        ok = True
        for sid in self.ids:
            counts, comm, err = self.packet.read2ByteTxRx(
                self.port, sid, ADDR_PRESENT_POSITION)
            if comm == COMM_SUCCESS:
                positions.append(counts_to_radians(counts))
            else:
                positions.append(math.nan)  # mark unreadable joints, keep going
                ok = False

        msg = JointState()
        # Stamp every message with the current time; consumers rely on this.
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.joint_names)
        msg.position = positions
        # velocity and effort are left empty for now (valid: empty arrays).
        self.pub.publish(msg)

        if not ok and not self._warned:
            self.get_logger().warn(
                'One or more servos did not respond; publishing NaN for those. '
                '(This warning is shown once.)')
            self._warned = True


def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    try:
        rclpy.spin(node)  # process timers/callbacks until Ctrl-C
    except KeyboardInterrupt:
        pass
    finally:
        node.port.closePort()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
