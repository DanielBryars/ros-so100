#!/usr/bin/env python3
"""A minimal ROS2 node that reads SO100 servo positions and publishes them.

It publishes sensor_msgs/JointState on the /joint_states topic. That is the
standard topic/message pair for joint telemetry in ROS2, and it is exactly what
RViz + robot_state_publisher consume later for visualization. The raw encoder counts the
angles were computed from go out on joint_counts (std_msgs/Int32MultiArray, same joint
order; -1 for a servo that did not answer), for checking the calibration.

Read-only by default: torque is never enabled and you can move the arm by hand and watch
the numbers change. This is the leader driver.

With `commandable:=true` (and a calibration) it is also the follower driver: it accepts
position targets in radians on `joint_command` (sensor_msgs/JointState, by joint name) once
torque has been switched on with the `enable` service (std_srvs/SetBool). Safety:
  - targets are clamped to the joint limits (less `limit_margin`),
  - the setpoint sent to the servos moves at no more than `max_velocity` rad/s,
  - the setpoint never leads the measured position by more than `max_lag` rad, so a blocked
    joint pushes with a bounded force instead of winding up,
  - the servos' Torque_Limit is set to `torque_limit` (0-1000 = 0-100%) while enabled,
  - if commands stop for `command_timeout` s the arm stops where it is and holds.
Enabling holds the present pose. Disabling drops torque: the arm falls, so park it first.
Torque is left as it is when the node exits (the servos keep holding their last goal).
"""
import json
import math
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray
from std_srvs.srv import SetBool

# The Feetech SDK is vendored as a subpackage so this works without pip.
from .scservo_sdk import PortHandler, PacketHandler, GroupSyncWrite, COMM_SUCCESS

# STS/SMS registers. Present position, speed and load are 2 bytes each from address 56,
# so one 6-byte read gets all three.
ADDR_TORQUE_ENABLE = 40
ADDR_GOAL_POSITION = 42
ADDR_TORQUE_LIMIT = 48
ADDR_PRESENT_POSITION = 56
COUNTS_PER_RAD = 4096.0 / (2.0 * math.pi)


def sign_magnitude(value, sign_bit):
    """Feetech speed and load are a magnitude with a separate sign bit."""
    magnitude = value & ((1 << sign_bit) - 1)
    return -magnitude if value & (1 << sign_bit) else magnitude

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


def load_calibrated_mapping(calibration_file, urdf_file, joint_names):
    """Build per-joint (count_min, count_max, rad_lower, rad_upper) for a calibrated mapping.

    The LeRobot calibration JSON gives each motor's encoder range (range_min..range_max);
    the URDF gives the matching joint limits in radians. Mapping one linearly onto the other
    is exactly what the thesis pipeline does (via its -100..100 "normalised" units), so the
    angles published here are the ones the MuJoCo sim would be driven to.
    """
    with open(calibration_file) as f:
        calib = json.load(f)
    limits = {}
    for joint in ET.parse(urdf_file).getroot().findall('joint'):
        limit = joint.find('limit')
        if limit is not None:
            limits[joint.get('name')] = (float(limit.get('lower')), float(limit.get('upper')))
    mapping = []
    for name in joint_names:
        lower, upper = limits[name]
        mapping.append((calib[name]['range_min'], calib[name]['range_max'], lower, upper))
    return mapping


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
        # Optional calibrated mapping (both must be set). Without them the node keeps the
        # original uncalibrated counts_to_radians() behaviour.
        calibration_file = self.declare_parameter('calibration_file', '').value
        urdf_file = self.declare_parameter('urdf_file', '').value
        self.mapping = None
        if calibration_file and urdf_file:
            self.mapping = load_calibrated_mapping(calibration_file, urdf_file, self.joint_names)
            self.get_logger().info('Calibrated mapping from %s' % calibration_file)

        # --- Follower mode: accept commands (off by default; see the module docstring). ---
        self.commandable = self.declare_parameter('commandable', False).value
        self.max_velocity = self.declare_parameter('max_velocity', 0.5).value      # rad/s
        self.max_lag = self.declare_parameter('max_lag', 0.2).value                # rad
        self.limit_margin = self.declare_parameter('limit_margin', 0.02).value     # rad
        self.torque_limit = self.declare_parameter('torque_limit', 400).value      # 0-1000
        self.command_timeout = self.declare_parameter('command_timeout', 0.2).value  # s
        if self.commandable and self.mapping is None:
            raise RuntimeError('commandable needs calibration_file and urdf_file')
        self.enabled = False
        self.measured = None      # last complete reading, rad
        self.setpoint = None      # what the servos are being told, rad
        self.command = None       # where the setpoint is heading, rad
        self.command_time = 0.0

        # --- Open the serial bus to the servos. ---
        self.packet = PacketHandler(0)  # 0 = STS/SMS little-endian framing
        self.port = None
        self._next_reconnect = 0.0
        self.open_port()

        # --- Create the publisher: topic name, message type, queue depth. ---
        # Relative name: it is /joint_states normally, /leader/joint_states in namespace leader.
        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self.counts_pub = self.create_publisher(Int32MultiArray, 'joint_counts', 10)

        # --- Create a timer that fires the callback at rate_hz. ---
        self.timer = self.create_timer(1.0 / self.rate_hz, self.publish_once)
        self._warned = False
        if self.commandable:
            self.create_subscription(JointState, 'joint_command', self.on_command, 1)
            self.create_service(SetBool, 'enable', self.on_enable)
            self.get_logger().info('Commandable: torque is off until the enable service is called')

    # ---- follower mode ----

    def to_counts(self, i, rad):
        c_min, c_max, lower, upper = self.mapping[i]
        return int(round(c_min + (rad - lower) / (upper - lower) * (c_max - c_min)))

    def clamp(self, i, rad):
        _, _, lower, upper = self.mapping[i]
        return min(max(rad, lower + self.limit_margin), upper - self.limit_margin)

    def write_all(self, address, length, values):
        group = GroupSyncWrite(self.port, self.packet, address, length)
        for sid, v in zip(self.ids, values):
            group.addParam(sid, [(v >> (8 * b)) & 0xFF for b in range(length)])
        if group.txPacket() != COMM_SUCCESS:
            raise RuntimeError('sync write to address %d failed' % address)

    def on_enable(self, request, response):
        try:
            if request.data:
                if self.port is None or self.measured is None:
                    raise RuntimeError('no reading from the arm yet')
                self.setpoint = list(self.measured)   # hold where it is: no jump on enable
                self.command = list(self.measured)
                self.write_all(ADDR_TORQUE_LIMIT, 2, [int(self.torque_limit)] * len(self.ids))
                self.write_all(ADDR_GOAL_POSITION, 2,
                               [self.to_counts(i, q) for i, q in enumerate(self.setpoint)])
                self.write_all(ADDR_TORQUE_ENABLE, 1, [1] * len(self.ids))
                self.enabled = True
            else:
                self.enabled = False
                if self.port is not None:
                    self.write_all(ADDR_TORQUE_ENABLE, 1, [0] * len(self.ids))
            response.success = True
            response.message = 'torque on, holding' if self.enabled else 'torque off'
            self.get_logger().warning(response.message)
        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error('enable failed: %s' % e)
        return response

    def on_command(self, msg):
        if not self.enabled:
            return
        for name, q in zip(msg.name, msg.position):
            if name in self.joint_names and math.isfinite(q):
                i = self.joint_names.index(name)
                self.command[i] = self.clamp(i, q)
        self.command_time = time.monotonic()

    def drive(self):
        """Move the setpoint one tick towards the command and send it."""
        if self.command_timeout > 0 and time.monotonic() - self.command_time > self.command_timeout:
            self.command = list(self.setpoint)   # commands stopped: stop here and hold
        step = self.max_velocity / self.rate_hz
        for i, target in enumerate(self.command):
            q = self.setpoint[i] + min(max(target - self.setpoint[i], -step), step)
            q = min(max(q, self.measured[i] - self.max_lag), self.measured[i] + self.max_lag)
            self.setpoint[i] = q
        self.write_all(ADDR_GOAL_POSITION, 2,
                       [self.to_counts(i, q) for i, q in enumerate(self.setpoint)])

    def open_port(self):
        self.port = PortHandler(self.port_name)
        if not self.port.openPort():
            raise RuntimeError('Failed to open serial port %s' % self.port_name)
        if not self.port.setBaudRate(self.baud):
            raise RuntimeError('Failed to set baud %d' % self.baud)
        self.get_logger().info(
            'Opened %s @ %d bps, reading IDs %s' % (self.port_name, self.baud, self.ids))

    def publish_once(self):
        """Read and publish, surviving a USB drop-out.

        The adapter occasionally drops off the bus and re-enumerates (seen as "disabled by
        hub (EMI?)" in dmesg). Nothing is published while it is gone; we retry once a second.
        """
        now = time.monotonic()
        if self.port is None:
            if now < self._next_reconnect:
                return
            try:
                self.open_port()
                self._warned = False
            except Exception as e:  # device node not back yet
                self.port = None
                self._next_reconnect = now + 1.0
                self.get_logger().warning('Reconnect failed: %s' % e, throttle_duration_sec=10.0)
                return
        try:
            self.read_and_publish()
        except Exception as e:  # SerialException, termios.error, or a short read
            self.get_logger().error('Serial link lost (%s); will reconnect.' % e)
            try:
                self.port.closePort()
            except Exception:
                pass
            self.port = None
            self._next_reconnect = now + 1.0

    def read_and_publish(self):
        """Read every servo and publish one JointState message."""
        positions, velocities, efforts, raw = [], [], [], []
        ok = True
        for i, sid in enumerate(self.ids):
            data, comm, err = self.packet.readTxRx(self.port, sid, ADDR_PRESENT_POSITION, 6)
            if comm != COMM_SUCCESS or len(data) != 6:
                positions.append(math.nan)  # mark unreadable joints, keep going
                velocities.append(math.nan)
                efforts.append(math.nan)
                raw.append(-1)
                ok = False
                continue
            counts = data[0] | (data[1] << 8)
            raw.append(counts)
            if self.mapping is not None:
                c_min, c_max, lower, upper = self.mapping[i]
                # Deliberately not clamped: an out-of-limit angle is a calibration clue.
                positions.append(lower + (counts - c_min) / (c_max - c_min) * (upper - lower))
            else:
                positions.append(counts_to_radians(counts))
            velocities.append(sign_magnitude(data[2] | (data[3] << 8), 15) / COUNTS_PER_RAD)
            # Load as a fraction of full torque (-1..1), not N m.
            efforts.append(sign_magnitude(data[4] | (data[5] << 8), 10) / 1000.0)

        msg = JointState()
        # Stamp every message with the current time; consumers rely on this.
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.joint_names)
        msg.position = positions
        msg.velocity = velocities
        msg.effort = efforts
        self.pub.publish(msg)
        self.counts_pub.publish(Int32MultiArray(data=raw))

        if ok:
            self.measured = positions
            if self.enabled:
                self.drive()
        if not ok and not self._warned:
            self.get_logger().warning(
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
        if node.port is not None:
            node.port.closePort()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
