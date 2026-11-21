"""Show the leader and follower arms side by side in RViz, with calibrated joint angles.

Uses the SO-101 URDF (same joint names and zero convention as the MuJoCo sim) and the
LeRobot calibration JSONs in this package's config/, so what RViz shows is the pose the sim
would take for each arm. Read-only unless `drive:=true`, which lets each arm accept
/<arm>/joint_command once its torque is switched on with the /<arm>/enable service
(see joint_state_publisher.py for the safety limits). Also starts joint_readout, a table
of raw counts and calibrated angles for both arms on /joint_readout (shown in RViz).

Machine with the arms plugged in (publishers only, RViz elsewhere):
    ros2 launch so100_telemetry dual_arm.launch.py

Machine with the screen (RViz only, subscribes over the LAN):
    ros2 launch so100_telemetry dual_arm.launch.py publishers:=false use_rviz:=true

Everything on one machine:
    ros2 launch so100_telemetry dual_arm.launch.py use_rviz:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Stable names for the two controller boards (USB serial numbers), so the arms do not
# swap when /dev/ttyACM* enumeration order changes.
LEADER_PORT = '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AB0181835-if00'
FOLLOWER_PORT = '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B3E089561-if00'


def generate_launch_description():
    desc_share = get_package_share_directory('so101_description')
    urdf_path = os.path.join(desc_share, 'urdf', 'so101_new_calib.urdf')
    rviz_config = os.path.join(desc_share, 'rviz', 'dual_arm.rviz')
    config_dir = os.path.join(get_package_share_directory('so100_telemetry'), 'config')
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    def arm(name, port_arg, y, commandable=False):
        return [
            Node(
                package='so100_telemetry', executable='joint_state_publisher', namespace=name,
                parameters=[{
                    'port': LaunchConfiguration(port_arg),
                    'rate_hz': 30.0,
                    'calibration_file': os.path.join(config_dir, name + '_so100.json'),
                    'urdf_file': urdf_path,
                    'commandable': commandable,
                }],
            ),
            Node(
                package='robot_state_publisher', executable='robot_state_publisher', namespace=name,
                parameters=[{'robot_description': robot_description, 'frame_prefix': name + '/'}],
            ),
            Node(
                package='tf2_ros', executable='static_transform_publisher', namespace=name,
                arguments=['--y', str(y), '--frame-id', 'world', '--child-frame-id', name + '/base_link'],
            ),
        ]

    drive = ParameterValue(LaunchConfiguration('drive'), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument('publishers', default_value='true',
                              description='Read the arms and publish joint states + TF'),
        DeclareLaunchArgument('use_rviz', default_value='false',
                              description='Launch RViz2 on this machine'),
        DeclareLaunchArgument('drive', default_value='false',
                              description='Let the arms accept joint commands (torque still '
                                          'needs the /<arm>/enable service)'),
        DeclareLaunchArgument('leader_port', default_value=LEADER_PORT),
        DeclareLaunchArgument('follower_port', default_value=FOLLOWER_PORT),
        GroupAction(arm('leader', 'leader_port', -0.2, drive) + arm('follower', 'follower_port', 0.2, drive)
                    + [Node(package='so100_telemetry', executable='joint_readout')],
                    condition=IfCondition(LaunchConfiguration('publishers'))),
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config],
             condition=IfCondition(LaunchConfiguration('use_rviz'))),
    ])
