"""Launch RViz visualization of the real SO100 arm.

Starts three nodes:
  1. robot_state_publisher - loads the URDF and turns joint angles into 3D
     transforms (TF) that RViz can draw.
  2. our joint_state_publisher - reads the real servos and publishes
     /joint_states, told to use the URDF's joint names.
  3. rviz2 (optional) - the 3D viewer, preloaded with the arm's RViz config.

Run on the laptop's own screen:
    ros2 launch so100_telemetry display.launch.py

Run WITHOUT rviz (e.g. when viewing from another machine):
    ros2 launch so100_telemetry display.launch.py use_rviz:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Order matches servo bus IDs 1..6; these are the joint names used in the URDF.
URDF_JOINT_NAMES = [
    'Shoulder_Rotation',  # ID 1
    'Shoulder_Pitch',     # ID 2
    'Elbow',              # ID 3
    'Wrist_Pitch',        # ID 4
    'Wrist_Roll',         # ID 5
    'Gripper',            # ID 6
]


def generate_launch_description():
    desc_share = get_package_share_directory('so_arm_100_description')
    urdf_path = os.path.join(desc_share, 'urdf', 'so_arm_100_5dof.urdf')
    rviz_config = os.path.join(desc_share, 'rviz', 'so_arm_100.rviz')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    use_rviz = LaunchConfiguration('use_rviz')
    port = LaunchConfiguration('port')

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='Launch RViz2 on this machine'),
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0',
                              description='Serial port of the SO100 controller'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='so100_telemetry',
            executable='joint_state_publisher',
            parameters=[{'joint_names': URDF_JOINT_NAMES, 'port': port}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            condition=IfCondition(use_rviz),
        ),
    ])
