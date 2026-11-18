"""Real cameras on the pipeline's topic names (docs/ros2-pipeline.md).

    ros2 launch so100_telemetry cameras.launch.py
    ros2 launch so100_telemetry cameras.launch.py leader_wrist:=true   # also the leader's camera

Publishes /wrist_cam/image_raw and /overhead_cam/image_raw (+ camera_info) at 640x480, 30 fps,
rgb8, and optionally /leader/wrist_cam/image_raw. Set a device to '' to skip that camera.
Rectification (image_rect) is added once intrinsics exist.

The two arm cameras are identical (same USB id and no serial number), so /dev/v4l/by-id
cannot tell them apart; the defaults use by-path, i.e. which USB socket each is plugged into.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

BY_PATH = '/dev/v4l/by-path/pci-0000:11:00.0-usb-0:%s:1.0-video-index0'
NUROUM = '/dev/v4l/by-id/usb-AuditoryWorks_Nuroum_V11_AW09LCAB8BB8019850-video-index0'


def camera(name, device, condition=None, frame=None):
    return Node(
        package='v4l2_camera', executable='v4l2_camera_node', name='camera', namespace=name,
        parameters=[{
            'video_device': device,
            'image_size': [640, 480],
            'time_per_frame': [1, 30],
            'pixel_format': 'YUYV',
            'output_encoding': 'rgb8',
            'camera_frame_id': frame or name,
        }],
        condition=condition, output='screen')


def generate_launch_description():
    def is_set(arg):
        return IfCondition(PythonExpression(["'", LaunchConfiguration(arg), "' != ''"]))

    return LaunchDescription([
        DeclareLaunchArgument('wrist_device', default_value=BY_PATH % '1.1',
                              description='follower wrist camera'),
        DeclareLaunchArgument('overhead_device', default_value=NUROUM),
        DeclareLaunchArgument('leader_wrist', default_value='false'),
        DeclareLaunchArgument('leader_wrist_device', default_value=BY_PATH % '1.3'),
        camera('wrist_cam', LaunchConfiguration('wrist_device'), is_set('wrist_device')),
        camera('overhead_cam', LaunchConfiguration('overhead_device'), is_set('overhead_device')),
        camera('leader/wrist_cam', LaunchConfiguration('leader_wrist_device'),
               IfCondition(LaunchConfiguration('leader_wrist')), frame='leader_wrist_cam'),
    ])
