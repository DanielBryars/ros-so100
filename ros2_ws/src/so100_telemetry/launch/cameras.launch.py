"""Real cameras on the pipeline's topic names (docs/ros2-pipeline.md).

    ros2 launch so100_telemetry cameras.launch.py
    ros2 launch so100_telemetry cameras.launch.py overhead_device:=/dev/video4

Publishes /wrist_cam/image_raw + camera_info (and /overhead_cam/... when overhead_device is
set) at 640x480, 30 fps, rgb8. Rectification (image_rect) is added once intrinsics exist.

The two arm cameras are identical (same USB id and no serial number), so /dev/v4l/by-id
cannot tell them apart; the defaults use by-path, i.e. which USB socket each is plugged into.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

BY_PATH = '/dev/v4l/by-path/pci-0000:11:00.0-usb-0:%s:1.0-video-index0'


def camera(name, device, condition=None):
    return Node(
        package='v4l2_camera', executable='v4l2_camera_node', name='camera', namespace=name,
        parameters=[{
            'video_device': device,
            'image_size': [640, 480],
            'time_per_frame': [1, 30],
            'pixel_format': 'YUYV',
            'output_encoding': 'rgb8',
            'camera_frame_id': name,
        }],
        condition=condition, output='screen')


def generate_launch_description():
    overhead = LaunchConfiguration('overhead_device')
    return LaunchDescription([
        DeclareLaunchArgument('wrist_device', default_value=BY_PATH % '1.1',
                              description='follower wrist camera'),
        DeclareLaunchArgument('overhead_device', default_value='',
                              description='overhead camera device; empty = not started'),
        camera('wrist_cam', LaunchConfiguration('wrist_device')),
        camera('overhead_cam', overhead,
               IfCondition(PythonExpression(["'", overhead, "' != ''"]))),
    ])
