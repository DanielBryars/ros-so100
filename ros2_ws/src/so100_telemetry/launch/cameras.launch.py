"""Real cameras on the pipeline's topic names (docs/ros2-pipeline.md).

    ros2 launch so100_telemetry cameras.launch.py
    ros2 launch so100_telemetry cameras.launch.py leader_wrist:=true   # also the leader's camera

Publishes /wrist_cam/image_raw and /overhead_cam/image_raw (+ camera_info) at 640x480, 30 fps,
rgb8, and optionally /leader/wrist_cam/image_raw. Set a device to '' to skip that camera.
camera_info carries the intrinsics from config/<camera>.yaml once that file exists
(tools/calibration/calibrate_intrinsics.py writes it); rectification (image_rect) comes after.

The two arm cameras are identical (same USB id and no serial number), so /dev/v4l/by-id
cannot tell them apart; the defaults use by-path, i.e. which USB socket each is plugged into.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

BY_PATH = '/dev/v4l/by-path/pci-0000:11:00.0-usb-0:%s:1.0-video-index0'
CONFIG_DIR = os.path.join(get_package_share_directory('so100_telemetry'), 'config')
# Overhead: the colour stream of the RealSense D435 (raw YUYV only; it has no MJPEG mode).
REALSENSE_COLOUR = ('/dev/v4l/by-id/usb-Intel_R__RealSense_TM__Depth_Camera_435_Intel_R__RealSense_TM__'
                    'Depth_Camera_435_822213020221-video-index0')


# Fixed exposure, gain and white balance, and the colour gains that remove the tint each
# camera's manual white balance leaves. Found with tools/calibration/lock_exposure.py on
# 2026-11-18 under the bench's room lighting; redo them if the lighting changes.
WRIST_LOCKED = ('auto_exposure=1,exposure_time_absolute=100,gain=2,'
                'white_balance_automatic=0,white_balance_temperature=3259', '1.249,1.000,1.252')
OVERHEAD_LOCKED = ('auto_exposure=1,exposure_time_absolute=300,gain=36,'
                   'white_balance_automatic=0,white_balance_temperature=3950', '1.000,1.143,1.001')
AUTO = 'auto_exposure=3,white_balance_automatic=1'


def when_locked(value):
    return PythonExpression(["'", value, "' if '", LaunchConfiguration('locked'), "' == 'true' else ",
                             "'", AUTO if '=' in value else '', "'"])


def camera(name, device, condition=None, frame=None, fourcc='MJPG', locked=('', '')):
    # Our own node, not v4l2_camera: it captures MJPEG, which avoids the torn frames the
    # wrist cameras produce in raw YUYV (see camera_node.py).
    return Node(
        package='so100_telemetry', executable='camera_node', name='camera', namespace=name,
        parameters=[{
            'device': device,
            'width': 640, 'height': 480, 'fps': 30, 'fourcc': fourcc,
            'frame_id': frame or name,
            'controls': when_locked(locked[0]),
            'colour_gains': when_locked(locked[1]),
            'camera_info_file': os.path.join(CONFIG_DIR, (frame or name) + '.yaml'),
        }],
        condition=condition, output='screen')


def generate_launch_description():
    def is_set(arg):
        return IfCondition(PythonExpression(["'", LaunchConfiguration(arg), "' != ''"]))

    return LaunchDescription([
        DeclareLaunchArgument('locked', default_value='true',
                              description='false = auto exposure and white balance, no colour gains'),
        DeclareLaunchArgument('wrist_device', default_value=BY_PATH % '1.3',
                              description='follower wrist camera'),
        DeclareLaunchArgument('overhead_device', default_value=REALSENSE_COLOUR),
        DeclareLaunchArgument('overhead_fourcc', default_value='YUYV',
                              description='MJPG for an ordinary webcam'),
        DeclareLaunchArgument('leader_wrist', default_value='false'),
        DeclareLaunchArgument('leader_wrist_device', default_value=BY_PATH % '1.1'),
        camera('wrist_cam', LaunchConfiguration('wrist_device'), is_set('wrist_device'),
               locked=WRIST_LOCKED),
        camera('overhead_cam', LaunchConfiguration('overhead_device'), is_set('overhead_device'),
               fourcc=LaunchConfiguration('overhead_fourcc'), locked=OVERHEAD_LOCKED),
        camera('leader/wrist_cam', LaunchConfiguration('leader_wrist_device'),
               IfCondition(LaunchConfiguration('leader_wrist')), frame='leader_wrist_cam'),
    ])
