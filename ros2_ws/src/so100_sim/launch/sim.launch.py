"""Bring up the MuJoCo backend, optionally with the VR view and leader-arm teleop.

    ros2 launch so100_sim sim.launch.py                       # headless, cameras + joint states
    ros2 launch so100_sim sim.launch.py vr:=true teleop:=true # in the headset, driven by the leader

teleop:=true only starts the relay /leader/joint_states -> /joint_command; the leader driver
itself comes from so100_telemetry (dual_arm.launch.py).

The sim node needs mujoco/pyopenxr from the venv, so it is started with that interpreter. For
VR on this desktop the usual env applies (docs/desktop-setup.md: PRIME offload, DISPLAY).
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_python = os.environ.get('SO100_PYTHON', os.path.expanduser('~/venvs/so100/bin/python'))
    return LaunchDescription([
        DeclareLaunchArgument('vr', default_value='false'),
        DeclareLaunchArgument('teleop', default_value='false'),
        DeclareLaunchArgument('floor', default_value='black'),
        DeclareLaunchArgument('python', default_value=default_python),
        ExecuteProcess(
            cmd=[LaunchConfiguration('python'), '-u', '-m', 'so100_sim.sim_node', '--ros-args',
                 '-p', ['vr:=', LaunchConfiguration('vr')],
                 '-p', ['floor:=', LaunchConfiguration('floor')]],
            output='screen'),
        Node(package='so100_sim', executable='teleop_relay', output='screen',
             condition=IfCondition(LaunchConfiguration('teleop'))),
    ])
