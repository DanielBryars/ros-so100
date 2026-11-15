# ros-so100

Experimenting with an SO100 arm (upgraded Feetech **STS3250** servos) using ROS2.
Working notes in [docs/notes.md](docs/notes.md).

## Read the motors (no ROS needed)

```bash
python3 tools/scan_feetech.py --port /dev/ttyACM0
```

## Launch everything on the laptop (publishers + RViz)

```bash
source /opt/ros/lyrical/setup.bash
source ~/git/ros-so100/ros2_ws/install/setup.bash
ros2 launch so100_telemetry display.launch.py
```

## Just RViz on the laptop

Both `source` lines are required — the second one (the workspace overlay) is what
lets RViz resolve the arm's meshes (`package://so_arm_100_description/...`).

```bash
source /opt/ros/lyrical/setup.bash
source ~/git/ros-so100/ros2_ws/install/setup.bash
rviz2 -d ~/git/ros-so100/ros2_ws/src/so_arm_100_description/rviz/so_arm_100.rviz
```

## Just the telemetry node (publish /joint_states)

```bash
source /opt/ros/lyrical/setup.bash
source ~/git/ros-so100/ros2_ws/install/setup.bash
ros2 run so100_telemetry joint_state_publisher
# inspect it:
ros2 topic echo /joint_states
```

## RViz on Windows (WSL2) over the LAN

See [docs/wsl-rviz.md](docs/wsl-rviz.md) for the one-time setup. To run:

```bash
# laptop: publishers only
ros2 launch so100_telemetry display.launch.py use_rviz:=false
# Windows WSL: RViz on the desktop, subscribing over the LAN
bash tools/wsl/run_rviz_wsl.sh
```

## Leader + follower side by side (calibrated, SO-101 model)

Both arms are plugged into the desktop (`danb-ubuntu`). The launch file reads them by USB
serial number, maps encoder counts to radians with the LeRobot calibration JSONs in
`so100_telemetry/config/`, and shows them on the SO-101 URDF, which has the same joint names
and zero convention as the MuJoCo sim. Read-only: torque is never enabled.

```bash
source /opt/ros/lyrical/setup.bash
source ~/git/ros-so100/ros2_ws/install/setup.bash
ros2 launch so100_telemetry dual_arm.launch.py                    # publishers (machine with the arms)
ros2 launch so100_telemetry dual_arm.launch.py publishers:=false use_rviz:=true   # RViz on another machine
ros2 launch so100_telemetry dual_arm.launch.py use_rviz:=true     # everything on one machine
```

Topics: `/leader/joint_states`, `/follower/joint_states` (30 Hz, radians), TF frames prefixed
`leader/` and `follower/` under `world`. A machine running only RViz needs this workspace
built too, for the meshes in `so101_description`.
