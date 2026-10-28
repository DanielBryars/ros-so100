# ros-so100 — working notes

Experimenting with an SO100 arm (upgraded Feetech motors) using ROS2 directly on
the host OS (not the container build for now).

## Environment

| Item | Value |
|------|-------|
| Host | `danb-ux301la` @ 192.168.1.220 (ssh as `danb`) |
| OS | Ubuntu, kernel 7.0.0-38-generic, x86_64 |
| ROS2 | distro `lyrical` (`/opt/ros/lyrical`) |
| Python | system `python3` = 3.14.4 |

Notes on the Python environment: this host's system Python has **no `pip`,
`ensurepip`, or `venv`**, and `sudo` is password-gated. `pyserial` 3.5 *is*
installed system-wide. To avoid needing sudo, the Feetech SDK is **vendored**
into `vendor/` instead of pip-installed.

If you later want pip/venv available system-wide:

```bash
sudo apt install python3-pip python3.14-venv
```

## Reading the motors

The SO100 servos sit on a half-duplex TTL serial bus behind a USB driver board,
which appears as `/dev/ttyUSB0` (CH340) or `/dev/ttyACM0`. The `danb` user is
already in the `dialout` group, so no permission changes are needed.

Scanner: `tools/scan_feetech.py`. It pings servo IDs across common baud rates and
dumps each servo's register table (ID, model, firmware, position, voltage,
temperature, angle limits, mode).

```bash
# arm powered + USB connected:
python3 tools/scan_feetech.py                 # auto-detect port, try common bauds
python3 tools/scan_feetech.py --baud 1000000  # STS3215 default baud
python3 tools/scan_feetech.py --ids 1-6 --json
```

STS/SMS servos use little-endian framing (protocol end = 0), default baud
1,000,000. Baud-rate register index map: 0=1M, 1=500k, 2=250k, 3=128k, 4=115200,
5=76800, 6=57600, 7=38400.

## Status / TODO

- [x] SSH access from Windows workstation to laptop (key auth as `danb`).
- [x] Git hooked up on the Windows side (`F:\git\ros-so100`, same GitHub remote).
- [x] Feetech SDK vendored; scanner written and syntax-checked.
- [x] Plug in the SO100 and scan — done (6x STS3250 on /dev/ttyACM0).
- [x] Servo model confirmed: Feetech STS3250 (model 2825), fw 3.10.
- [ ] Decide ROS2 bring-up approach on `lyrical`.

## First motor scan (2026-10-28)

Board enumerated as QinHeng USB serial (`1a86:55d3`) at **`/dev/ttyACM0`**
(CDC-ACM, not ttyUSB). Connected via a USB-C to USB-A cable. A separate USB
camera (`05a3:9230`, Sonix `USB2.0_CAM1`) is also on the arm.

`python3 tools/scan_feetech.py --port /dev/ttyACM0` found **6 servos at 1 Mbps**:

| ID | Model | Model# | FW | Voltage | Temp |
|----|-------|--------|----|---------|------|
| 1  | STS3250 | 2825 | 3.10 | 12.5 V | 21 C |
| 2  | STS3250 | 2825 | 3.10 | 12.5 V | 21 C |
| 3  | STS3250 | 2825 | 3.10 | 12.6 V | 21 C |
| 4  | STS3250 | 2825 | 3.10 | 12.5 V | 21 C |
| 5  | STS3250 | 2825 | 3.10 | 12.6 V | 21 C |
| 6  | STS3250 | 2825 | 3.10 | 11.8 V | 20 C |

The "upgraded" motors are **Feetech STS3250** (model number 2825), vs the
STS3215 (777) on a stock SO100. All on bus IDs 1-6, 1 Mbps, position mode,
torque disabled, full 0-4095 angle range. Supply ~12 V.

## ROS2: publishing joint telemetry (2026-10-28)

Built a colcon workspace at `ros2_ws/` with one package `so100_telemetry`. Its
node `joint_state_publisher` reads the 6 servo positions from `/dev/ttyACM0` and
publishes `sensor_msgs/JointState` on `/joint_states` at 10 Hz. Read-only: never
enables torque. The Feetech SDK is vendored into the package as the subpackage
`so100_telemetry/scservo_sdk` so it builds/installs without pip.

Position is published in radians via `(counts - 2048) / 4096 * 2*pi` — an
UNCALIBRATED mapping (no per-joint home offset or sign yet; that arrives with the
URDF). `velocity`/`effort` left empty for now.

Build and run:

```bash
source /opt/ros/lyrical/setup.bash
cd ~/git/ros-so100/ros2_ws
colcon build --packages-select so100_telemetry
source install/setup.bash

# terminal A — the node:
ros2 run so100_telemetry joint_state_publisher
# terminal B — watch the data (source /opt/ros/lyrical/setup.bash first):
ros2 topic echo /joint_states
```

Parameters (override with `--ros-args -p name:=value`): `port` (/dev/ttyACM0),
`baud` (1000000), `rate_hz` (10.0), `ids` ([1..6]), `joint_names`.

Verified: node `/so100_joint_state_publisher`, topic `/joint_states` at a steady
10.000 Hz, positions match the raw scan counts.

### ROS2 CLI learned here
- `ros2 node list` — running nodes
- `ros2 topic list` — active topics
- `ros2 topic echo <topic>` — subscribe and print messages
- `ros2 topic hz <topic>` — measure publish rate
- `ros2 topic info <topic> -v` — see publishers/subscribers and QoS

### Next
- [ ] Add a URDF + robot_state_publisher, then visualize in RViz.
- [ ] Optionally add velocity/effort from Present_Speed/Present_Load.

## RViz visualization (2026-10-28)

Visualized the live arm in RViz on the laptop's own display.

- **Model:** reused the community description package `brukg/SO-100-arm`
  (`so_arm_100_description`, Apache-2.0), copied into `ros2_ws/src/`. It ships a
  pre-generated plain `urdf/so_arm_100_5dof.urdf`, so **no xacro needed** (xacro is
  not installed here). Built with colcon; meshes resolve via `package://`.
- **Launch:** `so100_telemetry/launch/display.launch.py` starts
  robot_state_publisher (loads URDF), our joint_state_publisher (reads servos), and
  rviz2 (with the package's rviz config). Toggle RViz with `use_rviz:=false`.
- **Joint name mapping** (servo ID -> URDF joint): 1 Shoulder_Rotation,
  2 Shoulder_Pitch, 3 Elbow, 4 Wrist_Pitch, 5 Wrist_Roll, 6 Gripper. Passed to our
  node via the `joint_names` parameter (no code change).
- **Verified:** `/joint_states`, `/robot_description`, `/tf` all present; live TF
  base_link -> Moving_Jaw reflects the real pose; RViz got OpenGL 4.6 (direct render).

Run it:

```bash
source /opt/ros/lyrical/setup.bash
source ~/git/ros-so100/ros2_ws/install/setup.bash
ros2 launch so100_telemetry display.launch.py
```

### Gotchas worth remembering
- **GUI over SSH:** launch on the laptop's display with `DISPLAY=:0` and
  `XAUTHORITY=/home/danb/.Xauthority` (discover it from a :0 process's /proc environ).
  Use `setsid nohup ... &` + `disown` to keep it alive after SSH exits.
- **`pkill -f <pattern>` self-matches over SSH**: the pattern text is in the remote
  shell's own argv, so `pkill -f joint_state_publisher` kills its own session
  (ssh exit 255). Kill by PID, or use a pattern that excludes the shell.
- **`timeout ros2 run ...`** can orphan the actual node (child survives the killed
  parent). Check with `pgrep -af` and clean up by PID.

### Uncalibrated pose
Positions are raw radians (no home offset / direction sign), so the RViz pose is
approximate and some joints may move inverted. Calibration is the next step.

### Next
- [ ] Calibrate per-joint zero offset + direction sign.
- [ ] Optional: RViz on the Windows box via WSL2 (mirrored networking + ROS2 in WSL).

## RViz on Windows via WSL2 — WORKING (2026-10-28)

Ran RViz on the Windows PC in WSL2 (Ubuntu 24.04, ROS2 Jazzy), viewing the
laptop's live arm over the LAN via Fast DDS. Full writeup: docs/wsl-rviz.md;
helper scripts in tools/wsl/.

Key steps: mirrored WSL networking (shared LAN), install ros-jazzy-rviz2, open
the Windows Hyper-V firewall for WSL inbound (default was Block — this was the
final blocker), stage meshes locally for package:// resolution. Jazzy<->Lyrical
DDS interop works for JointState/TF/robot_description.
