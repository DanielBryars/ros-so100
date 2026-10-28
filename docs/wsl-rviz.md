# RViz on Windows (WSL2) viewing the laptop's arm over the LAN

Goal: run RViz on the Windows PC and watch the SO100, while the laptop
(`192.168.1.220`) does the actual servo reading and publishing. ROS2 has no
central master, so nodes discover each other over the network automatically —
once the plumbing allows it.

## Architecture

```
 Laptop (Ubuntu 26.04, ROS2 Lyrical)            Windows PC
 ---------------------------------              -----------------------------
 so100 joint_state_publisher  --\                WSL2 Ubuntu 24.04 (ROS2 Jazzy)
 robot_state_publisher          >-- /joint_states, /tf, ---  rviz2 (WSLg window)
                                     /robot_description  (Fast DDS over the LAN)
```

Jazzy (WSL) and Lyrical (laptop) are different ROS2 distros, but both default to
the **Fast DDS** middleware and the standard messages (JointState, TFMessage,
robot_description=std_msgs/String) are wire-stable, so they interoperate.

## One-time setup (Windows side)

1. **Mirrored networking** so WSL shares the host LAN (NAT blocks DDS discovery):
   copy `tools/wsl/wslconfig.example` to `C:\Users\<you>\.wslconfig`, then
   `wsl --shutdown` (this also stops docker-desktop). After restart, WSL's eth0
   gets a `192.168.1.x` address on the same subnet as the laptop.

2. **Install RViz in WSL** (needs the WSL sudo password; run in a WSL terminal):
   `bash tools/wsl/install_ros2_jazzy.sh` — adds the ROS2 apt repo and installs
   `ros-jazzy-rviz2`.

3. **Open the Windows (Hyper-V) firewall for WSL inbound** — the default is
   Block, which drops the laptop's discovery packets. In an **elevated**
   PowerShell:
   ```powershell
   Set-NetFirewallHyperVVMSetting -Name '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -DefaultInboundAction Allow
   ```
   Revert with `-DefaultInboundAction Block`. The GUID is WSL's vmCreatorId.

4. **Stage the meshes** so RViz can resolve `package://so_arm_100_description`
   (the URDF arrives over the network, but mesh files must be local):
   `bash tools/wsl/stage_meshes.sh` — copies the STLs into `~/so100_viz`.

## Run it

- On the laptop: `ros2 launch so100_telemetry display.launch.py use_rviz:=false`
  (publishers only; add rviz on the laptop too if you like).
- In WSL: `bash tools/wsl/run_rviz_wsl.sh` — opens RViz on the Windows desktop,
  subscribed to the laptop over the LAN.

## Gotchas learned

- The apt install of only `ros-jazzy-rviz2` leaves the prefix `setup.sh`
  mis-wired (empty `AMENT_PREFIX_PATH`) and omits the `ros2` CLI. `run_rviz_wsl.sh`
  works around it by eval-ing `_local_setup_util.py` directly.
- `$HOME` and complex quoting get mangled when piping through
  git-bash -> wsl.exe -> bash. Run WSL commands from PowerShell, or (better)
  write a script file with LF endings and run `wsl -d Ubuntu -- bash <file>`.
- Verify discovery from the laptop with `ros2 node list`: the WSL `/rviz` node
  should appear once the firewall is open.
