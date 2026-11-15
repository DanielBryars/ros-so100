# Linux desktop setup — `danb-ubuntu` (192.168.1.73)

The sim / VR / inference hub for the IJCNN 2027 work (see [ijcnn-2027-tasks.md](ijcnn-2027-tasks.md)).
Set up 2026-11-09. Everything below was done over SSH as `danb` (key auth, passwordless sudo).

## Hardware / OS

| | |
|---|---|
| OS | Ubuntu 26.04.1 LTS (resolute), kernel 7.0 |
| GPU | RTX 3080 Ti 12 GB, NVIDIA driver 595.99.02 (CUDA 13 capable) |
| CPU / RAM | 16 cores, 59 GB |
| Disk | 903 GB NVMe, ~880 GB free |
| Network | wired `enp11s0`, 192.168.1.73 |
| Session | GNOME on Wayland |
| Python | system 3.14.4 with pip + venv |

## ROS2

`ros-lyrical-desktop` was already installed (same distro as the laptop). Added:

```bash
sudo apt install ros-lyrical-image-pipeline ros-lyrical-v4l2-camera \
  ros-lyrical-camera-calibration ros-lyrical-image-transport-plugins \
  ros-lyrical-camera-info-manager
```

Fast DDS, `ROS_DOMAIN_ID` unset (=0). With no configuration `ros2 node list` already shows
the laptop's `/so100_joint_state_publisher` and `/robot_state_publisher` over the LAN.

The `ros-so100` workspace is cloned at `~/git/ros-so100` and built:

```bash
source /opt/ros/lyrical/setup.bash
cd ~/git/ros-so100/ros2_ws && colcon build --symlink-install
source install/setup.bash
ros2 launch so100_telemetry display.launch.py use_rviz:=true port:=/dev/ttyACM0
```

## Python environment — `~/venvs/so100`

Created with `--system-site-packages` so `rclpy` (built against the system Python 3.14)
is importable in the same process as Torch and MuJoCo. Verified: `import rclpy, torch,
mujoco, xr, cv2, lerobot` all work, CUDA is visible, and a tiny ACT policy does a
forward pass on the GPU.

```bash
source ~/venvs/so100/bin/activate
source /opt/ros/lyrical/setup.bash     # not `set -u` clean; source before any strict script
```

What is in it and why:

- **torch 2.15 + torchvision 0.29 (CUDA 13.2 build)**, torchcodec 0.17. Oldest Torch with
  Python 3.14 wheels is 2.9, so the thesis's nightly 2.11 pin was replaced.
- **lerobot fork** `DanielBryars/lerobot @ c55fbe1` (0.4.3), editable at `~/git/lerobot`,
  installed with `--no-deps` because its `torch<2.8` / `torchvision<0.23` pins are stale.
  Its runtime deps were installed by hand with the same version ranges minus Torch.
- **mujoco 3.17.0.** The thesis used 3.4.0, which has no Python 3.14 wheel (first 3.x with
  one is 3.8.0; 4.0 is a major bump). Scenes load unchanged; EGL headless rendering of
  `scenes/so101_with_wrist_cam.xml` works (`MUJOCO_GL=egl`, both cameras, ~25 ms).
- **pyopenxr 1.1, glfw, PyOpenGL** for the VR teleop port.
- Robot plugins, editable, `--no-deps`: `~/git/lerobot-scratch/lerobot_robot_sts3250`
  (`@ 02eb78a`) and `~/git/lerobot-thesis/src/lerobot_robot_sim`.
- `lerobot-thesis` is cloned at `~/git/lerobot-thesis` with the SO-ARM100 mesh submodule.

Not installed: jax/flax/openpi (pi0 experiments only), pywin32.

## Groups / udev

`danb` added to `dialout` (serial arms) and `plugdev` (adb over USB). Takes effect on next
login. `adb` + `android-sdk-platform-tools-common` provide the Android udev rules.

## VR — WiVRn (Flatpak)

Meta Link is Windows-only, so the Quest 3 talks to Linux via WiVRn (Monado-based OpenXR
runtime + streamer). Installed system-wide:

```bash
sudo apt install flatpak adb vulkan-tools mesa-utils
sudo flatpak remote-add --if-not-exists --system flathub https://dl.flathub.org/repo/flathub.flatpakrepo
sudo flatpak install -y --system flathub io.github.wivrn.wivrn      # 26.9
# the matching GL extension org.freedesktop.Platform.GL.nvidia-595-99-02 was pulled in automatically
```

`~/.config/openxr/1/active_runtime.json` is symlinked to the Flatpak's
`files/share/openxr/1/openxr_wivrn.json` so host (non-Flatpak) OpenXR apps such as pyopenxr
find WiVRn. The dashboard may rewrite this on first run; that's fine.

### First-run checklist (needs the GUI and the headset)

1. `flatpak run io.github.wivrn.wivrn` → dashboard. Follow its wizard; it offers to install
   the **WiVRn client from the Meta Store** onto the Quest 3. Client and server must be the
   **same version** (26.9).
2. Wired: needs **Developer Mode** on the Quest first (Meta Horizon phone app → Devices →
   headset → Headset Settings → Developer Mode; the account must be registered as a developer
   at developer.oculus.com). Without it the headset enumerates with no ADB interface (only
   vendor-specific subclasses 138/139/140), `adb devices` is empty, no "allow USB debugging"
   prompt appears, and the dashboard says "could not connect" (hit 2026-11-10). Then: plug the
   Quest 3 into a USB 3 (5 Gb/s) port, `adb devices`, accept USB debugging on the headset,
   then "Connect via USB" in the dashboard.
   Wi-Fi alternative needs no Developer Mode: install the WiVRn client from the Meta Horizon
   Store, pick `danb-ubuntu` in the client, enter the pairing PIN the dashboard shows (also
   logged in `~/.var/app/io.github.wivrn.wivrn/.local/state/wivrn/wivrn-dashboard/server_logs_*.txt`).
   The dashboard's "port 9757 may need to be enabled" hint is generic; `ufw` is inactive here.
3. Smoke test (done 2026-11-10, Wi-Fi): `tools/vr/xr_hello.py` in the `ros-so100` repo creates an
   instance and reports runtime `WiVRn v26.9`, system `Meta Quest 3 on WiVRn`, per-eye
   2476x2594. `tools/vr/xr_render_linux.py` opens a GLFW window, creates a session with the
   Xlib/GLX binding, two `GL_SRGB8_ALPHA8` swapchains (3 images each), goes
   IDLE→READY→SYNCHRONIZED→VISIBLE→FOCUSED and holds **90 fps**; the server log shows NVENC
   streaming and the headset requesting IDR frames. Run it over SSH with:

   ```bash
   source ~/venvs/so100/bin/activate
   export XDG_RUNTIME_DIR=/run/user/1000 DISPLAY=:0 WAYLAND_DISPLAY=wayland-0           XAUTHORITY=$(ls /run/user/1000/.mutter-Xwaylandauth.*)           __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia
   python xr_render_linux.py 20      # seconds
   ```

4. Known caveats: NVIDIA path is less optimised than AMD (higher latency, keep default
   render resolution); if NVENC sits in P-state P2 see WiVRn `docs/nvenc.md`; one 2025 guide
   recommends an X11 session over GNOME-Wayland if the compositor misbehaves.

### Gotchas found porting the thesis VR viewer

- **Monitor is on the Ryzen iGPU** (card2 = `10:00.0` AMD Granite Ridge, HDMI). A plain GL
  context therefore lands on Mesa/radeonsi, not the 3080 Ti. Either move the HDMI cable to the
  NVIDIA card or always set `__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia`
  for anything that renders into WiVRn (the server encodes on the NVIDIA card).
- **pyopenxr 1.1.6501 has no `xr.ContextObject`/`xr.api2`** (the 1.0 helper is gone); build
  the session by hand as the thesis viewer already does.
- **Graphics binding:** `vr_openxr_viewer.py` / `teleop_sim_vr.py` raise "Only Windows is
  currently supported". The Linux branch is `xr.GraphicsBindingOpenGLXlibKHR` fed from
  `OpenGL.GLX.glXGetCurrentDisplay/Drawable/Context` (GLFW on XWayland, `DISPLAY=:0`); the
  fields are typed pointers (`LP_struct__XDisplay`, `LP_struct___GLXcontextRec`), see `P()` in
  `tools/vr/xr_render_linux.py`. Field is `glx_fbconfig` (null is fine).
- **Session lifecycle:** Monado/WiVRn requires waiting for the `READY` event before
  `begin_session` (the Windows viewer calls it immediately; Quest Link tolerated that).
- **`xr.sync_actions` raises `SessionNotFocused`** in pyopenxr even though
  `XR_SESSION_NOT_FOCUSED` is a success code — catch it.
- Head/controller poses report `view_state_flags=0` (invalid) while the headset is off the head
  in standby, even though the stream is running; tracking only starts once it is worn
  (confirmed 2026-11-15: worn → flags `0xf`, head/grip poses and triggers live at 90 fps).
- Pressing the Meta button drops the session to VISIBLE/SYNCHRONIZED and pauses `wait_frame`;
  it resumes to FOCUSED on return. The loop must tolerate this.

### Linux port of the thesis VR scripts (`tools/vr/`)

`tools/vr/vr_common.py` (shared `VRRenderer`, `ArmReader`, scene loading),
`vr_openxr_viewer.py` and `teleop_sim_vr.py` are the Linux ports, written 2026-11-15. The
originals in `lerobot-thesis/scripts/recording/` are untouched; the port only reads the
thesis repo for the scene XML, meshes and `utils/constants.py` (`LEROBOT_THESIS` overrides
the default `~/git/lerobot-thesis`).

```bash
cd ~/git/ros-so100/tools/vr        # with the env block from the smoke test above
python vr_openxr_viewer.py --seconds 20 --snapshot /tmp/eye.png   # smoke test, saves a left-eye frame
python teleop_sim_vr.py --test                                    # scene + controllers, no arm
python teleop_sim_vr.py --port /dev/ttyACM0                       # leader arm drives the sim
python teleop_sim_vr.py --no-vr --test                            # desktop MuJoCo viewer
```

Status (2026-11-15, Quest 3 over Wi-Fi, headset worn): both scripts go
IDLE→READY→SYNCHRONIZED→VISIBLE→FOCUSED, hold **90 fps** with tracking valid, recentre on the
first tracked frame and shut down cleanly; a left-eye frame captured from the live session
shows the robot, bowl and overhead-camera footprint. `--no-vr --test` also runs. Arm teleop
(`--teleop`, or `teleop_sim_vr.py` without `--test`) is **untested**: no arm on this desktop.

Differences from the Windows originals:

- Session begins on `READY`, layers are only submitted when `should_render`, and the loop
  idles while the headset is asleep or the Meta menu is open.
- Paced by the headset (90 Hz) with real-time physics, instead of a 30 fps sleep with 10
  substeps per frame (which ran the sim at 0.6x real time).
- Arm joints are addressed by name. In `so101_with_wrist_cam.xml` they are `qpos[7:13]`;
  `qpos[0:7]` is the Duplo block's free joint, which the old viewer's `--teleop` wrote to.
- The arm is read in a background thread in both scripts; thumbstick speeds are per second.
- The scene recentres itself on the first tracked frame. Keyboard controls are on the
  desktop mirror window (WASD/QE, R or Space, P) rather than the console.

Gotchas:

- `~/.config/openxr/1/active_runtime.json` only exists while the headset is connected: the
  WiVRn dashboard removes it when the server session ends (seen 2026-11-15 15:55 after the
  Quest dropped off Wi-Fi). "The loader was unable to find or load a runtime" means reconnect
  the WiVRn client on the headset first.
- Arm calibration: `leader_so100.json` / `follower_so100.json` were copied on 2026-11-15 from
  `lerobot-thesis/calibration/` into `~/.cache/huggingface/lerobot/calibration/`
  (`teleoperators/so100_leader_sts3250/`, `robots/so100_follower_sts3250/`). `scservo_sdk` was
  installed into `~/venvs/so100` with `pip install --no-deps vendor/feetech-servo-sdk-1.0.0`
  (then `git checkout` the egg-info files pip rewrites).

## Scratch

`~/claude-scratch/` holds setup logs (`setup_venv*.log`) and test renders.
