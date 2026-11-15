# IJCNN 2027 — task list

Goal: resubmit vACT+ (coord-conditioned ACT) with a **real-robot** result, using ROS2 as
the common sim/real contract so camera geometry is identical in both.

- Deadline: **31 Jan 2027** (regular papers, 6 pages IEEE two-column). Notification 15 Mar.
- Code: `D:\git\ai\lerobot-thesis`. Paper: `D:\git\ai\overleaf-thesis`. Notes: this repo.
- Go/no-go moment: vanilla ACT transferring to the real arm (end of phase 2). If that
  fails, fix appearance gap before touching the coord model.

## Phase 0 — Design (this week)

- [ ] **Design the ROS2 pipeline.** Topic contract (joint states, 2x image + camera_info,
      target point, action), node layout, which machine runs what, recording path
      (bag vs direct LeRobot writer), how the coord enters in sim vs real.
      *Proposal written 2026-11-15: [ros2-pipeline.md](ros2-pipeline.md), awaiting review
      (also proposes the message rates below).*
- [x] Decide the sim/VR host → **Linux desktop `danb-ubuntu` (192.168.1.73)**, Ubuntu
      26.04.1 + ROS2 Lyrical (same distro as the laptop), RTX 3080 Ti 12 GB, Python 3.14
      (torch/mujoco/pyopenxr all resolve). Quest 3 via WiVRn/Monado over USB-C.
      **Fallback:** RoboStack on Windows. **Last resort:** drop VR, multi-view MuJoCo
      window + leader arm.
- [x] Desktop setup (2026-11-09, see [desktop-setup.md](desktop-setup.md)): ROS2 image
      packages, Flatpak + WiVRn 26.9, adb, repos cloned (`ros-so100`, `lerobot-thesis`,
      `lerobot` fork, `lerobot-scratch`), `~/venvs/so100` with torch 2.15 cu13 + lerobot +
      mujoco 3.17 + pyopenxr + rclpy; ACT forward pass on GPU and EGL scene render verified.
- [x] WiVRn spike (2026-11-10): Meta Store client paired over **Wi-Fi** (USB needs Quest
      Developer Mode, not enabled yet); `tools/vr/xr_render_linux.py` holds a FOCUSED OpenXR
      session at 90 fps on the 3080 Ti via PRIME offload. Gotchas in
      [desktop-setup.md](desktop-setup.md#gotchas-found-porting-the-thesis-vr-viewer).
      Confirmed 2026-11-15 with the headset worn: view flags 0xf, head + both Touch Plus
      grip poses and trigger values tracking at 90 fps.
- [x] Port `vr_openxr_viewer.py` / `teleop_sim_vr.py` to Linux (2026-11-15): ports live in
      `tools/vr/` (thesis repo untouched); MuJoCo scene in the headset, FOCUSED at 90 fps.
      Arm teleop through them is untested (needs an arm + leader calibration on the desktop).
      See [desktop-setup.md](desktop-setup.md#linux-port-of-the-thesis-vr-scripts-toolsvr).
- [ ] Decide message rates: cameras 30 fps, policy 10–30 Hz, wired Ethernet or compressed
      image transport between laptop and PC.

## Phase 1 — Infrastructure (weeks 1–2)

- [ ] RoboStack spike on Windows: `rclpy` + MuJoCo + pyopenxr + CUDA Torch in one env.
- [ ] Calibrate both real webcams (ChArUco board already in `lerobot-thesis/calibration/`)
      → `CameraInfo` yaml per camera.
- [ ] Laptop nodes: camera publishers (`v4l2_camera`/`usb_cam`) with `camera_info`,
      `image_proc` rectification, follower command subscriber (joint targets → servos).
      Joint-state publisher already exists (`so100_telemetry`).
- [ ] Measure real overhead extrinsics vs robot base (board + FK) → scene XML camera pose.
- [ ] Build MuJoCo cameras from rectified K (`resolution`, `focalpixel`, `principalpixel`)
      so sim and real images share geometry.
- [ ] One policy node that subscribes to the contract and doesn't know sim from real.

## Phase 2 — Vanilla ACT, sim → real (weeks 3–5)

- [ ] Sim backend node: MuJoCo publishes the same topics; VR teleop commands via ROS2.
- [ ] Record single-block continuous-position demos via ROS2 → LeRobot v3 dataset.
- [ ] Domain randomisation pass (textures, lighting) in MuJoCo; decide wrist-only vs both.
- [ ] Train vanilla ACT. Eval in sim through the policy node.
- [ ] **Deploy on real arm. Go/no-go.** Log success rate + failure modes.

## Phase 3 — vACT+ coord model, sim → real (weeks 6–8)

- [ ] Re-record two-block forced-ambiguity set, continuous positions (Tier C style).
- [ ] Retrain vACT+ (wrist-only first per the May findings).
- [ ] Real demo: click-to-pick disambiguation. Coord from calibrated overhead click
      (homography/unproject), optionally a detector later.
- [ ] n≈10–20 real trials per target, video capture.

## Phase 4 — Paper (weeks 9–12)

- [ ] Compress CoRL draft to 6 pages: sim results one section, real transfer centrepiece.
- [ ] IEEE template, figures regenerated, bib reconciled.
- [ ] Check special-session list (published ~15 Dec) for a robotics/embodied track.

## Parking lot

- Detector/VLM → coord end-to-end (Phase 2.4 from the CoRL plan).
- Calibrate per-joint zero/direction for the URDF (pose currently lucky-aligned).
