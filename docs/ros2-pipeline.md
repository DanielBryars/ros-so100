# ROS2 pipeline design (Phase 0)

Status: **proposal, 2026-11-15** — not yet reviewed. Task list: [ijcnn-2027-tasks.md](ijcnn-2027-tasks.md).

The point of the pipeline is one thing: the policy, the recorder and the coord source talk
to a set of topics and cannot tell whether MuJoCo or the real arm is behind them. Everything
below is in service of that, and anything that does not help sim→real transfer is left out
(no ros2_control, no MoveIt, no custom message types).

## What the thesis code already fixes

Facts the design has to fit (from `lerobot-thesis`; file references in the survey notes at
the end):

| | Thesis convention |
|---|---|
| Dataset | LeRobot v3, 30 fps, `observation.state` (6), `action` (6), `observation.images.wrist_cam` and `.overhead_cam` at 640x480 |
| Units | normalised: joints −100..100, gripper 0..100, mapped linearly onto the sim joint limits |
| Action | absolute joint position targets (the leader arm's pose) |
| Coord | world XY of the target block in the robot base frame (metres), normalised with fixed bounds X (0.10, 0.38), Y (−0.28, 0.27); constant per episode; injected as `observation.environment_state` from `meta/episode_scenes.json` |
| Policy | ACT, chunk 100, executed open-loop by default; ViT resizes to 224 internally |
| Real cameras | no intrinsics exist yet; the coord came from a 6-point pixel→world homography |

Three things in the old path that the new one should not inherit:

1. **Time base.** Recording stepped 20 ms of physics per frame but ran and labelled frames at
   33 ms, so sim dynamics were 0.6x real time. A real arm cannot do that.
2. **Gripper zero.** Real calibration puts gripper 0 at the zero pose (0 rad); sim puts 0 at
   −0.175 rad. About 10° of disagreement hidden inside "normalised" units.
3. **Camera geometry.** Sim used nominal `fovy`; real used whatever the webcam gave.

## Decisions

**1. Radians on the wire, normalised units only inside the learning nodes.** Topics carry
calibrated radians in the MuJoCo joint convention, with the LeRobot joint names
(`shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_roll`, `gripper`). The
policy and recorder convert to the thesis normalised units with one shared function, so
datasets and training code keep their format. This makes the gripper-zero mismatch
impossible to hide: each backend has to say where its zero is in radians.

**2. Real time everywhere, 30 Hz control.** The sim steps physics against the wall clock (as
the VR port now does) and a control tick is 1/30 s in both worlds. No `use_sim_time`. Old
checkpoints were trained on the 0.6x dynamics, so they are a smoke test for the plumbing, not
a baseline; Phase 2 re-records anyway.

**3. Rectified pinhole images are the camera contract.** The policy consumes
`image_rect` + `camera_info`. Real: `v4l2_camera` → `image_proc` rectify. Sim: MuJoCo cameras
built from the real rectified K (`focalpixel` / `principalpixel`), published as already
rectified. Geometry then matches by construction and the remaining gap is appearance.

**4. The coord is a `PointStamped` in the base frame, from one node in both worlds.** A click
on the overhead `image_rect` is unprojected with `camera_info` and the camera→base transform
onto the table plane. In sim the same node runs against the sim overhead image, and the sim
also publishes the ground-truth block position, so click error is measurable in sim before it
matters on the bench. The homography stays as a fallback.

**5. Record straight to a LeRobot dataset; bags are for debugging.** A recorder node samples
at 30 Hz and writes LeRobot v3 with the thesis feature keys, plus `meta/episode_scenes.json`
(0-indexed) so `PickupCoordinateDataset` works unchanged. Converting bags afterwards would
add a second place for sync bugs. `ros2 bag record` (mcap) can still run alongside on the
real arm when a failure needs replaying.

**6. The sim node hosts the VR renderer in-process.** The headset view needs the full MuJoCo
state (blocks, bowl), not just joint angles, so mirroring it into a second process buys
nothing. One loop, paced by the headset at 90 Hz when VR is on or a 90 Hz timer when it is
not: step physics, render VR eyes, and every third frame render the two policy cameras
(about 1 ms each) and publish.

**7. One machine if the bench allows it.** Recommended: plug the follower arm and both
cameras into the desktop, so images never cross the network. If the arm has to stay on the
laptop, see "Laptop split" below. The topic contract is the same either way.

## Topic contract

| Topic | Type | Rate | Publisher (sim / real) | Notes |
|---|---|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | 90 / ~100 Hz | sim node / follower driver | measured position, rad |
| `/joint_command` | `sensor_msgs/JointState` | 30 Hz policy, leader rate in teleop | policy node or leader driver | position targets, rad; exactly one publisher at a time |
| `/leader/joint_states` | `sensor_msgs/JointState` | ~100 Hz | leader driver | teleop source; remapped onto `/joint_command` |
| `/wrist_cam/image_rect`, `/overhead_cam/image_rect` | `sensor_msgs/Image` rgb8 640x480 | 30 Hz | sim node / `image_proc` | what the policy sees |
| `/wrist_cam/camera_info`, `/overhead_cam/camera_info` | `sensor_msgs/CameraInfo` | 30 Hz | sim node / camera driver | rectified K |
| `/target_point` | `geometry_msgs/PointStamped` (`base`) | latched | target node | the coord, metres |
| `/sim/target_truth` | `geometry_msgs/PointStamped` | latched | sim node only | ground truth, never read by the policy |
| `/tf_static` | | | launch file | `base` → `overhead_cam` extrinsics |

Services: `/sim/reset` (`std_srvs/Trigger`, re-randomises the block), `/follower/enable`
(`SetBool`, torque on/off), `/policy/enable` (`SetBool`), `/recorder/start|save|discard`
(`Trigger`).

QoS: images best-effort, depth 1. Joint topics reliable, depth 1. `/target_point` reliable +
transient-local so late joiners get it.

## Nodes

```
                 ┌────────────── backend (pick one) ──────────────┐
 leader driver ─▶│ sim node: MuJoCo + VR + 2 cameras              │─▶ /joint_states
   /joint_command│   or                                           │─▶ /*_cam/image_rect, camera_info
 policy node  ──▶│ follower driver + v4l2_camera x2 + image_proc  │
                 └────────────────────────────────────────────────┘
 target node (click on overhead) ─▶ /target_point ─▶ policy node, recorder node
 recorder node ◀─ everything above ─▶ LeRobot v3 dataset
```

- **Sim node** (`so100_sim`). As decision 6. Applies `/joint_command` to `ctrl`, holds the
  last command if none arrives.
- **Follower driver** (extends `so100_telemetry`). Sync-reads all six servos, publishes
  calibrated radians, and accepts `/joint_command` behind safety limits: clamp to joint
  limits, cap the per-tick step, hold position if commands stop for 200 ms, torque only via
  the enable service. Counts→radians uses the existing calibration JSON (range centre and
  651.9 counts/rad). The same node in read-only mode is the leader driver.
- **Policy node** (`so100_lerobot`). 30 Hz timer: latest state + images + coord → the
  thesis `prepare_obs_for_policy` format → action chunk → `/joint_command`. Parameters for
  checkpoint, `n_action_steps` and temporal ensembling (100-step open loop is risky on
  hardware; 20–40 or ensembling is the likely real setting). Does nothing until enabled, and
  ramps from the current pose to the training home pose first.
- **Recorder node** (`so100_lerobot`). 30 Hz timer, takes the latest sample of each topic,
  refuses the frame (and counts it) if any input is older than 50 ms. `observation.state`
  from `/joint_states`, `action` from `/joint_command`, both converted to normalised units.
- **Target node**. Shows the overhead image, publishes the clicked point; in sim can instead
  forward ground truth.

Launch files: `sim.launch.py` and `real.launch.py` bring up a backend; `teleop.launch.py`,
`record.launch.py` and `policy.launch.py` are shared and never mention which.

The URDF in `so_arm_100_description` uses different joint names (`Shoulder_Rotation`, …).
RViz is not on the critical path, so the plan is a small relay or a renamed copy when it is
next needed, not a rework now.

## Message rates (the other open Phase 0 item)

Cameras 640x480 rgb8 at 30 fps, policy and recorder at 30 Hz, joint states at 90–100 Hz.
Raw images are 27.6 MB/s per camera: free on one machine, about 440 Mbit/s for two across a
network.

### Laptop split (only if the arm stays on the laptop)

The laptop (`danb-ux301la`, a Zenbook UX301LA) probably has no built-in Ethernet (not
checked: it was off when this was written), and raw images are not viable over Wi-Fi. Use `compressed_image_transport` (JPEG, ~5 MB/s for both cameras) and decode on the
desktop, and apply the same JPEG round-trip to the sim images so the policy sees the same
artefacts in training. A USB Ethernet adapter would make raw transport possible instead.

## Build order

1. **Sim node + leader teleop over ROS, in VR.** Needs the leader arm on the desktop.
2. **Recorder node** → a small dataset → train vanilla ACT → **policy node** closing the
   loop in sim. This proves the contract end to end without any real hardware.
3. **Follower driver + cameras**: intrinsics (ChArUco), overhead extrinsics, sim cameras
   rebuilt from the real K. Needs the follower arm and cameras.
4. Target node, then real deployment (the go/no-go).

## Answers to the open questions (2026-11-15)

- Both arms and all three cameras are plugged into the desktop, so decision 7 holds and the
  laptop split is not needed. The laptop is only a second RViz screen.
- The existing zero poses were checked in RViz against the SO-101 model
  (`dual_arm.launch.py`) and look right, so `leader_so100.json` / `follower_so100.json` are
  kept as they are. The ~10 degree gripper-zero difference between real and sim is still to
  be handled in the follower driver.
- The overhead camera is the Nuroum V11. It is not in its final position yet, so overhead
  extrinsics wait; its crop/scale mode at 640x480 still decides K.

## Survey notes

Sources for the table above: `src/lerobot_robot_sim/__init__.py` (units, features, sim
stepping), `scripts/recording/record_sim_vr_pickplace.py` (record loop, dataset API),
`utils/training.py` (coord normalisation, `prepare_obs_for_policy`, eval),
`scripts/inference/deploy_real_robot.py` (real loop, home pose, safety ramp),
`calibration/` (motor calibration, homography, ChArUco board; no intrinsics),
`REAL_ROBOT_PLAN.md` and `corl_experiments/EXPERIMENTS_LOG.md` (failure modes). One thing to
check when datasets are reused: the log says the 220-episode set has 1-indexed
`episode_scenes`, which silently gives episode 0 a (0, 0) coord.

## References for camera calibration

Two talks Daniel attended that bear on finding where the cameras are relative to the arm
(build-order step 3). Both were identified by searching the conference programmes; confirm
they are the right ones.

- **"ROS 2 roboreg: marker-free hand-eye calibration"**, ROSCon UK 2026, Edinburgh, 23 October
  2026 (<https://roscon.org.uk/2026/>). Registers the camera to the arm with no calibration
  target: it uses the joint positions and the robot model to predict what the arm should look
  like in the image and solves for the camera pose that makes that match. Code believed to be
  <https://github.com/lbr-stack/roboreg> (not checked against the talk).
- **"AutoTrack: Smart, Easy and Automated Camera Tracking"**, Siemen Lens, Blender Conference
  2026, Amsterdam (<https://www.youtube.com/watch?v=zvnUGii2TIY>; add-on at
  <https://extensions.blender.org/add-ons/siemen-lens-blender-autotrack/>). Automatic feature
  tracking and camera solve from video, so 3D objects can be composited into the footage.
  Relevant to the moving wrist camera: a camera path solved from video, compared with the
  gripper path from the joint angles, gives the camera-to-gripper transform without a target.
