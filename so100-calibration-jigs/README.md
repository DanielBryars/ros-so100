# SO-100 homing and calibration jigs: briefing

Everything found on the bench (18–19 November 2026) that matters for designing a jig to
re-zero the two SO-100 arms. Written to be read cold in a separate CAD session.

## Why a jig

The overhead camera to arm base transform is stuck at 2–3 cm because the arm's joint zeros
are wrong by several degrees. Two independent methods (silhouette fit, touch test) disagree
by 26 mm, and the cause is in the arm, not the camera:

- The calibration files were made by moving each joint through its range and mapping that
  range linearly onto the **SO-101** model's joint limits. Nothing ever tied a servo count
  to a known physical angle.
- The model used for sim, RViz and counts→radians is an **SO-101**. The real arms are
  **SO-100**.

Evidence that the zeros are off:

| Observation | Size |
|---|---|
| Follower forearm at commanded zero (should be horizontal) | slopes down about 8°, of which about 2° is sag under load |
| Touch-test fit when joint offsets are allowed | shoulder lift +8°, wrist flex −16° (xy error drops from 7.6 mm to 3.4 mm) |
| Sag under load, follower | 0.02–0.04 rad (1–2°) |
| Sag under load, leader | 0.06–0.12 rad (3.5–7°) |

The fitted offsets are indicative only; they also absorb the SO-101 vs SO-100 shape mismatch.

## What the jig has to deliver

One pose per arm in which **every joint's physical angle is known from geometry**, held
with the servos unpowered. In that pose I read the six servo counts and shift each joint's
calibration range so the counts read the known angles.

So the output of the CAD work that I need back is:

1. The pose the jig holds, as **the angle of each link relative to the previous link (or the
   base plate)**, taken from the CAD assembly. Physical geometry, not model joint values.
2. Which feature on each link defines that angle (e.g. "top face of forearm parallel to
   base plate").
3. For the gripper: whether it is closed, or open to a known gap.

I convert those to model joint angles for whichever model we end up using.

## The arms

| | Follower | Leader |
|---|---|---|
| Kit | SO-100 | SO-100 |
| Servos | Feetech STS3250, IDs 1–6, 1 Mbps | Feetech, IDs 1–6 |
| End of arm | Gripper with a wrist camera mounted on the **side** of the gripper | Handle with trigger |
| Encoder | 4096 counts per turn, 651.9 counts/rad | same |

Joint order and servo IDs: 1 shoulder_pan, 2 shoulder_lift, 3 elbow_flex, 4 wrist_flex,
5 wrist_roll, 6 gripper.

The wrist camera sticks out sideways and reaches the desk before the jaws when the wrist is
rolled. A jig for the follower has to clear it; a jig for the leader has to clear the handle.
The two arms therefore probably need different end-of-arm cradles, or a jig that references
only up to the wrist plus a separate roll/gripper reference.

## Model conventions (SO-101 model, currently used by the pipeline)

Files: `ros2_ws/src/so101_description/urdf/so101_new_calib.urdf`,
`scenes/so101_with_wrist_cam.xml`.

Base frame: x forward (away from the base, along the arm at pan 0), y left, z up. The origin
is 38.8 mm behind the pan axis. The desk surface is at z = −2.4 mm. The base plate is
87 × 110 mm in the model.

**Zero pose (all joints 0 rad):** upper arm vertical, forearm horizontal pointing forward,
gripper horizontal pointing forward, jaws opening vertically (moving jaw on top), gripper
slightly open. Renders: `model_zero_pose_side.png`, `model_zero_pose_three_quarter.png`,
`model_zero_pose_top.png` in this folder. These show SO-101 parts; use them for the pose
only, never for surfaces.

Positive directions and limits:

| Joint | Positive direction | Limits (deg) | Limits (rad) |
|---|---|---|---|
| shoulder_pan | arm swings toward −y (to the right, seen from behind) | ±110 | ±1.9199 |
| shoulder_lift | upper arm tilts forward | ±100 | ±1.7453 |
| elbow_flex | forearm tip goes down | ±96.8 | ±1.69 |
| wrist_flex | gripper tip goes down | ±95 | ±1.6581 |
| wrist_roll | (not checked on the bench) | −157.2 … +162.8 | −2.7438 … +2.8412 |
| gripper | opens | −10 … +100 | −0.1745 … +1.7453 |

Gripper closed is about −0.15 to −0.17 rad; 0 rad is slightly open.

### Link dimensions (from the URDF joint origins)

| From → to | Along the link | Offset |
|---|---|---|
| Base origin → pan axis | 38.8 mm forward, 62.4 mm up | |
| Pan axis → shoulder lift axis | 30.4 mm forward, 54.2 mm up | 18.3 mm to the side |
| Shoulder lift axis → elbow axis (upper arm) | 112.57 mm | 28 mm |
| Elbow axis → wrist flex axis (forearm) | 134.9 mm | 5.2 mm |
| Wrist flex axis → wrist roll body | 61.1 mm | 18.1 mm |

The SO-100 URDF in the repo (`ros2_ws/src/so_arm_100_description/urdf/`) gives the same
chain to within 1–3 mm up to the wrist (e.g. wrist roll 60.1 vs 61.1 mm). Link **shapes**,
the gripper and the jaw differ.

### Joint axis positions in the base frame (mm)

x forward, y left, z up from the base frame origin. Height above the desk = z + 2.4.

| Point | Zero pose (0,0,0,0,0) | Raised home (0,−0.3,0.3,0,0) | Folded park (0,−1.607,1.549,1.199,0) |
|---|---|---|---|
| Pan axis (shoulder origin) | 38.8, 0, 62.4 | same | same |
| Shoulder lift axis | 69.2, −18.3, 116.6 | same | same |
| Elbow axis | 97.2, −18.3, 229.2 | 62.7, −18.3, 232.4 | −44.3, −18.3, 140.5 |
| Wrist flex axis | 232.1, −18.3, 234.4 | 197.6, −18.3, 237.6 | 90.1, −18.3, 153.5 |
| Wrist roll body | 293.2, −0.2, 234.4 | 258.7, −0.2, 237.6 | 115.6, −0.2, 98.0 |
| Jaw tip (SO-101 jaw, not valid for SO-100) | 397.7, 0.1, 224.2 | 363.2, 0.1, 227.4 | 149.8, 0.1, −1.3 |

Heights above the desk at the zero pose: shoulder lift axis 119.0, elbow axis 231.6, wrist
flex axis 236.8 mm. Model undersides at zero (SO-101 shapes, indicative only): upper arm
107.0, forearm 216.6, wrist 221.1, gripper body 201.0 mm.

The folded park pose is where both arms were left on 19 November with torque off: upper arm
back, forearm folded forward over it, gripper tipped down. The arms stay there unpowered.
It is close to the folded rest pose of the FreeCAD assembly (`SO_5DOF_ARM100_08k v1`).

## Current calibration

Files: `ros2_ws/src/so100_telemetry/config/follower_so100.json`, `leader_so100.json`.
`homing_offset` is 0 for every joint. The driver
(`so100_telemetry/joint_state_publisher.py`) maps counts linearly:

    rad = lower + (counts − range_min) / (range_max − range_min) × (upper − lower)

with `lower`/`upper` the model limits above. All ranges have the same slope, 651.9 counts/rad
(4096 per turn), so re-zeroing is a pure shift of `range_min` and `range_max` by the same
number of counts per joint.

| Joint | Follower range_min–max | Follower count at 0 rad | Leader range_min–max | Leader count at 0 rad |
|---|---|---|---|---|
| shoulder_pan | 796–3299 | 2047.5 | 774–3277 | 2025.5 |
| shoulder_lift | 910–3185 | 2047.5 | 133–2408 | 1270.5 |
| elbow_flex | 1008–3211 | 2109.5 | 1145–3348 | 2246.5 |
| wrist_flex | 846–3007 | 1926.5 | 1555–3716 | 2635.5 |
| wrist_roll | 284–3925 | 2073 | 257–3898 | 2046 |
| gripper | 2057–3308 | 2171 | 2048–3299 | 2162 |

One degree is 11.4 counts, so a jig that fixes a joint to ±0.5° pins the zero to about
±6 counts.

Count wrapping: the driver has no wrap handling. It works because no joint's range crosses
the 0/4095 seam. The leader shoulder lift is closest (range starts at 133). If re-zeroing
pushes a range across the seam, the driver needs unwrapping added first; that is a known
to-do, not a jig constraint.

## Design guidance

- **Design against the SO-100 CAD, not the SO-101 model.** Every surface the jig touches
  must come from the SO-100 STEP parts (TheRobotStudio SO-ARM100 repository; I have not
  checked which STEP files it ships).
- **Reference printed link faces, not servo horns.** The horn-to-link angle is exactly what
  is unknown: horns go on at arbitrary spline positions.
- **Locate the base plate first**, in a pocket or against two edges, so everything else is
  relative to it.
- **Choose a pose the arm rests in under gravity** with torque off, so nothing has to be
  clamped. The folded rest pose of the CAD assembly is a good candidate. The model zero
  pose is a poor one: the arm is cantilevered and has to be held up.
- **Put contacts far from the joint axes.** Angular error is clearance divided by lever arm:
  0.2 mm at 100 mm is 0.11°; at 20 mm it is 0.57°.
- **Constrain each joint once.** One contact per joint that fixes its angle; over-constraint
  makes the arm bind and sit wrong.
- **Clearance** about 0.2 mm on printed mating faces.
- **Gripper:** closed is its own reference (jaws touching); no jig feature needed unless a
  known open gap is preferred.
- **Wrist roll** needs something flat to register on, such as the gripper's side face against
  a jig wall, and must clear the follower's side-mounted camera.
- **Pan** can be set by the jig's centreline: forearm or gripper centred between two walls
  that are square to the base plate pocket.
- **Backlash and sag:** the servos have play, and the leader sags several degrees under
  load. A resting pose loads each joint in one direction, which is repeatable. Note which
  way each joint is loaded in the jig.

A jig defined by physical geometry survives a later switch of the sim model from SO-101 to
SO-100 (the SO-100 URDF has different joint names and a different zero pose). A jig built
to reproduce "SO-101 zero" would not.

## What happens after the jig exists

1. Put each arm in its jig with torque off and read the counts (`/follower/joint_counts`,
   `/leader/joint_counts`, or the count columns in `tools/jog.py`).
2. Convert the jig's physical pose to model joint angles.
3. Shift `range_min`/`range_max` per joint so the jig counts read those angles.
4. Check with `tools/jog.py`: command zero and measure the forearm against the desk.
5. Redo the camera to base transform (silhouette fit, touch test, depth as a third check).

## Open questions

- Which model the sim uses long term (SO-101 as now, or an SO-100 model). Does not block the
  jig if the pose is defined by geometry.
- Whether the SO-ARM100 repository has STEP files for every SO-100 part in this build,
  including the wrist camera mount and the leader handle.
- Wrist roll positive direction has not been checked on the real arm.
- The calibration board card thickness (3 mm assumed) is unmeasured; irrelevant to the jig
  but relevant to the touch test afterwards.
