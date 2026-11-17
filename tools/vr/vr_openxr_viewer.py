#!/usr/bin/env python
"""MuJoCo scene in the headset, optionally mirroring a physical arm (kinematic, no physics).

Linux/WiVRn port of lerobot-thesis/scripts/recording/vr_openxr_viewer.py.

    python vr_openxr_viewer.py                       # view the scene, free-running physics
    python vr_openxr_viewer.py --teleop              # arm pose copied straight into qpos
    python vr_openxr_viewer.py --teleop --port /dev/ttyACM0
    python vr_openxr_viewer.py --seconds 20 --snapshot /tmp/eye.png   # smoke test

Env needed on this desktop is in docs/desktop-setup.md (PRIME offload; DISPLAY over SSH).
"""
import argparse
import time

import mujoco

from vr_common import (DEFAULT_PORT, DEFAULT_SCENE, ArmReader, FpsMeter, VRRenderer,
                       arm_qpos_indices, load_scene)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--teleop", "-t", action="store_true", help="mirror a physical arm (kinematic)")
    ap.add_argument("--follower", action="store_true", help="the arm is a follower, not a leader")
    ap.add_argument("--port", "-p", default=DEFAULT_PORT, help=f"arm serial port (default {DEFAULT_PORT})")
    ap.add_argument("--scene", default=DEFAULT_SCENE, help="scene file name in scenes/, or a path")
    ap.add_argument("--floor", choices=["black", "scene"], default="black",
                    help="black = matte black ground with no reflection (default); scene = as in the XML")
    ap.add_argument("--fov-overlays", action="store_true", help="draw the wrist/overhead camera footprints")
    ap.add_argument("--no-mirror", action="store_true", help="do not mirror the left eye to the desktop window")
    ap.add_argument("--seconds", type=float, default=0, help="exit after this long (0 = run until closed)")
    ap.add_argument("--snapshot", help="save one left-eye frame to this image file")
    args = ap.parse_args()

    model, data = load_scene(args.scene, floor=args.floor)
    qidx = arm_qpos_indices(model)
    arm = ArmReader(args.port, is_follower=args.follower) if args.teleop else None
    vr = VRRenderer(model, data, mirror=not args.no_mirror)
    vr.show_wrist_cam_fov = vr.show_overhead_cam_fov = args.fov_overlays
    meter = FpsMeter()
    max_steps = int(0.05 / model.opt.timestep)
    sim_debt = 0.0
    t0 = time.perf_counter()
    try:
        vr.init_all()
        if arm:
            arm.connect()
        print("Controls: sticks move/yaw/height; X or A recentre now; Y or B recentre in 5 s "
              "(get to the real arm first);\n          thumbstick click = put the sim robot where that "
              "controller is. Desktop window: WASD/QE, R, T (delayed), P.")
        while vr.render_frame():
            vr.check_keyboard()
            extra = ""
            if arm:
                rad, age = arm.latest()
                if rad is not None:
                    data.qpos[qidx] = rad
                    extra = f" | arm data age {age * 1000:.1f} ms"
                mujoco.mj_forward(model, data)  # kinematics only
            else:
                sim_debt += vr.frame_dt  # real-time physics, paced by the headset
                steps = min(int(sim_debt / model.opt.timestep), max_steps)
                sim_debt -= steps * model.opt.timestep
                for _ in range(steps):
                    mujoco.mj_step(model, data)
            meter.tick(vr, extra)

            elapsed = time.perf_counter() - t0
            if args.snapshot and vr.frames_rendered == 30:
                vr.snapshot_path = args.snapshot
            if args.seconds and elapsed > args.seconds:
                break
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        print(f"Rendered {vr.frames_rendered} frames in {time.perf_counter() - t0:.1f} s; "
              f"final state {vr.session_state.name}")
        if arm:
            arm.close()
        vr.cleanup()


if __name__ == "__main__":
    main()
