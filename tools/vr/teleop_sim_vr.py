#!/usr/bin/env python
"""Teleoperate the SO101 MuJoCo sim from a physical arm, in VR or on screen (with physics).

Linux/WiVRn port of lerobot-thesis/scripts/recording/teleop_sim_vr.py. The arm's joint
angles become position-actuator targets and the sim is stepped in real time, paced by the
headset (90 Hz) instead of the old fixed 30 fps sleep.

    python teleop_sim_vr.py                    # VR, leader arm
    python teleop_sim_vr.py --port /dev/ttyACM0
    python teleop_sim_vr.py --follower         # drive from a follower arm instead
    python teleop_sim_vr.py --test             # VR, no arm
    python teleop_sim_vr.py --no-vr [--test]   # MuJoCo desktop viewer instead of the headset

Env needed on this desktop is in docs/desktop-setup.md (PRIME offload; DISPLAY over SSH).
"""
import argparse
import time

import mujoco

from vr_common import DEFAULT_PORT, DEFAULT_SCENE, ArmReader, FpsMeter, VRRenderer, load_scene


class RealTimeStepper:
    """Steps physics by wall-clock time, capped so a stall cannot cause a burst."""

    def __init__(self, model, data, max_catchup=0.05):
        self.model, self.data = model, data
        self.max_steps = int(max_catchup / model.opt.timestep)
        self.debt = 0.0
        self.last = time.perf_counter()

    def step(self):
        now = time.perf_counter()
        self.debt += now - self.last
        self.last = now
        n = min(int(self.debt / self.model.opt.timestep), self.max_steps)
        self.debt = min(self.debt - n * self.model.opt.timestep, self.model.opt.timestep)
        for _ in range(n):
            mujoco.mj_step(self.model, self.data)


def apply_arm(arm, data):
    """Copy the latest arm reading into the actuator targets. Returns False if the arm died."""
    if arm is None:
        return True
    if arm.failed:
        return False
    rad, _ = arm.latest()
    if rad is not None:
        data.ctrl[:6] = rad
    return True


def run_vr(model, data, arm, args):
    vr = VRRenderer(model, data, mirror=not args.no_mirror)
    vr.show_wrist_cam_fov = vr.show_overhead_cam_fov = args.fov_overlays
    meter = FpsMeter()
    try:
        vr.init_all()
        if arm:
            arm.connect()
        print("Controls: sticks move/yaw/height; X or A recentre now; Y or B recentre in 5 s "
              "(get to the real arm first);\n          thumbstick click = put the sim robot where that "
              "controller is. Desktop window: WASD/QE, R, T (delayed), P.")
        stepper = RealTimeStepper(model, data)
        t0 = time.perf_counter()
        while apply_arm(arm, data):
            stepper.step()
            if not vr.render_frame():
                break
            vr.check_keyboard()
            meter.tick(vr)
            if args.seconds and time.perf_counter() - t0 > args.seconds:
                break
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        print(f"Rendered {vr.frames_rendered} frames; final state {vr.session_state.name}")
        if arm:
            arm.close()
        vr.cleanup()


def run_screen(model, data, arm, args):
    import mujoco.viewer
    viewer = mujoco.viewer.launch_passive(model, data)
    try:
        if arm:
            arm.connect()
        print("Screen mode: close the viewer window or press Ctrl+C to exit.")
        stepper = RealTimeStepper(model, data)
        t0 = time.perf_counter()
        while viewer.is_running() and apply_arm(arm, data):
            stepper.step()
            viewer.sync()
            time.sleep(1.0 / 60)
            if args.seconds and time.perf_counter() - t0 > args.seconds:
                break
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        viewer.close()
        if arm:
            arm.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", "-p", default=DEFAULT_PORT, help=f"arm serial port (default {DEFAULT_PORT})")
    ap.add_argument("--test", "-t", action="store_true", help="no arm required")
    ap.add_argument("--no-vr", action="store_true", help="render to a desktop window instead of the headset")
    ap.add_argument("--follower", action="store_true", help="use a follower arm instead of the leader")
    ap.add_argument("--scene", default=DEFAULT_SCENE, help="scene file name in scenes/, or a path")
    ap.add_argument("--floor", choices=["black", "scene"], default="black",
                    help="black = matte black ground with no reflection (default); scene = as in the XML")
    ap.add_argument("--fov-overlays", action="store_true", help="draw the wrist/overhead camera footprints")
    ap.add_argument("--no-mirror", action="store_true", help="do not mirror the left eye to the desktop window")
    ap.add_argument("--seconds", type=float, default=0, help="exit after this long (0 = run until closed)")
    args = ap.parse_args()

    model, data = load_scene(args.scene, floor=args.floor)
    arm = None if args.test else ArmReader(args.port, is_follower=args.follower)
    (run_screen if args.no_vr else run_vr)(model, data, arm, args)


if __name__ == "__main__":
    main()
