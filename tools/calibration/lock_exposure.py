#!/usr/bin/env python
"""Find fixed exposure, gain and white balance for a camera from a sheet of white paper,
so the camera can run with its auto modes off.

    python lock_exposure.py --camera wrist_cam --device /dev/v4l/by-path/...
    python lock_exposure.py --camera overhead_cam --device /dev/v4l/by-id/...

Put white paper (the ChArUco board will do) where the camera sees it, covering at least a
tenth of the image, under the lighting you will work in, and keep the scene still. The camera
must be streaming on /<camera>/image_raw (cameras.launch.py). The script switches to manual
and searches:
  - exposure: the shortest of 10/20/30 ms that gets the paper to the target level with no
    more than half the gain range (multiples of 10 ms do not flicker under 50 Hz mains light),
  - gain: so the paper sits at --white-level (default 225 of 255: bright, not clipped),
  - white balance temperature: so red = blue on the paper; any remaining tint (cheap cameras
    leave a green cast in manual mode) is given as fixed R, G, B gains for the camera node.
Auto modes are a poor reference on a dark table: they brighten the scene until white clips.
It leaves the camera on the result and prints the `controls` and `colour_gains` values for
cameras.launch.py, which applies them every time the camera is opened. Start the cameras with
`locked:=false` for this, so the image it measures has no gains applied yet.
Run with system python3 after sourcing ROS. Needs v4l2-ctl (v4l-utils).
"""
import argparse
import subprocess
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class Probe:
    def __init__(self, camera, device):
        self.device = device
        self.node = rclpy.create_node('lock_exposure')
        self.frames = []
        self.node.create_subscription(Image, f'/{camera}/image_raw', self.on_image,
                                      qos_profile_sensor_data)

    def on_image(self, msg):
        a = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.width, 3)
        px = a[::2, ::2].reshape(-1, 3).astype(np.float32)
        grey = px.mean(axis=1)
        paper = px[grey > 0.85 * np.percentile(grey, 95)].mean(axis=0)   # mean R, G, B of the paper
        self.frames.append((paper.max(), paper[0] / paper[2], *paper))

    def set(self, **ctrls):
        for name, value in ctrls.items():
            subprocess.run(['v4l2-ctl', '-d', self.device, '--set-ctrl', f'{name}={value}'],
                           check=True)

    def limits(self, name):
        out = subprocess.run(['v4l2-ctl', '-d', self.device, '--list-ctrls'],
                             capture_output=True, text=True, check=True).stdout
        line = next(ln for ln in out.splitlines() if ln.split() and ln.split()[0] == name)
        f = dict(kv.split('=') for kv in line.split(':', 1)[1].split() if '=' in kv)
        return int(f['min']), int(f['max']), int(f.get('step', 1))

    def measure(self, settle=0.7, frames=10):
        """(paper brightest channel, red/blue, R, G, B) after a control change has taken effect."""
        end = time.time() + settle
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.frames.clear()
        end = time.time() + 5.0
        while len(self.frames) < frames:
            if time.time() > end:
                raise RuntimeError('no images arriving; is the camera node running?')
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return np.mean(self.frames, axis=0)


def search(lo, hi, step, value_at, target):
    """Integer bisection for a control whose measured value rises with the setting."""
    while hi - lo > step:
        mid = (lo + hi) // 2 // step * step
        if value_at(mid) < target:
            lo = mid
        else:
            hi = mid
    return min((lo, hi), key=lambda v: abs(value_at(v) - target))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--camera', required=True, help='camera namespace, e.g. wrist_cam')
    ap.add_argument('--device', required=True, help='the V4L2 device the camera node has open')
    ap.add_argument('--white-level', type=int, default=225, help='where white paper should sit, 0-255')
    args = ap.parse_args()

    rclpy.init()
    p = Probe(args.camera, args.device)
    target = args.white_level

    g_lo, g_hi, g_step = p.limits('gain')
    t_lo, t_hi, t_step = p.limits('white_balance_temperature')
    p.set(auto_exposure=1, white_balance_automatic=0,
          white_balance_temperature=(t_lo + t_hi) // 2 // t_step * t_step)

    cache = {}

    def lum(gain):
        if gain not in cache:
            p.set(gain=gain)
            cache[gain] = p.measure()[0]
        return cache[gain]

    def find_gain():
        cache.clear()
        return search(g_lo, g_hi, g_step, lum, target)

    for exposure in (100, 200, 300):   # units of 100 us
        p.set(exposure_time_absolute=exposure)
        cache.clear()
        if lum((g_lo + g_hi) // 2 // g_step * g_step) >= target or exposure == 300:
            break
    p.set(gain=find_gain())   # so the paper is not clipped while white balance is searched

    rb = {}

    def red_blue(temp):   # a higher temperature setting makes the image warmer (more red)
        if temp not in rb:
            p.set(white_balance_temperature=temp)
            rb[temp] = p.measure()[1]
        return rb[temp]

    rising = red_blue(t_hi) > red_blue(t_lo)
    temp = search(t_lo, t_hi, t_step, red_blue if rising else (lambda t: -red_blue(t)),
                  1.0 if rising else -1.0)
    p.set(white_balance_temperature=temp)
    gain = find_gain()        # the white balance setting shifts the levels a little
    p.set(gain=gain)
    final = p.measure(frames=20)
    # Whatever tint the camera's own white balance cannot remove (cheap cameras leave a green
    # cast in manual mode) is taken out by fixed per-channel gains in the camera node. They are
    # scaled so none is below 1, which keeps clipped highlights white.
    gains = final[2:].max() / final[2:]
    print(f'paper R,G,B {final[2]:.0f},{final[3]:.0f},{final[4]:.0f} (brightest channel at {target})')
    print('controls: ' + ','.join(f'{k}={v}' for k, v in [
        ('auto_exposure', 1), ('exposure_time_absolute', exposure), ('gain', gain),
        ('white_balance_automatic', 0), ('white_balance_temperature', temp)]))
    print('colour_gains: [%.3f, %.3f, %.3f]' % tuple(gains))
    rclpy.shutdown()


if __name__ == '__main__':
    main()
