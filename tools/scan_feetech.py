#!/usr/bin/env python3
"""Scan a Feetech STS/SMS serial bus (e.g. SO100 arm) and dump each servo.

No external install needed: uses the vendored scservo_sdk under ../vendor and
system pyserial. Run with the arm powered and its USB driver board connected.

Examples:
  python3 scan_feetech.py                      # auto-detect port, try common bauds
  python3 scan_feetech.py --port /dev/ttyACM0 --baud 1000000
  python3 scan_feetech.py --ids 1-12 --json
"""
import argparse
import glob
import json
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "vendor", "feetech-servo-sdk-1.0.0"))
import scservo_sdk as scs  # noqa: E402

# STS/SMS control table (name, address, length), little-endian (protocol end = 0)
TABLE = [
    ("Firmware_Major", 0, 1), ("Firmware_Minor", 1, 1),
    ("Model", 3, 2), ("ID", 5, 1), ("Baud_Rate", 6, 1),
    ("Min_Angle_Limit", 9, 2), ("Max_Angle_Limit", 11, 2),
    ("Max_Temp_Limit", 13, 1), ("Max_Voltage_Limit", 14, 1), ("Min_Voltage_Limit", 15, 1),
    ("Max_Torque_Limit", 16, 2), ("Operating_Mode", 33, 1), ("Torque_Enable", 40, 1),
    ("Goal_Position", 42, 2), ("Present_Position", 56, 2), ("Present_Speed", 58, 2),
    ("Present_Load", 60, 2), ("Present_Voltage", 62, 1), ("Present_Temperature", 63, 1),
    ("Moving", 66, 1), ("Present_Current", 69, 2),
]
BAUD_INDEX = {0: 1000000, 1: 500000, 2: 250000, 3: 128000, 4: 115200, 5: 76800, 6: 57600, 7: 38400}
MODELS = {777: "STS3215", 2825: "STS3250", 1540: "SCS0009"}  # best-effort labels
COMMON_BAUDS = [1000000, 500000, 115200, 57600, 250000, 128000]
UNKNOWN = "unknown"


def autoport():
    for p in sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")):
        return p
    return None


def read_reg(ph, port, sid, addr, length):
    if length == 1:
        v, c, e = ph.read1ByteTxRx(port, sid, addr)
    elif length == 2:
        v, c, e = ph.read2ByteTxRx(port, sid, addr)
    else:
        v, c, e = ph.read4ByteTxRx(port, sid, addr)
    return (v if c == scs.COMM_SUCCESS else None), c, e


def scan_one_baud(portname, baud, ids):
    port = scs.PortHandler(portname)
    ph = scs.PacketHandler(0)  # STS/SMS little-endian
    if not port.openPort():
        return None, "could not open port"
    if not port.setBaudRate(baud):
        port.closePort()
        return None, "could not set baud"
    found = []
    for sid in ids:
        model, c, e = ph.ping(port, sid)
        if c != scs.COMM_SUCCESS:
            continue
        regs = {}
        for name, addr, length in TABLE:
            val, rc, re = read_reg(ph, port, sid, addr, length)
            regs[name] = val
        found.append({
            "id": sid,
            "model_number": model,
            "model_name": MODELS.get(model, UNKNOWN),
            "regs": regs,
        })
    port.closePort()
    return found, None


def fmt_servo(s):
    r = s["regs"]
    volt = r.get("Present_Voltage")
    br = r.get("Baud_Rate")
    out = [
        "  ID {:<3} | model {} ({}) | fw {}.{}".format(
            s["id"], s["model_number"], s["model_name"],
            r.get("Firmware_Major"), r.get("Firmware_Minor")),
        "    baud_idx={} ({} bps)  mode={}  torque_en={}".format(
            br, BAUD_INDEX.get(br, "?"), r.get("Operating_Mode"), r.get("Torque_Enable")),
        "    pos={}  goal={}  speed={}  load={}  current={}".format(
            r.get("Present_Position"), r.get("Goal_Position"), r.get("Present_Speed"),
            r.get("Present_Load"), r.get("Present_Current")),
        "    voltage={} V  temp={} C  angle_limits=[{}, {}]".format(
            volt / 10.0 if volt is not None else None, r.get("Present_Temperature"),
            r.get("Min_Angle_Limit"), r.get("Max_Angle_Limit")),
    ]
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None)
    ap.add_argument("--baud", type=int, default=None, help="single baud; default tries common ones")
    ap.add_argument("--ids", default="1-20", help="e.g. 1-20 or 1,2,3")
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args()

    if "-" in a.ids:
        lo, hi = a.ids.split("-")
        ids = list(range(int(lo), int(hi) + 1))
    else:
        ids = [int(x) for x in a.ids.split(",")]

    portname = a.port or autoport()
    if not portname:
        print("No serial port found (no /dev/ttyUSB* or /dev/ttyACM*). "
              "Is the arm plugged in and powered?", file=sys.stderr)
        sys.exit(2)

    bauds = [a.baud] if a.baud else COMMON_BAUDS
    print("Scanning {}  ids {}-{}  bauds {}".format(portname, ids[0], ids[-1], bauds),
          file=sys.stderr)
    for baud in bauds:
        found, err = scan_one_baud(portname, baud, ids)
        if err:
            print("  baud {}: {}".format(baud, err), file=sys.stderr)
            continue
        if found:
            if a.json:
                print(json.dumps({"port": portname, "baud": baud, "servos": found}, indent=2))
            else:
                print("\n== {} servo(s) at {} bps on {} ==".format(len(found), baud, portname))
                for s in found:
                    print(fmt_servo(s))
            return
        print("  baud {}: no servos responded".format(baud), file=sys.stderr)
    print("No servos found at any baud. Check wiring, power, and port.", file=sys.stderr)
    sys.exit(1)


if __name__ == "__main__":
    main()
