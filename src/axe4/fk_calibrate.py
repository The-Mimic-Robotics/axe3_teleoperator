"""
AXE4 FK calibration wizard: motor signs and offsets for planar FK.

Run once per arm to get correct motor_cfg. Writes to src/axe4/config/axe4_axis_calibration.json
(merge with existing keys). Leader loads motor_cfg from that file.

  python src/axe4/fk_calibrate.py --port /dev/ttyACM0 [--id axe]
"""

import argparse
import json
import time
from pathlib import Path

import numpy as np

from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode

CALIB_PATH = Path(__file__).resolve().parent / "config" / "axe4_axis_calibration.json"
CALIB_ROOT = Path.home() / ".cache/huggingface/lerobot/calibration/teleoperators/axe4_leader"
NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "elbow_super_flex"]
SHORT = ["pan", "lift", "elbow", "super"]
SAMPLE_N, SAMPLE_HZ = 15, 30
DESIRED_HOME_DEG = [0.0, 90.0, -90.0, -90.0]  # q1=0, q2=90, q3=-90, q4=-90 at rest


def connect(port: str, arm_id: str):
    motors = {
        "shoulder_pan": Motor(1, "sts3215", MotorNormMode.DEGREES),
        "shoulder_lift": Motor(2, "sts3215", MotorNormMode.DEGREES),
        "elbow_flex": Motor(3, "sts3215", MotorNormMode.DEGREES),
        "elbow_super_flex": Motor(4, "sts3215", MotorNormMode.DEGREES),
    }
    calib = None
    path = CALIB_ROOT / f"{arm_id}.json"
    if path.exists():
        with open(path) as f:
            raw = json.load(f)
        calib = {n: MotorCalibration(id=raw[n]["id"], drive_mode=raw[n]["drive_mode"],
                  homing_offset=raw[n]["homing_offset"], range_min=raw[n]["range_min"],
                  range_max=raw[n]["range_max"]) for n in motors}
    bus = FeetechMotorsBus(port=port, motors=motors, calibration=calib)
    bus.connect()
    if calib:
        bus.write_calibration(calib)
    bus.disable_torque()
    for m in motors:
        bus.write("Operating_Mode", m, OperatingMode.POSITION.value)
    return bus


def sample(bus):
    readings = []
    for _ in range(SAMPLE_N):
        readings.append(bus.sync_read("Present_Position"))
        time.sleep(1.0 / SAMPLE_HZ)
    return {k: float(np.mean([r[k] for r in readings])) for k in readings[0]}


def delta(a, b):
    return {k: a[k] - b[k] for k in a}


def fmt(d):
    return "  ".join(f"{SHORT[i]}={d[NAMES[i]]:+7.1f}" for i in range(4))


def prompt_and_sample(bus, msg):
    print(f"\n>>> {msg}")
    input("    Press ENTER when ready... ")
    print("    Sampling...", end="", flush=True)
    d = sample(bus)
    print(f"  got: {fmt(d)}")
    return d


def live_monitor(bus, ref, duration=999):
    print("    (move now — live feedback below, press Ctrl+C when in position)\n")
    try:
        while True:
            deg = bus.sync_read("Present_Position")
            dd = delta(deg, ref)
            top = max(dd, key=lambda k: abs(dd[k]))
            top_i = NAMES.index(top)
            tag = f"  <- {SHORT[top_i]} {dd[top]:+.1f}" if abs(dd[top]) > 2 else ""
            cols = "  ".join(f"{SHORT[i]}={dd[NAMES[i]]:+6.1f}" for i in range(4))
            print(f"    {cols}{tag}              ", end="\r")
            time.sleep(0.04)
    except KeyboardInterrupt:
        print()
    d = sample(bus)
    print(f"    Captured: {fmt(d)}")
    return d


def run_wizard(bus):
    data = {}
    print("\n" + "=" * 65)
    print("AXE4 FK CALIBRATION WIZARD")
    print("=" * 65)
    print("Follow prompts. At each step: move arm, then ENTER (or Ctrl+C for live steps).\n")

    print("--- PHASE 1: HOME ---")
    print("Put arm in rest pose: upper arm UP, forearm HORIZONTAL, handle DOWN.")
    data["HOME"] = prompt_and_sample(bus, "Hold HOME position.")
    home = data["HOME"]

    print("\n--- PHASE 2: JOINT LIMITS ---")
    print("Move ONLY the specified joint to each extreme; Ctrl+C to capture.\n")
    for name, msg in [
        ("PAN_LEFT", "Rotate BASE as far LEFT."),
        ("PAN_RIGHT", "Return HOME, then BASE as far RIGHT."),
        ("LIFT_MAX", "Return HOME. SHOULDER to one extreme."),
        ("LIFT_MIN", "Return HOME. SHOULDER to other extreme."),
        ("ELBOW_MAX", "Return HOME. ELBOW to one extreme."),
        ("ELBOW_MIN", "Return HOME. ELBOW to other extreme."),
        ("SUPER_MAX", "Return HOME. WRIST/SUPER to one extreme."),
        ("SUPER_MIN", "Return HOME. WRIST/SUPER to other extreme."),
    ]:
        print(f"\n--- {name} ---\n>>> {msg}")
        data[name] = live_monitor(bus, home)

    print("\n--- PHASE 3: EEF DIRECTIONS ---")
    print("From HOME, move the grip in each direction; Ctrl+C to capture.\n")
    data["EEF_HOME"] = prompt_and_sample(bus, "Return to HOME first.")
    eef_home = data["EEF_HOME"]
    for name, msg in [
        ("EEF_FORWARD", "Move grip FORWARD (away from you)."),
        ("EEF_BACKWARD", "Return HOME. Move grip BACKWARD."),
        ("EEF_LEFT", "Return HOME. Move grip LEFT."),
        ("EEF_RIGHT", "Return HOME. Move grip RIGHT."),
        ("EEF_UP", "Return HOME. Move grip UP."),
        ("EEF_DOWN", "Return HOME. Move grip DOWN."),
    ]:
        print(f"\n--- {name} ---\n>>> {msg}")
        data[name] = live_monitor(bus, eef_home)

    return data, home, eef_home


def analyze(data, home, eef_home):
    pairs = [
        ("shoulder_pan", "PAN_LEFT", "PAN_RIGHT"),
        ("shoulder_lift", "LIFT_MAX", "LIFT_MIN"),
        ("elbow_flex", "ELBOW_MAX", "ELBOW_MIN"),
        ("elbow_super_flex", "SUPER_MAX", "SUPER_MIN"),
    ]
    ranges = {}
    for motor, ka, kb in pairs:
        da = data[ka][motor] - home[motor]
        db = data[kb][motor] - home[motor]
        lo, hi = min(da, db), max(da, db)
        ranges[motor] = {"min_delta": round(lo, 1), "max_delta": round(hi, 1), "range": round(hi - lo, 1)}

    eef_deltas = {}
    for direction in ["FORWARD", "BACKWARD", "LEFT", "RIGHT", "UP", "DOWN"]:
        key = f"EEF_{direction}"
        if key in data:
            eef_deltas[direction] = delta(data[key], eef_home)

    signs = [None, None, None, None]
    if "LEFT" in eef_deltas and "RIGHT" in eef_deltas:
        dl, dr = eef_deltas["LEFT"]["shoulder_pan"], eef_deltas["RIGHT"]["shoulder_pan"]
        signs[0] = 1 if (abs(dl) > abs(dr) and dl > 0) or (abs(dr) >= abs(dl) and dr < 0) else -1
    if "UP" in eef_deltas and "DOWN" in eef_deltas:
        du, dd = eef_deltas["UP"]["shoulder_lift"], eef_deltas["DOWN"]["shoulder_lift"]
        if abs(du) > 5 or abs(dd) > 5:
            signs[1] = 1 if du > 0 else -1
        else:
            df = eef_deltas.get("FORWARD", {}).get("shoulder_lift", 0)
            signs[1] = -1 if df > 0 else 1
    if "UP" in eef_deltas and "DOWN" in eef_deltas:
        du, dd = eef_deltas["UP"]["elbow_flex"], eef_deltas["DOWN"]["elbow_flex"]
        signs[2] = 1 if (abs(du) > abs(dd) and du > 0) or (abs(dd) >= abs(du) and dd < 0) else -1
    if "DOWN" in eef_deltas and "FORWARD" in eef_deltas:
        df, dd = eef_deltas["FORWARD"]["elbow_super_flex"], eef_deltas["DOWN"]["elbow_super_flex"]
        if abs(dd) > abs(df):
            signs[3] = 1 if dd < 0 else -1
        else:
            signs[3] = -1 if df > 0 else 1
    defaults = [1, -1, -1, -1]
    for i in range(4):
        if signs[i] is None:
            signs[i] = defaults[i]

    offsets = [0.0, 0.0, 0.0, 0.0]
    for i in range(4):
        m_home = home[NAMES[i]]
        offsets[i] = DESIRED_HOME_DEG[i] - signs[i] * m_home

    motor_cfg = [{"name": NAMES[i], "sign": signs[i], "offset": round(offsets[i], 1)} for i in range(4)]
    return motor_cfg, ranges


def main():
    ap = argparse.ArgumentParser(description="AXE4 FK calibration (motor signs + offsets)")
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--id", default="axe", help="lerobot calibration id")
    args = ap.parse_args()

    bus = connect(args.port, args.id)
    try:
        data, home, eef_home = run_wizard(bus)
        motor_cfg, ranges = analyze(data, home, eef_home)
    finally:
        bus.disconnect()

    print("\n--- Result: motor_cfg ---")
    for m in motor_cfg:
        print(f"  {m['name']:20s}  sign={m['sign']:+d}  offset={m['offset']:+.1f}")

    existing = {}
    if CALIB_PATH.exists():
        with open(CALIB_PATH) as f:
            existing = json.load(f)
    existing["motor_cfg"] = motor_cfg
    existing["fk_ranges_deg"] = ranges

    CALIB_PATH.parent.mkdir(parents=True, exist_ok=True)
    with open(CALIB_PATH, "w") as f:
        json.dump(existing, f, indent=2)

    print(f"\nSaved motor_cfg → {CALIB_PATH}")
    print("Restart teleop to use new FK calibration.")


if __name__ == "__main__":
    main()
