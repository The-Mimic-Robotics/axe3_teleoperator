#!/usr/bin/env python3
"""
AXE4 FK Calibration Wizard.

Guided session to determine:
  - Correct sign for each motor
  - Correct offset for each motor
  - Joint limits
  - EEF direction mapping

Saves results to axe4_FK/calibration_result.json
then prints the corrected MOTOR_CFG you can paste into live_eef.py.

Usage:
    conda activate lerobot
    python axe4_FK/calibrate.py --port /dev/ttyACM0
"""

import argparse
import time
import json
import numpy as np
from pathlib import Path

# ΓöÇΓöÇ Motor bus setup (same as live_eef.py) ΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇ

CALIB_ROOT = (
    Path.home()
    / ".cache/huggingface/lerobot/calibration/teleoperators/axe4_leader"
)


def connect(port, arm_id):
    from lerobot.motors import Motor, MotorCalibration, MotorNormMode
    from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode

    motors = {
        "shoulder_pan":      Motor(1, "sts3215", MotorNormMode.DEGREES),
        "shoulder_lift":     Motor(2, "sts3215", MotorNormMode.DEGREES),
        "elbow_flex":        Motor(3, "sts3215", MotorNormMode.DEGREES),
        "elbow_super_flex":  Motor(4, "sts3215", MotorNormMode.DEGREES),
    }

    calib = None
    path = CALIB_ROOT / f"{arm_id}.json"
    if path.exists():
        with open(path) as f:
            raw = json.load(f)
        calib = {}
        for name, info in raw.items():
            calib[name] = MotorCalibration(
                id=info["id"],
                drive_mode=info["drive_mode"],
                homing_offset=info["homing_offset"],
                range_min=info["range_min"],
                range_max=info["range_max"],
            )

    bus = FeetechMotorsBus(port=port, motors=motors, calibration=calib)
    bus.connect()

    if calib:
        bus.write_calibration(calib)
        print(f"Calibration loaded (id='{arm_id}')")

    bus.disable_torque()
    for m in motors:
        bus.write("Operating_Mode", m, OperatingMode.POSITION.value)

    return bus


# ΓöÇΓöÇ Helpers ΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇ

NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "elbow_super_flex"]
SHORT = ["pan", "lift", "elbow", "super"]

SAMPLE_N = 15
SAMPLE_HZ = 30


def sample(bus):
    """Average a burst of readings to reduce noise."""
    readings = []
    for _ in range(SAMPLE_N):
        readings.append(bus.sync_read("Present_Position"))
        time.sleep(1.0 / SAMPLE_HZ)
    out = {}
    for k in readings[0]:
        out[k] = float(np.mean([r[k] for r in readings]))
    return out


def delta(a, b):
    return {k: a[k] - b[k] for k in a}


def fmt(d):
    return "  ".join(f"{SHORT[i]}={d[NAMES[i]]:+7.1f}" for i in range(4))


def banner(text):
    print()
    print("=" * 65)
    print(text)
    print("=" * 65)


def prompt_and_sample(bus, msg):
    print(f"\n>>> {msg}")
    input("    Press ENTER when ready... ")
    print("    Sampling...", end="", flush=True)
    d = sample(bus)
    print(f"  got: {fmt(d)}")
    return d


def live_monitor(bus, ref, duration=999):
    """Show live motor deltas until user presses ENTER (via KeyboardInterrupt hack)."""
    print("    (move now ΓÇö live feedback below, press Ctrl+C when in position)\n")
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


# ΓöÇΓöÇ Main calibration session ΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇ

def run(bus):
    data = {}

    banner("AXE4 FK CALIBRATION WIZARD")
    print("""
This will guide you through a series of arm poses.
For each step:
  1. Move the arm as instructed
  2. Press ENTER (or Ctrl+C for live-monitored steps)
  3. Hold still while it samples

The result: correct motor signs, offsets, and joint limits.
""")

    # ΓöÇΓöÇ PHASE 1: HOME ΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇ
    banner("PHASE 1: HOME POSITION")
    print("Put the arm in its natural REST pose (arch/portal shape).")
    print("Links should be roughly 90 deg from each other:")
    print("  upper arm UP, forearm HORIZONTAL, handle DOWN")
    data["HOME"] = prompt_and_sample(bus, "Hold the arm in HOME position.")
    home = data["HOME"]

    # ΓöÇΓöÇ PHASE 2: INDIVIDUAL JOINT RANGES ΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇ
    banner("PHASE 2: INDIVIDUAL JOINT LIMITS")
    print("For each joint, move it to both extremes.")
    print("Move ONLY the specified joint, keep others still.")
    print("Live monitoring: move the joint, then Ctrl+C to capture.\n")

    joint_steps = [
        ("PAN_LEFT",     "Rotate the BASE (joint 1) as far LEFT as it goes."),
        ("PAN_RIGHT",    "Return to HOME. Now rotate BASE as far RIGHT."),
        ("LIFT_MAX",     "Return to HOME. Push SHOULDER (joint 2) to one extreme."),
        ("LIFT_MIN",     "Return to HOME. Push SHOULDER to the OTHER extreme."),
        ("ELBOW_MAX",    "Return to HOME. Push ELBOW (joint 3) to one extreme."),
        ("ELBOW_MIN",    "Return to HOME. Push ELBOW to the OTHER extreme."),
        ("SUPER_MAX",    "Return to HOME. Push WRIST/SUPER (joint 4) to one extreme."),
        ("SUPER_MIN",    "Return to HOME. Push WRIST/SUPER to the OTHER extreme."),
    ]

    for name, msg in joint_steps:
        print(f"\n--- {name} ---")
        print(f">>> {msg}")
        data[name] = live_monitor(bus, home)

    # ΓöÇΓöÇ PHASE 3: EEF DIRECTIONS ΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇ
    banner("PHASE 3: EEF DIRECTIONS")
    print("From HOME, move the GRIP/HANDLE in each direction.")
    print("Use whatever combination of joints feels natural.")
    print("Live monitoring: move, then Ctrl+C to capture.\n")

    data["EEF_HOME"] = prompt_and_sample(bus, "Return to HOME position first.")
    eef_home = data["EEF_HOME"]

    eef_steps = [
        ("EEF_FORWARD",  "Move the grip FORWARD (away from you)."),
        ("EEF_BACKWARD", "Return to HOME. Move the grip BACKWARD (toward you)."),
        ("EEF_LEFT",     "Return to HOME. Move the grip LEFT."),
        ("EEF_RIGHT",    "Return to HOME. Move the grip RIGHT."),
        ("EEF_UP",       "Return to HOME. Move the grip UP."),
        ("EEF_DOWN",     "Return to HOME. Move the grip DOWN."),
    ]

    for name, msg in eef_steps:
        print(f"\n--- {name} ---")
        print(f">>> {msg}")
        data[name] = live_monitor(bus, eef_home)

    # ΓöÇΓöÇ SAVE RAW DATA ΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇ
    out_path = Path("axe4_FK/calibration_data.json")
    with open(out_path, "w") as f:
        json.dump(data, f, indent=2)
    print(f"\nRaw data saved ΓåÆ {out_path}")

    # ΓöÇΓöÇ ANALYZE ΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇ
    analyze(data)


# ΓöÇΓöÇ Analysis ΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇ

def analyze(data):
    banner("ANALYSIS")

    home = data["HOME"]
    eef_home = data.get("EEF_HOME", home)

    # ΓöÇΓöÇ Joint ranges ΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇ
    print("\n--- Joint ranges (motor degrees from HOME) ---")
    pairs = [
        ("shoulder_pan",      "PAN_LEFT",    "PAN_RIGHT"),
        ("shoulder_lift",     "LIFT_MAX",    "LIFT_MIN"),
        ("elbow_flex",        "ELBOW_MAX",   "ELBOW_MIN"),
        ("elbow_super_flex",  "SUPER_MAX",   "SUPER_MIN"),
    ]

    ranges = {}
    for motor, key_a, key_b in pairs:
        da = data[key_a][motor] - home[motor]
        db = data[key_b][motor] - home[motor]
        lo = min(da, db)
        hi = max(da, db)
        ranges[motor] = {"min_delta": round(lo, 1), "max_delta": round(hi, 1),
                         "range": round(hi - lo, 1)}
        print(f"  {motor:20s}  [{lo:+7.1f} .. {hi:+7.1f}]  range={hi-lo:.1f} deg")

    # ΓöÇΓöÇ EEF direction analysis ΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇ
    print("\n--- EEF direction motor deltas from HOME ---")
    print(f"  {'Direction':12s}  {'pan':>7s}  {'lift':>7s}  {'elbow':>7s}  {'super':>7s}  biggest_motor")

    eef_deltas = {}
    for direction in ["FORWARD", "BACKWARD", "LEFT", "RIGHT", "UP", "DOWN"]:
        key = f"EEF_{direction}"
        if key not in data:
            continue
        dd = delta(data[key], eef_home)
        eef_deltas[direction] = dd

        top = max(dd, key=lambda k: abs(dd[k]))
        vals = "  ".join(f"{dd[NAMES[i]]:+7.1f}" for i in range(4))
        print(f"  {direction:12s}  {vals}  {top}")

    # ΓöÇΓöÇ Sign determination ΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇ
    print("\n--- Sign determination ---")
    print("Convention: +q1=LEFT, +q2=more-upward, +q3=open-elbow, +q4=open-wrist")

    signs = [None, None, None, None]

    # Pan sign: LEFT should give +q1
    if "LEFT" in eef_deltas and "RIGHT" in eef_deltas:
        d_left = eef_deltas["LEFT"]["shoulder_pan"]
        d_right = eef_deltas["RIGHT"]["shoulder_pan"]
        if abs(d_left) > abs(d_right):
            signs[0] = +1 if d_left > 0 else -1
        else:
            signs[0] = +1 if d_right < 0 else -1
        print(f"  Pan:   LEFT gave pan delta {d_left:+.1f}, "
              f"RIGHT gave {d_right:+.1f}  ΓåÆ  sign = {signs[0]:+d}")

    # Lift sign: UP should increase q2 (more upward)
    if "UP" in eef_deltas and "DOWN" in eef_deltas:
        d_up = eef_deltas["UP"]["shoulder_lift"]
        d_down = eef_deltas["DOWN"]["shoulder_lift"]
        if abs(d_up) > 5 or abs(d_down) > 5:
            # UP means more vertical = larger q2, so +q2 when moving up
            signs[1] = +1 if d_up > 0 else -1
            print(f"  Lift:  UP gave lift delta {d_up:+.1f}, "
                  f"DOWN gave {d_down:+.1f}  ΓåÆ  sign = {signs[1]:+d}")
        else:
            print(f"  Lift:  UP/DOWN deltas too small ({d_up:+.1f}/{d_down:+.1f}), "
                  "using FORWARD/BACKWARD")
            d_fwd = eef_deltas.get("FORWARD", {}).get("shoulder_lift", 0)
            d_bwd = eef_deltas.get("BACKWARD", {}).get("shoulder_lift", 0)
            # FORWARD = lean forward = decrease q2
            signs[1] = -1 if d_fwd > 0 else +1
            print(f"         FWD gave lift delta {d_fwd:+.1f}  ΓåÆ  sign = {signs[1]:+d}")

    # Elbow sign: FORWARD should open elbow (increase a3 toward horizontal)
    # At home a3=0 (horizontal), going forward means a3 stays ~0 but arm reaches out
    # More practically: look at which direction elbow motor moves during UP vs DOWN
    if "UP" in eef_deltas and "DOWN" in eef_deltas:
        d_up = eef_deltas["UP"]["elbow_flex"]
        d_down = eef_deltas["DOWN"]["elbow_flex"]
        d_fwd = eef_deltas.get("FORWARD", {}).get("elbow_flex", 0)
        biggest = max([d_up, d_down, d_fwd], key=abs)
        # UP typically means elbow opens (forearm goes more upward) ΓåÆ ╬öq3 > 0
        if abs(d_up) > abs(d_down):
            signs[2] = +1 if d_up > 0 else -1
        else:
            signs[2] = +1 if d_down < 0 else -1
        print(f"  Elbow: UP gave elbow delta {d_up:+.1f}, "
              f"DOWN gave {d_down:+.1f}  ΓåÆ  sign = {signs[2]:+d}")

    # Super sign: similar logic
    if "DOWN" in eef_deltas and "FORWARD" in eef_deltas:
        d_fwd = eef_deltas["FORWARD"]["elbow_super_flex"]
        d_down = eef_deltas["DOWN"]["elbow_super_flex"]
        d_up = eef_deltas.get("UP", {}).get("elbow_super_flex", 0)
        if abs(d_down) > abs(d_fwd):
            signs[3] = +1 if d_down < 0 else -1
        else:
            signs[3] = -1 if d_fwd > 0 else +1
        print(f"  Super: FWD gave super delta {d_fwd:+.1f}, "
              f"DOWN gave {d_down:+.1f}  ΓåÆ  sign = {signs[3]:+d}")

    # Fill in any None signs with best guess
    defaults = [+1, -1, -1, -1]
    for i in range(4):
        if signs[i] is None:
            signs[i] = defaults[i]
            print(f"  {SHORT[i]:6s}: could not determine, using default {signs[i]:+d}")

    # ΓöÇΓöÇ Offset determination ΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇ
    print("\n--- Offset determination ---")
    print("At HOME, the FK angles should be: q1=0, q2=+90, q3=-90, q4=-90")
    desired_home = [0.0, 90.0, -90.0, -90.0]
    offsets = [0.0, 0.0, 0.0, 0.0]

    for i in range(4):
        m_home = home[NAMES[i]]
        offsets[i] = desired_home[i] - signs[i] * m_home
        print(f"  {SHORT[i]:6s}: motor_home={m_home:+7.1f}  sign={signs[i]:+d}  "
              f"ΓåÆ offset = {desired_home[i]} - ({signs[i]:+d})*({m_home:.1f}) = {offsets[i]:+.1f}")

    # ΓöÇΓöÇ Output ΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇ
    banner("RESULT ΓÇö paste this into axe4_FK/live_eef.py")
    print()
    print("MOTOR_CFG = [")
    print(f'    ("shoulder_pan",       {signs[0]:+d},  {offsets[0]:+7.1f}),')
    print(f'    ("shoulder_lift",      {signs[1]:+d},  {offsets[1]:+7.1f}),')
    print(f'    ("elbow_flex",         {signs[2]:+d},  {offsets[2]:+7.1f}),')
    print(f'    ("elbow_super_flex",   {signs[3]:+d},  {offsets[3]:+7.1f}),')
    print("]")

    print("\n--- Joint limits (degrees from HOME) ---")
    for i, motor in enumerate(NAMES):
        r = ranges[motor]
        print(f"  {SHORT[i]:6s}: [{r['min_delta']:+.1f}, {r['max_delta']:+.1f}]  "
              f"(range {r['range']:.1f} deg)")

    # Save result
    result = {
        "signs": signs,
        "offsets": offsets,
        "ranges": {NAMES[i]: ranges[NAMES[i]] for i in range(4)},
        "motor_cfg": [
            {"name": NAMES[i], "sign": signs[i], "offset": round(offsets[i], 1)}
            for i in range(4)
        ],
    }
    out_path = Path("axe4_FK/calibration_result.json")
    with open(out_path, "w") as f:
        json.dump(result, f, indent=2)
    print(f"\nResult saved ΓåÆ {out_path}")
    print("\nNow update MOTOR_CFG in axe4_FK/live_eef.py and re-run live mode.")


# ΓöÇΓöÇ Entry point ΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇ

def main():
    ap = argparse.ArgumentParser(description="AXE4 FK calibration wizard")
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--id", default="axe", help="calibration file ID")
    ap.add_argument("--analyze-only", type=str, default=None,
                    help="skip collection, analyze existing JSON file")
    args = ap.parse_args()

    if args.analyze_only:
        with open(args.analyze_only) as f:
            data = json.load(f)
        analyze(data)
        return

    bus = connect(args.port, args.id)
    try:
        run(bus)
    finally:
        bus.disconnect()


if __name__ == "__main__":
    main()
