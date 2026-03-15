"""
AXE4 axis calibration — determines output axis mapping, joint direction signs,
max joint limits, and coordinate-frame orientation.

Procedure
---------
1. For each of FORWARD, LEFT, UP: capture home then moved FK samples.
2. Infer which FK axis each motion maps to and the sign.
3. Record joint-angle ranges (max travel seen during calibration).
4. Write calibration JSON to src/axe4/config/axe4_axis_calibration.json
"""

import argparse
import json
import time
from pathlib import Path

import numpy as np

from lerobot.teleoperators.axe4_leader.axe4_leader import axe4Leader
from lerobot.teleoperators.axe4_leader.config_axe4_leader import axe4LeaderConfig

CALIB_PATH = Path("src/axe4/config/axe4_axis_calibration.json")
SAMPLE_SECONDS = 1.2
SAMPLE_HZ = 60
TICKS_PER_REV = 4096.0

DIRECTION_NAMES = {0: "X", 1: "Y", 2: "Z"}
SIGN_NAMES = {1.0: "+", -1.0: "-"}


def sample_joint_mean(leader: axe4Leader, seconds: float) -> dict[str, float]:
    samples = []
    dt = 1.0 / SAMPLE_HZ
    n = max(1, int(seconds * SAMPLE_HZ))
    for _ in range(n):
        samples.append(leader.bus.sync_read("Present_Position", normalize=False))
        time.sleep(dt)
    return {k: float(np.mean([s[k] for s in samples])) for k in samples[0]}


def sample_joint_range(leader: axe4Leader, seconds: float):
    """Record the min/max of each joint over *seconds*."""
    mins, maxs = {}, {}
    dt = 1.0 / SAMPLE_HZ
    n = max(1, int(seconds * SAMPLE_HZ))
    for _ in range(n):
        snap = leader.bus.sync_read("Present_Position", normalize=False)
        for k, v in snap.items():
            mins[k] = min(mins.get(k, v), v)
            maxs[k] = max(maxs.get(k, v), v)
        time.sleep(dt)
    return mins, maxs


def fk_xyz(leader: axe4Leader, joints_deg: dict[str, float]) -> np.ndarray:
    x, y, z = leader.compute_forward_kinematics(joints_deg)
    return np.array([x, y, z], dtype=np.float32)


def raw_to_abs_deg(raw: dict[str, float]) -> dict[str, float]:
    """Raw encoder ticks → absolute motor degrees (for FK)."""
    return {k: float(raw[k] * 360.0 / TICKS_PER_REV) for k in raw}


def infer_row(delta_fk: np.ndarray):
    idx = int(np.argmax(np.abs(delta_fk)))
    sign = 1.0 if delta_fk[idx] >= 0 else -1.0
    row = np.zeros(3, dtype=np.float32)
    row[idx] = sign
    return row, idx, sign


def main():
    parser = argparse.ArgumentParser(description="AXE4 axis + direction calibration")
    parser.add_argument("--port", default="/dev/ttyACM0", help="Servo driver serial port")
    parser.add_argument("--teleop-id", default="axe4", help="Teleoperator id")
    args = parser.parse_args()

    cfg = axe4LeaderConfig(port=args.port, id=args.teleop_id, transport="none", handle_source="udp")
    leader = axe4Leader(cfg)
    leader.connect(calibrate=False)

    print("=" * 60)
    print("AXE4 Axis Calibration")
    print("=" * 60)
    print("This calibrates direction mapping (Forward/Left/Up)")
    print("and records joint limits.\n")

    targets = [
        ("FORWARD", "Push the handle FORWARD (away from you) and hold"),
        ("LEFT",    "Push the handle to your LEFT and hold"),
        ("UP",      "Lift the handle UP and hold"),
    ]

    rows = []
    used_axes: set[int] = set()
    report: dict = {}
    global_min_raw: dict[str, float] = {}
    global_max_raw: dict[str, float] = {}

    try:
        # --- Direction calibration ---
        for target_name, prompt in targets:
            input(f"\n[{target_name}] Bring arm to HOME position, then press ENTER... ")
            home_raw = sample_joint_mean(leader, SAMPLE_SECONDS)
            home_fk = fk_xyz(leader, raw_to_abs_deg(home_raw))

            input(f"[{target_name}] {prompt}. Press ENTER... ")
            moved_raw = sample_joint_mean(leader, SAMPLE_SECONDS)
            moved_fk = fk_xyz(leader, raw_to_abs_deg(moved_raw))

            delta_fk = moved_fk - home_fk
            row, idx, sign = infer_row(delta_fk)
            rows.append(row)
            used_axes.add(idx)

            joint_delta = {k: float(moved_raw[k] - home_raw[k]) for k in home_raw}
            report[target_name] = {
                "fk_delta_xyz": delta_fk.tolist(),
                "joint_delta_raw": joint_delta,
            }

            axis_name = DIRECTION_NAMES[idx]
            sign_name = SIGN_NAMES[sign]
            print(f"  → mapped to {sign_name}{axis_name}   (fk_delta={delta_fk})")

        # --- Joint limits ---
        print("\n" + "-" * 60)
        print("JOINT LIMIT RECORDING")
        print("Move EVERY joint through its FULL range of motion.")
        input("Press ENTER when ready, then move joints. Press ENTER again to stop.\n")

        stop = False
        import threading

        def _wait():
            nonlocal stop
            input()
            stop = True

        t = threading.Thread(target=_wait, daemon=True)
        t.start()

        while not stop:
            snap = leader.bus.sync_read("Present_Position", normalize=False)
            for k, v in snap.items():
                global_min_raw[k] = min(global_min_raw.get(k, v), v)
                global_max_raw[k] = max(global_max_raw.get(k, v), v)
            time.sleep(1.0 / SAMPLE_HZ)

        joint_limits_raw = {
            k: {"min": global_min_raw.get(k, 0), "max": global_max_raw.get(k, 0)}
            for k in leader.bus.motors
        }
        joint_limits_deg = {
            k: {
                "min_deg": round(v["min"] * 360.0 / TICKS_PER_REV, 2),
                "max_deg": round(v["max"] * 360.0 / TICKS_PER_REV, 2),
                "range_deg": round((v["max"] - v["min"]) * 360.0 / TICKS_PER_REV, 2),
            }
            for k, v in joint_limits_raw.items()
        }

        print("\nRecorded joint ranges:")
        for name, lim in joint_limits_deg.items():
            print(f"  {name:25s}  {lim['min_deg']:8.2f}°  →  {lim['max_deg']:8.2f}°  "
                  f"(range {lim['range_deg']:.2f}°)")

    finally:
        leader.disconnect()

    if len(used_axes) != 3:
        print("\nCalibration FAILED: two target motions mapped to the same FK axis.")
        print("Try larger moves and hold steady during each sample.")
        return

    calib = {
        "output_axis_map": np.stack(rows, axis=0).tolist(),
        "direction_report": report,
        "joint_limits_raw": {k: {"min": int(v["min"]), "max": int(v["max"])} for k, v in joint_limits_raw.items()},
        "joint_limits_deg": joint_limits_deg,
    }

    CALIB_PATH.parent.mkdir(parents=True, exist_ok=True)
    with CALIB_PATH.open("w", encoding="utf-8") as f:
        json.dump(calib, f, indent=2)

    print(f"\nCalibration saved → {CALIB_PATH}")
    print("Restart the teleop process to load the new calibration.")


if __name__ == "__main__":
    main()
