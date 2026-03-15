import argparse
import json
import time
from pathlib import Path

import numpy as np

from lerobot.teleoperators.axe4_leader.axe4_leader import axe4Leader
from lerobot.teleoperators.axe4_leader.config_axe4_leader import axe4LeaderConfig

CALIB_PATH = Path("tester_eef/axe4_axis_calibration.json")
SAMPLE_SECONDS = 1.2
SAMPLE_HZ = 60
TICKS_PER_REV = 4096.0


def sample_joint_mean(leader: axe4Leader, seconds: float) -> dict[str, float]:
    samples = []
    dt = 1.0 / SAMPLE_HZ
    n = max(1, int(seconds * SAMPLE_HZ))
    for _ in range(n):
        # Use raw register values so this script works without LeRobot motor calibration.
        samples.append(leader.bus.sync_read("Present_Position", normalize=False))
        time.sleep(dt)
    out = {}
    for k in samples[0]:
        out[k] = float(np.mean([s[k] for s in samples]))
    return out


def fk_xyz(leader: axe4Leader, joints_deg: dict[str, float]) -> np.ndarray:
    x, y, z = leader.compute_forward_kinematics(joints_deg)
    return np.array([x, y, z], dtype=np.float32)


def raw_to_relative_deg(raw_joints: dict[str, float], raw_home: dict[str, float]) -> dict[str, float]:
    # Approximate conversion for direction inference only.
    return {k: float((raw_joints[k] - raw_home[k]) * 360.0 / TICKS_PER_REV) for k in raw_joints}


def infer_row(delta_fk: np.ndarray):
    idx = int(np.argmax(np.abs(delta_fk)))
    sign = 1.0 if delta_fk[idx] >= 0 else -1.0
    row = np.zeros(3, dtype=np.float32)
    row[idx] = sign
    return row, idx, sign


def main():
    parser = argparse.ArgumentParser(description="Calibrate AXE4 output axis mapping from motor readings.")
    parser.add_argument("--port", default="/dev/ttyACM0", help="Leader serial port, e.g. /dev/ttyACM0")
    parser.add_argument("--teleop-id", default="axe4", help="Teleoperator id (for existing motor calibration)")
    args = parser.parse_args()

    cfg = axe4LeaderConfig(port=args.port, id=args.teleop_id)
    leader = axe4Leader(cfg)
    leader.connect(calibrate=False)

    print("AXE4 motor-based axis calibration")
    print("Reads Present_Position from motors (not UDP).")
    print("")

    targets = [
        ("FORWARD", "Move teleop FORWARD and hold"),
        ("LEFT", "Move teleop LEFT and hold"),
        ("UP", "Move teleop UP and hold"),
    ]

    rows = []
    used_axes = set()
    report = {}

    try:
        for target_name, prompt in targets:
            input(f"[{target_name}] Return near HOME, then press ENTER to capture HOME sample...")
            home_raw = sample_joint_mean(leader, SAMPLE_SECONDS)
            home_fk = fk_xyz(leader, raw_to_relative_deg(home_raw, home_raw))

            input(f"[{target_name}] {prompt}. Press ENTER to capture MOVED sample...")
            moved_raw = sample_joint_mean(leader, SAMPLE_SECONDS)
            moved_fk = fk_xyz(leader, raw_to_relative_deg(moved_raw, home_raw))

            delta_fk = moved_fk - home_fk
            row, idx, sign = infer_row(delta_fk)
            rows.append(row)
            used_axes.add(idx)

            joint_delta = {k: float(moved_raw[k] - home_raw[k]) for k in home_raw}
            report[target_name] = {
                "fk_delta_xyz": delta_fk.tolist(),
                "joint_delta_raw": joint_delta,
            }

            axis_name = ["X", "Y", "Z"][idx]
            sign_name = "+" if sign > 0 else "-"
            print(f"  fk delta: {delta_fk}, dominant: {sign_name}{axis_name}")
            print(f"  joints delta: {joint_delta}")
    finally:
        leader.disconnect()

    if len(used_axes) != 3:
        print("")
        print("Calibration failed: two target motions mapped to same FK axis.")
        print("Try larger moves and hold arm steady during each sample.")
        return

    calib = {
        "output_axis_map": np.stack(rows, axis=0).tolist(),
        "direction_report": report,
    }

    CALIB_PATH.parent.mkdir(parents=True, exist_ok=True)
    with CALIB_PATH.open("w", encoding="utf-8") as f:
        json.dump(calib, f, indent=2)

    print("")
    print(f"Saved calibration to: {CALIB_PATH}")
    print("Restart teleop process to load it in axe4_leader.")


if __name__ == "__main__":
    main()
