#!/usr/bin/env python3
"""
SO101 calibration (save locally under so101_FK/calibration/).

This runs the same calibration procedure as `lerobot-calibrate` for SO101, but saves
the resulting JSON in this repo (instead of the HF cache) so `so101_FK/live_eef.py`
can load it deterministically.

Example:
  conda activate lerobot

  # Follower
  python so101_FK/calibrate.py --device so101_follower --port /dev/ttyACM0 --id my_follower

  # Leader
  python so101_FK/calibrate.py --device so101_leader --port /dev/ttyACM1 --id my_leader
"""

import argparse
from pathlib import Path

import draccus

from lerobot.motors.motors_bus import MotorCalibration


def _device_kind(device: str) -> tuple[str, str]:
    if device == "so101_follower":
        return "robots", "so101_follower"
    if device == "so101_leader":
        return "teleoperators", "so101_leader"
    raise ValueError(f"Unknown device: {device}")


def _local_calib_path(device: str, arm_id: str) -> Path:
    kind, name = _device_kind(device)
    root = Path(__file__).resolve().parent / "calibration" / kind / name
    root.mkdir(parents=True, exist_ok=True)
    return root / f"{arm_id}.json"


def _connect_so101_bus(port: str):
    from lerobot.motors import Motor, MotorNormMode
    from lerobot.motors.feetech import FeetechMotorsBus

    motors = {
        "shoulder_pan": Motor(1, "sts3215", MotorNormMode.DEGREES),
        "shoulder_lift": Motor(2, "sts3215", MotorNormMode.DEGREES),
        "elbow_flex": Motor(3, "sts3215", MotorNormMode.DEGREES),
        "wrist_flex": Motor(4, "sts3215", MotorNormMode.DEGREES),
        "wrist_roll": Motor(5, "sts3215", MotorNormMode.DEGREES),
        "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
    }
    bus = FeetechMotorsBus(port=port, motors=motors, calibration=None)
    bus.connect()
    return bus


def run_calibration(port: str) -> dict[str, MotorCalibration]:
    """
    Minimal, deterministic calibration that matches SO101 follower/leader implementation:
      - torque off
      - set position mode
      - set_half_turn_homings()
      - record_ranges_of_motion()
      - write_calibration()
    """
    from lerobot.motors.feetech import OperatingMode

    bus = _connect_so101_bus(port)
    try:
        bus.disable_torque()
        for motor in bus.motors:
            bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)

        input("Move the arm to the middle of its range of motion and press ENTER....")
        homing_offsets = bus.set_half_turn_homings()

        print(
            "Move all joints sequentially through their entire ranges of motion.\n"
            "Recording positions. Press ENTER to stop..."
        )
        range_mins, range_maxes = bus.record_ranges_of_motion()

        calib: dict[str, MotorCalibration] = {}
        for motor, m in bus.motors.items():
            calib[motor] = MotorCalibration(
                id=m.id,
                drive_mode=0,
                homing_offset=homing_offsets[motor],
                range_min=range_mins[motor],
                range_max=range_maxes[motor],
            )

        bus.write_calibration(calib)
        return calib
    finally:
        bus.disconnect()


def save_calibration(calib: dict[str, MotorCalibration], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f, draccus.config_type("json"):
        draccus.dump(calib, f, indent=4)


def main():
    ap = argparse.ArgumentParser(description="Calibrate SO101 and save under so101_FK/calibration/")
    ap.add_argument("--device", choices=["so101_follower", "so101_leader"], required=True)
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--id", required=True)
    args = ap.parse_args()

    out = _local_calib_path(args.device, args.id)
    print(f"Saving calibration to: {out}")

    calib = run_calibration(args.port)
    save_calibration(calib, out)
    print("Calibration saved.")


if __name__ == "__main__":
    main()

