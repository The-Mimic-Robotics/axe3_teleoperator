#!/usr/bin/env python3
"""Live EEF visualizer for AXE modular leaders (3- or 4-joint)."""

import argparse
import json
import time
from pathlib import Path

import numpy as np

from lerobot.teleoperators.axe_leader.config_axe_leader import axeLeaderConfig
from lerobot.teleoperators.axe_leader.fk import forward_kinematics, load_motor_cfg, motor_deg_to_angles


def direction_label(x: float, y: float, z: float, threshold: float = 0.001) -> str:
    vec = [x, y, z]
    idx = max(range(3), key=lambda i: abs(vec[i]))
    if abs(vec[idx]) < threshold:
        return "STOP"
    if idx == 0:
        return "FORWARD" if vec[idx] > 0 else "BACKWARD"
    if idx == 1:
        return "LEFT" if vec[idx] > 0 else "RIGHT"
    return "UP" if vec[idx] > 0 else "DOWN"


def _repo_root() -> Path:
    # .../src/lerobot/teleoperators/axe_leader/calibrate/live_eef.py -> repo
    return Path(__file__).resolve().parents[5]


def _repo_calibration_root() -> Path:
    return _repo_root() / "calibration" / "teleoperators" / "axe_leader"


def connect(port: str, arm_id: str, joint_names: tuple[str, ...], motor_ids: tuple[int, ...]):
    from lerobot.motors import Motor, MotorCalibration, MotorNormMode
    from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode

    motors = {
        name: Motor(mid, "sts3215", MotorNormMode.DEGREES)
        for name, mid in zip(joint_names, motor_ids, strict=False)
    }

    calib_root = _repo_calibration_root()
    calib_root.mkdir(parents=True, exist_ok=True)
    calib_path = calib_root / f"{arm_id}.json"
    calib = None
    if calib_path.exists():
        with open(calib_path) as f:
            raw = json.load(f)
        calib = {}
        for name, info in raw.items():
            if name not in motors:
                continue
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
    bus.disable_torque()
    for m in motors:
        bus.write("Operating_Mode", m, OperatingMode.POSITION.value)
    return bus


def _draw(ax, chain, eef, cmd):
    import matplotlib.pyplot as plt

    ax.cla()
    ax.plot(chain[:, 0], chain[:, 1], chain[:, 2], "-o", lw=3, ms=7, color="steelblue", label="arm")
    ax.scatter(*eef, s=140, c="red", marker="^", zorder=5, label="EEF")

    lim = 0.5
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_zlim(-0.2, 0.6)
    ax.set_xlabel("X (fwd)")
    ax.set_ylabel("Y (left)")
    ax.set_zlabel("Z (up)")
    ax.set_title(
        f"EEF [{eef[0]:.3f}, {eef[1]:.3f}, {eef[2]:.3f}]\\n"
        f"Cmd [{cmd[0]:+.4f}, {cmd[1]:+.4f}, {cmd[2]:+.4f}]",
        fontfamily="monospace",
        fontsize=11,
    )
    ax.legend(loc="upper left")
    plt.tight_layout()
    plt.pause(0.03)


def run_live(bus, cfg: axeLeaderConfig, viewer: bool = True):
    print("\n" + "=" * 65)
    print("LIVE EEF POSE   Frame: X fwd | Y left | Z up")
    print("=" * 65)
    time.sleep(0.3)

    motor_cfg = load_motor_cfg(cfg)

    def _read_present_position() -> dict[str, float]:
        try:
            return bus.sync_read("Present_Position")
        except RuntimeError as e:
            if "has no calibration registered" not in str(e):
                raise
            return bus.sync_read("Present_Position", normalize=False)

    home_deg = _read_present_position()
    home_q = motor_deg_to_angles(home_deg, motor_cfg)
    home_eef, _ = forward_kinematics(home_q, cfg.link_lengths_m)

    ax = None
    if viewer:
        try:
            import matplotlib.pyplot as plt
            from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

            plt.ion()
            fig = plt.figure(figsize=(9, 8))
            ax = fig.add_subplot(111, projection="3d")
            ax.view_init(elev=20, azim=-150)
        except Exception as e:
            print(f"[INFO] Viewer disabled (matplotlib not installed): {e}")
            print("[INFO] Running terminal-only mode. Use --no-viewer to silence this message.")
            ax = None

    try:
        while True:
            deg = _read_present_position()
            q = motor_deg_to_angles(deg, motor_cfg)
            eef, chain = forward_kinematics(q, cfg.link_lengths_m)
            cmd = eef - home_eef
            label = direction_label(float(cmd[0]), float(cmd[1]), float(cmd[2]))

            print(
                f"Cmd: [{cmd[0]: 8.4f}, {cmd[1]: 8.4f}, {cmd[2]: 8.4f}]  ({label:8s})",
                end="\r",
            )

            if ax is not None:
                _draw(ax, chain, eef, cmd)

    except KeyboardInterrupt:
        print("\n\nDone.")
    finally:
        if ax is not None:
            import matplotlib.pyplot as plt

            plt.close("all")


def main():
    ap = argparse.ArgumentParser(description="AXE modular standalone FK + EEF")
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--id", default="axe", help="calibration file ID")
    ap.add_argument("--num-joints", type=int, default=3, choices=[3, 4])
    ap.add_argument(
        "--links-m",
        type=float,
        nargs="+",
        default=[0.060, 0.210, 0.250],
        help="One link length per joint in meters (first is shoulder_pan segment)",
    )
    ap.add_argument("--axis-calibration-path", type=str, default="")
    ap.add_argument("--no-viewer", action="store_true", help="terminal only, no matplotlib")
    args = ap.parse_args()

    joints = []
    names_tail = ["shoulder_lift", "elbow_flex", "elbow_super_flex"]
    for idx in range(max(args.num_joints, 0)):
        link_len = args.links_m[idx] if idx < len(args.links_m) else 0.0
        if idx == 0:
            name = "shoulder_pan"
        else:
            name = names_tail[idx - 1]
        joints.append(
            {
                "name": name,
                "id": idx + 1,
                "model": "sts3215",
                "link_length_m": float(link_len),
            }
        )

    arm_cfg = {
        "axis_calibration_path": args.axis_calibration_path,
        "joints": joints,
    }

    cfg = axeLeaderConfig(
        arm=arm_cfg,
    )
    bus = connect(args.port, args.id, cfg.joint_names, cfg.motor_ids)
    try:
        run_live(bus, cfg, viewer=not args.no_viewer)
    finally:
        bus.disconnect()


if __name__ == "__main__":
    main()
