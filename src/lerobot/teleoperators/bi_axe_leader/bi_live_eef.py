#!/usr/bin/env python3
"""Bimanual Live EEF visualizer for BiAxeLeader (left + right arms)."""

import argparse
import json
import time
from pathlib import Path

import math

import numpy as np

from lerobot.teleoperators.axe_leader.fk import forward_kinematics, motor_deg_to_angles


def load_motor_cfg_from_file(path):
    """Load motor_cfg from calibration JSON file."""
    if not path or not Path(path).exists():
        return None
    with open(path) as f:
        data = json.load(f)
    motor_cfg = data.get("motor_cfg")
    if not motor_cfg:
        return None
    return [(m["name"], int(m["sign"]), float(m["offset"])) for m in motor_cfg]


def direction_label(x: float, y: float, z: float, threshold: float = 0.001) -> str:
    vec = [x, y, z]
    idx = max(range(3), key=lambda i: abs(vec[i]))
    if abs(vec[idx]) < threshold:
        return "STOP"
    if idx == 0:
        return "FWD" if vec[idx] > 0 else "BWD"
    if idx == 1:
        return "LEFT" if vec[idx] > 0 else "RIGHT"
    return "UP" if vec[idx] > 0 else "DOWN"


def connect_arm(port: str, arm_id: str, joint_names, motor_ids, calib_root: Path):
    from lerobot.motors import Motor, MotorCalibration, MotorNormMode
    from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode

    motors = {
        name: Motor(mid, "sts3215", MotorNormMode.DEGREES)
        for name, mid in zip(joint_names, motor_ids, strict=False)
    }

    calib_path = calib_root / f"{arm_id}.json"
    calib = None
    if calib_path.exists():
        print(f"Loading motor calibration from: {calib_path}")
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
    else:
        print(f"No motor calibration found at: {calib_path}")

    bus = FeetechMotorsBus(port=port, motors=motors, calibration=calib)
    bus.connect()
    if calib:
        bus.write_calibration(calib)
    bus.disable_torque()
    for m in motors:
        bus.write("Operating_Mode", m, OperatingMode.POSITION.value)
    return bus


def _draw_bimanual(ax, left_chain, left_eef, right_chain, right_eef, left_cmd, right_cmd, arm_separation):
    import matplotlib.pyplot as plt

    ax.cla()

    # Offset arms: left at Y=+separation/2, right at Y=-separation/2
    left_offset = np.array([0.0, arm_separation / 2, 0.0])
    right_offset = np.array([0.0, -arm_separation / 2, 0.0])

    left_chain_offset = left_chain + left_offset
    right_chain_offset = right_chain + right_offset
    left_eef_offset = left_eef + left_offset
    right_eef_offset = right_eef + right_offset

    # Draw left arm (blue)
    ax.plot(left_chain_offset[:, 0], left_chain_offset[:, 1], left_chain_offset[:, 2],
            "-o", lw=3, ms=7, color="steelblue", label="Left arm")
    ax.scatter(*left_eef_offset, s=140, c="blue", marker="^", zorder=5)

    # Draw right arm (green)
    ax.plot(right_chain_offset[:, 0], right_chain_offset[:, 1], right_chain_offset[:, 2],
            "-o", lw=3, ms=7, color="forestgreen", label="Right arm")
    ax.scatter(*right_eef_offset, s=140, c="green", marker="^", zorder=5)

    lim = 0.6
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_zlim(-0.2, 0.6)
    ax.set_xlabel("X (fwd)")
    ax.set_ylabel("Y (left)")
    ax.set_zlabel("Z (up)")

    left_label = direction_label(left_cmd[0], left_cmd[1], left_cmd[2])
    right_label = direction_label(right_cmd[0], right_cmd[1], right_cmd[2])

    ax.set_title(
        f"L: [{left_cmd[0]:+.3f}, {left_cmd[1]:+.3f}, {left_cmd[2]:+.3f}] ({left_label})\n"
        f"R: [{right_cmd[0]:+.3f}, {right_cmd[1]:+.3f}, {right_cmd[2]:+.3f}] ({right_label})",
        fontfamily="monospace",
        fontsize=10,
    )
    ax.legend(loc="upper left")
    plt.tight_layout()
    plt.pause(0.03)


def run_bimanual_live(
    left_bus,
    right_bus,
    left_motor_cfg,
    right_motor_cfg,
    link_lengths_m,
    arm_separation,
    viewer=True,
    *,
    right_planar_elbow_offset_rad: float = -math.pi / 2.0,
):
    print("\n" + "=" * 70)
    print("BIMANUAL LIVE EEF   Frame: X fwd | Y left | Z up")
    print(f"Arm separation: {arm_separation}m")
    print("=" * 70)
    print(f"Left motor_cfg: {left_motor_cfg}")
    print(f"Right motor_cfg: {right_motor_cfg}")
    time.sleep(0.3)

    def _read_pos(bus):
        try:
            return bus.sync_read("Present_Position")
        except RuntimeError as e:
            if "has no calibration registered" not in str(e):
                raise
            return bus.sync_read("Present_Position", normalize=False)

    # Capture home positions
    left_home_deg = _read_pos(left_bus)
    left_home_q = motor_deg_to_angles(left_home_deg, left_motor_cfg)
    left_home_eef, _ = forward_kinematics(left_home_q, link_lengths_m)

    right_home_deg = _read_pos(right_bus)
    right_home_q = motor_deg_to_angles(right_home_deg, right_motor_cfg)
    right_home_eef, _ = forward_kinematics(
        right_home_q,
        link_lengths_m,
        planar_mirror=True,
        planar_mirror_elbow_offset_rad=right_planar_elbow_offset_rad,
    )

    ax = None
    if viewer:
        try:
            import matplotlib.pyplot as plt
            from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

            plt.ion()
            fig = plt.figure(figsize=(10, 8))
            ax = fig.add_subplot(111, projection="3d")
            ax.view_init(elev=20, azim=-150)
        except Exception as e:
            print(f"[INFO] Viewer disabled: {e}")
            ax = None

    try:
        while True:
            # Left arm
            left_deg = _read_pos(left_bus)
            left_q = motor_deg_to_angles(left_deg, left_motor_cfg)
            left_eef, left_chain = forward_kinematics(left_q, link_lengths_m)
            left_cmd = left_eef - left_home_eef

            # Right arm
            right_deg = _read_pos(right_bus)
            right_q = motor_deg_to_angles(right_deg, right_motor_cfg)
            right_eef, right_chain = forward_kinematics(
                right_q,
                link_lengths_m,
                planar_mirror=True,
                planar_mirror_elbow_offset_rad=right_planar_elbow_offset_rad,
            )
            right_cmd = right_eef - right_home_eef

            left_label = direction_label(left_cmd[0], left_cmd[1], left_cmd[2])
            right_label = direction_label(right_cmd[0], right_cmd[1], right_cmd[2])

            print(
                f"L:[{left_cmd[0]:+.3f},{left_cmd[1]:+.3f},{left_cmd[2]:+.3f}]({left_label:5s}) "
                f"R:[{right_cmd[0]:+.3f},{right_cmd[1]:+.3f},{right_cmd[2]:+.3f}]({right_label:5s})",
                end="\r",
            )

            if ax is not None:
                _draw_bimanual(ax, left_chain, left_eef, right_chain, right_eef, left_cmd, right_cmd, arm_separation)

    except KeyboardInterrupt:
        print("\n\nDone.")
    finally:
        if ax is not None:
            import matplotlib.pyplot as plt
            plt.close("all")


def main():
    ap = argparse.ArgumentParser(description="Bimanual AXE3 Live EEF Viewer")
    ap.add_argument("--left-port", default="COM8", help="Left arm serial port")
    ap.add_argument("--right-port", default="COM9", help="Right arm serial port")
    ap.add_argument("--left-id", default="axe_left", help="Left arm motor calibration ID")
    ap.add_argument("--right-id", default="axe_right", help="Right arm motor calibration ID")
    ap.add_argument("--left-axis-calib", type=str, default="", help="Left arm axis calibration JSON")
    ap.add_argument("--right-axis-calib", type=str, default="", help="Right arm axis calibration JSON")
    ap.add_argument("--calib-dir", type=str, default="", help="Calibration directory")
    ap.add_argument("--links-m", type=float, nargs="+", default=[0.060, 0.210, 0.250])
    ap.add_argument("--arm-separation", type=float, default=0.5, help="Distance between arm bases (meters)")
    ap.add_argument(
        "--right-elbow-offset-deg",
        type=float,
        default=-90.0,
        help="Extra elbow (q3) offset in FK for the right arm after planar mirror (default -90; try +90 if wrong)",
    )
    ap.add_argument(
        "--debug-fk",
        action="store_true",
        help="After connect, print one FK diagnostic block (same as debug_fk_snapshot module).",
    )
    ap.add_argument("--no-viewer", action="store_true")
    args = ap.parse_args()

    joint_names = ("shoulder_pan", "shoulder_lift", "elbow_flex")
    motor_ids = (1, 2, 3)
    link_lengths_m = tuple(args.links_m[:3])

    # Calibration directory
    if args.calib_dir:
        calib_root = Path(args.calib_dir)
    else:
        calib_root = Path(__file__).resolve().parents[4] / "calibration" / "teleoperators" / "bi_axe_leader"

    print(f"Calibration root: {calib_root}")
    print(f"Calibration root exists: {calib_root.exists()}")

    # Load axis calibrations
    left_axis_path = args.left_axis_calib or str(calib_root / "axis_left.json")
    right_axis_path = args.right_axis_calib or str(calib_root / "axis_right.json")

    print(f"Left axis path: {left_axis_path}")
    print(f"Left axis exists: {Path(left_axis_path).exists()}")
    print(f"Right axis path: {right_axis_path}")
    print(f"Right axis exists: {Path(right_axis_path).exists()}")

    left_motor_cfg = load_motor_cfg_from_file(left_axis_path)
    if left_motor_cfg:
        print(f"Loaded left axis calibration: {left_axis_path}")
        print(f"Left motor_cfg: {left_motor_cfg}")
    else:
        left_motor_cfg = [(name, 1, 0.0) for name in joint_names]
        print("Using identity left motor_cfg")

    right_motor_cfg = load_motor_cfg_from_file(right_axis_path)
    if right_motor_cfg:
        print(f"Loaded right axis calibration: {right_axis_path}")
        print(f"Right motor_cfg: {right_motor_cfg}")
    else:
        right_motor_cfg = [(name, 1, 0.0) for name in joint_names]
        print("Using identity right motor_cfg")

    # Connect both arms
    print(f"\nConnecting left arm on {args.left_port}...")
    left_bus = connect_arm(args.left_port, args.left_id, joint_names, motor_ids, calib_root)

    print(f"Connecting right arm on {args.right_port}...")
    right_bus = connect_arm(args.right_port, args.right_id, joint_names, motor_ids, calib_root)

    if args.debug_fk:
        from lerobot.teleoperators.bi_axe_leader.debug_fk_snapshot import run_fk_snapshot_print

        def _snap_read(bus):
            try:
                return bus.sync_read("Present_Position")
            except RuntimeError as e:
                if "has no calibration registered" not in str(e):
                    raise
                return bus.sync_read("Present_Position", normalize=False)

        run_fk_snapshot_print(
            _snap_read(left_bus),
            _snap_read(right_bus),
            left_motor_cfg,
            right_motor_cfg,
            link_lengths_m,
            right_elbow_off_rad=math.radians(args.right_elbow_offset_deg),
            left_axis_path=left_axis_path,
            right_axis_path=right_axis_path,
        )

    try:
        run_bimanual_live(
            left_bus,
            right_bus,
            left_motor_cfg,
            right_motor_cfg,
            link_lengths_m,
            args.arm_separation,
            viewer=not args.no_viewer,
            right_planar_elbow_offset_rad=math.radians(args.right_elbow_offset_deg),
        )
    finally:
        left_bus.disconnect()
        right_bus.disconnect()


if __name__ == "__main__":
    main()
