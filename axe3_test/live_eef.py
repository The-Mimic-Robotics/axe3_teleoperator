#!/usr/bin/env python3
"""Live EEF visualizer for AXE modular leaders (3- or 4-joint) - STANDALONE TEST VERSION."""

import argparse
import json
import time
from pathlib import Path

import numpy as np

# ============ INLINE FK (from working b57f863 commit) ============

# Raw AXE3-style FK frame -> semantic frame (X=fwd, Y=left, Z=up)
_FRAME_CORRECTION = np.array(
    [
        [0.0, 0.0, -1.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    ],
    dtype=np.float64,
)


def _rot_y(angle_rad: float) -> np.ndarray:
    c = np.cos(angle_rad)
    s = np.sin(angle_rad)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=np.float64)


def _rot_z(angle_rad: float) -> np.ndarray:
    c = np.cos(angle_rad)
    s = np.sin(angle_rad)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)


def forward_kinematics(q, link_lengths_m):
    if q.size == 0:
        zero = np.zeros(3, dtype=np.float64)
        return zero, np.array([zero])

    q1 = float(q[0]) if q.size >= 1 else 0.0
    q2 = float(q[1]) if q.size >= 2 else 0.0
    q3 = float(q[2]) if q.size >= 3 else 0.0

    l1 = float(link_lengths_m[0]) if len(link_lengths_m) >= 1 else 0.0
    l2 = float(link_lengths_m[1]) if len(link_lengths_m) >= 2 else 0.0
    l3 = float(link_lengths_m[2]) if len(link_lengths_m) >= 3 else 0.0

    v1 = np.array([l1, 0.0, 0.0], dtype=np.float64)
    v2 = np.array([l2, 0.0, 0.0], dtype=np.float64)
    v3 = np.array([l3, 0.0, 0.0], dtype=np.float64)

    p0 = np.zeros(3, dtype=np.float64)
    p1_local = v1
    p2_local = p1_local + _rot_y(q2) @ v2
    p3_local = p2_local + _rot_y(q2 + q3) @ v3

    rz = _rot_z(q1)
    p1 = rz @ p1_local
    p2 = rz @ p2_local
    p3 = rz @ p3_local

    chain = np.vstack([p0, p1, p2, p3])
    chain = (_FRAME_CORRECTION @ chain.T).T
    return chain[-1], chain


def motor_deg_to_angles(deg_dict, motor_cfg):
    q = np.zeros(len(motor_cfg), dtype=np.float64)
    for i, (name, sign, off) in enumerate(motor_cfg):
        q[i] = np.radians(sign * deg_dict[name] + off)
    return q


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


# ============ END INLINE FK ============


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


def connect(port: str, arm_id: str, joint_names, motor_ids, calib_root: Path):
    from lerobot.motors import Motor, MotorCalibration, MotorNormMode
    from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode

    motors = {
        name: Motor(mid, "sts3215", MotorNormMode.DEGREES)
        for name, mid in zip(joint_names, motor_ids, strict=False)
    }

    calib_root.mkdir(parents=True, exist_ok=True)
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
        f"EEF [{eef[0]:.3f}, {eef[1]:.3f}, {eef[2]:.3f}]\n"
        f"Cmd [{cmd[0]:+.4f}, {cmd[1]:+.4f}, {cmd[2]:+.4f}]",
        fontfamily="monospace",
        fontsize=11,
    )
    ax.legend(loc="upper left")
    plt.tight_layout()
    plt.pause(0.03)


def run_live(bus, motor_cfg, link_lengths_m, viewer: bool = True):
    print("\n" + "=" * 65)
    print("LIVE EEF POSE   Frame: X fwd | Y left | Z up")
    print("=" * 65)
    print(f"Motor config: {motor_cfg}")
    print(f"Link lengths: {link_lengths_m}")
    time.sleep(0.3)

    def _read_present_position() -> dict[str, float]:
        try:
            return bus.sync_read("Present_Position")
        except RuntimeError as e:
            if "has no calibration registered" not in str(e):
                raise
            return bus.sync_read("Present_Position", normalize=False)

    home_deg = _read_present_position()
    home_q = motor_deg_to_angles(home_deg, motor_cfg)
    home_eef, _ = forward_kinematics(home_q, link_lengths_m)

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
            print(f"[INFO] Viewer disabled: {e}")
            ax = None

    try:
        while True:
            deg = _read_present_position()
            q = motor_deg_to_angles(deg, motor_cfg)
            eef, chain = forward_kinematics(q, link_lengths_m)
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
    ap = argparse.ArgumentParser(description="AXE3 standalone FK + EEF tester")
    ap.add_argument("--port", default="COM8")
    ap.add_argument("--id", default="axe", help="motor calibration file ID (e.g. axe)")
    ap.add_argument("--axis-calib", type=str, default="", help="path to axis calibration JSON (signs/offsets)")
    ap.add_argument("--motor-calib-dir", type=str, default="", help="directory for motor EEPROM calibration")
    ap.add_argument("--links-m", type=float, nargs="+", default=[0.060, 0.210, 0.250])
    ap.add_argument("--no-viewer", action="store_true")
    args = ap.parse_args()

    joint_names = ("shoulder_pan", "shoulder_lift", "elbow_flex")
    motor_ids = (1, 2, 3)
    link_lengths_m = tuple(args.links_m[:3])

    # Load axis calibration (signs/offsets)
    motor_cfg = None
    axis_calib_path = args.axis_calib
    if not axis_calib_path:
        # Default to calibration_result.json in same folder
        axis_calib_path = str(Path(__file__).resolve().parent / "calibration_result.json")
    
    motor_cfg = load_motor_cfg_from_file(axis_calib_path)
    if motor_cfg:
        print(f"Loaded axis calibration from: {axis_calib_path}")
    else:
        # Default identity mapping
        motor_cfg = [(name, 1, 0.0) for name in joint_names]
        print("Using identity motor_cfg (sign=+1, offset=0)")

    # Motor calibration directory
    if args.motor_calib_dir:
        calib_root = Path(args.motor_calib_dir)
    else:
        calib_root = Path(__file__).resolve().parent  # axe3_test folder

    bus = connect(args.port, args.id, joint_names, motor_ids, calib_root)
    try:
        run_live(bus, motor_cfg, link_lengths_m, viewer=not args.no_viewer)
    finally:
        bus.disconnect()


if __name__ == "__main__":
    main()
