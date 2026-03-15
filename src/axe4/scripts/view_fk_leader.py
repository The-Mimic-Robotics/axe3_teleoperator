"""
Live view of what the leader's FK produces (src/lerobot/teleoperators/axe4_leader/fk.py).

Same idea as axe4_FK/live_eef.py but uses the leader's load_motor_cfg, motor_deg_to_angles,
and forward_kinematics so you can verify directions match.

  python src/axe4/scripts/view_fk_leader.py --port /dev/ttyACM0 [--id axe] [--no-viewer]
"""

import argparse
import time
from pathlib import Path

import numpy as np

from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode

from lerobot.teleoperators.axe4_leader.fk import (
    load_motor_cfg,
    motor_deg_to_angles,
    forward_kinematics,
)

CALIB_ROOT = (
    Path.home()
    / ".cache/huggingface/lerobot/calibration/teleoperators/axe4_leader"
)

DIR_LABELS = {
    0: ("FORWARD", "BACKWARD"),
    1: ("LEFT", "RIGHT"),
    2: ("UP", "DOWN"),
}


def _load_calib(arm_id):
    path = CALIB_ROOT / f"{arm_id}.json"
    if not path.exists():
        return None
    import json
    with open(path) as f:
        raw = json.load(f)
    return {
        name: MotorCalibration(
            id=info["id"],
            drive_mode=info["drive_mode"],
            homing_offset=info["homing_offset"],
            range_min=info["range_min"],
            range_max=info["range_max"],
        )
        for name, info in raw.items()
    }


def connect(port: str, arm_id: str):
    motors = {
        "shoulder_pan": Motor(1, "sts3215", MotorNormMode.DEGREES),
        "shoulder_lift": Motor(2, "sts3215", MotorNormMode.DEGREES),
        "elbow_flex": Motor(3, "sts3215", MotorNormMode.DEGREES),
        "elbow_super_flex": Motor(4, "sts3215", MotorNormMode.DEGREES),
    }
    calib = _load_calib(arm_id)
    bus = FeetechMotorsBus(port=port, motors=motors, calibration=calib)
    bus.connect()
    if calib:
        bus.write_calibration(calib)
    bus.disable_torque()
    for m in motors:
        bus.write("Operating_Mode", m, OperatingMode.POSITION.value)
    return bus


def _fmt(d):
    return "  ".join(f"{k[:6]:>6s}={v:+.1f}" for k, v in d.items())


def _draw(ax, chain, eef, cmd, label):
    import matplotlib.pyplot as plt
    ax.cla()
    ax.plot(chain[:, 0], chain[:, 1], chain[:, 2],
            "-o", lw=3, ms=7, color="steelblue", label="arm")
    ax.scatter(*eef, s=140, c="red", marker="^", zorder=5, label="EEF")
    lim = 0.4
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_zlim(-0.15, 0.45)
    ax.set_xlabel("X (fwd)")
    ax.set_ylabel("Y (left)")
    ax.set_zlabel("Z (up)")
    ax.set_title(
        f"Leader FK  EEF [{eef[0]:.3f}, {eef[1]:.3f}, {eef[2]:.3f}]\n"
        f"Cmd [{cmd[0]:+.4f}, {cmd[1]:+.4f}, {cmd[2]:+.4f}]  ({label})",
        fontfamily="monospace", fontsize=11,
    )
    ax.legend(loc="upper left")
    plt.tight_layout()
    plt.pause(0.03)


def main():
    ap = argparse.ArgumentParser(description="View leader FK output (same as live_eef style)")
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--id", default="axe")
    ap.add_argument("--no-viewer", action="store_true", help="Terminal only, no matplotlib")
    args = ap.parse_args()

    motor_cfg = load_motor_cfg()
    bus = connect(args.port, args.id)

    print()
    print("=" * 65)
    print("LEADER FK LIVE   Frame: X fwd | Y left | Z up")
    print("(uses src/lerobot/teleoperators/axe4_leader/fk.py)")
    print("=" * 65)

    time.sleep(0.4)
    home_deg = bus.sync_read("Present_Position")
    home_q = motor_deg_to_angles(home_deg, motor_cfg)
    home_eef, home_chain = forward_kinematics(*home_q)

    print(f"Home deg : {_fmt(home_deg)}")
    print(f"Home q   : [{np.degrees(home_q[0]):+.1f}, {np.degrees(home_q[1]):+.1f}, "
          f"{np.degrees(home_q[2]):+.1f}, {np.degrees(home_q[3]):+.1f}] deg")
    print(f"Home EEF : [{home_eef[0]:+.4f}, {home_eef[1]:+.4f}, {home_eef[2]:+.4f}] m")
    print()

    ax = None
    if not args.no_viewer:
        try:
            import matplotlib.pyplot as plt
            from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
            plt.ion()
            fig = plt.figure(figsize=(9, 8))
            ax = fig.add_subplot(111, projection="3d")
            ax.view_init(elev=20, azim=-150)
        except Exception as e:
            print(f"[WARN] Viewer failed: {e}")

    try:
        while True:
            deg = bus.sync_read("Present_Position")
            q = motor_deg_to_angles(deg, motor_cfg)
            eef, chain = forward_kinematics(*q)
            cmd = eef - home_eef

            idx = int(np.argmax(np.abs(cmd)))
            if abs(cmd[idx]) > 0.008:
                label = DIR_LABELS[idx][0 if cmd[idx] > 0 else 1]
            else:
                label = "---"

            print(
                f"Cmd: [{cmd[0]: 8.4f}, {cmd[1]: 8.4f}, {cmd[2]: 8.4f}]  ({label:8s})",
                end="\r",
            )

            if ax is not None:
                _draw(ax, chain, eef, cmd, label)

            time.sleep(0.04)
    except KeyboardInterrupt:
        print("\n\nDone.")
    finally:
        bus.disconnect()
        if ax is not None:
            import matplotlib.pyplot as plt
            plt.close("all")


if __name__ == "__main__":
    main()
