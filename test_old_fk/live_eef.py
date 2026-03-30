#!/usr/bin/env python3
"""
AXE4 ΓÇö standalone FK + EEF pose.

Simple planar 3-link chain + vertical pan.  No URDF.
Just trig: each link lives in a vertical plane swept by the base pan.

    conda activate lerobot

    # Verify motor-to-joint mapping
    python axe4_FK/live_eef.py --port /dev/ttyACM0 --mode identify

    # Live EEF pose + 3D viewer
    python axe4_FK/live_eef.py --port /dev/ttyACM0

    # Terminal only (no matplotlib)
    python axe4_FK/live_eef.py --port /dev/ttyACM0 --no-viewer
"""

import argparse
import time
import json
import numpy as np
from pathlib import Path

# ΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉ
# ARM GEOMETRY (metres)
#
# Physical layout at rest (arch / portal shape):
#
#        ΓöîΓöÇΓöÇΓöÇ upper arm (L1, pointing UP) ΓöÇΓöÇΓöÇΓöÉ
#   shoulder                                 elbow
#        Γöé                                     Γöé
#      base offset                    forearm (L2, horizontal)
#        Γöé                                     Γöé
#      [BASE]                           super-flex joint
#                                              Γöé
#                                        handle (L3, pointing DOWN)
#                                              Γöé
#                                            [GRIP]
# ΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉ

L_BASE = 0.075    # horizontal offset from pan axis to shoulder pivot
L1     = 0.249    # upper arm
L2     = 0.282    # forearm
L3     = 0.120    # handle to grip point

# ΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇ
# MOTOR ΓåÆ ANGLE MAPPING
#
#   (motor_name,   sign,   offset_deg)
#
# angle = sign * motor_degrees + offset     (then convert to radians)
#
# Pan (q1): angle in the horizontal plane, 0 = forward
# Lift/elbow/super (q2, q3, q4): incremental angles in the vertical plane
#   cumulative:  a2=q2,  a3=q2+q3,  a4=q2+q3+q4
#   where 0┬░ = horizontal, +90┬░ = straight up, -90┬░ = straight down
#
# At home (motors Γëê 0┬░) the arm should be in the arch pose:
#   a2 = +90┬░  (upper arm UP)
#   a3 =   0┬░  (forearm HORIZONTAL)
#   a4 = -90┬░  (handle DOWN)
#
# So offsets must give: q2=90, q3=-90, q4=-90 when motors read 0.
# ΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇΓöÇ


MOTOR_CFG = [
    # (motor_name,          sign, offset_deg)
    # All signs -1: +motor ΓåÆ -angle.  Offsets keep home pose correct.
    ("shoulder_pan",         -1, +168.5),
    ("shoulder_lift",        -1,  +78.8),
    ("elbow_flex",           -1, +114.0),
    ("elbow_super_flex",     -1, +113.6),
]


# ΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉ
# FK MATH ΓÇö pure trig, no matrices
# ΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉ

def forward_kinematics(q1, q2, q3, q4):
    """
    Planar 3-link chain + pan.  Returns (eef_xyz[3], chain[5├ù3]).

    q1 : pan angle (rad), 0 = forward, + = left
    q2 : shoulder lift  (rad, incremental)
    q3 : elbow flex     (rad, incremental)
    q4 : super flex     (rad, incremental)

    chain rows: [base, shoulder, elbow, super_joint, eef]
    """
    a2 = q2
    a3 = q2 + q3
    a4 = q2 + q3 + q4

    c1, s1 = np.cos(q1), np.sin(q1)

    # Build the chain in (r, z) then sweep by pan
    # r = horizontal distance from pan axis,  z = height
    joints_rz = [(0.0, 0.0)]                                   # base

    r, z = L_BASE, 0.0                                         # shoulder
    joints_rz.append((r, z))

    r += L1 * np.cos(a2);  z += L1 * np.sin(a2)               # elbow
    joints_rz.append((r, z))

    r += L2 * np.cos(a3);  z += L2 * np.sin(a3)               # super joint
    joints_rz.append((r, z))

    r += L3 * np.cos(a4);  z += L3 * np.sin(a4)               # eef
    joints_rz.append((r, z))

    chain = np.array([[ri * c1, ri * s1, zi] for ri, zi in joints_rz])
    return chain[-1], chain


def motor_deg_to_angles(deg_dict):
    """Calibrated motor degrees ΓåÆ (q1, q2, q3, q4) in radians."""
    q = np.zeros(4)
    for i, (name, sign, off) in enumerate(MOTOR_CFG):
        q[i] = np.radians(sign * deg_dict[name] + off)
    return q


# ΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉ
# MOTOR BUS  (uses lerobot only for serial protocol)
# ΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉ

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

    calib = _load_calib(arm_id)
    bus = FeetechMotorsBus(port=port, motors=motors, calibration=calib)
    bus.connect()

    if calib:
        bus.write_calibration(calib)
        print(f"Calibration loaded  (id='{arm_id}')")
    else:
        print(f"[WARN] No calibration for id='{arm_id}'")

    bus.disable_torque()
    for m in motors:
        bus.write("Operating_Mode", m, OperatingMode.POSITION.value)

    return bus


def _load_calib(arm_id):
    from lerobot.motors import MotorCalibration

    path = CALIB_ROOT / f"{arm_id}.json"
    if not path.exists():
        return None
    with open(path) as f:
        raw = json.load(f)
    out = {}
    for name, info in raw.items():
        out[name] = MotorCalibration(
            id=info["id"],
            drive_mode=info["drive_mode"],
            homing_offset=info["homing_offset"],
            range_min=info["range_min"],
            range_max=info["range_max"],
        )
    return out


# ΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉ
# MODE: IDENTIFY
# ΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉ

def run_identify(bus):
    print()
    print("=" * 65)
    print("JOINT IDENTIFICATION")
    print("Move ONE physical joint at a time.")
    print("Watch which motor name shows the biggest delta.")
    print("=" * 65)

    time.sleep(0.3)
    ref = bus.sync_read("Present_Position")
    print(f"\nRef: {_fmt(ref)}\n")
    print("Move a joint nowΓÇª  Ctrl+C to quit.\n")

    try:
        while True:
            deg = bus.sync_read("Present_Position")
            delta = {k: deg[k] - ref[k] for k in deg}
            top = max(delta, key=lambda k: abs(delta[k]))
            top_val = delta[top]
            tag = f"  <-  {top}  {top_val:+.1f}deg" if abs(top_val) > 2 else ""

            cols = "  ".join(f"{k[:6]:>6s}={delta[k]:+7.1f}" for k in delta)
            print(f"  {cols}{tag}            ", end="\r")
            time.sleep(0.04)
    except KeyboardInterrupt:
        print("\n")


# ΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉ
# MODE: LIVE
# ΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉ

DIR_LABELS = {
    0: ("FORWARD", "BACKWARD"),
    1: ("LEFT",    "RIGHT"),
    2: ("UP",      "DOWN"),
}


def run_live(bus, viewer=True):
    print()
    print("=" * 65)
    print("LIVE EEF POSE   Frame: X fwd | Y left | Z up")
    print("=" * 65)

    time.sleep(0.4)
    home_deg = bus.sync_read("Present_Position")
    home_q = motor_deg_to_angles(home_deg)
    home_eef, home_chain = forward_kinematics(*home_q)

    print(f"Home deg : {_fmt(home_deg)}")
    print(f"Home q   : [{np.degrees(home_q[0]):+.1f}, {np.degrees(home_q[1]):+.1f}, "
          f"{np.degrees(home_q[2]):+.1f}, {np.degrees(home_q[3]):+.1f}]")
    print(f"Home EEF : [{home_eef[0]:+.4f}, {home_eef[1]:+.4f}, {home_eef[2]:+.4f}] m")
    print()

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
            print(f"[WARN] Viewer failed: {e}")
            ax = None

    try:
        while True:
            deg = bus.sync_read("Present_Position")
            q = motor_deg_to_angles(deg)
            eef, chain = forward_kinematics(*q)
            cmd = eef - home_eef

            idx = int(np.argmax(np.abs(cmd)))
            if abs(cmd[idx]) > 0.008:
                label = DIR_LABELS[idx][0 if cmd[idx] > 0 else 1]
            else:
                label = "---"

            print(
                f"Cmd: [{cmd[0]: 8.4f}, {cmd[1]: 8.4f}, {cmd[2]: 8.4f}]"
                f"  ({label:8s})",
                end="\r",
            )

            if ax is not None:
                _draw(ax, chain, eef, cmd, label)

    except KeyboardInterrupt:
        print("\n\nDone.")
    finally:
        if ax is not None:
            import matplotlib.pyplot as plt
            plt.close("all")


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
        f"EEF [{eef[0]:.3f}, {eef[1]:.3f}, {eef[2]:.3f}]\n"
        f"Cmd [{cmd[0]:+.4f}, {cmd[1]:+.4f}, {cmd[2]:+.4f}]  ({label})",
        fontfamily="monospace", fontsize=11,
    )
    ax.legend(loc="upper left")
    plt.tight_layout()
    plt.pause(0.03)


def _fmt(d):
    return "  ".join(f"{k}={v:+.1f}" for k, v in d.items())


# ΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉ
# MAIN
# ΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉΓòÉ

def main():
    ap = argparse.ArgumentParser(description="AXE4 standalone FK + EEF")
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--id", default="axe", help="calibration file ID")
    ap.add_argument("--mode", choices=["live", "identify"], default="live")
    ap.add_argument("--no-viewer", action="store_true",
                    help="terminal only, no matplotlib")
    args = ap.parse_args()

    bus = connect(args.port, args.id)
    try:
        if args.mode == "identify":
            run_identify(bus)
        else:
            run_live(bus, viewer=not args.no_viewer)
    finally:
        bus.disconnect()


if __name__ == "__main__":
    main()
