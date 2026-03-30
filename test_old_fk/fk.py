"""
Planar 3-link + pan FK for AXE4. No URDF in pipeline.

Output frame: arch in the XZ (forward) plane. +X = forward, +Y = left, Z = up.
Applied as 90┬░ rotation so arm that physically extends "left" in pan frame is reported as forward/back.
"""

import json
import numpy as np
from pathlib import Path

# Arm geometry (m), from CAD / axe4_arm.yaml
L_BASE = 0.075   # pan axis to shoulder pivot
L1 = 0.249      # upper arm
L2 = 0.282      # forearm
L3 = 0.120      # handle to grip

JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "elbow_super_flex"]

# Default motor_cfg if no calibration file (sign, offset_deg)
_DEFAULT_MOTOR_CFG = [
    ("shoulder_pan", -1, 168.5),
    ("shoulder_lift", -1, 78.8),
    ("elbow_flex", -1, 114.0),
    ("elbow_super_flex", -1, 113.6),
]


def _find_calibration_path() -> Path | None:
    """Resolve src/axe4/config/axe4_axis_calibration.json from this package."""
    # .../src/lerobot/teleoperators/axe4_leader/fk.py -> src
    src = Path(__file__).resolve().parent.parent.parent.parent
    p = src / "axe4" / "config" / "axe4_axis_calibration.json"
    return p if p.exists() else None


def load_motor_cfg() -> list[tuple[str, int, float]]:
    """Load (name, sign, offset_deg) from axe4_axis_calibration.json if present."""
    path = _find_calibration_path()
    if not path:
        return _DEFAULT_MOTOR_CFG.copy()
    try:
        with open(path) as f:
            data = json.load(f)
    except (json.JSONDecodeError, OSError):
        return _DEFAULT_MOTOR_CFG.copy()
    motor_cfg = data.get("motor_cfg")
    if not motor_cfg or len(motor_cfg) != 4:
        return _DEFAULT_MOTOR_CFG.copy()
    out = []
    for m in motor_cfg:
        name = m.get("name")
        sign = int(m.get("sign", 1))
        offset = float(m.get("offset", 0.0))
        if name in JOINT_NAMES:
            out.append((name, sign, offset))
    if len(out) != 4:
        return _DEFAULT_MOTOR_CFG.copy()
    # Ensure order matches JOINT_NAMES
    by_name = {n: (n, s, o) for n, s, o in out}
    return [by_name[n] for n in JOINT_NAMES]


def motor_deg_to_angles(deg_dict: dict[str, float], motor_cfg: list[tuple[str, int, float]]) -> np.ndarray:
    """Raw motor degrees -> (q1,q2,q3,q4) in radians."""
    q = np.zeros(4)
    for i, (name, sign, off) in enumerate(motor_cfg):
        q[i] = np.radians(sign * deg_dict[name] + off)
    return q


def forward_kinematics(q1: float, q2: float, q3: float, q4: float) -> tuple[np.ndarray, np.ndarray]:
    """
    Planar 3-link + pan. Returns (eef_xyz, chain_5x3).
    q1=pan (rad), q2..q4 incremental in vertical plane; 0┬░ = horizontal, +90┬░ = up.
    """
    a2, a3, a4 = q2, q2 + q3, q2 + q3 + q4
    c1, s1 = np.cos(q1), np.sin(q1)

    joints_rz = [(0.0, 0.0)]
    r, z = L_BASE, 0.0
    joints_rz.append((r, z))
    r += L1 * np.cos(a2); z += L1 * np.sin(a2)
    joints_rz.append((r, z))
    r += L2 * np.cos(a3); z += L2 * np.sin(a3)
    joints_rz.append((r, z))
    r += L3 * np.cos(a4); z += L3 * np.sin(a4)
    joints_rz.append((r, z))

    chain = np.array([[ri * c1, ri * s1, zi] for ri, zi in joints_rz])
    # Rotate 90┬░ about Z so arch lies in forward (XZ) plane: (x,y,z) -> (-y, x, z)
    chain = np.column_stack([-chain[:, 1], chain[:, 0], chain[:, 2]])
    return chain[-1], chain
