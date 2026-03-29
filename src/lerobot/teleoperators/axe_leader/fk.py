"""Dynamic FK helpers for AXE modular leader teleoperator."""

import json
import logging
from pathlib import Path

import numpy as np

from .config_axe_leader import axeLeaderConfig

logger = logging.getLogger(__name__)

# Raw AXE3-style FK frame -> semantic frame (X=fwd, Y=left, Z=up)
# Derived from observed mapping:
#   read DOWN/UP   -> real FWD/BWD
#   read FWD/BWD   -> real LEFT/RIGHT
#   read LEFT/RIGHT-> real UP/DOWN
_FRAME_CORRECTION = np.array(
    [
        [0.0, 0.0, -1.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    ],
    dtype=np.float64,
)


def _default_axis_calibration_path() -> Path | None:
    # .../src/lerobot/teleoperators/axe_leader/fk.py -> repo root
    repo_root = Path(__file__).resolve().parents[4]
    candidates = [
        repo_root / "src" / "lerobot" / "teleoperators" / "axe_leader" / "calibrate" / "calibration_result.json",
        repo_root / "src" / "lerobot" / "teleoperators" / "axe_leader" / "calibrate" / "axis_calibration.json",
    ]
    for path in candidates:
        if path.exists():
            return path
    return None


def _resolve_axis_calibration_path(config: axeLeaderConfig) -> Path | None:
    if config.axis_calibration_path:
        p = Path(config.axis_calibration_path)
        return p if p.exists() else None
    return _default_axis_calibration_path()


def _load_axis_calibration_data(config: axeLeaderConfig) -> tuple[Path | None, dict | None]:
    path = _resolve_axis_calibration_path(config)
    if not path:
        return None, None
    try:
        with open(path) as f:
            return path, json.load(f)
    except (json.JSONDecodeError, OSError):
        logger.warning(f"Failed to read axis calibration file '{path}'.")
        return path, None


def load_motor_cfg(config: axeLeaderConfig) -> list[tuple[str, int, float]]:
    """Load (name, sign, offset_deg) from axis calibration json, fallback to identity mapping."""
    identity = [(name, 1, 0.0) for name in config.joint_names]
    active_names = set(config.joint_names)

    path, data = _load_axis_calibration_data(config)
    if path is None or data is None:
        logger.warning("No axis calibration file found for axe_leader; using sign=+1 offset=0 for all joints.")
        return identity

    motor_cfg = data.get("motor_cfg")
    if not isinstance(motor_cfg, list):
        logger.warning(f"Axis calibration file '{path}' missing motor_cfg; using identity motor mapping.")
        return identity

    out: list[tuple[str, int, float]] = []
    converted_tick_offsets = False
    for m in motor_cfg:
        if not isinstance(m, dict):
            continue
        name = m.get("name")
        if name not in active_names:
            continue
        sign = int(m.get("sign", 1))
        offset = float(m.get("offset", 0.0))
        # Backward compatibility: older axe_leader calibrator mistakenly stored
        # offsets in encoder ticks. Detect and convert to degrees.
        if abs(offset) > 720.0:
            offset = offset * 360.0 / 4096.0
            converted_tick_offsets = True
        out.append((name, sign, offset))

    if converted_tick_offsets:
        logger.warning(
            f"Axis calibration file '{path}' appears to contain tick-domain offsets; "
            "auto-converted offsets to degrees. Re-running FK calibration is recommended."
        )

    if len(out) != config.num_joints:
        logger.warning(
            f"Axis calibration file '{path}' has {len(out)} active motor_cfg entries, "
            f"expected {config.num_joints}; using identity motor mapping."
        )
        return identity

    by_name = {name: (name, sign, offset) for name, sign, offset in out}
    try:
        return [by_name[name] for name in config.joint_names]
    except KeyError:
        logger.warning(f"Axis calibration file '{path}' is missing at least one active joint; using identity.")
        return identity


def motor_deg_to_angles(deg_dict: dict[str, float], motor_cfg: list[tuple[str, int, float]]) -> np.ndarray:
    """Raw motor degrees -> joint angles in radians for active joints."""
    q = np.zeros(len(motor_cfg), dtype=np.float64)
    for i, (name, sign, off) in enumerate(motor_cfg):
        q[i] = np.radians(sign * deg_dict[name] + off)
    return q


def _rot_y(angle_rad: float) -> np.ndarray:
    c = np.cos(angle_rad)
    s = np.sin(angle_rad)
    return np.array(
        [
            [c, 0.0, s],
            [0.0, 1.0, 0.0],
            [-s, 0.0, c],
        ],
        dtype=np.float64,
    )


def _rot_z(angle_rad: float) -> np.ndarray:
    c = np.cos(angle_rad)
    s = np.sin(angle_rad)
    return np.array(
        [
            [c, -s, 0.0],
            [s, c, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def forward_kinematics(
    q: np.ndarray,
    link_lengths_m: list[float] | tuple[float, ...],
) -> tuple[np.ndarray, np.ndarray]:
    """AXE3-style FK with configurable link lengths.

    Joint convention:
        - q1: pan around +Z
        - q2: shoulder in X-Z plane (around +Y)
        - q3: elbow relative to shoulder in X-Z plane (around +Y)

    Link convention:
        - link_lengths_m[0]: pan/base segment
        - link_lengths_m[1]: upper-arm segment
        - link_lengths_m[2]: forearm segment
    """
    if q.size == 0:
        zero = np.zeros(3, dtype=np.float64)
        return zero, np.array([zero])

    if len(link_lengths_m) < q.size:
        raise ValueError(
            f"link_lengths_m must include one length per joint. "
            f"Got {len(link_lengths_m)} lengths for {q.size} joints."
        )

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
