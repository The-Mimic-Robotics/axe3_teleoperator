#!/usr/bin/env python3
"""FK diagnostic: prints one pasteable block per arm so we can see where right diverges.

Run from repo (with src on PYTHONPATH), arms powered and connected:

  conda run -n lerobot python -m lerobot.teleoperators.bi_axe_leader.debug_fk_snapshot

Put LEFT and RIGHT in the *same physical pose* (e.g. both straight home), run again, paste both blocks.

What we compare:
  - Raw Present_Position (bus degrees) — same hardware pose should be similar or mirror-related.
  - q (rad) after motor_cfg — should track each other if mapping is consistent.
  - q2_fk, q3_fk — values *after* planar_mirror + elbow offset (right only); these drive the green chain.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

from lerobot.teleoperators.axe_leader.fk import forward_kinematics, motor_deg_to_angles

# Import after lerobot path is available
from lerobot.teleoperators.bi_axe_leader.bi_live_eef import connect_arm, load_motor_cfg_from_file


def run_fk_snapshot_print(
    left_deg: dict[str, float],
    right_deg: dict[str, float],
    left_motor_cfg: list,
    right_motor_cfg: list,
    link_lengths_m: tuple[float, ...],
    *,
    right_elbow_off_rad: float,
    joint_names: tuple[str, ...] = ("shoulder_pan", "shoulder_lift", "elbow_flex"),
    left_axis_path: str = "",
    right_axis_path: str = "",
) -> None:
    """Print the same diagnostic as the CLI (for --debug-fk in bi_live_eef)."""
    print("=" * 72)
    print("BI_AXE_FK_DEBUG_SNAPSHOT")
    if left_axis_path:
        print(f"  left_axis:  {left_axis_path}")
    if right_axis_path:
        print(f"  right_axis: {right_axis_path}")
    print(f"  link_lengths_m: {link_lengths_m}")
    print(f"  right planar_mirror=True, elbow_off_rad={right_elbow_off_rad:+.6f}")
    print("=" * 72)
    print()
    _print_arm_block("LEFT", left_deg, left_motor_cfg, link_lengths_m, planar_mirror=False, elbow_off_rad=0.0, joint_names=joint_names)
    _print_arm_block(
        "RIGHT",
        right_deg,
        right_motor_cfg,
        link_lengths_m,
        planar_mirror=True,
        elbow_off_rad=right_elbow_off_rad,
        joint_names=joint_names,
    )
    print("--- QUICK CHECKS ---")
    print("  Same physical pose on both arms? Compare Present_Position and q lines.")
    print("=" * 72)


def _read_pos(bus):
    try:
        return bus.sync_read("Present_Position")
    except RuntimeError as e:
        if "has no calibration registered" not in str(e):
            raise
        return bus.sync_read("Present_Position", normalize=False)


def _fmt_deg(d: dict[str, float], names: tuple[str, ...]) -> str:
    return "  " + ", ".join(f"{n}={d[n]:+.2f}" for n in names)


def _print_arm_block(
    label: str,
    deg: dict[str, float],
    motor_cfg: list,
    link_lengths_m: tuple[float, ...],
    *,
    planar_mirror: bool,
    elbow_off_rad: float,
    joint_names: tuple[str, ...],
):
    q = motor_deg_to_angles(deg, motor_cfg)
    q_deg = np.degrees(q)
    q1, q2, q3 = float(q[0]), float(q[1]), float(q[2])
    q2_fk, q3_fk = q2, q3
    if planar_mirror:
        q2_fk = -q2
        q3_fk = -q3 + elbow_off_rad

    eef, chain = forward_kinematics(
        q,
        link_lengths_m,
        planar_mirror=planar_mirror,
        planar_mirror_elbow_offset_rad=elbow_off_rad if planar_mirror else 0.0,
    )

    print(f"--- {label} ---")
    print(f"  motor_cfg: {motor_cfg}")
    print(f"  Present_Position (deg): {_fmt_deg(deg, joint_names)}")
    print(
        f"  q after motor_cfg (deg): pan={q_deg[0]:+.2f} lift={q_deg[1]:+.2f} elbow={q_deg[2]:+.2f}  (rad: {q1:+.4f}, {q2:+.4f}, {q3:+.4f})"
    )
    print(
        f"  q used in planar FK (deg): pan={np.degrees(q1):+.2f} lift={np.degrees(q2_fk):+.2f} elbow={np.degrees(q3_fk):+.2f}  "
        f"(elbow_off_rad={elbow_off_rad:+.4f})"
    )
    print(f"  EEF xyz (m): {eef[0]:+.5f} {eef[1]:+.5f} {eef[2]:+.5f}")
    print("  chain joints (m):")
    for i, row in enumerate(chain):
        print(f"    p{i}: {row[0]:+.5f} {row[1]:+.5f} {row[2]:+.5f}")
    print()


def main() -> None:
    ap = argparse.ArgumentParser(description="Print one FK snapshot per arm (paste for debugging).")
    ap.add_argument("--left-port", default="COM8")
    ap.add_argument("--right-port", default="COM9")
    ap.add_argument("--left-id", default="axe_left")
    ap.add_argument("--right-id", default="axe_right")
    ap.add_argument("--left-axis-calib", default="")
    ap.add_argument("--right-axis-calib", default="")
    ap.add_argument("--calib-dir", default="")
    ap.add_argument("--links-m", type=float, nargs="+", default=[0.060, 0.210, 0.250])
    ap.add_argument(
        "--right-elbow-offset-deg",
        type=float,
        default=-90.0,
        help="Must match bi_live_eef / bi_axe_leader right arm (-90 default).",
    )
    args = ap.parse_args()

    joint_names = ("shoulder_pan", "shoulder_lift", "elbow_flex")
    motor_ids = (1, 2, 3)
    link_lengths_m = tuple(args.links_m[:3])
    elbow_rad = math.radians(args.right_elbow_offset_deg)

    if args.calib_dir:
        calib_root = Path(args.calib_dir)
    else:
        calib_root = Path(__file__).resolve().parents[4] / "calibration" / "teleoperators" / "bi_axe_leader"

    left_axis = args.left_axis_calib or str(calib_root / "axis_left.json")
    right_axis = args.right_axis_calib or str(calib_root / "axis_right.json")

    left_motor_cfg = load_motor_cfg_from_file(left_axis)
    right_motor_cfg = load_motor_cfg_from_file(right_axis)
    if not left_motor_cfg:
        left_motor_cfg = [(n, 1, 0.0) for n in joint_names]
    if not right_motor_cfg:
        right_motor_cfg = [(n, 1, 0.0) for n in joint_names]

    print("=" * 72)
    print("BI_AXE_FK_DEBUG_SNAPSHOT  (paste this whole block)")
    print(f"  left_axis:  {left_axis}  exists={Path(left_axis).exists()}")
    print(f"  right_axis: {right_axis} exists={Path(right_axis).exists()}")
    print(f"  link_lengths_m: {link_lengths_m}")
    print(f"  right planar_mirror=True, planar_mirror_elbow_offset_rad={elbow_rad:+.6f} ({args.right_elbow_offset_deg:+.1f} deg)")
    print("=" * 72)
    print()

    left_bus = connect_arm(args.left_port, args.left_id, joint_names, motor_ids, calib_root)
    right_bus = connect_arm(args.right_port, args.right_id, joint_names, motor_ids, calib_root)
    try:
        ld = _read_pos(left_bus)
        rd = _read_pos(right_bus)

        run_fk_snapshot_print(
            ld,
            rd,
            left_motor_cfg,
            right_motor_cfg,
            link_lengths_m,
            right_elbow_off_rad=elbow_rad,
            joint_names=joint_names,
            left_axis_path=left_axis,
            right_axis_path=right_axis,
        )
        print("  If poses match physically but EEF/chains differ a lot: motor_cfg vs planar_mirror.")
        print("  If raw Present_Position differs a lot at same pose: encoder/homing (not only JSON).")
    finally:
        left_bus.disconnect()
        right_bus.disconnect()


if __name__ == "__main__":
    # Allow `python path/to/debug_fk_snapshot.py` if repo root is on PYTHONPATH
    main()
