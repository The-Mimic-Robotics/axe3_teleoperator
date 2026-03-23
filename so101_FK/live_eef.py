#!/usr/bin/env python3
"""
SO101 — live FK (EEF pose + twist) + simple link visualization.

This script mirrors the UX of `axe4_FK/live_eef.py` but uses an actual URDF FK solver
(via LeRobot's `RobotKinematics`, which uses the optional `placo` dependency).

Prereqs:
  - A calibrated SO101 device (leader or follower) via `lerobot-calibrate`
  - An SO101 URDF file. Recommended by LeRobot:
      https://github.com/TheRobotStudio/SO-ARM100/blob/main/Simulation/SO101/so101_new_calib.urdf

Examples:
  # Identify motor names by moving one joint at a time
  python so101_FK/live_eef.py --port /dev/ttyACM0 --device so101_leader --id my_leader --mode identify

  # Live EEF pose + twist (+ optional viewer)
  python so101_FK/live_eef.py --port /dev/ttyACM0 --device so101_follower --id my_follower \
    --urdf_path ~/SO-ARM100/Simulation/SO101/so101_new_calib.urdf --target_frame gripper

  # Terminal only
  python so101_FK/live_eef.py ... --no-viewer
"""

import argparse
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np


SO101_MOTOR_ORDER = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]


@dataclass(frozen=True)
class DeviceSpec:
    kind: str  # "robots" | "teleoperators"
    name: str  # "so101_follower" | "so101_leader"


def _device_spec(device: str) -> DeviceSpec:
    if device == "so101_follower":
        return DeviceSpec(kind="robots", name="so101_follower")
    if device == "so101_leader":
        return DeviceSpec(kind="teleoperators", name="so101_leader")
    raise ValueError(f"Unknown device: {device}")


def _load_calibration(device: str, arm_id: str):
    """
    Load LeRobot calibration dict[str, MotorCalibration] if present.

    Preference order:
      1) Local to this repo: so101_FK/calibration/{robots|teleoperators}/{device}/{id}.json
      2) LeRobot default cache: ~/.cache/huggingface/lerobot/calibration/{robots|teleoperators}/{device}/{id}.json
    """
    import draccus

    from lerobot.motors.motors_bus import MotorCalibration
    from lerobot.utils.constants import HF_LEROBOT_CALIBRATION

    spec = _device_spec(device)
    local_root = Path(__file__).resolve().parent / "calibration"
    local_path = local_root / spec.kind / spec.name / f"{arm_id}.json"
    cache_path = HF_LEROBOT_CALIBRATION / spec.kind / spec.name / f"{arm_id}.json"

    calib_path = local_path if local_path.exists() else cache_path
    if not calib_path.exists():
        # Return both paths for better diagnostics.
        return None, (local_path, cache_path)

    with open(calib_path) as f, draccus.config_type("json"):
        calib = draccus.load(dict[str, MotorCalibration], f)
    return calib, calib_path


def connect(port: str, device: str, arm_id: str):
    """
    Connect to SO101 via Feetech bus, applying calibration if available.
    """
    from lerobot.motors import Motor, MotorNormMode
    from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode

    motors = {
        "shoulder_pan": Motor(1, "sts3215", MotorNormMode.DEGREES),
        "shoulder_lift": Motor(2, "sts3215", MotorNormMode.DEGREES),
        "elbow_flex": Motor(3, "sts3215", MotorNormMode.DEGREES),
        "wrist_flex": Motor(4, "sts3215", MotorNormMode.DEGREES),
        "wrist_roll": Motor(5, "sts3215", MotorNormMode.DEGREES),
        "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
    }

    calib, calib_path = _load_calibration(device, arm_id)
    bus = FeetechMotorsBus(port=port, motors=motors, calibration=calib)
    bus.connect()

    if calib:
        bus.write_calibration(calib)
        print(f"Calibration loaded: {calib_path}")
    else:
        if isinstance(calib_path, tuple):
            local_path, cache_path = calib_path
            print(f"[WARN] No calibration found at:")
            print(f"  - {local_path}")
            print(f"  - {cache_path}")
            print("Run: python so101_FK/calibrate.py ...  to generate one in so101_FK/calibration/")
        else:
            print(f"[WARN] No calibration found at: {calib_path}")

    # Read-only / safe: disable torque and set position mode.
    bus.disable_torque()
    for m in motors:
        bus.write("Operating_Mode", m, OperatingMode.POSITION.value)

    return bus


def _fmt(d: dict[str, float]) -> str:
    return "  ".join(f"{k}={float(v):+7.2f}" for k, v in d.items())


def run_identify(bus):
    print()
    print("=" * 72)
    print("JOINT IDENTIFICATION")
    print("Move ONE physical joint at a time.")
    print("Watch which motor name shows the biggest delta.")
    print("=" * 72)

    time.sleep(0.3)
    ref = bus.sync_read("Present_Position")
    print(f"\nRef: {_fmt(ref)}\n")
    print("Move a joint now…  Ctrl+C to quit.\n")

    try:
        while True:
            deg = bus.sync_read("Present_Position")
            delta = {k: float(deg[k] - ref[k]) for k in deg}
            top = max(delta, key=lambda k: abs(delta[k]))
            top_val = delta[top]
            tag = f"  <-  {top}  {top_val:+.1f}deg" if abs(top_val) > 2 else ""
            cols = "  ".join(f"{k[:10]:>10s}={delta[k]:+7.1f}" for k in delta)
            print(f"  {cols}{tag}            ", end="\r")
            time.sleep(0.04)
    except KeyboardInterrupt:
        print("\n")


def _parse_urdf_chain_links(urdf_path: str, tip_link_or_frame: str) -> list[str]:
    """
    Return link names along the kinematic chain from the URDF root link to `tip_link_or_frame`.

    This is used only for visualization (to draw link positions). If parsing fails, returns [].
    """
    import xml.etree.ElementTree as ET

    try:
        root = ET.parse(urdf_path).getroot()
    except Exception:
        return []

    joints = root.findall("joint")
    parent_of: dict[str, str] = {}
    child_of: dict[str, str] = {}
    for j in joints:
        p = j.find("parent")
        c = j.find("child")
        if p is None or c is None:
            continue
        parent = p.attrib.get("link")
        child = c.attrib.get("link")
        if parent and child:
            parent_of[child] = parent
            child_of[parent] = child  # not 1-1 globally, but enough for root finding below

    # Root link: appears as a parent but never as a child.
    all_parents = {p for p in child_of.keys()}
    all_children = {c for c in parent_of.keys()}
    roots = sorted(all_parents - all_children)
    if not roots:
        # Fallback: any link that isn't a child
        roots = sorted({l.attrib.get("name") for l in root.findall("link")} - all_children)
        roots = [r for r in roots if r]
    root_link = roots[0] if roots else None
    if not root_link:
        return []

    # Tip might be a frame name; we can only chain links. Try exact link match first.
    link_names = {l.attrib.get("name") for l in root.findall("link")}
    tip_link = tip_link_or_frame if tip_link_or_frame in link_names else None
    if tip_link is None:
        # Heuristics: common tip links
        for cand in (tip_link_or_frame.replace("_frame", ""), "gripper", "tool0", "ee", "end_effector"):
            if cand in link_names:
                tip_link = cand
                break
    if tip_link is None:
        return []

    chain = [tip_link]
    while chain[-1] in parent_of:
        chain.append(parent_of[chain[-1]])
        if len(chain) > 64:
            break
    chain.reverse()

    # Ensure chain starts at root if possible.
    if root_link in chain:
        chain = chain[chain.index(root_link) :]
    return chain


def _urdf_links_for_joint_chain(urdf_path: str, joint_names: list[str], target_frame: str) -> list[str]:
    """
    Build a *link* chain for visualization using URDF joint definitions.

    We prefer to follow the commanded joint order (joint_names) to avoid ambiguity in branched URDFs.
    Returned list is link names in order, starting from the first joint's parent link.
    """
    import xml.etree.ElementTree as ET

    try:
        robot = ET.parse(urdf_path).getroot()
    except Exception:
        return []

    joints = {}
    for j in robot.findall("joint"):
        name = j.attrib.get("name")
        if not name:
            continue
        p = j.find("parent")
        c = j.find("child")
        if p is None or c is None:
            continue
        parent = p.attrib.get("link")
        child = c.attrib.get("link")
        if parent and child:
            joints[name] = (parent, child)

    # Resolve target_frame to a link if user passed a joint name like "gripper".
    # For SO101 new calib URDF, the commonly desired tip link is "gripper_frame_link".
    link_names = {l.attrib.get("name") for l in robot.findall("link")}
    target_link = target_frame if target_frame in link_names else None
    if target_link is None:
        for cand in ("gripper_frame_link", "gripper_link", "tool0", "ee_link"):
            if cand in link_names:
                target_link = cand
                break

    chain: list[str] = []

    # Start at first joint's parent
    if joint_names:
        first = joints.get(joint_names[0])
        if first:
            chain.append(first[0])

    # Append each joint's child in order
    for jn in joint_names:
        pc = joints.get(jn)
        if not pc:
            continue
        parent, child = pc
        if not chain:
            chain.append(parent)
        if chain[-1] != parent and parent not in chain:
            # URDF mismatch; still add parent to keep polyline sane
            chain.append(parent)
        chain.append(child)

    # If a target link exists and isn't already included, append via parent traversal if possible.
    if target_link and (not chain or chain[-1] != target_link):
        parent_of = {child: parent for (parent, child) in joints.values()}
        tip = target_link
        back = [tip]
        while tip in parent_of and len(back) < 64:
            tip = parent_of[tip]
            back.append(tip)
        back.reverse()
        # Merge onto our chain tail if they intersect, else just append the target.
        inter = None
        for i, name in enumerate(back):
            if name in chain:
                inter = name
        if inter:
            idx = chain.index(inter)
            chain = chain[: idx + 1]
            chain.extend([n for n in back[back.index(inter) + 1 :] if n not in chain])
        else:
            chain.append(target_link)

    # De-dupe preserving order
    seen = set()
    out = []
    for n in chain:
        if n and n not in seen:
            out.append(n)
            seen.add(n)
    return out


def _try_get_frame_positions(kin, frame_names: Iterable[str]) -> np.ndarray:
    """
    Query placo wrapper for positions of each frame name; skips missing frames.
    Returns Nx3.
    """
    pts = []
    for name in frame_names:
        try:
            t = kin.robot.get_T_world_frame(name)
            pts.append(np.array(t[:3, 3], dtype=float))
        except Exception:
            continue
    return np.array(pts, dtype=float) if pts else np.zeros((0, 3), dtype=float)


def _compute_twist(prev_T: np.ndarray, curr_T: np.ndarray, dt: float) -> tuple[np.ndarray, np.ndarray]:
    """
    Finite-difference twist in world frame:
      v = dp/dt
      w = log(R_prev^T R_curr)/dt
    """
    from lerobot.utils.rotation import Rotation

    if dt <= 0:
        return np.zeros(3), np.zeros(3)

    dp = curr_T[:3, 3] - prev_T[:3, 3]
    v = dp / dt

    r_rel = prev_T[:3, :3].T @ curr_T[:3, :3]
    rotvec = Rotation.from_matrix(r_rel).as_rotvec()
    w = rotvec / dt
    return v, w


def run_live(
    bus,
    urdf_path: str,
    target_frame: str,
    joint_names: list[str],
    viewer: bool = True,
):
    from lerobot.model.kinematics import RobotKinematics
    from lerobot.utils.rotation import Rotation

    print()
    print("=" * 72)
    print("LIVE EEF POSE + TWIST   Frame: URDF world")
    print("=" * 72)

    # Kinematics solver (requires placo)
    # Prefer a link tip frame if user passed a joint name (common: "gripper").
    tip_frame = target_frame
    if target_frame == "gripper":
        tip_frame = "gripper_frame_link"

    kin = RobotKinematics(
        urdf_path=urdf_path,
        target_frame_name=tip_frame,
        joint_names=joint_names,
    )

    # Build chain links for plotting. First try ordered joint-chain, then fall back to generic traversal.
    chain_links = _urdf_links_for_joint_chain(urdf_path, joint_names=joint_names, target_frame=tip_frame)
    if not chain_links:
        chain_links = _parse_urdf_chain_links(urdf_path, tip_link_or_frame=tip_frame)

    # Viewer init
    ax = None
    if viewer:
        try:
            import matplotlib.pyplot as plt
            from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

            plt.ion()
            fig = plt.figure(figsize=(9, 8))
            ax = fig.add_subplot(111, projection="3d")
            ax.view_init(elev=22, azim=-145)
        except Exception as e:
            print(f"[WARN] Viewer failed: {e}")
            ax = None

    # Prime
    time.sleep(0.25)
    deg = bus.sync_read("Present_Position")
    q_deg = np.array([float(deg[n]) for n in joint_names], dtype=float)
    t_prev = kin.forward_kinematics(q_deg)
    t0 = time.perf_counter()

    pos0 = t_prev[:3, 3].copy()
    rot0 = Rotation.from_matrix(t_prev[:3, :3]).as_rotvec()

    print(f"Using joints : {joint_names}")
    print(f"Target frame : {tip_frame}")
    print(f"Home joints  : {_fmt({n: deg[n] for n in joint_names})}")
    print(f"Home pos (m) : [{pos0[0]:+.4f}, {pos0[1]:+.4f}, {pos0[2]:+.4f}]")
    print(f"Home rotvec  : [{rot0[0]:+.4f}, {rot0[1]:+.4f}, {rot0[2]:+.4f}] rad")
    if chain_links:
        print(f"Viz chain    : {chain_links[0]} -> ... -> {chain_links[-1]}  ({len(chain_links)} links)")
    else:
        print("Viz chain    : [WARN] could not parse chain from URDF (will show EEF only)")
    print()

    try:
        while True:
            t1 = time.perf_counter()
            dt = t1 - t0

            deg = bus.sync_read("Present_Position")
            q_deg = np.array([float(deg[n]) for n in joint_names], dtype=float)
            t_curr = kin.forward_kinematics(q_deg)

            pos = t_curr[:3, 3]
            rotvec = Rotation.from_matrix(t_curr[:3, :3]).as_rotvec()
            v, w = _compute_twist(t_prev, t_curr, dt)

            print(
                f"pos[m]=[{pos[0]:+7.4f},{pos[1]:+7.4f},{pos[2]:+7.4f}]  "
                f"rotvec[rad]=[{rotvec[0]:+7.4f},{rotvec[1]:+7.4f},{rotvec[2]:+7.4f}]  "
                f"v[m/s]=[{v[0]:+7.3f},{v[1]:+7.3f},{v[2]:+7.3f}]  "
                f"w[rad/s]=[{w[0]:+7.3f},{w[1]:+7.3f},{w[2]:+7.3f}]",
                end="\r",
            )

            if ax is not None:
                _draw(ax, kin, chain_links, t_curr)

            t_prev = t_curr
            t0 = t1

    except KeyboardInterrupt:
        print("\n\nDone.")
    finally:
        if ax is not None:
            import matplotlib.pyplot as plt

            plt.close("all")


def _draw(ax, kin, chain_links: list[str], t_eef: np.ndarray):
    import matplotlib.pyplot as plt

    ax.cla()

    # Link positions (best effort)
    pts = _try_get_frame_positions(kin, chain_links) if chain_links else np.zeros((0, 3), dtype=float)
    if len(pts) >= 2:
        ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], "-o", lw=3, ms=6, color="steelblue", label="arm")

    eef = np.array(t_eef[:3, 3], dtype=float)
    ax.scatter(*eef, s=140, c="red", marker="^", zorder=5, label="EEF")

    # Simple axis triad at EEF
    R = t_eef[:3, :3]
    scale = 0.05
    ax.quiver(eef[0], eef[1], eef[2], *(R[:, 0] * scale), color="r", linewidth=2)
    ax.quiver(eef[0], eef[1], eef[2], *(R[:, 1] * scale), color="g", linewidth=2)
    ax.quiver(eef[0], eef[1], eef[2], *(R[:, 2] * scale), color="b", linewidth=2)

    # Bounds: auto around chain / eef
    cloud = pts if len(pts) else eef.reshape(1, 3)
    mins = cloud.min(axis=0)
    maxs = cloud.max(axis=0)
    pad = 0.15
    ax.set_xlim(float(mins[0] - pad), float(maxs[0] + pad))
    ax.set_ylim(float(mins[1] - pad), float(maxs[1] + pad))
    ax.set_zlim(float(mins[2] - pad), float(maxs[2] + pad))

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title(f"EEF [{eef[0]:.3f}, {eef[1]:.3f}, {eef[2]:.3f}] (URDF world)", fontfamily="monospace")
    ax.legend(loc="upper left")
    plt.tight_layout()
    plt.pause(0.03)


def main():
    ap = argparse.ArgumentParser(description="SO101 live FK (EEF pose + twist)")
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--device", choices=["so101_follower", "so101_leader"], default="so101_follower")
    ap.add_argument("--id", required=True, help="calibration file ID (used by lerobot-calibrate)")

    ap.add_argument("--mode", choices=["live", "identify"], default="live")
    ap.add_argument("--no-viewer", action="store_true", help="terminal only, no matplotlib")

    ap.add_argument(
        "--urdf_path",
        default=None,
        help="Path to SO101 URDF (file or folder). If omitted, uses vendored SO-ARM100 under so101_FK/.",
    )
    ap.add_argument("--target_frame", default="gripper", help="EEF frame name in URDF (default: gripper)")
    ap.add_argument(
        "--joints",
        default="shoulder_pan,shoulder_lift,elbow_flex,wrist_flex,wrist_roll",
        help="Comma-separated joints for FK (exclude gripper).",
    )

    args = ap.parse_args()

    vendored_default_urdf = (
        Path(__file__).resolve().parent / "SO-ARM100" / "Simulation" / "SO101" / "so101_new_calib.urdf"
    )

    def _resolve_urdf_path(p: str) -> str:
        path = Path(p).expanduser()
        if path.is_file():
            return str(path)
        if path.is_dir():
            # placo/pin sometimes expects "robot.urdf" inside a folder; handle both cases.
            for cand in (
                path / "robot.urdf",
                path / "so101_new_calib.urdf",
                path / "so101.urdf",
                path / "so101_new_calib.urdf.xacro",
            ):
                if cand.is_file():
                    return str(cand)
            # Fall back to first *.urdf in the directory (non-recursive).
            urdfs = sorted(path.glob("*.urdf"))
            if urdfs:
                return str(urdfs[0])
        return str(path)

    bus = connect(args.port, args.device, args.id)
    try:
        if args.mode == "identify":
            run_identify(bus)
            return

        if not args.urdf_path:
            if vendored_default_urdf.exists():
                args.urdf_path = str(vendored_default_urdf)
            else:
                raise SystemExit(
                    "Missing --urdf_path and no vendored URDF found. Recommended URDF: "
                    "https://github.com/TheRobotStudio/SO-ARM100/blob/main/Simulation/SO101/so101_new_calib.urdf"
                )

        urdf_path = _resolve_urdf_path(args.urdf_path)
        joint_names = [s.strip() for s in args.joints.split(",") if s.strip()]
        run_live(
            bus=bus,
            urdf_path=urdf_path,
            target_frame=args.target_frame,
            joint_names=joint_names,
            viewer=not args.no_viewer,
        )
    finally:
        bus.disconnect()


if __name__ == "__main__":
    main()

