#!/usr/bin/env python
"""
Configuration for the AXE modular leader teleoperator.

Uses a single arm structure for joint metadata and link geometry.
"""

from dataclasses import dataclass, field
from pathlib import Path

from ..config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("axe_leader")
@dataclass
class axeLeaderConfig(TeleoperatorConfig):
    """Config for AXE modular leader (single-arm)."""

    # Serial connection for this single leader arm instance.
    port: str = "/dev/ttyACM0"
    use_degrees: bool = True

    # Unified arm model definition.
    # `joints` entries must be ordered from pan -> distal joint.
    # Every joint includes `link_length_m` in meters.
    # For `shoulder_pan`, `link_length_m` represents the pan-to-next-joint segment.
    # Example joint entry:
    #   {"name": "shoulder_lift", "id": 2, "model": "sts3215", "link_length_m": 0.210}
    arm: dict[str, object] = field(
        default_factory=lambda: {
            "axis_calibration_path": "",
            "joints": [
                {"name": "shoulder_pan", "id": 1, "model": "sts3215", "link_length_m": 0.060},
                {"name": "shoulder_lift", "id": 2, "model": "sts3215", "link_length_m": 0.210},
                {"name": "elbow_flex", "id": 3, "model": "sts3215", "link_length_m": 0.250},
                # {"name": "elbow_super_flex", "id": 4, "model": "sts3215", "link_length_m": 0.120},
            ],
        }
    )

    # --- Handle / IMU source ---
    has_imu: bool = True
    handle_source: str = "ble"
    handle_device_name: str = "AXE4_left"
    # Optional direct UUID override for this device, e.g.
    # {"angle": "...", "quat": "...", "joy": "..."}
    handle_ble_uuids: dict[str, str] = field(default_factory=dict)
    # BLE UUID profiles keyed by side or custom label.
    # UUID ownership lives in config (not in HandleReader).
    handle_ble_profiles: dict[str, dict[str, object]] = field(
        default_factory=lambda: {
            "left": {
                "name": "AXE4_left",
                "uuids": {
                    "angle": "beb5483e-36e1-4688-b7f5-ea07361b26a8",
                    "quat": "828917c1-ea55-4d4a-a66e-fd202cea0645",
                    "joy": "9c661337-b499-497d-aa5b-0105316e6e22",
                },
            },
            "right": {
                "name": "AXE4_right",
                "uuids": {
                    "angle": "d1a68735-86b2-4d26-b8f2-1b633075c3f9",
                    "quat": "f3c83012-78d1-4e96-a14a-7bc991060932",
                    "joy": "2a8497d5-d852-4f01-90a6-16e51141bc25",
                },
            },
        }
    )
    imu_port: int = 5000
    imu_ip: str = "127.0.0.1"

    # --- Transport ---
    transport: str = "ros2"
    ros2_node_name: str = "axe_teleop"
    ros2_topic_prefix: str = "axe"
    udp_target_ip: str = "192.168.131.150"
    udp_target_port: int = 5005
    # When True (default), UDP sends only the legacy 28-byte eef_pose packet (<7f>).
    # When False, non-pose packets are sent with a 1-byte type tag to avoid collisions with
    # receivers that parse raw 28-byte pose packets.
    udp_pose_only: bool = False
    # When True, prints each UDP payload being sent (type + values) to terminal.
    udp_print_packets: bool = False

    # --- Teleop arming / filtering ---
    require_arm_key: bool = True
    arm_key: str = " "
    # One of: keyboard, handle_sw, handle_sw2, xbox_a, xbox_b
    arm_toggle_source: str = "keyboard"
    arm_toggle_cooldown_s: float = 0.3

    # Position/twist deadbands reduce small jitter when the leader is static.
    position_deadband_m: float = 0.003
    twist_deadband_m: float = 0.001

    # Right-hand arm in a mirrored pair: negate q2+q3 in FK (see fk.forward_kinematics).
    planar_mirror_fk: bool = False
    # Added to q3 after that negation (rad); default −π/2 on right arm in bi_axe_leader.
    planar_mirror_elbow_offset_rad: float = 0.0

    @property
    def joint_defs(self) -> list[dict[str, object]]:
        joints = self.arm.get("joints", []) if isinstance(self.arm, dict) else []
        return [j for j in joints if isinstance(j, dict)]

    @property
    def joint_names(self) -> tuple[str, ...]:
        return tuple(str(j["name"]) for j in self.joint_defs)

    @property
    def motor_ids(self) -> tuple[int, ...]:
        return tuple(int(j["id"]) for j in self.joint_defs)

    @property
    def motor_models(self) -> tuple[str, ...]:
        return tuple(str(j.get("model", "sts3215")) for j in self.joint_defs)

    @property
    def num_joints(self) -> int:
        return len(self.joint_defs)

    @property
    def link_lengths_m(self) -> tuple[float, ...]:
        return tuple(float(j.get("link_length_m", 0.0)) for j in self.joint_defs)

    @property
    def axis_calibration_path(self) -> str:
        if isinstance(self.arm, dict):
            return str(self.arm.get("axis_calibration_path", "") or "")
        return ""

    @property
    def resolved_handle_ble_uuids(self) -> dict[str, str]:
        required = {"angle", "quat", "joy"}

        if required.issubset(set(self.handle_ble_uuids.keys())):
            return {k: str(self.handle_ble_uuids[k]) for k in ("angle", "quat", "joy")}

        target = (self.handle_device_name or "").strip().lower()
        for profile in self.handle_ble_profiles.values():
            if not isinstance(profile, dict):
                continue
            name = str(profile.get("name", "")).strip().lower()
            uuids = profile.get("uuids", {})
            if name and name == target and isinstance(uuids, dict) and required.issubset(set(uuids.keys())):
                return {k: str(uuids[k]) for k in ("angle", "quat", "joy")}

        # Fallback to first valid profile (typically left)
        for profile in self.handle_ble_profiles.values():
            if not isinstance(profile, dict):
                continue
            uuids = profile.get("uuids", {})
            if isinstance(uuids, dict) and required.issubset(set(uuids.keys())):
                return {k: str(uuids[k]) for k in ("angle", "quat", "joy")}

        raise ValueError("No valid BLE UUID set found. Provide handle_ble_uuids with angle/quat/joy.")

    def __post_init__(self) -> None:
        if self.calibration_dir is None:
            repo_root = Path(__file__).resolve().parents[4]
            self.calibration_dir = repo_root / "calibration" / "teleoperators" / "axe_leader"

        if self.num_joints not in (3, 4):
            raise ValueError(f"arm.joints must define 3 or 4 joints, got {self.num_joints}")

        for idx, joint in enumerate(self.joint_defs):
            if "name" not in joint or "id" not in joint:
                raise ValueError(f"arm.joints[{idx}] must include 'name' and 'id'")
            if "link_length_m" not in joint:
                raise ValueError(f"arm.joints[{idx}] must include 'link_length_m'")

        valid_sources = {"keyboard", "handle_sw", "handle_sw2", "xbox_a", "xbox_b"}
        if self.arm_toggle_source not in valid_sources:
            raise ValueError(f"arm_toggle_source must be one of {sorted(valid_sources)}")

        if self.has_imu and self.handle_source == "ble":
            _ = self.resolved_handle_ble_uuids
