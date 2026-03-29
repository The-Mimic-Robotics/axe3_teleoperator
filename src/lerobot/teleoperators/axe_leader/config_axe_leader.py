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
    handle_device_name: str = "AXE3_left"
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
