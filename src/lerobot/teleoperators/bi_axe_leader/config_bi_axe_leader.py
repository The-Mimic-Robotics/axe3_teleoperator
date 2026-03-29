#!/usr/bin/env python

from dataclasses import dataclass, field

from ..config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("bi_axe_leader")
@dataclass
class BiAxeLeaderConfig(TeleoperatorConfig):
    # HuggingFace-style explicit left/right serial ports.
    left_arm_port: str = "/dev/ttyACM0"
    right_arm_port: str = "/dev/ttyACM1"

    # BLE handles auto-default to left/right names; override only if needed.
    left_handle_device_name: str = "AXE3_left"
    right_handle_device_name: str = "AXE3_right"

    # Shared AXE leader behavior
    use_degrees: bool = True
    arm: dict[str, object] = field(
        default_factory=lambda: {
            "axis_calibration_path": "",
            "joints": [
                {"name": "shoulder_pan", "id": 1, "model": "sts3215", "link_length_m": 0.060},
                {"name": "shoulder_lift", "id": 2, "model": "sts3215", "link_length_m": 0.210},
                {"name": "elbow_flex", "id": 3, "model": "sts3215", "link_length_m": 0.250},
            ],
        }
    )
    has_imu: bool = True
    handle_source: str = "ble"
    imu_port: int = 5000
    imu_ip: str = "127.0.0.1"

    # One shared transport target for both arms.
    transport: str = "udp"
    udp_target_ip: str = "127.0.0.1"
    udp_target_port: int = 5005
    udp_pose_only: bool = False
    udp_print_packets: bool = False

    # Arming / filtering
    require_arm_key: bool = True
    arm_key: str = " "
    arm_toggle_source: str = "keyboard"
    arm_toggle_cooldown_s: float = 0.3
    position_deadband_m: float = 0.003
    twist_deadband_m: float = 0.001

    def __post_init__(self) -> None:
        if not self.left_arm_port:
            raise ValueError("left_arm_port is required")
        if not self.right_arm_port:
            raise ValueError("right_arm_port is required")
