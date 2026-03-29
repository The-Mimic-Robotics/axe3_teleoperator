#!/usr/bin/env python

import logging
from functools import cached_property

from ..teleoperator import Teleoperator
from ..axe_leader.axe_leader import axeLeader
from ..axe_leader.config_axe_leader import axeLeaderConfig
from .config_bi_axe_leader import BiAxeLeaderConfig
from .udp_transport import BiAxeUDPTransport

logger = logging.getLogger(__name__)


class BiAxeLeader(Teleoperator):
    """Bimanual AXE leader using two axeLeader instances and one shared UDP target."""

    config_class = BiAxeLeaderConfig
    name = "bi_axe_leader"

    def __init__(self, config: BiAxeLeaderConfig):
        super().__init__(config)
        self.config = config

        left_cfg = axeLeaderConfig(
            id=f"{config.id}_left" if config.id else None,
            calibration_dir=config.calibration_dir,
            port=config.left_arm_port,
            use_degrees=config.use_degrees,
            arm=config.arm,
            has_imu=config.has_imu,
            handle_source=config.handle_source,
            handle_device_name=config.left_handle_device_name,
            imu_port=config.imu_port,
            imu_ip=config.imu_ip,
            transport="none",
            udp_target_ip=config.udp_target_ip,
            udp_target_port=config.udp_target_port,
            udp_pose_only=config.udp_pose_only,
            udp_print_packets=config.udp_print_packets,
            require_arm_key=config.require_arm_key,
            arm_key=config.arm_key,
            arm_toggle_source=config.arm_toggle_source,
            arm_toggle_cooldown_s=config.arm_toggle_cooldown_s,
            position_deadband_m=config.position_deadband_m,
            twist_deadband_m=config.twist_deadband_m,
        )
        right_cfg = axeLeaderConfig(
            id=f"{config.id}_right" if config.id else None,
            calibration_dir=config.calibration_dir,
            port=config.right_arm_port,
            use_degrees=config.use_degrees,
            arm=config.arm,
            has_imu=config.has_imu,
            handle_source=config.handle_source,
            handle_device_name=config.right_handle_device_name,
            imu_port=config.imu_port,
            imu_ip=config.imu_ip,
            transport="none",
            udp_target_ip=config.udp_target_ip,
            udp_target_port=config.udp_target_port,
            udp_pose_only=config.udp_pose_only,
            udp_print_packets=config.udp_print_packets,
            require_arm_key=config.require_arm_key,
            arm_key=config.arm_key,
            arm_toggle_source=config.arm_toggle_source,
            arm_toggle_cooldown_s=config.arm_toggle_cooldown_s,
            position_deadband_m=config.position_deadband_m,
            twist_deadband_m=config.twist_deadband_m,
        )

        self.left_arm = axeLeader(left_cfg)
        self.right_arm = axeLeader(right_cfg)

        self._bi_udp = BiAxeUDPTransport(
            ip=config.udp_target_ip,
            port=config.udp_target_port,
            pose_only=config.udp_pose_only,
            print_packets=config.udp_print_packets,
        )

        # Route both single-arm publishers to one shared tagged UDP endpoint.
        self.left_arm._transport = self._bi_udp.make_arm_transport("L")
        self.right_arm._transport = self._bi_udp.make_arm_transport("R")

    @cached_property
    def action_features(self) -> dict[str, type]:
        return {f"left_{key}": value for key, value in self.left_arm.action_features.items()} | {
            f"right_{key}": value for key, value in self.right_arm.action_features.items()
        }

    @cached_property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self.left_arm.is_connected and self.right_arm.is_connected

    def connect(self, calibrate: bool = True) -> None:
        self.left_arm.connect(calibrate)
        self.right_arm.connect(calibrate)

    @property
    def is_calibrated(self) -> bool:
        return self.left_arm.is_calibrated and self.right_arm.is_calibrated

    def calibrate(self) -> None:
        self.left_arm.calibrate()
        self.right_arm.calibrate()

    def configure(self) -> None:
        self.left_arm.configure()
        self.right_arm.configure()

    def setup_motors(self) -> None:
        self.left_arm.setup_motors()
        self.right_arm.setup_motors()

    def get_action(self) -> dict[str, float]:
        action_dict = {}
        left_action = self.left_arm.get_action()
        action_dict.update({f"left_{key}": value for key, value in left_action.items()})

        right_action = self.right_arm.get_action()
        action_dict.update({f"right_{key}": value for key, value in right_action.items()})
        return action_dict

    def send_feedback(self, feedback: dict[str, float]) -> None:
        left_feedback = {
            key.removeprefix("left_"): value for key, value in feedback.items() if key.startswith("left_")
        }
        right_feedback = {
            key.removeprefix("right_"): value for key, value in feedback.items() if key.startswith("right_")
        }

        if left_feedback:
            self.left_arm.send_feedback(left_feedback)
        if right_feedback:
            self.right_arm.send_feedback(right_feedback)

    def disconnect(self) -> None:
        self.left_arm.disconnect()
        self.right_arm.disconnect()
        self._bi_udp.shutdown()
