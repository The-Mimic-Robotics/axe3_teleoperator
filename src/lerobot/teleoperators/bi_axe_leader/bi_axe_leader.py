#!/usr/bin/env python

import logging
from functools import cached_property

from ..teleoperator import Teleoperator
from ..axe_leader.axe_leader import axeLeader
from ..axe_leader.config_axe_leader import axeLeaderConfig
from .config_bi_axe_leader import BiAxeLeaderConfig

logger = logging.getLogger(__name__)


class BiAxeLeader(Teleoperator):
    """Bimanual AXE leader teleoperator using two independent axeLeader instances."""

    config_class = BiAxeLeaderConfig
    name = "bi_axe_leader"

    def __init__(self, config: BiAxeLeaderConfig):
        super().__init__(config)
        self.config = config

        left_cfg_dict = {
            **config.shared,
            **config.left_arm,
            "id": config.left_arm.get("id", f"{config.id}_left" if config.id else None),
            "calibration_dir": config.calibration_dir,
        }
        right_cfg_dict = {
            **config.shared,
            **config.right_arm,
            "id": config.right_arm.get("id", f"{config.id}_right" if config.id else None),
            "calibration_dir": config.calibration_dir,
        }

        self.left_arm = axeLeader(axeLeaderConfig(**left_cfg_dict))
        self.right_arm = axeLeader(axeLeaderConfig(**right_cfg_dict))

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
