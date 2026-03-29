#!/usr/bin/env python

from dataclasses import dataclass, field

from ..config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("bi_axe_leader")
@dataclass
class BiAxeLeaderConfig(TeleoperatorConfig):
    left_arm: dict[str, object] = field(
        default_factory=lambda: {
            "port": "/dev/ttyACM0",
            "handle_device_name": "AXE3_left",
            "ros2_topic_prefix": "axe_left",
        }
    )
    right_arm: dict[str, object] = field(
        default_factory=lambda: {
            "port": "/dev/ttyACM1",
            "handle_device_name": "AXE3_right",
            "ros2_topic_prefix": "axe_right",
        }
    )
    shared: dict[str, object] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if "port" not in self.left_arm:
            raise ValueError("left_arm must include 'port'")
        if "port" not in self.right_arm:
            raise ValueError("right_arm must include 'port'")
