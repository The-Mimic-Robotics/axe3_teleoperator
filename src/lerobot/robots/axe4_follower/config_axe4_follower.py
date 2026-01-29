#!/usr/bin/env python

#mimic mathias Desrochers eltopchi1@gmail.com

from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig
from typing import Optional

from ..config import RobotConfig


@RobotConfig.register_subclass("axe4_follower")
@dataclass
class axe4FollowerConfig(RobotConfig):


    udp_ip: str = "127.0.0.1"
    udp_port: int = 5005

    # Port to connect to the arm
    port: str | None = None

    disable_torque_on_disconnect: bool = True

    # `max_relative_target` limits the magnitude of the relative positional target vector for safety purposes.
    # Set this to a positive scalar to have the same value for all motors, or a dictionary that maps motor
    # names to the max_relative_target value for that motor.
    max_relative_target: float | dict[str, float] | None = None

    # cameras
    cameras: dict[str, CameraConfig] = field(default_factory=dict)

    # Set to `True` for backward compatibility with previous policies/dataset
    use_degrees: bool = True
