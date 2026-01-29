#!/usr/bin/env python

#mimic mathias Desrochers eltopchi1@gmail.com

from dataclasses import dataclass

from ..config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("axe4_leader")
@dataclass
class axe4LeaderConfig(TeleoperatorConfig):
    # Port to connect to the arm
    port: str

    use_degrees: bool = True
    imu_port: int = 5000
    imu_ip: str = "127.0.0.1"

