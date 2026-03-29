#!/usr/bin/env python

#mimic mathias Desrochers eltopchi1@gmail.com

from dataclasses import dataclass

from ..config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("axe3_leader")
@dataclass
class axe3LeaderConfig(TeleoperatorConfig):
    # Port to connect to the arm
    port: str

    use_degrees: bool = True
    handle_source: str = "ble"
    handle_device_name: str = "AXE3_left"
    imu_port: int = 5000
    imu_ip: str = "127.0.0.1"

