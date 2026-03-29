#!/usr/bin/env python
"""
Configuration for the AXE4 leader teleoperator.

Port, handle source (BLE vs UDP), transport (ros2/udp/none), and UDP/ROS2 options.
"""

# MISC Robotics - Achal Patel achalypatel3403@gmail.com
# MISC Robotics - Mathias Desrochers eltopchi1@gmail.com

from dataclasses import dataclass

from ..config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("axe4_leader")
@dataclass
class axe4LeaderConfig(TeleoperatorConfig):
    """Config for axe4 leader: port, handle_source (ble/udp), transport (ros2/udp/none), UDP/ROS2 params."""

    # Serial port for the Feetech servo driver
    port: str = "/dev/ttyACM0"

    use_degrees: bool = True

    # --- Handle / IMU source ---
    # "ble"  : BLE handle reader (default)
    handle_source: str = "ble"
    handle_device_name: str = "AXE3_left"

    # "udp"  : legacy UDP IMU reader (C++ imu_udp bridge)
    imu_port: int = 5000
    imu_ip: str = "127.0.0.1"

    # --- Transport (how pose data leaves the leader) ---
    # "ros2" : publish to ROS 2 topics (default)
    # "udp"  : send packed floats over UDP
    # "none" : data only returned via get_action()
    transport: str = "ros2"
    udp_target_ip: str = "127.0.0.1"
    udp_target_port: int = 5005
    # When True (default), UDP sends only the legacy 28-byte eef_pose packet (<7f>).
    # When False, non-pose packets are sent with a 1-byte type tag to avoid collisions with
    # receivers that parse raw 28-byte pose packets.
    udp_pose_only: bool = True

    # --- Teleop arming / filtering ---
    # When True, teleop stays disarmed until `arm_key` is pressed in the terminal running
    # lerobot-teleoperate. Arming captures current FK pose as home reference.
    require_arm_key: bool = True
    arm_key: str = " "
    # Ignore tiny FK changes around rest so robot does not drift while leader is stationary.
    position_deadband_m: float = 0.003
    # Zero very small per-step deltas to avoid unintended twist/velocity nudges.
    twist_deadband_m: float = 0.001