#!/usr/bin/env python

import logging
import struct
import socket
from typing import Any

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from ..robot import Robot 
from .config_axe3_follower import axe3FollowerConfig

logger = logging.getLogger(__name__)

class axe3Follower(Robot):
    """
    A Proxy Robot that sends actions over UDP instead of controlling physical motors.
    """
    config_class = axe3FollowerConfig
    name = "axe3_udp_follower"

    def __init__(self, config: axe3FollowerConfig):
        self.config = config
        self.cameras = make_cameras_from_configs(config.cameras)
        
        # 1. UDP Setup using Config
        self.sock = None
        self.address = (config.udp_ip, config.udp_port)
        self._is_connected = False
        self.id = "udp_follower"
        
        # 2. State tracking
        self._current_state = {
            "x": 0.0, "y": 0.0, "z": 0.0,
            "imu.qw": 1.0, "imu.qx": 0.0, "imu.qy": 0.0, "imu.qz": 0.0,
        }

    @property
    def observation_features(self) -> dict[str, type | tuple]:
        features = {key: float for key in self._current_state.keys()}
        cam_features = {
            cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3) 
            for cam in self.cameras
        }
        return {**features, **cam_features}

    @property
    def action_features(self) -> dict[str, type]:
        return {key: float for key in self._current_state.keys()}

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        logger.info(f"Connecting to UDP Target at {self.address}...")
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._is_connected = True
        
        for cam in self.cameras.values():
            cam.connect()

        logger.info(f"{self} connected (UDP Proxy).")

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        pass

    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        
        obs_dict = self._current_state.copy()
        for cam_key, cam in self.cameras.items():
            obs_dict[cam_key] = cam.async_read()
        return obs_dict

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        # Update internal state
        for key, val in action.items():
            if key in self._current_state:
                self._current_state[key] = float(val)

        # 2. Extract Data for Packet
        # Safety: Default to 0/Identity if keys missing
        x = float(action.get("x", self._current_state["x"]))
        y = float(action.get("y", self._current_state["y"]))
        z = float(action.get("z", self._current_state["z"]))
        
        qw = float(action.get("imu.qw", self._current_state["imu.qw"]))
        qx = float(action.get("imu.qx", self._current_state["imu.qx"]))
        qy = float(action.get("imu.qy", self._current_state["imu.qy"]))
        qz = float(action.get("imu.qz", self._current_state["imu.qz"]))

        # 3. Pack Binary (Little Endian, 7 floats) -> 28 bytes
        try:
            packet = struct.pack('<fffffff', x, y, z, qw, qx, qy, qz)
            self.sock.sendto(packet, self.address)
        except Exception as e:
            logger.error(f"Failed to send UDP packet: {e}")

        return action
    
    def configure(self) -> None:
        pass

    def disconnect(self):
        if self.sock:
            self.sock.close()
        for cam in self.cameras.values():
            cam.disconnect()
        self._is_connected = False
        logger.info(f"{self} disconnected.")