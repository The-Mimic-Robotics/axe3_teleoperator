#!/usr/bin/env python

# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import logging
import time
import socket
import struct
import numpy as np
import math

from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import (
    FeetechMotorsBus,
    OperatingMode,
)
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from ..teleoperator import Teleoperator
from .config_axe3_leader import axe3LeaderConfig

logger = logging.getLogger(__name__)


class axe3Leader(Teleoperator):
    """
    SO-101 Leader Arm designed by TheRobotStudio and Hugging Face.
    Modified to include IMU data via UDP.
    """

    config_class = axe3LeaderConfig
    name = "axe3_leader"

    def __init__(self, config: axe3LeaderConfig):
        super().__init__(config)
        self.config = config
        norm_mode_body = MotorNormMode.DEGREES if config.use_degrees else MotorNormMode.RANGE_M100_100
        
        # 1. Setup Motors
        self.bus = FeetechMotorsBus(
            port=self.config.port,
            motors={
                "shoulder_pan": Motor(1, "sts3215", norm_mode_body),
                "shoulder_lift": Motor(2, "sts3215", norm_mode_body),
                "elbow_flex": Motor(3, "sts3215", norm_mode_body),
            },
            calibration=self.calibration,
        )

        # 2. Setup UDP for IMU
        # We bind to the IP/Port to listen for data from the C++ script
        self.imu_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.imu_sock.bind((self.config.imu_ip, self.config.imu_port))
        self.imu_sock.setblocking(False) # Non-blocking to avoid freezing the robot
        
        # Default IMU state (Identity Quaternion w=1, x=0, y=0, z=0)
        self.latest_imu_data = [1.0, 0.0, 0.0, 0.0]

    @property
    def action_features(self) -> dict[str, type]:
        # We define the 3 motors AND the 4 quaternion values
        features = {f"{motor}.pos": float for motor in self.bus.motors}
        features["imu.qw"] = float
        features["imu.qx"] = float
        features["imu.qy"] = float
        features["imu.qz"] = float
        return features

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        self.bus.connect()
        if not self.is_calibrated and calibrate:
            logger.info(
                "Mismatch between calibration values in the motor and the calibration file or no calibration file found"
            )
            self.calibrate()

        self.configure()
        logger.info(f"{self} connected. Listening for IMU on {self.config.imu_ip}:{self.config.imu_port}")

    @property
    def is_calibrated(self) -> bool:
        return self.bus.is_calibrated

    def calibrate(self) -> None:
        if self.calibration:
            user_input = input(
                f"Press ENTER to use provided calibration file associated with the id {self.id}, or type 'c' and press ENTER to run calibration: "
            )
            if user_input.strip().lower() != "c":
                logger.info(f"Writing calibration file associated with the id {self.id} to the motors")
                self.bus.write_calibration(self.calibration)
                return

        logger.info(f"\nRunning calibration of {self}")
        self.bus.disable_torque()
        for motor in self.bus.motors:
            self.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)

        input(f"Move {self} to the middle of its range of motion and press ENTER....")
        homing_offsets = self.bus.set_half_turn_homings()

        print(
            "Move all joints sequentially through their entire ranges "
            "of motion.\nRecording positions. Press ENTER to stop..."
        )
        range_mins, range_maxes = self.bus.record_ranges_of_motion()

        self.calibration = {}
        for motor, m in self.bus.motors.items():
            self.calibration[motor] = MotorCalibration(
                id=m.id,
                drive_mode=0,
                homing_offset=homing_offsets[motor],
                range_min=range_mins[motor],
                range_max=range_maxes[motor],
            )

        self.bus.write_calibration(self.calibration)
        self._save_calibration()
        print(f"Calibration saved to {self.calibration_fpath}")

    def configure(self) -> None:
        self.bus.disable_torque()
        self.bus.configure_motors()
        for motor in self.bus.motors:
            self.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)

    def setup_motors(self) -> None:
        for motor in reversed(self.bus.motors):
            input(f"Connect the controller board to the '{motor}' motor only and press enter.")
            self.bus.setup_motor(motor)
            print(f"'{motor}' motor id set to {self.bus.motors[motor].id}")

    def get_action(self) -> dict[str, float]:
        start = time.perf_counter()
        
        # 1. Read Motors
        raw_action = self.bus.sync_read("Present_Position")
        # action = {f"{motor}.pos": val for motor, val in action.items()}

        x, y, z = self.compute_forward_kinematics(raw_action)
        
        # 2. Read IMU (Drain the buffer to get the latest packet)
        try:
            while True:
                # 4 floats = 16 bytes
                data, _ = self.imu_sock.recvfrom(16)
                # Unpack 4 floats (f f f f)
                self.latest_imu_data = struct.unpack('4f', data)
        except BlockingIOError:
            # No more data in buffer, use the latest known value
            pass
        except Exception as e:
            logger.warning(f"UDP Read Error: {e}")

        # 3. Merge Data
        # action["imu.qw"] = self.latest_imu_data[0]
        # action["imu.qx"] = self.latest_imu_data[1]
        # action["imu.qy"] = self.latest_imu_data[2]
        # action["imu.qz"] = self.latest_imu_data[3]

        action = {
            "x": x,
            "y": y,
            "z": z,
            "imu.qw": self.latest_imu_data[0],
            "imu.qx": self.latest_imu_data[1],
            "imu.qy": self.latest_imu_data[2],
            "imu.qz": self.latest_imu_data[3],
        }



        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read action: {dt_ms:.1f}ms")
        return action

    def send_feedback(self, feedback: dict[str, float]) -> None:
        # TODO(rcadene, aliberts): Implement force feedback
        raise NotImplementedError

    def disconnect(self) -> None:
        if not self.is_connected:
            DeviceNotConnectedError(f"{self} is not connected.")

        self.bus.disconnect()
        if hasattr(self, 'imu_sock'):
            self.imu_sock.close()
            
        logger.info(f"{self} disconnected.")



    
    def compute_forward_kinematics(self, joints_degrees):
            """
            Computes the X, Y, Z position of the end effector.
            Input: Dictionary with 'shoulder_pan', 'shoulder_lift', 'elbow_flex' in DEGREES.
            Output: (x, y, z) tuple in METERS.
            """
            
            # 1. Convert to Radians
            # Note: We negate some angles if the rotation direction is opposite to standard right-hand rule.
            # usually: Pan (+) = Left, Lift (+) = Down/Forward, Elbow (+) = Down/In
            q1 = np.radians(joints_degrees["shoulder_pan"])
            q2 = np.radians(joints_degrees["shoulder_lift"])
            q3 = np.radians(joints_degrees["elbow_flex"])

            v1 = np.array([0.01, 0, 0.02]) # x,y,z
            v2 = np.array([0.05, 0, 0.35])
            v3 = np.array([0.35, 0, 0])


            p1 = v1
            p2 = rot_y(q2) @ v2
            p3 = rot_y(q2 + q3) @ v3

            arm_in_2d_plane = p1 + p2 + p3

            final_pos = rot_z(q1) @ arm_in_2d_plane

            # 4. Apply Pan Rotation (Joint 1)
            x_final = final_pos[0]
            y_final = final_pos[1]
            z_final = final_pos[2]

            return x_final, y_final, z_final
    
def rot_y(ang):
    #rotate vector arrnd y axis
    c = np.cos(ang)
    s = np.sin(ang)
    return np.array([
        [c, 0, s],
        [0, 1, 0],
        [-s, 0, c]
    ])

def rot_z(ang):
    #rotate vector arrnd y axis
    c = np.cos(ang)
    s = np.sin(ang)
    return np.array([
        [c, -s, 0],
        [s, c, 0],
        [0, 0, 1]
    ])