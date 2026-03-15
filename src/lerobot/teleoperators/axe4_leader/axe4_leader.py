#!/usr/bin/env python

# MISC Robotics - Achal Patel achalypatel3403@gmail.com
# MISC Robotics - Mathias Desrochers eltopchi1@gmail.com

import logging
import time
import numpy as np

from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import (
    FeetechMotorsBus,
    OperatingMode,
)
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from ..teleoperator import Teleoperator
from .config_axe4_leader import axe4LeaderConfig
from .handle_reader import HandleReader, LegacyIMUReader
from .transport import create_transport
from .fk import load_motor_cfg, motor_deg_to_angles, forward_kinematics

logger = logging.getLogger(__name__)


class axe4Leader(Teleoperator):
    """
    AXE4 leader: 4× STS3215 + BLE handle (IMU). Pose from planar FK + handle quaternion.
    Publishes: eef_pose, eef_position, eef_twist (deltas), imu, joy.
    """

    config_class = axe4LeaderConfig
    name = "axe4_leader"

    def __init__(self, config: axe4LeaderConfig):
        super().__init__(config)
        self.config = config
        norm_mode = MotorNormMode.DEGREES if config.use_degrees else MotorNormMode.RANGE_M100_100

        self.bus = FeetechMotorsBus(
            port=self.config.port,
            motors={
                "shoulder_pan": Motor(1, "sts3215", norm_mode),
                "shoulder_lift": Motor(2, "sts3215", norm_mode),
                "elbow_flex": Motor(3, "sts3215", norm_mode),
                "elbow_super_flex": Motor(4, "sts3215", norm_mode),
            },
            calibration=self.calibration,
        )

        if config.handle_source == "ble":
            self._handle = HandleReader(device_name=config.handle_device_name)
        else:
            self._handle = LegacyIMUReader(ip=config.imu_ip, port=config.imu_port)

        self._transport = create_transport(
            config.transport,
            udp_ip=config.udp_target_ip,
            udp_port=config.udp_target_port,
            udp_pose_only=getattr(config, "udp_pose_only", True),
        )

        self._motor_cfg = load_motor_cfg()
        self._home_xyz = None
        self._prev_xyz = None

    # ------------------------------------------------------------------
    # Teleoperator interface
    # ------------------------------------------------------------------
    @property
    def action_features(self) -> dict[str, type]:
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
                "Mismatch between calibration values in the motor and the calibration file "
                "or no calibration file found"
            )
            self.calibrate()

        self.configure()
        self._handle.start()
        logger.info(
            f"{self} connected.  handle_source={self.config.handle_source}  "
            f"transport={self.config.transport}"
        )

    @property
    def is_calibrated(self) -> bool:
        return self.bus.is_calibrated

    def calibrate(self) -> None:
        if self.calibration:
            user_input = input(
                f"Press ENTER to use provided calibration file associated with the id {self.id}, "
                "or type 'c' and press ENTER to run calibration: "
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

    # ------------------------------------------------------------------
    # Main action loop
    # ------------------------------------------------------------------
    def get_action(self) -> dict[str, float]:
        start = time.perf_counter()
        motor_degrees = self.bus.sync_read("Present_Position")
        q = motor_deg_to_angles(motor_degrees, self._motor_cfg)
        raw_xyz, _ = forward_kinematics(*q)
        raw_xyz = np.asarray(raw_xyz, dtype=np.float32)

        if self._home_xyz is None:
            self._home_xyz = raw_xyz.copy()
            self._prev_xyz = raw_xyz.copy()

        rel_xyz = raw_xyz - self._home_xyz
        delta_xyz = raw_xyz - self._prev_xyz
        self._prev_xyz = raw_xyz.copy()

        hs = self._handle.state

        self._transport.publish_eef_pose(
            rel_xyz[0], rel_xyz[1], rel_xyz[2],
            hs.qw, hs.qx, hs.qy, hs.qz,
        )
        self._transport.publish_eef_position(rel_xyz[0], rel_xyz[1], rel_xyz[2])
        self._transport.publish_eef_twist(delta_xyz[0], delta_xyz[1], delta_xyz[2], 0.0, 0.0, 0.0)
        self._transport.publish_imu(
            hs.qw, hs.qx, hs.qy, hs.qz,
            hs.roll, hs.pitch, hs.yaw,
        )
        self._transport.publish_buttons(hs.sw, hs.sw2, hs.joy_x, hs.joy_y, hs.joy_z)

        action = {
            "x": float(rel_xyz[0]),
            "y": float(rel_xyz[1]),
            "z": float(rel_xyz[2]),
            "pos_x": float(rel_xyz[0]),
            "pos_y": float(rel_xyz[1]),
            "pos_z": float(rel_xyz[2]),
            "imu.qw": hs.qw,
            "imu.qx": hs.qx,
            "imu.qy": hs.qy,
            "imu.qz": hs.qz,
        }
        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read action: {dt_ms:.1f}ms")
        return action

    def send_feedback(self, feedback: dict[str, float]) -> None:
        raise NotImplementedError

    def disconnect(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        self.bus.disconnect()
        self._handle.stop()
        self._transport.shutdown()
        logger.info(f"{self} disconnected.")

    def compute_forward_kinematics(self, joints_degrees: dict[str, float]) -> tuple[float, float, float]:
        """Motor degrees -> (x, y, z) in m. Frame: X fwd, Y left, Z up. Uses planar FK + motor_cfg."""
        q = motor_deg_to_angles(joints_degrees, self._motor_cfg)
        eef, _ = forward_kinematics(*q)
        return float(eef[0]), float(eef[1]), float(eef[2])
