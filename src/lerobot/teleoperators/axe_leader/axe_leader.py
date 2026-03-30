#!/usr/bin/env python
"""AXE modular leader teleoperator (standalone implementation)."""

import logging
import os
import select
import sys
import time

import numpy as np

try:
    import termios
    import tty
except Exception:
    termios = None
    tty = None

from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from ..teleoperator import Teleoperator
from .config_axe_leader import axeLeaderConfig
from .fk import forward_kinematics, load_motor_cfg, motor_deg_to_angles
from .handle_reader import HandleReader, HandleState, LegacyIMUReader
from .transport import create_transport

logger = logging.getLogger(__name__)


class _ArmingKeyReader:
    """Non-blocking single-key reader for terminal arming toggle."""

    def __init__(self, key: str):
        self._key = (key or "t")[0]
        self._enabled = sys.stdin.isatty() and os.name != "nt" and termios is not None and tty is not None
        self._fd = None
        self._old_term = None
        if self._enabled:
            try:
                self._fd = sys.stdin.fileno()
                self._old_term = termios.tcgetattr(self._fd)
                tty.setcbreak(self._fd)
            except Exception as e:
                logger.warning(f"Failed to enable arm key reader: {e}")
                self._enabled = False
                self._fd = None
                self._old_term = None

    @property
    def enabled(self) -> bool:
        return self._enabled

    def poll_toggle(self) -> bool:
        if not self._enabled or self._fd is None:
            return False
        try:
            ready, _, _ = select.select([sys.stdin], [], [], 0.0)
            if not ready:
                return False
            ch = sys.stdin.read(1)
            return ch.lower() == self._key.lower()
        except Exception:
            return False

    def close(self) -> None:
        if self._enabled and self._fd is not None and self._old_term is not None:
            try:
                termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_term)
            except Exception:
                pass
        self._enabled = False
        self._fd = None
        self._old_term = None


class axeLeader(Teleoperator):
    """Configurable AXE leader teleoperator supporting 3 or 4 motor joints."""

    config_class = axeLeaderConfig
    name = "axe_leader"

    def __init__(self, config: axeLeaderConfig):
        super().__init__(config)
        self.config: axeLeaderConfig = config

        norm_mode = MotorNormMode.DEGREES if config.use_degrees else MotorNormMode.RANGE_M100_100

        motors = {
            name: Motor(mid, model, norm_mode)
            for name, mid, model in zip(config.joint_names, config.motor_ids, config.motor_models, strict=False)
        }

        filtered_calibration = {
            name: calib for name, calib in self.calibration.items() if name in motors
        }
        self.calibration = filtered_calibration
        self.bus = FeetechMotorsBus(
            port=self.config.port,
            motors=motors,
            calibration=self.calibration,
        )

        self._transport_kind = config.transport
        self._transport = self._create_transport_with_fallback(self._transport_kind)

        if config.has_imu:
            if config.handle_source == "ble":
                    self._handle = HandleReader(
                        device_name=config.handle_device_name,
                        char_uuid_angle=config.resolved_handle_ble_uuids["angle"],
                        char_uuid_quat=config.resolved_handle_ble_uuids["quat"],
                        char_uuid_joy=config.resolved_handle_ble_uuids["joy"],
                    )
            else:
                self._handle = LegacyIMUReader(ip=config.imu_ip, port=config.imu_port)
        else:
            self._handle: HandleReader | LegacyIMUReader | None = None

        self._motor_cfg = load_motor_cfg(self.config)

        self._home_xyz = None
        self._prev_xyz = None
        self._cmd_xyz = None
        self._armed = not bool(getattr(self.config, "require_arm_key", False))
        self._arm_reader = None
        if getattr(self.config, "require_arm_key", False):
            self._arm_reader = _ArmingKeyReader(getattr(self.config, "arm_key", "t"))
            if getattr(self.config, "arm_toggle_source", "keyboard") == "keyboard" and not self._arm_reader.enabled:
                logger.warning("Arm-key gating requested but stdin is not interactive; teleop will start armed.")
                self._armed = True

        self._last_arm_toggle_t = 0.0
        self._prev_arm_button_pressed = False

    def _create_transport_with_fallback(self, kind: str):
        try:
            return create_transport(
                kind,
                udp_ip=self.config.udp_target_ip,
                udp_port=self.config.udp_target_port,
                udp_pose_only=getattr(self.config, "udp_pose_only", True),
                udp_print_packets=getattr(self.config, "udp_print_packets", False),
                ros2_node_name=self.config.ros2_node_name,
                ros2_topic_prefix=self.config.ros2_topic_prefix,
            )
        except ImportError as e:
            if kind != "ros2":
                raise
            logger.warning(
                "ROS2 transport requested but ROS2 Python packages are unavailable; "
                "falling back to UDP transport for this session."
            )
            self._transport_kind = "udp"
            return create_transport(
                "udp",
                udp_ip=self.config.udp_target_ip,
                udp_port=self.config.udp_target_port,
                udp_pose_only=getattr(self.config, "udp_pose_only", True),
                udp_print_packets=getattr(self.config, "udp_print_packets", False),
            )

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
        # `lerobot-calibrate` calls connect(calibrate=False) before device.calibrate().
        # In that flow, BLE/IMU is not needed and can add noisy reconnect logs.
        if self._handle is not None and calibrate:
            self._handle.start()

        if self._arm_reader and self._arm_reader.enabled and self.config.arm_toggle_source == "keyboard":
            logger.info(
                f"Teleop DISARMED. Press '{self.config.arm_key}' to arm and capture home; "
                f"press again to disarm/pause."
            )
        elif self.config.arm_toggle_source != "keyboard":
            logger.info(
                f"Teleop arm toggle source set to '{self.config.arm_toggle_source}'. "
                "Press configured handle/xbox button to arm/disarm."
            )

        logger.info(
            f"{self} connected.  joints={self.config.num_joints}  "
            f"handle_source={self.config.handle_source if self._handle is not None else 'none'}  "
            f"transport={self._transport_kind}"
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
        try:
            homing_offsets = self.bus.set_half_turn_homings()
        except Exception as e:
            expected = [(name, m.id) for name, m in self.bus.motors.items()]
            raise ConnectionError(
                "Motor communication failed while writing homing offsets. "
                f"Expected motors (name,id): {expected}. "
                "Check power, USB/TTL wiring, baud/port, and motor IDs. "
                "If IDs are unknown/mismatched, run setup_motors() first. "
                f"Original error: {e}"
            ) from e

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

    def _toggle_button_pressed(self, hs: HandleState) -> bool:
        source = getattr(self.config, "arm_toggle_source", "keyboard")
        if source in {"handle_sw", "xbox_a"}:
            return int(hs.sw) == 1
        if source in {"handle_sw2", "xbox_b"}:
            return int(hs.sw2) == 1
        return False

    def _poll_arm_toggle(self, hs: HandleState) -> bool:
        now = time.perf_counter()
        cooldown_s = float(getattr(self.config, "arm_toggle_cooldown_s", 0.3))
        if now - self._last_arm_toggle_t < cooldown_s:
            return False

        source = getattr(self.config, "arm_toggle_source", "keyboard")
        if source == "keyboard":
            return bool(self._arm_reader and self._arm_reader.poll_toggle())

        pressed = self._toggle_button_pressed(hs)
        edge = pressed and not self._prev_arm_button_pressed
        self._prev_arm_button_pressed = pressed
        if edge:
            self._last_arm_toggle_t = now
        return edge

    def get_action(self) -> dict[str, float]:
        """Read motors, run dynamic FK, publish pose/twist/imu/joy, return action dict."""
        start = time.perf_counter()
        motor_degrees = self.bus.sync_read("Present_Position")
        q = motor_deg_to_angles(motor_degrees, self._motor_cfg)
        raw_xyz, _ = forward_kinematics(
            q,
            link_lengths_m=self.config.link_lengths_m,
        )
        raw_xyz = np.asarray(raw_xyz, dtype=np.float32)

        hs = self._handle.state if self._handle is not None else HandleState()

        if self._poll_arm_toggle(hs):
            self._armed = not self._armed
            if self._armed:
                self._home_xyz = raw_xyz.copy()
                self._cmd_xyz = np.zeros(3, dtype=np.float32)
                self._prev_xyz = self._cmd_xyz.copy()
                logger.info("Teleop ARMED. Home captured from current pose.")
            else:
                self._home_xyz = None
                self._cmd_xyz = np.zeros(3, dtype=np.float32)
                self._prev_xyz = self._cmd_xyz.copy()
                logger.info("Teleop DISARMED. Publishing zero position/twist.")

            self._last_arm_toggle_t = time.perf_counter()

        if not self._armed:
            rel_xyz = np.zeros(3, dtype=np.float32)
            delta_xyz = np.zeros(3, dtype=np.float32)
            self._home_xyz = raw_xyz.copy()
            self._cmd_xyz = rel_xyz.copy()
            self._prev_xyz = rel_xyz.copy()
        else:
            if self._home_xyz is None:
                self._home_xyz = raw_xyz.copy()
                self._cmd_xyz = np.zeros(3, dtype=np.float32)
                self._prev_xyz = self._cmd_xyz.copy()

            raw_rel = raw_xyz - self._home_xyz
            if self._cmd_xyz is None:
                self._cmd_xyz = raw_rel.copy()
                self._prev_xyz = self._cmd_xyz.copy()

            pos_db = float(getattr(self.config, "position_deadband_m", 0.0))
            if np.max(np.abs(raw_rel - self._cmd_xyz)) >= pos_db:
                self._cmd_xyz = raw_rel.copy()
            rel_xyz = self._cmd_xyz.copy()

            prev_cmd = self._prev_xyz.copy()
            delta_xyz = rel_xyz - prev_cmd
            tw_db = float(getattr(self.config, "twist_deadband_m", 0.0))
            delta_xyz[np.abs(delta_xyz) < tw_db] = 0.0
            self._prev_xyz = rel_xyz.copy()

        self._transport.publish_eef_pose(
            rel_xyz[0],
            rel_xyz[1],
            rel_xyz[2],
            hs.qw,
            hs.qx,
            hs.qy,
            hs.qz,
        )
        self._transport.publish_eef_pose_absolute(
            raw_xyz[0],
            raw_xyz[1],
            raw_xyz[2],
            hs.qw,
            hs.qx,
            hs.qy,
            hs.qz,
        )
        self._transport.publish_eef_position(rel_xyz[0], rel_xyz[1], rel_xyz[2])
        self._transport.publish_eef_position_absolute(raw_xyz[0], raw_xyz[1], raw_xyz[2])
        self._transport.publish_eef_twist(delta_xyz[0], delta_xyz[1], delta_xyz[2], 0.0, 0.0, 0.0)
        self._transport.publish_imu(
            hs.qw,
            hs.qx,
            hs.qy,
            hs.qz,
            hs.roll,
            hs.pitch,
            hs.yaw,
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
        if self._handle is not None:
            self._handle.stop()
        self._transport.shutdown()
        if self._arm_reader:
            self._arm_reader.close()
        logger.info(f"{self} disconnected.")

    def compute_forward_kinematics(self, joints_degrees: dict[str, float]) -> tuple[float, float, float]:
        """Motor degrees -> (x, y, z) in meters for active joints."""
        q = motor_deg_to_angles(joints_degrees, self._motor_cfg)
        eef, _ = forward_kinematics(
            q,
            link_lengths_m=self.config.link_lengths_m,
        )
        return float(eef[0]), float(eef[1]), float(eef[2])
