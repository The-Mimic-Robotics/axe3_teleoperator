#!/usr/bin/env python3
"""
SO101 leader -> RViz (RobotModel) bridge.

Goal: you only have a real SO101 *leader*. This script reads leader joints and publishes ROS topics so RViz can
render the SO101 URDF (RobotModel) and show measured EEF pose/twist.

What it does:
  - Launches `robot_state_publisher` with the SO101 URDF (robot_description)
  - Launches `rviz2` with a pre-made config that already shows RobotModel + TF (`so101_FK/so101.rviz`)
  - Reads the SO101 leader (Feetech bus) and publishes:
      - /joint_states                    (sensor_msgs/JointState)
      - /so101/eef_pose_measured         (geometry_msgs/PoseStamped)   from FK
      - /so101/eef_twist_measured        (geometry_msgs/TwistStamped)  finite-difference twist from FK

Run:
  conda activate lerobot
  python so101_FK/rviz_control.py --port /dev/ttyACM0 --id blue_leader
"""

import argparse
import subprocess
import time
from pathlib import Path
from typing import Optional
import tempfile
import sys

import numpy as np


SO101_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]


def _resolve_urdf_path(p: str) -> str:
    path = Path(p).expanduser()
    if path.is_file():
        return str(path)
    if path.is_dir():
        for cand in (path / "robot.urdf", path / "so101_new_calib.urdf", path / "so101.urdf"):
            if cand.is_file():
                return str(cand)
        urdfs = sorted(path.glob("*.urdf"))
        if urdfs:
            return str(urdfs[0])
    return str(path)


def _read_text(path: str) -> str:
    with open(path) as f:
        return f.read()


def _device_spec(device: str) -> tuple[str, str]:
    if device != "so101_leader":
        raise ValueError("This script is leader-only. Use --device so101_leader.")
    return "teleoperators", "so101_leader"


def _load_calibration(device: str, arm_id: str):
    """
    Load calibration dict[str, MotorCalibration] if present.
    Preference order:
      1) so101_FK/calibration/{robots|teleoperators}/{device}/{id}.json
      2) ~/.cache/huggingface/lerobot/calibration/{robots|teleoperators}/{device}/{id}.json
    """
    import draccus

    from lerobot.motors.motors_bus import MotorCalibration
    from lerobot.utils.constants import HF_LEROBOT_CALIBRATION

    kind, name = _device_spec(device)
    local_root = Path(__file__).resolve().parent / "calibration"
    local_path = local_root / kind / name / f"{arm_id}.json"
    cache_path = HF_LEROBOT_CALIBRATION / kind / name / f"{arm_id}.json"

    calib_path = local_path if local_path.exists() else cache_path
    if not calib_path.exists():
        return None, (local_path, cache_path)

    with open(calib_path) as f, draccus.config_type("json"):
        calib = draccus.load(dict[str, MotorCalibration], f)
    return calib, calib_path


def _connect_bus(port: str, device: str, arm_id: str):
    from lerobot.motors import Motor, MotorNormMode
    from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode

    motors = {
        "shoulder_pan": Motor(1, "sts3215", MotorNormMode.DEGREES),
        "shoulder_lift": Motor(2, "sts3215", MotorNormMode.DEGREES),
        "elbow_flex": Motor(3, "sts3215", MotorNormMode.DEGREES),
        "wrist_flex": Motor(4, "sts3215", MotorNormMode.DEGREES),
        "wrist_roll": Motor(5, "sts3215", MotorNormMode.DEGREES),
        "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
    }

    calib, calib_path = _load_calibration(device, arm_id)
    bus = FeetechMotorsBus(port=port, motors=motors, calibration=calib)
    bus.connect()

    if calib:
        bus.write_calibration(calib)
        print(f"Calibration loaded: {calib_path}")
    else:
        if isinstance(calib_path, tuple):
            local_path, cache_path = calib_path
            print("[WARN] No calibration found at:")
            print(f"  - {local_path}")
            print(f"  - {cache_path}")
            print("Run: python so101_FK/calibrate.py --device so101_leader --port ... --id ...")

    bus.disable_torque()
    for m in motors:
        bus.write("Operating_Mode", m, OperatingMode.POSITION.value)

    return bus


def main():
    ap = argparse.ArgumentParser(description="SO101 RViz viz + control")
    vendored_default_urdf = (
        Path(__file__).resolve().parent / "SO-ARM100" / "Simulation" / "SO101" / "so101_new_calib.urdf"
    )
    ap.add_argument("--urdf_path", default=str(vendored_default_urdf))
    ap.add_argument("--target_frame", default="gripper_frame_link")
    ap.add_argument("--rate", type=float, default=30.0)

    ap.add_argument("--no-launch_rviz", action="store_true", help="Do not auto-launch rviz2")
    ap.add_argument("--no-launch_state_publisher", action="store_true", help="Do not auto-launch robot_state_publisher")

    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--device", choices=["so101_leader"], default="so101_leader")
    ap.add_argument("--id", required=True, help="Calibration id for the leader")
    ap.add_argument("--publish_prefix", default="/so101", help="Prefix for published topics")
    args = ap.parse_args()

    try:
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import PoseStamped, TwistStamped
        from sensor_msgs.msg import JointState
    except ImportError as e:
        raise SystemExit(
            "ROS2 python libs not available in this environment. "
            "Run inside a ROS2-enabled environment where `rclpy` is importable."
        ) from e

    from lerobot.model.kinematics import RobotKinematics
    from lerobot.utils.rotation import Rotation

    urdf_path = _resolve_urdf_path(args.urdf_path)
    urdf_xml = _read_text(urdf_path)

    rsp_proc: Optional[subprocess.Popen] = None
    rviz_proc: Optional[subprocess.Popen] = None

    if not args.no_launch_state_publisher:
        # Avoid passing huge XML on CLI; write a temporary params yaml.
        # Use wildcard node selector so it works regardless of node name.
        params_yaml = "/**:\n  ros__parameters:\n    robot_description: |\n" + "\n".join(
            f"      {line}" for line in urdf_xml.splitlines()
        )
        tf = tempfile.NamedTemporaryFile("w", prefix="so101_rsp_", suffix=".yaml", delete=False)
        tf.write(params_yaml)
        tf.flush()
        tf.close()
        rsp_proc = subprocess.Popen(
            [
                "ros2",
                "run",
                "robot_state_publisher",
                "robot_state_publisher",
                "--ros-args",
                "--params-file",
                tf.name,
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    if not args.no_launch_rviz:
        rviz_cfg = Path(__file__).resolve().parent / "so101.rviz"
        cmd = ["rviz2"]
        if rviz_cfg.exists():
            cmd += ["-d", str(rviz_cfg)]
        rviz_proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    if not rclpy.ok():
        rclpy.init()

    class So101Viz(Node):
        def __init__(self):
            super().__init__("so101_rviz_control")
            self.pub_js = self.create_publisher(JointState, "/joint_states", 10)
            self.pub_eef_pose = self.create_publisher(PoseStamped, f"{args.publish_prefix}/eef_pose_measured", 10)
            self.pub_eef_twist = self.create_publisher(TwistStamped, f"{args.publish_prefix}/eef_twist_measured", 10)
            self.bus = _connect_bus(args.port, args.device, args.id)

            self.kin = RobotKinematics(
                urdf_path=urdf_path,
                target_frame_name=args.target_frame,
                joint_names=SO101_JOINTS,
            )

            self._last_fk_t = time.perf_counter()
            self._last_fk_t = time.perf_counter()
            self._last_fk_T = None

            self.timer = self.create_timer(1.0 / args.rate, self._tick)

        def _publish_joint_state(self, q_deg: np.ndarray):
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = SO101_JOINTS
            msg.position = np.deg2rad(q_deg).tolist()  # JointState expects radians
            self.pub_js.publish(msg)

        def _read_hw_joints_deg(self) -> np.ndarray:
            # If calibration is missing, normalized reads will throw; fall back to raw.
            try:
                deg = self.bus.sync_read("Present_Position")
            except RuntimeError:
                deg = self.bus.sync_read("Present_Position", normalize=False)
            return np.array([float(deg[n]) for n in SO101_JOINTS], dtype=float)

        def _publish_fk(self, q_deg: np.ndarray):
            # Publish measured EEF pose + twist from FK.
            T = self.kin.forward_kinematics(q_deg)
            now = time.perf_counter()
            dt = now - self._last_fk_t
            self._last_fk_t = now

            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = "base_link"
            pose.pose.position.x = float(T[0, 3])
            pose.pose.position.y = float(T[1, 3])
            pose.pose.position.z = float(T[2, 3])
            quat = Rotation.from_matrix(T[:3, :3]).as_quat()  # [x,y,z,w]
            pose.pose.orientation.x = float(quat[0])
            pose.pose.orientation.y = float(quat[1])
            pose.pose.orientation.z = float(quat[2])
            pose.pose.orientation.w = float(quat[3])
            self.pub_eef_pose.publish(pose)

            twist = TwistStamped()
            twist.header.stamp = pose.header.stamp
            twist.header.frame_id = "base_link"
            if self._last_fk_T is not None and dt > 0:
                dp = T[:3, 3] - self._last_fk_T[:3, 3]
                v = dp / dt
                dR = self._last_fk_T[:3, :3].T @ T[:3, :3]
                w = Rotation.from_matrix(dR).as_rotvec() / dt
                twist.twist.linear.x = float(v[0])
                twist.twist.linear.y = float(v[1])
                twist.twist.linear.z = float(v[2])
                twist.twist.angular.x = float(w[0])
                twist.twist.angular.y = float(w[1])
                twist.twist.angular.z = float(w[2])
            self._last_fk_T = T
            self.pub_eef_twist.publish(twist)

        def _tick(self):
            q = self._read_hw_joints_deg()
            self._publish_joint_state(q)
            self._publish_fk(q)

        def shutdown(self):
            self.bus.disconnect()

        def _handle_keys(self):
            from lerobot.utils.rotation import Rotation

            step_deg = getattr(self, "_step_deg", 2.0)
            step_m = getattr(self, "_step_m", 0.01)
            step_w = getattr(self, "_step_w", 0.05)

            while self._keys:
                ch = self._keys.popleft()
                if ch == "q":
                    raise KeyboardInterrupt
                if ch == "j":
                    self._interactive_submode = "joint"
                if ch == "p":
                    self._interactive_submode = "pose"
                if ch == "t":
                    self._interactive_submode = "twist"

                if ch in "12345":
                    self._selected_joint = int(ch) - 1

                if ch == ",":
                    step_deg = max(0.1, step_deg / 1.5)
                if ch == ".":
                    step_deg = min(25.0, step_deg * 1.5)
                if ch == "-":
                    self._q_cmd[self._selected_joint] -= step_deg
                if ch == "=":
                    self._q_cmd[self._selected_joint] += step_deg

                if self._interactive_submode in ("pose", "twist"):
                    if self._pose_cmd is None:
                        q0 = self._read_hw_joints_deg() if (self.bus is not None) else self._q_cmd.copy()
                        self._pose_cmd = self.kin.forward_kinematics(q0)

                    T = self._pose_cmd.copy()

                    # Translation
                    if ch == "w":
                        T[:3, 3] += np.array([+step_m, 0, 0])
                    if ch == "s":
                        T[:3, 3] += np.array([-step_m, 0, 0])
                    if ch == "a":
                        T[:3, 3] += np.array([0, +step_m, 0])
                    if ch == "d":
                        T[:3, 3] += np.array([0, -step_m, 0])
                    if ch == "r":
                        T[:3, 3] += np.array([0, 0, +step_m])
                    if ch == "f":
                        T[:3, 3] += np.array([0, 0, -step_m])

                    # Rotation (small rotvec increments in world frame)
                    rot = np.zeros(3, dtype=float)
                    if ch == "i":
                        rot[0] += step_w
                    if ch == "k":
                        rot[0] -= step_w
                    if ch == "j":
                        rot[1] += step_w
                    if ch == "l":
                        rot[1] -= step_w
                    if ch == "u":
                        rot[2] += step_w
                    if ch == "o":
                        rot[2] -= step_w
                    if np.linalg.norm(rot) > 0:
                        dR = Rotation.from_rotvec(rot).as_matrix()
                        T[:3, :3] = T[:3, :3] @ dR

                    self._pose_cmd = T

            self._step_deg = step_deg
            self._step_m = step_m
            self._step_w = step_w

    node = So101Viz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if rviz_proc is not None:
            rviz_proc.terminate()
        if rsp_proc is not None:
            rsp_proc.terminate()


if __name__ == "__main__":
    main()

