"""
MuJoCo EEF tester — drives a floating box from AXE4 teleop data.

Source (--source): ros2 (subscribe topic) or udp (listen packets).

Topic / mode:
  axe4/eef_position  position relative to home (absolute, drift-free)
  axe4/eef_pose      position + orientation (absolute or delta by --mode)
  axe4/eef_twist     velocity control: TwistStamped linear xyz integrated as pos += vel*dt
                     (same as cartesian_velocity on real arms; use --topic axe4/eef_twist)

MISC Robotics - Achal Patel achalypatel3403@gmail.com
MISC Robotics - Mathias Desrochers eltopchi1@gmail.com
"""

import argparse
import time
import socket
import struct
import threading
import numpy as np
import mujoco
import mujoco.viewer

# --- defaults ---
UDP_IP = "127.0.0.1"
UDP_PORT = 5005

# Shared state:  [x, y, z, qw, qx, qy, qz]
current_cmd = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
new_data_flag = False   # set True by listener, cleared by renderer
lock = threading.Lock()
running = True

# MuJoCo scene.  Camera looks along +X (from behind), Y = left, Z = up.
XML_SCENE = """
<mujoco>
  <option timestep="0.002" gravity="0 0 0"/>
  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global azimuth="180" elevation="-15"/>
  </visual>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0"
             width="512" height="308"/>
    <texture name="grid" type="2d" builtin="checker" width="512" height="512"
             rgb1=".1 .2 .3" rgb2=".2 .3 .4"/>
    <material name="grid" texture="grid" texrepeat="1 1" texuniform="true"
              reflectance=".2"/>
  </asset>
  <worldbody>
    <light pos="0 0 1.5" dir="0 0 -1" directional="true"/>
    <geom name="floor" size="0 0 .05" type="plane" material="grid" condim="3"/>

    <!-- fixed origin axes:  red = +X (fwd),  green = +Y (left),  blue = +Z (up) -->
    <geom fromto="0 0 0.001 0.3 0 0.001" size="0.003" rgba="1 0.3 0.3 0.5" type="cylinder"/>
    <geom fromto="0 0 0.001 0 0.3 0.001" size="0.003" rgba="0.3 1 0.3 0.5" type="cylinder"/>
    <geom fromto="0 0 0.001 0 0 0.3"     size="0.003" rgba="0.3 0.3 1 0.5" type="cylinder"/>

    <body name="end_effector" pos="0 0 0.5">
      <freejoint name="ee_joint"/>
      <geom type="box" size=".03 .03 .03" rgba="0.9 0.2 0.2 1" mass="0.01"/>
      <!-- body axes: red = X, green = Y, blue = Z -->
      <geom fromto="0 0 0 0.08 0 0" size="0.004" rgba="1 0 0 1" type="cylinder"/>
      <geom fromto="0 0 0 0 0.08 0" size="0.004" rgba="0 1 0 1" type="cylinder"/>
      <geom fromto="0 0 0 0 0 0.08" size="0.004" rgba="0 0 1 1" type="cylinder"/>
    </body>
  </worldbody>
</mujoco>
"""


def direction_label(x, y, z, threshold=0.005):
    """Return a human-readable direction string from an XYZ vector."""
    vec = [x, y, z]
    idx = max(range(3), key=lambda i: abs(vec[i]))
    if abs(vec[idx]) < threshold:
        return "   STOP"
    labels = [("FORWARD", "BACKWARD"), ("LEFT", "RIGHT"), ("UP", "DOWN")]
    return f"{'   ' if vec[idx] > 0 else '   '}{labels[idx][0 if vec[idx] > 0 else 1]:>8s}"


# ----------------------------------------------------------------
# Listeners  (run in background thread, write to current_cmd)
# ----------------------------------------------------------------
def udp_listener():
    """Receive 28-byte UDP packets:  7 floats (x, y, z, qw, qx, qy, qz)."""
    global current_cmd, new_data_flag, running

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    sock.settimeout(0.2)
    print(f"[UDP] Listening on {UDP_IP}:{UDP_PORT}...")

    while running:
        try:
            data, _ = sock.recvfrom(1024)
            if len(data) == 28:
                values = struct.unpack('<fffffff', data)
                with lock:
                    current_cmd[:] = list(values)
                    new_data_flag = True
        except socket.timeout:
            continue
        except Exception as e:
            print(f"[UDP] error: {e}")
    sock.close()


_ros2_topic = "axe4/eef_position"
_ros2_velocity_mode = False  # True when subscribing to eef_twist


def ros2_listener():
    """Subscribe to PoseStamped (eef_pose / eef_position) or TwistStamped (eef_twist)."""
    global current_cmd, new_data_flag, running, _ros2_topic, _ros2_velocity_mode
    try:
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import PoseStamped, TwistStamped
    except ImportError:
        print("[ROS2] rclpy not available — falling back to UDP")
        udp_listener()
        return

    if not rclpy.ok():
        rclpy.init()

    class _Sub(Node):
        def __init__(self, topic: str, velocity_mode: bool):
            super().__init__("axe4_eef_tester")
            self._velocity_mode = velocity_mode
            if velocity_mode:
                self.sub = self.create_subscription(TwistStamped, topic, self._cb_twist, 10)
            else:
                self.sub = self.create_subscription(PoseStamped, topic, self._cb_pose, 10)

        def _cb_pose(self, msg):
            global new_data_flag
            p = msg.pose.position
            o = msg.pose.orientation
            with lock:
                current_cmd[:] = [p.x, p.y, p.z, o.w, o.x, o.y, o.z]
                new_data_flag = True

        def _cb_twist(self, msg):
            global new_data_flag
            t = msg.twist.linear
            with lock:
                current_cmd[0] = t.x
                current_cmd[1] = t.y
                current_cmd[2] = t.z
                current_cmd[3:7] = [1.0, 0.0, 0.0, 0.0]
                new_data_flag = True

    node = _Sub(_ros2_topic, _ros2_velocity_mode)
    kind = "TwistStamped (velocity)" if _ros2_velocity_mode else "PoseStamped"
    print(f"[ROS2] Subscribed to /{_ros2_topic} ({kind})")
    while running:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()


# ----------------------------------------------------------------
# Main
# ----------------------------------------------------------------
def main():
    global running, _ros2_topic, _ros2_velocity_mode

    parser = argparse.ArgumentParser(description="AXE4 EEF tester (MuJoCo)")
    parser.add_argument("--source", choices=["ros2", "udp"], default="ros2",
                        help="Data source (default: ros2)")
    parser.add_argument("--topic", default="axe4/eef_position",
                        help="ROS 2 topic: axe4/eef_position (absolute), axe4/eef_pose, axe4/eef_twist (velocity)")
    parser.add_argument("--mode", choices=["absolute", "delta", "velocity"], default=None,
                        help="Mode. Default: auto from topic (velocity for eef_twist, absolute for eef_position)")
    parser.add_argument("--no-rotation", action="store_true",
                        help="Ignore orientation — identity quaternion")
    parser.add_argument("--debug", action="store_true",
                        help="Print received values every 0.5 s")
    args = parser.parse_args()

    _ros2_topic = args.topic
    if "twist" in args.topic:
        _ros2_velocity_mode = True
    else:
        _ros2_velocity_mode = False

    if args.mode is not None:
        use_absolute = (args.mode == "absolute")
        use_velocity = (args.mode == "velocity")
    else:
        use_velocity = _ros2_velocity_mode
        use_absolute = (not use_velocity) and ("position" in args.topic)

    mode_name = "VELOCITY (integrate twist)" if use_velocity else ("ABSOLUTE" if use_absolute else "DELTA")

    model = mujoco.MjModel.from_xml_string(XML_SCENE)
    data = mujoco.MjData(model)

    # Starting position for the box
    HOME_POS = np.array([0.0, 0.0, 0.5])

    listener_fn = ros2_listener if args.source == "ros2" else udp_listener
    t = threading.Thread(target=listener_fn, daemon=True)
    t.start()

    print(f"[SIM] source={args.source}  topic=/{_ros2_topic}  mode={mode_name}")
    if use_velocity:
        print("[SIM] Velocity mode: integrating twist.linear as position delta per message.")
    print("[SIM] Origin axes: red = +X (forward)  green = +Y (left)  blue = +Z (up)")
    print("[SIM] Close the viewer window to exit.")

    last_debug = 0.0
    identity_quat = np.array([1.0, 0.0, 0.0, 0.0])

    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running():
                with lock:
                    cmd = current_cmd.copy()

                xyz = np.array(cmd[:3], dtype=np.float64)
                quat = np.array(cmd[3:7], dtype=np.float64)

                # Validate quaternion
                qnorm = np.linalg.norm(quat)
                if qnorm < 1e-6:
                    quat = identity_quat.copy()
                else:
                    quat = quat / qnorm

                if use_velocity:
                    # Velocity / twist: integrate linear part (leader sends delta per step or velocity)
                    MAX_STEP = 0.02
                    delta = np.clip(xyz, -MAX_STEP, MAX_STEP)
                    data.qpos[0:3] += delta
                    data.qpos[0] = float(np.clip(data.qpos[0], -2.0, 2.0))
                    data.qpos[1] = float(np.clip(data.qpos[1], -2.0, 2.0))
                    data.qpos[2] = float(np.clip(data.qpos[2], 0.05, 2.0))
                elif use_absolute:
                    data.qpos[0] = float(np.clip(HOME_POS[0] + xyz[0], -2.0, 2.0))
                    data.qpos[1] = float(np.clip(HOME_POS[1] + xyz[1], -2.0, 2.0))
                    data.qpos[2] = float(np.clip(HOME_POS[2] + xyz[2], 0.05, 2.0))
                else:
                    MAX_DELTA_STEP = 0.01
                    delta = np.clip(xyz, -MAX_DELTA_STEP, MAX_DELTA_STEP)
                    data.qpos[0:3] += delta
                    data.qpos[0] = float(np.clip(data.qpos[0], -2.0, 2.0))
                    data.qpos[1] = float(np.clip(data.qpos[1], -2.0, 2.0))
                    data.qpos[2] = float(np.clip(data.qpos[2], 0.05, 2.0))

                if args.no_rotation:
                    data.qpos[3:7] = identity_quat
                else:
                    data.qpos[3:7] = quat

                mujoco.mj_forward(model, data)
                viewer.sync()

                # Debug output
                now = time.time()
                if args.debug and now - last_debug > 0.5:
                    last_debug = now
                    pos = data.qpos[0:3]
                    label = direction_label(cmd[0], cmd[1], cmd[2])
                    print(
                        f"[DBG] xyz=[{cmd[0]:+.4f}, {cmd[1]:+.4f}, {cmd[2]:+.4f}]  "
                        f"box=[{pos[0]:+.4f}, {pos[1]:+.4f}, {pos[2]:+.4f}]  "
                        f"dir={label}"
                    )

                time.sleep(0.016)
    except KeyboardInterrupt:
        pass
    finally:
        running = False
        t.join(timeout=2)
        print("[SIM] Closed.")


if __name__ == "__main__":
    main()
