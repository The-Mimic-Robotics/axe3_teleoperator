# AXE4 Teleoperator

# MISC Robotics - Achal Patel achalypatel3403@gmail.com
# MISC Robotics - Mathias Desrochers eltopchi1@gmail.com

Pose comes from **planar 3-link + pan FK** (motor angles) and **handle IMU** (orientation). No URDF in the pose pipeline; URDF is for visualization only (e.g. `urdf_view`).

## System Map

```mermaid
graph LR
    subgraph Handle
        IMU[JY901 IMU] -->|I2C| ESP32[ESP32-C3]
        JOY[Joystick + Buttons] --> ESP32
    end

    ESP32 -->|BLE notify<br/>quat / euler / joy| HOST[handle_reader]

    subgraph AXE4 Leader
        STS[4× STS3215] -->|Present_Position| FK[planar FK]
        FK -->|xyz| LEADER[axe4_leader]
        HOST -->|quat, buttons| LEADER
    end

    subgraph "ROS 2 Topics (Kinova-compatible)"
        LEADER -->|PoseStamped| POSE[/axe4/eef_pose]
        LEADER -->|PoseStamped| POS[/axe4/eef_position]
        LEADER -->|TwistStamped| TWIST[/axe4/eef_twist]
        LEADER -->|Imu| IMUT[/axe4/imu]
        LEADER -->|Joy| JOYT[/axe4/joy]
    end

    subgraph "UDP :5005"
        LEADER -.->|pose / twist / imu / joy| UDP[UDP]
    end

    POSE --> KINOVA[kinova_teleop]
    POS --> KINOVA
    TWIST --> KINOVA
    UDP -.-> KINOVA
```

## Setup

```bash
conda activate lerobot
pip install bleak
```

BLE handle advertises as **"Handle ESP32"** by default; set `--teleop.handle_device_name=YourName` if your firmware uses a different name.

## Connect servo driver

```bash
ls /dev/ttyACM*
```

## Calibrate (first time)

**FK motor signs + offsets** (required for correct pose):

```bash
python src/axe4/fk_calibrate.py --port /dev/ttyACM0 [--id axe]
```

**Axis / direction mapping** (optional, for reporting):

```bash
python src/axe4/axis_calibrator.py --port /dev/ttyACM0
```

## Run (ROS 2 — default)

```bash
lerobot-teleoperate \
    --robot.type=axe4_follower \
    --robot.cameras={} \
    --teleop.type=axe4_leader \
    --teleop.port=/dev/ttyACM0 \
    --teleop.transport=ros2 \
    --teleop.handle_source=ble \
    --teleop.id=axe \
    --display_data=false
```

## Run (UDP fallback)

```bash
lerobot-teleoperate \
    --robot.type=axe4_follower \
    --robot.cameras={} \
    --robot.udp_port=5005 \
    --teleop.type=axe4_leader \
    --teleop.port=/dev/ttyACM0 \
    --teleop.transport=udp \
    --teleop.handle_source=udp \
    --teleop.id=axe \
    --display_data=false
```

## Test

```bash
python src/axe4/scripts/eef_tester.py --source ros2
python src/axe4/scripts/eef_tester.py --source udp
python src/axe4/scripts/view_fk.py --port /dev/ttyACM0
python simple_udp_receiver.py
```

## Topics (frame: base_link, X fwd / Y left / Z up)

| Topic | Type | Description |
|-------|------|-------------|
| `/axe4/eef_pose` | PoseStamped | Position (relative to home) + orientation (handle quat) |
| `/axe4/eef_position` | PoseStamped | Position only (identity orientation) |
| `/axe4/eef_twist` | TwistStamped | Linear delta xyz per step (for cartesian_velocity-style control) |
| `/axe4/imu` | Imu | Handle orientation |
| `/axe4/joy` | Joy | Joystick axes + buttons |

## Jaco / Kinova integration — topic map

**AXE4 (this repo)** publishes the topics above. Your **Jaco-side bridge** (e.g. `kinova_teleop` / `arms_xbox_ctr`) should subscribe to them and publish to the Kinova driver as below.

### What to subscribe to (on the Jaco bridge) → control type

| Control type | Subscribe to (AXE4) | Use for |
|--------------|--------------------|--------|
| **Cartesian velocity** (real-time, smooth) | `/axe4/eef_twist` (TwistStamped) | Map `twist.linear` (and optionally `twist.angular`) to continuous velocity commands. Best for real-time teleop. |
| **Cartesian pose** (target position) | `/axe4/eef_position` (PoseStamped) or `/axe4/eef_pose` (PoseStamped) | Position (and orientation from eef_pose) relative to home. Use for pose targets or IK. |
| **Orientation only** | `/axe4/imu` (Imu) | Handle quaternion / euler for wrist orientation. |
| **Buttons / gripper** | `/axe4/joy` (Joy) | `axes` for joystick, `buttons` for triggers (e.g. gripper open/close). |

### What to publish (from the Jaco bridge) → Kinova driver

| To get this behaviour | Publish to (Kinova) | Message type |
|------------------------|---------------------|--------------|
| **Continuous Cartesian velocity** | `/<arm>/<arm>_driver/in/cartesian_velocity` | `kinova_msgs/PoseVelocity` (linear xyz, angular xyz) — same as Twist. Subscribe to `/axe4/eef_twist` and republish here (with frame/scaling as needed). |
| **Joint-space velocity** | `/<arm>/<arm>_driver/in/joint_velocity` | `kinova_msgs/JointVelocity` (deg/s). Requires IK or mapping from twist to joint velocities on the Jaco side. |
| **One-shot Cartesian pose** | Use action `/<arm>/<arm>_driver/pose_action/tool_pose` | Goal: `geometry_msgs/PoseStamped`. Subscribe to `/axe4/eef_pose` or `/axe4/eef_position`, send as action goal when you want a target. |
| **One-shot joint pose** | Use action `/<arm>/<arm>_driver/joints_action/joint_angles` | Goal: `JointAngles`. Only if you run IK on the Jaco side from `/axe4/eef_pose`. |

Example: for **real-time EEF following**, subscribe to `/axe4/eef_twist` and publish to `/<arm>/<arm>_driver/in/cartesian_velocity` (PoseVelocity). Optionally blend with `/axe4/eef_pose` for orientation. Use `/axe4/joy` for gripper.

## URDF (visualization only)

`src/axe4/config/axe4.urdf` matches the planar FK (same axes and lengths as `fk.py`). Use with `urdf_view` or any URDF viewer; **zero joint state** = home (arm up, forearm horizontal, handle down). Not used in pose computation.

## Config

| File | What |
|------|------|
| `src/axe4/config/axe4_arm.yaml` | Arm geometry (CAD) |
| `src/axe4/config/axe4.urdf` | URDF for **visualization only** (not used in pose computation) |
| `src/axe4/config/axe4_axis_calibration.json` | motor_cfg (signs/offsets), axis map, joint limits |

nathanael mccooeye nathanaelmccooeye@gmail.com
