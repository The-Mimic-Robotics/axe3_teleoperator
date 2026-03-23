# AXE4 Leader — Package overview and flow

Teleoperator: 4× Feetech STS3215 motors (arm) + BLE handle (IMU + joystick). Pose from planar FK; orientation from handle quaternion. Outputs: eef_pose, eef_position, eef_twist, imu, joy (ROS2 or UDP).

---

## File roles

| File | Role |
|------|------|
| `axe4_leader.py` | Main teleoperator: bus + handle + transport; `get_action()` runs FK and publishes. |
| `fk.py` | Planar 3-link + pan FK; motor_cfg load; no URDF. |
| `config_axe4_leader.py` | Config: port, handle_source, transport, UDP/ROS2 options. |
| `handle_reader.py` | BLE (HandleReader) or UDP (LegacyIMUReader) for IMU + joystick. |
| `transport/base.py` | Abstract `PoseTransport`; `NullTransport` no-op. |
| `transport/ros2_transport.py` | Publishes PoseStamped, TwistStamped, Imu, Joy. |
| `transport/udp_transport.py` | Sends pose (and optionally other) packets over UDP. |
| `transport/__init__.py` | `create_transport("ros2"\|"udp"\|"none", **kwargs)`. |

---

## Overall data flow

```mermaid
flowchart LR
    subgraph Hardware
        M[4× STS3215 motors]
        H[BLE Handle\nIMU + Joy]
    end

    subgraph axe4_leader
        direction TB
        A[axe4Leader.get_action]
        A --> B[bus.sync_read\nPresent_Position]
        A --> C[handle.state]
        B --> D[fk.motor_deg_to_angles]
        D --> E[fk.forward_kinematics]
        E --> F[raw_xyz]
        F --> G[rel_xyz, delta_xyz]
        C --> G
        G --> T[transport.publish_*]
    end

    M --> B
    H --> C
    T --> ROS[ROS2 topics\nor UDP]
```

---

## get_action() flow (axe4_leader.py)

```mermaid
flowchart TB
    A[get_action] --> B[sync_read Present_Position]
    B --> C[motor_deg_to_angles\nmotor_cfg]
    C --> D[forward_kinematics q1..q4]
    D --> E[raw_xyz]
    E --> F{_home_xyz set?}
    F -->|No| G[set _home_xyz, _prev_xyz]
    F -->|Yes| H[rel_xyz = raw - home]
    G --> H
    H --> I[delta_xyz = raw - prev]
    I --> J[handle.state]
    J --> K[publish_eef_pose, position, twist, imu, buttons]
    K --> L[return action dict]
```

---

## FK pipeline (fk.py)

```mermaid
flowchart LR
    subgraph Input
        D[deg_dict\nmotor degrees]
        MC[motor_cfg\nsign, offset per joint]
    end

    subgraph fk
        M[motor_deg_to_angles] --> Q["q1..q4 (rad)"]
        Q --> FK[forward_kinematics]
        FK --> RZ["(r,z) chain\nL_BASE, L1, L2, L3"]
        RZ --> XY["x=r·cos(q1)\ny=r·sin(q1)\nz=z"]
        XY --> ROT["90° Z rotation\n(-y, x, z)"]
        ROT --> EEF[eef_xyz, chain_5×3]
    end

    D --> M
    MC --> M
```

- **motor_cfg**: loaded from `src/axe4/config/axe4_axis_calibration.json` (key `motor_cfg`) or defaults.
- **forward_kinematics**: builds planar chain in (r,z), sweeps by pan q1, then applies 90° rotation so arch is in XZ (forward) plane.
- **Frame**: +X forward, +Y left, Z up.

---

## Transport selection

```mermaid
flowchart TB
    create_transport(kind) --> K{kind?}
    K -->|none| N[NullTransport]
    K -->|udp| U[UDPTransport\nip, port, pose_only]
    K -->|ros2| R[ROS2Transport\nnode_name]
    N --> OUT[PoseTransport]
    U --> OUT
    R --> OUT
```

---

## Key classes

- **axe4Leader** (axe4_leader.py): Connects bus, handle, transport; `connect()` / `get_action()` / `disconnect()`. First `get_action()` sets home; subsequent ones publish rel_xyz and delta_xyz.
- **HandleReader** (handle_reader.py): BLE thread; exposes `state` (roll, pitch, yaw, quat, joy, buttons). **LegacyIMUReader**: UDP fallback, quat only.
- **PoseTransport** (transport/base.py): Interface for publish_eef_pose, publish_eef_position, publish_eef_twist, publish_imu, publish_buttons.
