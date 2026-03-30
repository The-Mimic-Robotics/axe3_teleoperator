# AXE Leader Teleop (Single + Bimanual)

## 1) Single arm (`axe_leader`)

```bash
python -m lerobot.scripts.lerobot_teleoperate \
  --robot.type=axe4_follower \
  --robot.cameras='{}' \
  --robot.udp_ip=192.168.131.150 \
  --robot.udp_port=5005 \
  --teleop.type=axe_leader \
  --teleop.port=COM8 \
  --teleop.transport=udp \
  --teleop.udp_target_ip=192.168.131.150 \
  --teleop.udp_target_port=5005 \
  --teleop.handle_source=ble \
  --teleop.handle_device_name=AXE4_left \
  --teleop.id=axe \
  --teleop.udp_pose_only=false \
  --teleop.udp_print_packets=false \
  --display_data=false
```

## 2) Bimanual (`bi_axe_leader`)

Both BLE handles auto-default:
- left: `AXE4_left`
- right: `AXE4_right`

Both streams are sent to one UDP target with arm+message tags.

```bash
python -m lerobot.scripts.lerobot_teleoperate \
  --robot.type=axe4_follower \
  --robot.cameras='{}' \
  --robot.udp_ip=192.168.131.150 \
  --robot.udp_port=5005 \
  --teleop.type=bi_axe_leader \
  --teleop.left_arm_port=COM8 \
  --teleop.right_arm_port=COM9 \
  --teleop.transport=udp \
  --teleop.udp_target_ip=192.168.131.150 \
  --teleop.udp_target_port=5005 \
  --teleop.handle_source=ble \
  --teleop.id=axe_bi \
  --teleop.udp_pose_only=false \
  --teleop.udp_print_packets=false \
  --display_data=false
```

## 3) Optional: explicit BLE names (bimanual)

```bash
--teleop.left_handle_device_name=AXE3_left \
--teleop.right_handle_device_name=AXE4_right

or (for AXE4 handles):

```bash
--teleop.left_handle_device_name=AXE4_left \
--teleop.right_handle_device_name=AXE4_right
```

## 4) Optional: explicit BLE UUID overrides (bimanual)

If a handle advertises custom characteristics, override per arm:

```bash
--teleop.left_handle_ble_uuids='{"angle":"beb5483e-36e1-4688-b7f5-ea07361b26a8","quat":"828917c1-ea55-4d4a-a66e-fd202cea0645","joy":"9c661337-b499-497d-aa5b-0105316e6e22"}' \
--teleop.right_handle_ble_uuids='{"angle":"d1a68735-86b2-4d26-b8f2-1b633075c3f9","quat":"f3c83012-78d1-4e96-a14a-7bc991060932","joy":"2a8497d5-d852-4f01-90a6-16e51141bc25"}'
```
```

## Notes

- `udp_pose_only=true` sends minimal packets only.
- `udp_pose_only=false` sends all message types.
- `udp_print_packets=true` prints sent payload values in terminal.
