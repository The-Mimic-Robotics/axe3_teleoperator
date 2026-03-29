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
  --teleop.handle_device_name=AXE3_left \
  --teleop.id=axe \
  --teleop.udp_pose_only=false \
  --teleop.udp_print_packets=false \
  --display_data=false
```

## 2) Bimanual (`bi_axe_leader`)

Both BLE handles auto-default:
- left: `AXE3_left`
- right: `AXE3_right`

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
--teleop.right_handle_device_name=AXE3_right
```

## Notes

- `udp_pose_only=true` sends minimal packets only.
- `udp_pose_only=false` sends all message types.
- `udp_print_packets=true` prints sent payload values in terminal.
