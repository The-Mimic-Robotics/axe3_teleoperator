#!/usr/bin/env python

"""Tagged UDP transport for bimanual AXE leader teleoperation.

Packets are tagged with both arm and message type so one UDP target can receive
both streams unambiguously.

Format:
- Relative pose:      <cc7f>  arm('L'|'R'), 'O', x,y,z,qw,qx,qy,qz
- Absolute pose:      <cc7f>  arm('L'|'R'), 'A', x,y,z,qw,qx,qy,qz
- Relative position:  <cc3f>  arm('L'|'R'), 'P', x,y,z
- Absolute position:  <cc3f>  arm('L'|'R'), 'Q', x,y,z
- Twist:              <cc6f>  arm('L'|'R'), 'T', vx,vy,vz,wx,wy,wz
- IMU:                <cc7f>  arm('L'|'R'), 'I', qw,qx,qy,qz,roll,pitch,yaw
- Buttons:            <cc3f2B> arm('L'|'R'), 'J', joy_x,joy_y,joy_z,sw,sw2
"""

import logging
import socket
import struct

from ..axe_leader.transport.base import PoseTransport

logger = logging.getLogger(__name__)


class BiAxeUDPTransport:
    def __init__(
        self,
        ip: str = "127.0.0.1",
        port: int = 5005,
        pose_only: bool = False,
        print_packets: bool = False,
    ):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._addr = (ip, port)
        self._pose_only = pose_only
        self._print_packets = print_packets
        logger.info(f"BiAxeUDPTransport -> {ip}:{port} (pose_only={pose_only})")

    def _send(self, packet: bytes) -> None:
        try:
            self._sock.sendto(packet, self._addr)
        except OSError as exc:
            logger.warning(f"UDP send failed to {self._addr}: {exc}")

    def _debug_print(self, arm_tag: bytes, msg_tag: bytes, values) -> None:
        if not self._print_packets:
            return
        arm = arm_tag.decode("ascii", errors="ignore")
        msg = msg_tag.decode("ascii", errors="ignore")
        vals = ", ".join(f"{float(v):+.4f}" if isinstance(v, float) else str(v) for v in values)
        print(f"[UDP:BI:{arm}:{msg}] {vals}")

    def _typed(self, arm_tag: bytes, msg_tag: bytes, fmt: str, *values) -> bytes:
        if len(arm_tag) != 1 or len(msg_tag) != 1:
            raise ValueError("arm_tag and msg_tag must be 1 byte each")
        if not fmt.startswith("<"):
            raise ValueError(f"Expected little-endian fmt starting with '<', got {fmt!r}")
        return struct.pack("<cc" + fmt[1:], arm_tag, msg_tag, *values)

    def make_arm_transport(self, arm: str) -> PoseTransport:
        arm_upper = arm.upper()
        if arm_upper not in {"L", "R"}:
            raise ValueError("arm must be 'L' or 'R'")
        return _ArmTransport(self, arm_upper.encode("ascii"))

    def shutdown(self) -> None:
        self._sock.close()


class _ArmTransport(PoseTransport):
    def __init__(self, parent: BiAxeUDPTransport, arm_tag: bytes):
        self._parent = parent
        self._arm_tag = arm_tag

    def publish_eef_pose(self, x, y, z, qw, qx, qy, qz):
        self._parent._debug_print(self._arm_tag, b"O", (x, y, z, qw, qx, qy, qz))
        pkt = self._parent._typed(self._arm_tag, b"O", "<7f", x, y, z, qw, qx, qy, qz)
        self._parent._send(pkt)

    def publish_eef_pose_absolute(self, x, y, z, qw, qx, qy, qz):
        if self._parent._pose_only:
            return
        self._parent._debug_print(self._arm_tag, b"A", (x, y, z, qw, qx, qy, qz))
        pkt = self._parent._typed(self._arm_tag, b"A", "<7f", x, y, z, qw, qx, qy, qz)
        self._parent._send(pkt)

    def publish_eef_position(self, x, y, z):
        if self._parent._pose_only:
            return
        self._parent._debug_print(self._arm_tag, b"P", (x, y, z))
        pkt = self._parent._typed(self._arm_tag, b"P", "<3f", x, y, z)
        self._parent._send(pkt)

    def publish_eef_position_absolute(self, x, y, z):
        if self._parent._pose_only:
            return
        self._parent._debug_print(self._arm_tag, b"Q", (x, y, z))
        pkt = self._parent._typed(self._arm_tag, b"Q", "<3f", x, y, z)
        self._parent._send(pkt)

    def publish_eef_twist(self, vx, vy, vz, wx=0.0, wy=0.0, wz=0.0):
        if self._parent._pose_only:
            return
        self._parent._debug_print(self._arm_tag, b"T", (vx, vy, vz, wx, wy, wz))
        pkt = self._parent._typed(self._arm_tag, b"T", "<6f", vx, vy, vz, wx, wy, wz)
        self._parent._send(pkt)

    def publish_imu(self, qw, qx, qy, qz, roll=0.0, pitch=0.0, yaw=0.0):
        if self._parent._pose_only:
            return
        self._parent._debug_print(self._arm_tag, b"I", (qw, qx, qy, qz, roll, pitch, yaw))
        pkt = self._parent._typed(self._arm_tag, b"I", "<7f", qw, qx, qy, qz, roll, pitch, yaw)
        self._parent._send(pkt)

    def publish_buttons(self, sw, sw2, joy_x=0.0, joy_y=0.0, joy_z=0.0):
        if self._parent._pose_only:
            return
        self._parent._debug_print(self._arm_tag, b"J", (joy_x, joy_y, joy_z, int(sw), int(sw2)))
        pkt = self._parent._typed(self._arm_tag, b"J", "<3f2B", joy_x, joy_y, joy_z, sw, sw2)
        self._parent._send(pkt)

    def shutdown(self):
        # Parent owns socket lifecycle.
        pass
