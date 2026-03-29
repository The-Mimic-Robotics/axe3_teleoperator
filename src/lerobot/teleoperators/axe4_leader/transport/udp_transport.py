"""UDP transport for AXE4 teleop.

Compatibility behavior:
- `publish_eef_pose` always sends the legacy 28-byte packet `<7f>` so existing
    receivers (e.g. simple_udp_receiver and kinova bridge) keep working.
- When `pose_only=False`, non-pose messages are sent as typed packets with a
    1-byte tag prefix to avoid packet-size collisions with legacy pose parsing.
"""

import logging
import socket
import struct

from .base import PoseTransport

logger = logging.getLogger(__name__)


class UDPTransport(PoseTransport):
    def __init__(self, ip: str = "192.168.131.150", port: int = 5005, pose_only: bool = True):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._addr = (ip, port)
        self._pose_only = pose_only
        logger.info(f"UDPTransport → {ip}:{port} (pose_only={pose_only})")

    def _send(self, packet: bytes) -> None:
        try:
            self._sock.sendto(packet, self._addr)
        except OSError as exc:
            logger.warning(f"UDP send failed to {self._addr}: {exc}")

    @staticmethod
    def _typed_packet(tag: bytes, fmt: str, *values) -> bytes:
        if not fmt.startswith("<"):
            raise ValueError(f"Expected little-endian fmt starting with '<', got {fmt!r}")
        return struct.pack("<c" + fmt[1:], tag, *values)

    def publish_eef_pose(self, x, y, z, qw, qx, qy, qz):
        pkt = struct.pack("<7f", x, y, z, qw, qx, qy, qz)
        self._send(pkt)

    def publish_eef_position(self, x, y, z):
        if self._pose_only:
            return
        self._send(self._typed_packet(b"P", "<3f", x, y, z))

    def publish_eef_twist(self, vx, vy, vz, wx=0.0, wy=0.0, wz=0.0):
        if self._pose_only:
            return
        self._send(self._typed_packet(b"T", "<6f", vx, vy, vz, wx, wy, wz))

    def publish_imu(self, qw, qx, qy, qz, roll=0.0, pitch=0.0, yaw=0.0):
        if self._pose_only:
            return
        self._send(self._typed_packet(b"I", "<7f", qw, qx, qy, qz, roll, pitch, yaw))

    def publish_buttons(self, sw, sw2, joy_x=0.0, joy_y=0.0, joy_z=0.0):
        if self._pose_only:
            return
        self._send(self._typed_packet(b"J", "<3f2B", joy_x, joy_y, joy_z, sw, sw2))

    def shutdown(self):
        self._sock.close()
