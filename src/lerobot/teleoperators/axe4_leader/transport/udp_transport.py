"""UDP transport — packs pose data as raw floats and sends to a UDP endpoint."""

import logging
import socket
import struct

from .base import PoseTransport

logger = logging.getLogger(__name__)


class UDPTransport(PoseTransport):
    def __init__(self, ip: str = "127.0.0.1", port: int = 5005, pose_only: bool = True):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._addr = (ip, port)
        self._pose_only = pose_only
        logger.info(f"UDPTransport → {ip}:{port} (pose_only={pose_only})")

    def publish_eef_pose(self, x, y, z, qw, qx, qy, qz):
        pkt = struct.pack("<7f", x, y, z, qw, qx, qy, qz)
        self._sock.sendto(pkt, self._addr)

    def publish_eef_position(self, x, y, z):
        if self._pose_only:
            return
        pkt = struct.pack("<3f", x, y, z)
        self._sock.sendto(pkt, self._addr)

    def publish_eef_twist(self, vx, vy, vz, wx=0.0, wy=0.0, wz=0.0):
        if self._pose_only:
            return
        pkt = struct.pack("<6f", vx, vy, vz, wx, wy, wz)
        self._sock.sendto(pkt, self._addr)

    def publish_imu(self, qw, qx, qy, qz, roll=0.0, pitch=0.0, yaw=0.0):
        if self._pose_only:
            return
        pkt = struct.pack("<7f", qw, qx, qy, qz, roll, pitch, yaw)
        self._sock.sendto(pkt, self._addr)

    def publish_buttons(self, sw, sw2, joy_x=0.0, joy_y=0.0, joy_z=0.0):
        if self._pose_only:
            return
        pkt = struct.pack("<3f2B", joy_x, joy_y, joy_z, sw, sw2)
        self._sock.sendto(pkt, self._addr)

    def shutdown(self):
        self._sock.close()
