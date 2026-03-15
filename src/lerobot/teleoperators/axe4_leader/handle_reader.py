"""
BLE Handle Reader for the AXE4 teleoperator handle (ESP32 + JY901 IMU).

Connects to the ESP32 via Bluetooth Low Energy and receives:
  - IMU angles  (roll, pitch, yaw)   3 floats  12 bytes
  - IMU quats   (w, x, y, z)         4 floats  16 bytes
  - Joystick    (x, y, z, sw, sw2)   3 floats + 2 uint8  14 bytes

Replaces the old UDP-based IMU reader (imu_reader/).
Source firmware: https://github.com/The-Mimic-Robotics/handle_v1
"""

import asyncio
import logging
import struct
import threading
from dataclasses import dataclass, field

logger = logging.getLogger(__name__)

CHAR_UUID_ANGLE = "beb5483e-36e1-4688-b7f5-ea07361b26a8"
CHAR_UUID_QUAT = "828917c1-ea55-4d4a-a66e-fd202cea0645"
CHAR_UUID_JOY = "9c661337-b499-497d-aa5b-0105316e6e22"


@dataclass
class HandleState:
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    qw: float = 1.0
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    joy_x: float = 0.0
    joy_y: float = 0.0
    joy_z: float = 0.0
    sw: int = 0
    sw2: int = 0


class HandleReader:
    """Thread-safe BLE reader for the ESP32 handle controller."""

    def __init__(self, device_name: str = "Handle ESP32"):
        self.device_name = device_name
        self._state = HandleState()
        self._lock = threading.Lock()
        self._connected = False
        self._running = False
        self._thread: threading.Thread | None = None

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def state(self) -> HandleState:
        with self._lock:
            return HandleState(
                roll=self._state.roll,
                pitch=self._state.pitch,
                yaw=self._state.yaw,
                qw=self._state.qw,
                qx=self._state.qx,
                qy=self._state.qy,
                qz=self._state.qz,
                joy_x=self._state.joy_x,
                joy_y=self._state.joy_y,
                joy_z=self._state.joy_z,
                sw=self._state.sw,
                sw2=self._state.sw2,
            )

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()
        logger.info(f"HandleReader started, scanning for '{self.device_name}'")

    def stop(self) -> None:
        self._running = False
        self._connected = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=5.0)
        self._thread = None
        logger.info("HandleReader stopped")

    def _run_loop(self) -> None:
        while self._running:
            try:
                asyncio.run(self._connect_and_listen())
            except Exception as e:
                logger.warning(f"BLE connection lost: {e}. Reconnecting in 2s...")
            if self._running:
                import time
                time.sleep(2.0)

    async def _connect_and_listen(self) -> None:
        try:
            from bleak import BleakScanner, BleakClient
        except ImportError:
            logger.error("bleak not installed. Run: pip install bleak")
            self._running = False
            return

        logger.info(f"Scanning for '{self.device_name}'...")
        device = await BleakScanner.find_device_by_name(self.device_name, timeout=10.0)
        if not device:
            logger.warning(f"Could not find '{self.device_name}'. Is it powered on?")
            return

        logger.info(f"Found '{self.device_name}' at {device.address}. Connecting...")
        async with BleakClient(device) as client:
            self._connected = True
            logger.info("BLE connected. Subscribing to characteristics...")

            await client.start_notify(CHAR_UUID_ANGLE, self._on_angle)
            await client.start_notify(CHAR_UUID_QUAT, self._on_quat)
            await client.start_notify(CHAR_UUID_JOY, self._on_joy)

            while self._running and client.is_connected:
                await asyncio.sleep(0.05)

            await client.stop_notify(CHAR_UUID_ANGLE)
            await client.stop_notify(CHAR_UUID_QUAT)
            await client.stop_notify(CHAR_UUID_JOY)
            self._connected = False

    def _on_angle(self, _sender, data: bytearray) -> None:
        if len(data) == 12:
            roll, pitch, yaw = struct.unpack("<3f", data)
            with self._lock:
                self._state.roll = roll
                self._state.pitch = pitch
                self._state.yaw = yaw

    def _on_quat(self, _sender, data: bytearray) -> None:
        if len(data) == 16:
            w, x, y, z = struct.unpack("<4f", data)
            with self._lock:
                self._state.qw = w
                self._state.qx = x
                self._state.qy = y
                self._state.qz = z

    def _on_joy(self, _sender, data: bytearray) -> None:
        if len(data) == 14:
            jx, jy, jz, sw, sw2 = struct.unpack("<3f2B", data)
            with self._lock:
                self._state.joy_x = jx
                self._state.joy_y = jy
                self._state.joy_z = jz
                self._state.sw = sw
                self._state.sw2 = sw2


class LegacyIMUReader:
    """Fallback UDP-based IMU reader (old C++ imu_udp bridge).

    Reads 4 floats (qw, qx, qy, qz) from a UDP socket.
    """

    def __init__(self, ip: str = "127.0.0.1", port: int = 5000):
        import socket
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((ip, port))
        self._sock.setblocking(False)
        self._state = HandleState()
        self._lock = threading.Lock()

    @property
    def connected(self) -> bool:
        return True

    @property
    def state(self) -> HandleState:
        self._drain()
        with self._lock:
            return HandleState(
                qw=self._state.qw,
                qx=self._state.qx,
                qy=self._state.qy,
                qz=self._state.qz,
            )

    def _drain(self) -> None:
        try:
            while True:
                data, _ = self._sock.recvfrom(16)
                if len(data) == 16:
                    w, x, y, z = struct.unpack("<4f", data)
                    with self._lock:
                        self._state.qw = w
                        self._state.qx = x
                        self._state.qy = y
                        self._state.qz = z
        except BlockingIOError:
            pass

    def start(self) -> None:
        pass

    def stop(self) -> None:
        self._sock.close()
