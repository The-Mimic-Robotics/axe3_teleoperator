"""
BLE Handle Reader for AXE leader handle (ESP32 + IMU).

Receives:
  - IMU angles  (roll, pitch, yaw)   3 floats  12 bytes
  - IMU quats   (w, x, y, z)         4 floats  16 bytes
  - Joystick    (x, y, z, sw, sw2)   3 floats + 2 uint8  14 bytes
"""

import asyncio
import logging
import platform
import struct
import threading
from dataclasses import dataclass

logger = logging.getLogger(__name__)


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
    """Thread-safe BLE reader for ESP32 handle controller."""

    def __init__(
        self,
        device_name: str = "AXE4_left",
        char_uuid_angle: str = "beb5483e-36e1-4688-b7f5-ea07361b26a8",
        char_uuid_quat: str = "828917c1-ea55-4d4a-a66e-fd202cea0645",
        char_uuid_joy: str = "9c661337-b499-497d-aa5b-0105316e6e22",
    ):
        self.device_name = device_name
        self.char_uuid_angle = char_uuid_angle
        self.char_uuid_quat = char_uuid_quat
        self.char_uuid_joy = char_uuid_joy
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

    @staticmethod
    def _normalize_name(name: str) -> str:
        return "".join(ch for ch in name.lower() if ch.isalnum())

    @staticmethod
    def _is_likely_ble_address(value: str) -> bool:
        cleaned = value.strip().replace("-", ":")
        parts = cleaned.split(":")
        if len(parts) != 6:
            return False
        return all(len(p) == 2 and all(c in "0123456789abcdefABCDEF" for c in p) for p in parts)

    @staticmethod
    def _candidate_names(device) -> list[str]:
        names: list[str] = []

        for attr in ("name", "local_name"):
            value = getattr(device, attr, None)
            if isinstance(value, str) and value.strip():
                names.append(value.strip())

        metadata = getattr(device, "metadata", None)
        if isinstance(metadata, dict):
            local = metadata.get("local_name")
            if isinstance(local, str) and local.strip():
                names.append(local.strip())

        out: list[str] = []
        seen: set[str] = set()
        for n in names:
            key = n.lower()
            if key not in seen:
                seen.add(key)
                out.append(n)
        return out

    def _pick_device_by_name(self, devices: list, target_name: str):
        target = target_name.strip()
        if not target:
            return None

        target_lower = target.lower()
        target_norm = self._normalize_name(target)

        for dev in devices:
            for cand in self._candidate_names(dev):
                if cand.lower() == target_lower:
                    return dev

        for dev in devices:
            for cand in self._candidate_names(dev):
                if self._normalize_name(cand) == target_norm:
                    return dev

        for dev in devices:
            for cand in self._candidate_names(dev):
                cand_norm = self._normalize_name(cand)
                if target_norm in cand_norm or cand_norm in target_norm:
                    return dev

        return None

    async def _windows_preflight_pick(self, target: str):
        if platform.system() != "Windows":
            return None

        try:
            from bleak import BleakScanner
        except ImportError:
            return None

        devices = await BleakScanner.discover(timeout=4.0)
        picked = self._pick_device_by_name(devices, target)

        if picked is None:
            names: list[str] = []
            for dev in devices:
                for n in self._candidate_names(dev):
                    if n not in names:
                        names.append(n)
            if names:
                logger.info(f"Windows BLE preflight nearby names: {', '.join(names[:10])}")

        return picked

    async def _connect_and_listen(self) -> None:
        try:
            from bleak import BleakClient, BleakScanner
        except ImportError:
            logger.error("bleak not installed. Run: pip install bleak")
            self._running = False
            return

        logger.info(f"Scanning for '{self.device_name}'...")
        target = self.device_name.strip()
        device = await self._windows_preflight_pick(target)

        if not device:
            device = await BleakScanner.find_device_by_name(target, timeout=6.0)

        if not device and self._is_likely_ble_address(target):
            try:
                device = await BleakScanner.find_device_by_address(target, timeout=6.0)
            except Exception:
                device = None

        if not device:
            devices = await BleakScanner.discover(timeout=10.0)
            device = self._pick_device_by_name(devices, target)

        if not device:
            devices = await BleakScanner.discover(timeout=5.0)
            names = []
            for dev in devices:
                for n in self._candidate_names(dev):
                    if n not in names:
                        names.append(n)
            if names:
                logger.warning(f"Could not find '{self.device_name}'. Nearby BLE names: {', '.join(names[:10])}")
            else:
                logger.warning(f"Could not find '{self.device_name}'. Is it powered on and paired?")
            return

        logger.info(f"Found '{self.device_name}' at {device.address}. Connecting...")
        async with BleakClient(device) as client:
            self._connected = True
            logger.info("BLE connected. Subscribing to characteristics...")

            await client.start_notify(self.char_uuid_angle, self._on_angle)
            await client.start_notify(self.char_uuid_quat, self._on_quat)
            await client.start_notify(self.char_uuid_joy, self._on_joy)

            while self._running and client.is_connected:
                await asyncio.sleep(0.05)

            await client.stop_notify(self.char_uuid_angle)
            await client.stop_notify(self.char_uuid_quat)
            await client.stop_notify(self.char_uuid_joy)
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
    """Fallback UDP-based IMU reader (old C++ imu_udp bridge)."""

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
