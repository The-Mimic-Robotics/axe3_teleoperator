"""Abstract transport interface for AXE pose publishing."""

from abc import ABC, abstractmethod


class PoseTransport(ABC):
	"""Publishes teleoperation data (pose, position, twist/deltas, IMU, joy)."""

	@abstractmethod
	def publish_eef_pose(
		self, x: float, y: float, z: float,
		qw: float, qx: float, qy: float, qz: float,
	) -> None: ...

	@abstractmethod
	def publish_eef_position(self, x: float, y: float, z: float) -> None: ...

	def publish_eef_pose_absolute(
		self, x: float, y: float, z: float,
		qw: float, qx: float, qy: float, qz: float,
	) -> None:
		"""Optional absolute pose output. Default no-op for backward compatibility."""
		pass

	def publish_eef_position_absolute(self, x: float, y: float, z: float) -> None:
		"""Optional absolute position output. Default no-op for backward compatibility."""
		pass

	@abstractmethod
	def publish_eef_twist(
		self, vx: float, vy: float, vz: float,
		wx: float = 0.0, wy: float = 0.0, wz: float = 0.0,
	) -> None: ...

	@abstractmethod
	def publish_imu(
		self, qw: float, qx: float, qy: float, qz: float,
		roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0,
	) -> None: ...

	@abstractmethod
	def publish_buttons(
		self, sw: int, sw2: int,
		joy_x: float = 0.0, joy_y: float = 0.0, joy_z: float = 0.0,
	) -> None: ...

	def shutdown(self) -> None:
		pass


class NullTransport(PoseTransport):
	"""No-op transport. Data only via get_action()."""

	def publish_eef_pose(self, x, y, z, qw, qx, qy, qz):
		pass

	def publish_eef_position(self, x, y, z):
		pass

	def publish_eef_twist(self, vx, vy, vz, wx=0.0, wy=0.0, wz=0.0):
		pass

	def publish_imu(self, qw, qx, qy, qz, roll=0.0, pitch=0.0, yaw=0.0):
		pass

	def publish_buttons(self, sw, sw2, joy_x=0.0, joy_y=0.0, joy_z=0.0):
		pass
