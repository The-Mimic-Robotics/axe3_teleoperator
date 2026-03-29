"""ROS 2 transport for AXE teleoperation data."""

import logging
import threading

from .base import PoseTransport

logger = logging.getLogger(__name__)

try:
	import rclpy
	from geometry_msgs.msg import PoseStamped, TwistStamped
	from sensor_msgs.msg import Imu, Joy

	ROS2_AVAILABLE = True
except ImportError:
	ROS2_AVAILABLE = False


class ROS2Transport(PoseTransport):
	def __init__(self, node_name: str = "axe_teleop", topic_prefix: str = "axe"):
		if not ROS2_AVAILABLE:
			raise ImportError(
				"rclpy not installed. Install ROS 2 or: pip install rclpy geometry-msgs sensor-msgs"
			)
		if not rclpy.ok():
			rclpy.init()

		self._node = rclpy.create_node(node_name)
		prefix = topic_prefix.strip("/")
		self._pub_pose = self._node.create_publisher(PoseStamped, f"{prefix}/eef_pose", 10)
		self._pub_pose_abs = self._node.create_publisher(PoseStamped, f"{prefix}/eef_pose_absolute", 10)
		self._pub_position = self._node.create_publisher(PoseStamped, f"{prefix}/eef_position", 10)
		self._pub_position_abs = self._node.create_publisher(PoseStamped, f"{prefix}/eef_position_absolute", 10)
		self._pub_twist = self._node.create_publisher(TwistStamped, f"{prefix}/eef_twist", 10)
		self._pub_imu = self._node.create_publisher(Imu, f"{prefix}/imu", 10)
		self._pub_joy = self._node.create_publisher(Joy, f"{prefix}/joy", 10)

		self._spin_thread = threading.Thread(target=self._spin, daemon=True)
		self._spin_thread.start()

		logger.info(f"ROS2Transport node '{node_name}' started on topic prefix '{prefix}'")

	def _spin(self):
		rclpy.spin(self._node)

	def _stamp(self):
		return self._node.get_clock().now().to_msg()

	def publish_eef_pose(self, x, y, z, qw, qx, qy, qz):
		msg = PoseStamped()
		msg.header.stamp = self._stamp()
		msg.header.frame_id = "base_link"
		msg.pose.position.x = float(x)
		msg.pose.position.y = float(y)
		msg.pose.position.z = float(z)
		msg.pose.orientation.w = float(qw)
		msg.pose.orientation.x = float(qx)
		msg.pose.orientation.y = float(qy)
		msg.pose.orientation.z = float(qz)
		self._pub_pose.publish(msg)

	def publish_eef_pose_absolute(self, x, y, z, qw, qx, qy, qz):
		msg = PoseStamped()
		msg.header.stamp = self._stamp()
		msg.header.frame_id = "base_link"
		msg.pose.position.x = float(x)
		msg.pose.position.y = float(y)
		msg.pose.position.z = float(z)
		msg.pose.orientation.w = float(qw)
		msg.pose.orientation.x = float(qx)
		msg.pose.orientation.y = float(qy)
		msg.pose.orientation.z = float(qz)
		self._pub_pose_abs.publish(msg)

	def publish_eef_position(self, x, y, z):
		msg = PoseStamped()
		msg.header.stamp = self._stamp()
		msg.header.frame_id = "base_link"
		msg.pose.position.x = float(x)
		msg.pose.position.y = float(y)
		msg.pose.position.z = float(z)
		msg.pose.orientation.w = 1.0
		msg.pose.orientation.x = 0.0
		msg.pose.orientation.y = 0.0
		msg.pose.orientation.z = 0.0
		self._pub_position.publish(msg)

	def publish_eef_position_absolute(self, x, y, z):
		msg = PoseStamped()
		msg.header.stamp = self._stamp()
		msg.header.frame_id = "base_link"
		msg.pose.position.x = float(x)
		msg.pose.position.y = float(y)
		msg.pose.position.z = float(z)
		msg.pose.orientation.w = 1.0
		msg.pose.orientation.x = 0.0
		msg.pose.orientation.y = 0.0
		msg.pose.orientation.z = 0.0
		self._pub_position_abs.publish(msg)

	def publish_eef_twist(self, vx, vy, vz, wx=0.0, wy=0.0, wz=0.0):
		msg = TwistStamped()
		msg.header.stamp = self._stamp()
		msg.header.frame_id = "base_link"
		msg.twist.linear.x = float(vx)
		msg.twist.linear.y = float(vy)
		msg.twist.linear.z = float(vz)
		msg.twist.angular.x = float(wx)
		msg.twist.angular.y = float(wy)
		msg.twist.angular.z = float(wz)
		self._pub_twist.publish(msg)

	def publish_imu(self, qw, qx, qy, qz, roll=0.0, pitch=0.0, yaw=0.0):
		msg = Imu()
		msg.header.stamp = self._stamp()
		msg.header.frame_id = "imu_link"
		msg.orientation.w = float(qw)
		msg.orientation.x = float(qx)
		msg.orientation.y = float(qy)
		msg.orientation.z = float(qz)
		self._pub_imu.publish(msg)

	def publish_buttons(self, sw, sw2, joy_x=0.0, joy_y=0.0, joy_z=0.0):
		msg = Joy()
		msg.header.stamp = self._stamp()
		msg.axes = [float(joy_x), float(joy_y), float(joy_z)]
		msg.buttons = [int(sw), int(sw2)]
		self._pub_joy.publish(msg)

	def shutdown(self):
		self._node.destroy_node()
		if rclpy.ok():
			rclpy.shutdown()
