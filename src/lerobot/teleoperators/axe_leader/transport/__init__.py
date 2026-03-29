"""Transport wrappers for axe_leader (reusing AXE4 transport implementation)."""

from .base import PoseTransport
from .udp_transport import UDPTransport

__all__ = ["PoseTransport", "UDPTransport", "create_transport"]


def create_transport(kind: str, **kwargs) -> PoseTransport:
    from .base import NullTransport
    from .ros2_transport import ROS2Transport

    if kind == "none":
        return NullTransport()
    if kind == "udp":
        return UDPTransport(
            ip=kwargs.get("udp_ip", "127.0.0.1"),
            port=kwargs.get("udp_port", 5005),
            pose_only=kwargs.get("udp_pose_only", True),
            print_packets=kwargs.get("udp_print_packets", False),
        )
    if kind == "ros2":
        return ROS2Transport(
            node_name=kwargs.get("ros2_node_name", "axe_teleop"),
            topic_prefix=kwargs.get("ros2_topic_prefix", "axe"),
        )
    raise ValueError(f"Unknown transport kind: {kind!r}. Use 'ros2', 'udp', or 'none'.")
