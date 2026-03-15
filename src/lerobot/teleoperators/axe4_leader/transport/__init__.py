from .base import PoseTransport
from .udp_transport import UDPTransport

__all__ = ["PoseTransport", "UDPTransport", "create_transport"]


def create_transport(kind: str, **kwargs) -> PoseTransport:
    """Factory: create a transport by name.

    Args:
        kind: "ros2", "udp", or "none"
        **kwargs: forwarded to the transport constructor
    """
    if kind == "none":
        from .base import NullTransport
        return NullTransport()
    if kind == "udp":
        return UDPTransport(
            ip=kwargs.get("udp_ip", "127.0.0.1"),
            port=kwargs.get("udp_port", 5005),
            pose_only=kwargs.get("udp_pose_only", True),
        )
    if kind == "ros2":
        from .ros2_transport import ROS2Transport
        return ROS2Transport(
            node_name=kwargs.get("ros2_node_name", "axe4_teleop"),
        )
    raise ValueError(f"Unknown transport kind: {kind!r}. Use 'ros2', 'udp', or 'none'.")
