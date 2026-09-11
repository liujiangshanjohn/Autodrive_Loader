"""Camera driver skeleton for wheel-loader (GMSL TBD)."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class CameraDriverNode(Node):
    """Placeholder: later publish sensor_msgs/Image (or compressed) per camera."""

    def __init__(self) -> None:
        super().__init__('camera_driver_node')
        self.declare_parameter('camera_count', 5)
        self.declare_parameter('frame_id', 'camera_main')
        self.declare_parameter('publish_rate_hz', 10.0)

        self._count = int(self.get_parameter('camera_count').value)
        rate = float(self.get_parameter('publish_rate_hz').value)
        self._status_pub = self.create_publisher(String, '/camera/driver_status', 10)
        self._timer = self.create_timer(max(1.0 / max(rate, 0.1), 0.05), self._on_timer)
        self.get_logger().info(
            f'camera_driver skeleton up: cameras={self._count} (GMSL capture TODO)'
        )

    def _on_timer(self) -> None:
        msg = String()
        msg.data = f'camera_driver stub ok; cameras={self._count}; capture not wired'
        self._status_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraDriverNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
