"""Perception skeleton: obstacles / pile / unload point.

Inputs (later): camera images, /rslidar_points
Output: /loader/perception (JSON String stub)
"""

from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__('loader_perception_node')
        self.declare_parameter('publish_rate_hz', 5.0)
        rate = float(self.get_parameter('publish_rate_hz').value)

        self._sub_cam = self.create_subscription(
            String, '/camera/driver_status', self._on_cam, 10
        )
        # Real lidar: sensor_msgs/PointCloud2 on /rslidar_points — wire later.
        self._pub = self.create_publisher(String, '/loader/perception', 10)
        self._timer = self.create_timer(1.0 / max(rate, 0.1), self._tick)
        self._cam_ok = False
        self.get_logger().info('perception skeleton: publish /loader/perception')

    def _on_cam(self, msg: String) -> None:
        self._cam_ok = True

    def _tick(self) -> None:
        payload = {
            'obstacles': [],
            'pile': None,
            'unload_point': None,
            'camera_heartbeat': self._cam_ok,
            'status': 'stub',
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._pub.publish(msg)
        self._cam_ok = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
