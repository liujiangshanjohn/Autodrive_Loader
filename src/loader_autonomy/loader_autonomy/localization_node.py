"""Localization fusion skeleton: GNSS/INS + wheel speed + articulation.

Inputs (later): /chcnav/devpvt, chassis / CAN sensor topics
Output: /loader/localization (PoseStamped stub) + /loader/localization_status
"""

from __future__ import annotations

import json

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String

try:
    from cyber_msgs.msg import ChassisStatus
except ImportError:  # pragma: no cover - workspace may build msgs later
    ChassisStatus = None


class LocalizationNode(Node):
    def __init__(self) -> None:
        super().__init__('loader_localization_node')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('child_frame_id', 'base_link')
        rate = float(self.get_parameter('publish_rate_hz').value)
        self._frame = str(self.get_parameter('frame_id').value)

        if ChassisStatus is not None:
            self.create_subscription(
                ChassisStatus, '/chassis_status', self._on_chassis, 10
            )
        self._pose_pub = self.create_publisher(PoseStamped, '/loader/localization', 10)
        self._stat_pub = self.create_publisher(String, '/loader/localization_status', 10)
        self._timer = self.create_timer(1.0 / max(rate, 0.1), self._tick)
        self._have_chassis = False
        self.get_logger().info(
            'localization skeleton: fuse GNSS/INS + wheel + articulation (TODO)'
        )

    def _on_chassis(self, _msg) -> None:
        self._have_chassis = True

    def _tick(self) -> None:
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self._frame
        pose.pose.orientation.w = 1.0
        self._pose_pub.publish(pose)

        stat = String()
        stat.data = json.dumps(
            {
                'status': 'stub',
                'sources': ['gnss_ins', 'wheel_speed', 'articulation'],
                'chassis_seen': self._have_chassis,
            },
            ensure_ascii=False,
        )
        self._stat_pub.publish(stat)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalizationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
