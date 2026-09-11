"""Planning skeleton: path / speed / bucket / local avoid.

Input: /loader/mission_goal (+ localization later)
Output: /loader/plan
"""

from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PlanningNode(Node):
    def __init__(self) -> None:
        super().__init__('loader_planning_node')
        self.declare_parameter('publish_rate_hz', 10.0)
        rate = float(self.get_parameter('publish_rate_hz').value)

        self._goal = '{}'
        self.create_subscription(String, '/loader/mission_goal', self._on_goal, 10)
        self._pub = self.create_publisher(String, '/loader/plan', 10)
        self._timer = self.create_timer(1.0 / max(rate, 0.1), self._tick)
        self.get_logger().info('planning skeleton: /loader/mission_goal → /loader/plan')

    def _on_goal(self, msg: String) -> None:
        self._goal = msg.data

    def _tick(self) -> None:
        try:
            goal = json.loads(self._goal) if self._goal else {}
        except json.JSONDecodeError:
            goal = {'raw': self._goal}

        plan = {
            'phase': goal.get('phase', 'idle'),
            'path': [],
            'speed_mps': 0.0,
            'bucket_cmd': None,
            'status': 'stub',
        }
        msg = String()
        msg.data = json.dumps(plan, ensure_ascii=False)
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlanningNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
