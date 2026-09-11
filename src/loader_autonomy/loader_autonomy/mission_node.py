"""Mission V-cycle skeleton: approach pile → scoop → lift → unload → loop.

Inputs: /loader/perception, /loader/localization
Output: /loader/mission_state, /loader/mission_goal
"""

from __future__ import annotations

import json
from enum import Enum

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String


class Phase(str, Enum):
    IDLE = 'idle'
    APPROACH_PILE = 'approach_pile'
    SCOOP = 'scoop'
    LIFT = 'lift'
    HAUL = 'haul'
    UNLOAD = 'unload'


class MissionNode(Node):
    def __init__(self) -> None:
        super().__init__('loader_mission_node')
        self.declare_parameter('publish_rate_hz', 5.0)
        self.declare_parameter('auto_advance_stub', False)
        rate = float(self.get_parameter('publish_rate_hz').value)
        self._auto = bool(self.get_parameter('auto_advance_stub').value)

        self._phase = Phase.IDLE
        self._tick_count = 0

        self.create_subscription(String, '/loader/perception', self._on_perception, 10)
        self.create_subscription(PoseStamped, '/loader/localization', self._on_pose, 10)

        self._state_pub = self.create_publisher(String, '/loader/mission_state', 10)
        self._goal_pub = self.create_publisher(String, '/loader/mission_goal', 10)
        self._timer = self.create_timer(1.0 / max(rate, 0.1), self._tick)

        self._last_perception = '{}'
        self._have_pose = False
        self.get_logger().info('mission V-cycle skeleton: IDLE (wire transitions later)')

    def _on_perception(self, msg: String) -> None:
        self._last_perception = msg.data

    def _on_pose(self, _msg: PoseStamped) -> None:
        self._have_pose = True

    def _tick(self) -> None:
        self._tick_count += 1
        if self._auto and self._tick_count % 50 == 0:
            order = list(Phase)
            idx = order.index(self._phase)
            self._phase = order[(idx + 1) % len(order)]

        state = {
            'phase': self._phase.value,
            'have_pose': self._have_pose,
            'perception_echo': self._last_perception[:120],
            'status': 'stub',
        }
        s = String()
        s.data = json.dumps(state, ensure_ascii=False)
        self._state_pub.publish(s)

        goal = String()
        goal.data = json.dumps(
            {
                'phase': self._phase.value,
                'target': None,
                'note': 'planning consumes this goal',
            },
            ensure_ascii=False,
        )
        self._goal_pub.publish(goal)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
