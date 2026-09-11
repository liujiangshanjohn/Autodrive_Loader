"""Control + mode arbitration skeleton.

Inputs: /loader/plan, manual takeover flag, chassis feedback
Output: /vehicle_command (only when enable_vehicle_command:=true)

Safety default: do NOT publish real /vehicle_command until ProtoAdapter is filled.
"""

from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class ControlNode(Node):
    def __init__(self) -> None:
        super().__init__('loader_control_node')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('enable_vehicle_command', False)
        self.declare_parameter('default_mode', 'auto')  # auto | manual | estop

        rate = float(self.get_parameter('publish_rate_hz').value)
        self._enable_cmd = bool(self.get_parameter('enable_vehicle_command').value)
        self._mode = str(self.get_parameter('default_mode').value)

        self._plan = '{}'
        self._manual = False
        self._estop = False

        self.create_subscription(String, '/loader/plan', self._on_plan, 10)
        self.create_subscription(Bool, '/loader/manual_takeover', self._on_manual, 10)
        self.create_subscription(Bool, '/loader/estop', self._on_estop, 10)
        self.create_subscription(String, '/loader/chassis_feedback_stub', self._on_fb, 10)

        self._status_pub = self.create_publisher(String, '/loader/control_status', 10)
        # Real path: cyber_msgs/ProtoAdapter on /vehicle_command — fill protobuf later.
        self._cmd_debug_pub = self.create_publisher(String, '/loader/vehicle_command_debug', 10)

        self._timer = self.create_timer(1.0 / max(rate, 0.1), self._tick)
        self.get_logger().warn(
            f'control skeleton: mode={self._mode}, '
            f'enable_vehicle_command={self._enable_cmd} '
            f'(keep false until ControlCommand protobuf is ready)'
        )

    def _on_plan(self, msg: String) -> None:
        self._plan = msg.data

    def _on_manual(self, msg: Bool) -> None:
        self._manual = bool(msg.data)

    def _on_estop(self, msg: Bool) -> None:
        self._estop = bool(msg.data)

    def _on_fb(self, _msg: String) -> None:
        pass

    def _arbitrate(self) -> str:
        if self._estop:
            return 'estop'
        if self._manual:
            return 'manual'
        return 'auto'

    def _tick(self) -> None:
        mode = self._arbitrate()
        try:
            plan = json.loads(self._plan) if self._plan else {}
        except json.JSONDecodeError:
            plan = {}

        status = {
            'mode': mode,
            'plan_phase': plan.get('phase', 'idle'),
            'speed_mps': 0.0 if mode != 'auto' else plan.get('speed_mps', 0.0),
            'enable_vehicle_command': self._enable_cmd,
            'status': 'stub',
        }
        s = String()
        s.data = json.dumps(status, ensure_ascii=False)
        self._status_pub.publish(s)
        self._cmd_debug_pub.publish(s)

        if self._enable_cmd:
            self.get_logger().error(
                'enable_vehicle_command=true but ProtoAdapter publish not implemented yet'
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
