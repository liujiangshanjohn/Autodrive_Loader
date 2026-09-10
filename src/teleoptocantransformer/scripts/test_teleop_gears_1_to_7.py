#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
向 /controls/teleop 发布测试 JSON，固定发送 2 档。

用途：
1) 验证 teleop_converter_xiaosong 对 gear 字段的处理
2) 验证 cannode 对 pto_cmd -> 0x18FD0821 的映射
"""

import argparse
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String as StringMsg


class TeleopGearPublisher(Node):
    def __init__(
        self,
        publish_interval_sec: float,
        gear_switch_interval_sec: float,
        loop: bool,
        gear: int,
        cycle_gears: bool,
    ):
        super().__init__("teleop_gear_1_to_7_tester")

        qos_profile_teleop = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.pub = self.create_publisher(StringMsg, "/controls/teleop", qos_profile_teleop)

        self.publish_interval_sec = publish_interval_sec
        self.gear_switch_interval_sec = gear_switch_interval_sec
        self.loop = loop
        self.fixed_gear = gear
        self.cycle_gears = cycle_gears
        self.cycle_index = 0
        self.gear_sequence = list(range(1, 8))
        self.current_gear = self.gear_sequence[0] if self.cycle_gears else self.fixed_gear
        self.last_switch_time = time.time()

    def build_payload(self, gear: int) -> dict:
        # 仅保留与本测试相关的最小字段集合
        return {
            "request_remote_control": True,
            "emergency_stop": False,
            "parking_brake": False,
            "gear": gear,
        }

    def publish_once(self):
        gear = self.fixed_gear
        if self.cycle_gears:
            now = time.time()
            if (now - self.last_switch_time) >= self.gear_switch_interval_sec:
                self.cycle_index = (self.cycle_index + 1) % len(self.gear_sequence)
                self.current_gear = self.gear_sequence[self.cycle_index]
                self.last_switch_time = now
            gear = self.current_gear

        payload = self.build_payload(gear)
        msg = StringMsg()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.pub.publish(msg)
        self.get_logger().info(
            f"已发送 /controls/teleop: gear={gear}, payload={msg.data}"
        )

        end_time = time.time() + self.publish_interval_sec
        while rclpy.ok() and time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.01)

    def run(self):
        mode_desc = "循环档位 1~7" if self.cycle_gears else f"固定 gear={self.fixed_gear}"
        self.get_logger().info(
            f"开始发送 {mode_desc}，发送间隔 {self.publish_interval_sec:.3f}s"
            f"{', 档位切换间隔 ' + format(self.gear_switch_interval_sec, '.2f') + 's' if self.cycle_gears else ''}，"
            f"循环模式: {'开启' if self.loop else '关闭'}"
        )
        while rclpy.ok():
            self.publish_once()
            if not self.loop:
                break
        self.get_logger().info("测试结束。")


def parse_args():
    parser = argparse.ArgumentParser(
        description="发布 /controls/teleop 测试消息，固定发送指定档位（默认 2）"
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=0.05,
        help="发送间隔（秒），默认 0.05（20Hz）",
    )
    parser.add_argument(
        "--gear-switch-interval",
        type=float,
        default=3.0,
        help="循环档位时的切换间隔（秒），默认 3.0",
    )
    parser.add_argument(
        "--loop",
        action="store_true",
        help="循环发送（默认只发一轮）",
    )
    parser.add_argument(
        "--gear",
        type=int,
        default=2,
        help="固定发送的档位，范围 1~7，默认 2",
    )
    parser.add_argument(
        "--cycle-gears",
        action="store_true",
        help="按 1~7 档循环发送（开启后忽略 --gear）",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    fixed_gear = min(max(args.gear, 1), 7)
    node = TeleopGearPublisher(
        publish_interval_sec=max(args.interval, 0.01),
        gear_switch_interval_sec=max(args.gear_switch_interval, 0.01),
        loop=args.loop,
        gear=fixed_gear,
        cycle_gears=args.cycle_gears,
    )
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
