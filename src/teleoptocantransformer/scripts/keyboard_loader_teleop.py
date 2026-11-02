#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
装载机键盘遥控：发布 JSON 到 /controls/teleop

按键：
  W/S     前进档+油门 / 后退档+油门（按住持续，松开约 0.2s 后回零）
  A/D     左转 / 右转
  空格    刹车
  I/K     大臂升 / 大臂落
  J/L     铲斗升 / 铲斗落
  H       喇叭开关
  T       照明开关
  N       空档并清零运动轴
  E       急停（继续运行）
  Esc / Q 急停并退出

实车前请先：
  1) ros2 launch cannode cannode.launch.py
  2) ros2 launch teleoptocantransformer teleop_converter_loader.launch.py
  3) python3 .../keyboard_loader_teleop.py
  4) ros2 topic echo /can_frame_sent   或   /cannode/vcu_feedback
"""

from __future__ import annotations

import json
import select
import sys
import termios
import threading
import time
import tty

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class KeyboardLoaderTeleop(Node):
    def __init__(self) -> None:
        super().__init__('keyboard_loader_teleop')

        self.declare_parameter('publish_hz', 20.0)
        self.declare_parameter('key_hold_sec', 0.22)
        self.declare_parameter('throttle_level', 0.4)
        self.declare_parameter('brake_level', 0.6)
        self.declare_parameter('steer_level', 0.5)
        self.declare_parameter('boom_level', 0.5)
        self.declare_parameter('bucket_level', 0.5)

        self.publish_hz = float(self.get_parameter('publish_hz').value)
        self.key_hold_sec = float(self.get_parameter('key_hold_sec').value)
        self.throttle_level = float(self.get_parameter('throttle_level').value)
        self.brake_level = float(self.get_parameter('brake_level').value)
        self.steer_level = float(self.get_parameter('steer_level').value)
        self.boom_level = float(self.get_parameter('boom_level').value)
        self.bucket_level = float(self.get_parameter('bucket_level').value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.pub = self.create_publisher(String, '/controls/teleop', qos)

        self.lock = threading.Lock()
        self.key_stamp: dict[str, float] = {}
        self.horn = False
        self.light = False
        self.estop = False
        self.running = True

        self._print_help()
        self.create_timer(1.0 / max(self.publish_hz, 1.0), self._on_timer)
        self._kb_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self._kb_thread.start()

    def _print_help(self) -> None:
        self.get_logger().info(
            '\n======== 装载机键盘遥控 ========\n'
            '  W/S   前进/后退\n'
            '  A/D   左转/右转\n'
            '  空格  刹车\n'
            '  I/K   大臂升/落\n'
            '  J/L   铲斗升/落\n'
            '  H/T   喇叭/照明开关\n'
            '  N     空档清零\n'
            '  E     急停\n'
            '  Esc/Q 急停退出\n'
            '================================'
        )

    def _activate(self, key: str, rivals: tuple[str, ...] = ()) -> None:
        now = time.monotonic()
        self.key_stamp[key] = now
        for r in rivals:
            self.key_stamp.pop(r, None)

    def _is_active(self, key: str, now: float) -> bool:
        ts = self.key_stamp.get(key)
        return ts is not None and (now - ts) <= self.key_hold_sec

    def _keyboard_loop(self) -> None:
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while self.running and rclpy.ok():
                r, _, _ = select.select([sys.stdin], [], [], 0.05)
                if not r:
                    continue
                ch = sys.stdin.read(1)
                if not ch:
                    continue
                if ord(ch) == 27:
                    with self.lock:
                        self.estop = True
                        self.running = False
                    break
                key = ch.lower()
                with self.lock:
                    if key == 'q':
                        self.estop = True
                        self.running = False
                        break
                    if key == 'e':
                        self.estop = True
                        self.key_stamp.clear()
                        continue
                    if key == 'n':
                        self.estop = False
                        self.key_stamp.clear()
                        continue
                    if key == 'h':
                        self.horn = not self.horn
                        self.get_logger().info(f'喇叭: {"开" if self.horn else "关"}')
                        continue
                    if key == 't':
                        self.light = not self.light
                        self.get_logger().info(f'照明: {"开" if self.light else "关"}')
                        continue

                    self.estop = False
                    if key == 'w':
                        self._activate('w', ('s',))
                    elif key == 's':
                        self._activate('s', ('w',))
                    elif key == 'a':
                        self._activate('a', ('d',))
                    elif key == 'd':
                        self._activate('d', ('a',))
                    elif key == 'i':
                        self._activate('i', ('k',))
                    elif key == 'k':
                        self._activate('k', ('i',))
                    elif key == 'j':
                        self._activate('j', ('l',))
                    elif key == 'l':
                        self._activate('l', ('j',))
                    elif key == ' ':
                        self._activate(' ')
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def _build_payload(self) -> dict:
        now = time.monotonic()
        with self.lock:
            horn = self.horn
            light = self.light
            estop = self.estop
            w = self._is_active('w', now)
            s = self._is_active('s', now)
            a = self._is_active('a', now)
            d = self._is_active('d', now)
            i = self._is_active('i', now)
            k = self._is_active('k', now)
            j = self._is_active('j', now)
            l = self._is_active('l', now)
            space = self._is_active(' ', now)

        if estop:
            return {
                'gear': 'N',
                'throttle': 0.0,
                'brake': 1.0,
                'steering': 0.0,
                'boom': 0.0,
                'bucket': 0.0,
                'horn': False,
                'light': light,
                'emergency_stop': True,
            }

        gear = 'N'
        throttle = 0.0
        if w:
            gear = 'D'
            throttle = self.throttle_level
        elif s:
            gear = 'R'
            throttle = self.throttle_level

        steering = 0.0
        if a:
            steering = -self.steer_level
        elif d:
            steering = self.steer_level

        boom = self.boom_level if i else (-self.boom_level if k else 0.0)
        bucket = self.bucket_level if j else (-self.bucket_level if l else 0.0)
        brake = self.brake_level if space else 0.0

        return {
            'gear': gear,
            'throttle': throttle,
            'brake': brake,
            'steering': steering,
            'boom': boom,
            'bucket': bucket,
            'horn': horn,
            'light': light,
            'emergency_stop': False,
        }

    def _publish_estop(self) -> None:
        msg = String()
        msg.data = json.dumps(
            {
                'gear': 'N',
                'throttle': 0.0,
                'brake': 1.0,
                'steering': 0.0,
                'boom': 0.0,
                'bucket': 0.0,
                'horn': False,
                'light': False,
                'emergency_stop': True,
            }
        )
        self.pub.publish(msg)

    def _on_timer(self) -> None:
        if not self.running:
            self._publish_estop()
            rclpy.shutdown()
            return
        msg = String()
        msg.data = json.dumps(self._build_payload())
        self.pub.publish(msg)


def main() -> None:
    if not sys.stdin.isatty():
        print('错误：需要交互式终端运行键盘遥控', file=sys.stderr)
        sys.exit(1)

    rclpy.init()
    node = KeyboardLoaderTeleop()
    try:
        while rclpy.ok() and node.running:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        with node.lock:
            node.estop = True
            node.running = False
        node._publish_estop()
        time.sleep(0.05)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
