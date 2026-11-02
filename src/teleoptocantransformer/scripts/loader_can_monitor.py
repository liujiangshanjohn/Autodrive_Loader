#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
监控装载机 CAN 下发与反馈：
  - /can_frame_sent : cannode 实际发出的 0x100/0x101/0x102
  - /cannode/vcu_feedback : 解析后的前进/后退/转向等反馈
"""

from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def _u16_le(data: list[int], idx: int) -> int:
    if idx + 1 >= len(data):
        return 0
    return data[idx] | (data[idx + 1] << 8)


class LoaderCanMonitor(Node):
    def __init__(self) -> None:
        super().__init__('loader_can_monitor')
        self.create_subscription(String, '/can_frame_sent', self._on_sent, 10)
        self.create_subscription(String, '/cannode/vcu_feedback', self._on_fb, 10)
        self.get_logger().info('监听 /can_frame_sent 与 /cannode/vcu_feedback ...')

    def _on_sent(self, msg: String) -> None:
        try:
            obj = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        can_id = int(obj.get('can_id_dec', 0))
        data = obj.get('data', [])
        if not isinstance(data, list) or len(data) < 8:
            return

        if can_id == 0x100:
            b0 = int(data[0])
            hb = (int(data[7]) >> 7) & 1
            self.get_logger().info(
                f'[TX 0x100] 前进={b0 & 1} 后退={(b0 >> 1) & 1} '
                f'喇叭={(b0 >> 2) & 1} 照明={(b0 >> 3) & 1} 心跳={hb}'
            )
        elif can_id == 0x101:
            thr = _u16_le(data, 0)
            brk = _u16_le(data, 2)
            left = _u16_le(data, 4)
            right = _u16_le(data, 6)
            self.get_logger().info(
                f'[TX 0x101] 油门={thr}({thr / 10.0:.1f}%) 刹车={brk}({brk / 10.0:.1f}%) '
                f'左转={left} 右转={right}'
            )
        elif can_id == 0x102:
            bu = _u16_le(data, 0)
            bd = _u16_le(data, 2)
            ku = _u16_le(data, 4)
            kd = _u16_le(data, 6)
            self.get_logger().info(
                f'[TX 0x102] 大臂升={bu} 落={bd} 铲斗升={ku} 落={kd}'
            )

    def _on_fb(self, msg: String) -> None:
        try:
            obj = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if obj.get('protocol') != 'sjtu_loader':
            return
        self.get_logger().info(
            f'[FB] 设定前进={obj.get("fb_forward")} 后退={obj.get("fb_backward")} | '
            f'输出前进={obj.get("out_forward")} 后退={obj.get("out_backward")} | '
            f'自动={obj.get("auto_mode")} 油门出={obj.get("throttle_out")} '
            f'左转电流={obj.get("left_steer_current_mA")}mA '
            f'右转电流={obj.get("right_steer_current_mA")}mA '
            f'超时={obj.get("vcu_communication_timeout")}'
        )


def main() -> None:
    rclpy.init()
    node = LoaderCanMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
