#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
无 CAN 卡离线测试：模拟键盘指令 → 打印 teleop JSON 与将下发的 CAN 帧（0x100/0x101/0x102）

用法：
  # 1) 纯离线演示（不依赖 ROS / 不需 CAN）
  python3 src/teleoptocantransformer/scripts/test_loader_keyboard_offline.py

  # 2) 同时发布到 ROS，便于另一终端 echo
  python3 src/teleoptocantransformer/scripts/test_loader_keyboard_offline.py --ros

  # 3) 交互键盘（需 tty）
  python3 src/teleoptocantransformer/scripts/test_loader_keyboard_offline.py --interactive
"""

from __future__ import annotations

import argparse
import json
import select
import struct
import sys
import termios
import time
import tty


def clamp01(v: float) -> float:
    return max(0.0, min(1.0, v))


def clamp11(v: float) -> float:
    return max(-1.0, min(1.0, v))


def pct_to_u16(pct_0_100: float) -> int:
    return max(0, min(1000, int(round(pct_0_100 * 10.0))))


def axis_to_pair(axis: float) -> tuple[int, int]:
    axis = clamp11(axis)
    if axis > 0:
        return int(round(axis * 1000)), 0
    if axis < 0:
        return 0, int(round((-axis) * 1000))
    return 0, 0


def build_teleop(
    gear: str = 'N',
    throttle: float = 0.0,
    brake: float = 0.0,
    steering: float = 0.0,
    boom: float = 0.0,
    bucket: float = 0.0,
    horn: bool = False,
    light: bool = False,
    emergency_stop: bool = False,
) -> dict:
    return {
        'gear': gear,
        'throttle': clamp01(throttle),
        'brake': clamp01(brake),
        'steering': clamp11(steering),
        'boom': clamp11(boom),
        'bucket': clamp11(bucket),
        'horn': bool(horn),
        'light': bool(light),
        'emergency_stop': bool(emergency_stop),
    }


def encode_can(teleop: dict, heartbeat: bool = True) -> dict[str, bytes]:
    """按 LoaderCANParser / 协议映射生成 0x100/0x101/0x102 共 8 字节帧。"""
    estop = bool(teleop.get('emergency_stop'))
    gear = str(teleop.get('gear', 'N')).upper()

    # 0x100
    b0 = 0
    if not estop:
        if gear == 'D':
            b0 |= 0x01
        elif gear == 'R':
            b0 |= 0x02
        if teleop.get('horn'):
            b0 |= 0x04
        if teleop.get('light'):
            b0 |= 0x08
    frame100 = bytearray(8)
    frame100[0] = b0
    if heartbeat:
        frame100[7] |= 0x80

    # 0x101
    if estop:
        thr = 0
        brk = 1000
        left = right = 0
    else:
        thr = pct_to_u16(float(teleop.get('throttle', 0.0)) * 100.0)
        brk = pct_to_u16(float(teleop.get('brake', 0.0)) * 100.0)
        steer = float(teleop.get('steering', 0.0))
        if steer > 0:
            left, right = 0, pct_to_u16(steer * 100.0)
        elif steer < 0:
            left, right = pct_to_u16((-steer) * 100.0), 0
        else:
            left = right = 0
    frame101 = struct.pack('<HHHH', thr, brk, left, right)

    # 0x102
    if estop:
        boom_up = boom_down = bucket_up = bucket_down = 0
    else:
        boom_up, boom_down = axis_to_pair(float(teleop.get('boom', 0.0)))
        bucket_up, bucket_down = axis_to_pair(float(teleop.get('bucket', 0.0)))
    frame102 = struct.pack('<HHHH', boom_up, boom_down, bucket_up, bucket_down)

    return {
        '0x100': bytes(frame100),
        '0x101': bytes(frame101),
        '0x102': bytes(frame102),
    }


def fmt_hex(data: bytes) -> str:
    return ' '.join(f'{b:02X}' for b in data)


def decode_explain(can_id: str, data: bytes) -> str:
    if can_id == '0x100':
        b0, b7 = data[0], data[7]
        return (
            f"前进={(b0 >> 0) & 1} 后退={(b0 >> 1) & 1} "
            f"喇叭={(b0 >> 2) & 1} 照明={(b0 >> 3) & 1} 心跳={(b7 >> 7) & 1}"
        )
    if can_id == '0x101':
        thr, brk, left, right = struct.unpack('<HHHH', data)
        return (
            f"油门={thr}({thr / 10:.1f}%) 刹车={brk}({brk / 10:.1f}%) "
            f"左转={left} 右转={right}"
        )
    if can_id == '0x102':
        bu, bd, ku, kd = struct.unpack('<HHHH', data)
        return f"大臂升={bu} 落={bd} 铲斗升={ku} 落={kd}"
    return ''


def print_frame(step: str, teleop: dict) -> None:
    frames = encode_can(teleop)
    print('\n' + '=' * 64)
    print(f'[{step}] teleop JSON:')
    print(json.dumps(teleop, ensure_ascii=False))
    print('将下发 CAN（无卡时仅打印，不写总线）:')
    for cid in ('0x100', '0x101', '0x102'):
        data = frames[cid]
        print(f'  {cid}: [{fmt_hex(data)}]  {decode_explain(cid, data)}')


def run_demo() -> None:
    seq = [
        ('空闲/空档', build_teleop()),
        ('按住 W 前进', build_teleop(gear='D', throttle=0.4)),
        ('W + D 前进并右转', build_teleop(gear='D', throttle=0.4, steering=0.5)),
        ('按住 S 后退', build_teleop(gear='R', throttle=0.4)),
        ('A 左转', build_teleop(steering=-0.5)),
        ('空格刹车', build_teleop(brake=0.6)),
        ('I 大臂升', build_teleop(boom=0.5)),
        ('K 大臂落', build_teleop(boom=-0.5)),
        ('J 铲斗升', build_teleop(bucket=0.5)),
        ('L 铲斗落', build_teleop(bucket=-0.5)),
        ('喇叭+照明', build_teleop(horn=True, light=True)),
        ('急停 E', build_teleop(emergency_stop=True, brake=1.0)),
    ]
    print('离线演示：模拟键盘序列 → teleop → CAN 帧内容')
    for name, payload in seq:
        print_frame(name, payload)
        time.sleep(0.15)
    print('\n演示结束。接 CAN 卡后请走完整链路（见 README_LOADER.md）。')


def run_ros_demo() -> None:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    rclpy.init()
    node = Node('loader_keyboard_offline_demo')
    qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )
    pub = node.create_publisher(String, '/controls/teleop', qos)

    seq = [
        ('W 前进', build_teleop(gear='D', throttle=0.4)),
        ('W+D 前进右转', build_teleop(gear='D', throttle=0.4, steering=0.5)),
        ('S 后退', build_teleop(gear='R', throttle=0.4)),
        ('A 左转', build_teleop(steering=-0.5)),
        ('刹车', build_teleop(brake=0.6)),
        ('急停', build_teleop(emergency_stop=True, brake=1.0)),
        ('回空档', build_teleop()),
    ]

    print('已发布到 /controls/teleop，可用另一终端：')
    print('  ros2 topic echo /controls/teleop')
    for name, payload in seq:
        print_frame(name, payload)
        msg = String()
        msg.data = json.dumps(payload)
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.8)

    node.destroy_node()
    rclpy.shutdown()


def run_interactive() -> None:
    if not sys.stdin.isatty():
        print('需要交互式终端', file=sys.stderr)
        sys.exit(1)

    levels = {
        'throttle': 0.4,
        'brake': 0.6,
        'steer': 0.5,
        'boom': 0.5,
        'bucket': 0.5,
    }
    # 按一下保持该动作 hold_sec 秒；长按靠系统按键重复刷新
    hold: dict[str, float] = {}
    hold_sec = 0.45
    toggles = {'horn': False, 'light': False, 'emergency_stop': False}
    last_payload_str = ''

    print('交互模式：请先用鼠标点一下本终端窗口，再按键')
    print('  W/S 前进/后退  A/D 左转/右转  空格刹车')
    print('  I/K 大臂升/落  J/L 铲斗升/落  H喇叭 T照明 N空档 E急停')
    print('  Esc 或 Q 退出')
    print('(无 CAN，仅在状态变化时打印)\n')
    sys.stdout.flush()

    def current_payload() -> dict:
        now = time.monotonic()

        def active(k: str) -> bool:
            ts = hold.get(k)
            return ts is not None and (now - ts) <= hold_sec

        if toggles['emergency_stop']:
            return build_teleop(emergency_stop=True, brake=1.0, light=toggles['light'])

        gear, throttle = 'N', 0.0
        if active('w'):
            gear, throttle = 'D', levels['throttle']
        elif active('s'):
            gear, throttle = 'R', levels['throttle']

        steering = 0.0
        if active('a'):
            steering = -levels['steer']
        elif active('d'):
            steering = levels['steer']

        boom = levels['boom'] if active('i') else (-levels['boom'] if active('k') else 0.0)
        bucket = levels['bucket'] if active('j') else (-levels['bucket'] if active('l') else 0.0)
        brake = levels['brake'] if active(' ') else 0.0

        return build_teleop(
            gear=gear,
            throttle=throttle,
            brake=brake,
            steering=steering,
            boom=boom,
            bucket=bucket,
            horn=toggles['horn'],
            light=toggles['light'],
            emergency_stop=False,
        )

    def maybe_print(tag: str) -> None:
        nonlocal last_payload_str
        payload = current_payload()
        s = json.dumps(payload, sort_keys=True)
        if s != last_payload_str:
            last_payload_str = s
            print_frame(tag, payload)
            sys.stdout.flush()

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)  # raw 比 cbreak 更易在 IDE 终端收到按键
        maybe_print('就绪(空档)')
        while True:
            r, _, _ = select.select([sys.stdin], [], [], 0.05)
            if not r:
                # 按键超时回零时也刷新一次
                maybe_print('松开回零')
                continue

            ch = sys.stdin.read(1)
            if not ch:
                continue
            code = ord(ch)
            # Ctrl-C
            if code == 3:
                break
            if code == 27 or ch.lower() == 'q':
                toggles['emergency_stop'] = True
                hold.clear()
                maybe_print('退出急停')
                break

            key = ch.lower()
            now = time.monotonic()
            if key == 'e':
                toggles['emergency_stop'] = True
                hold.clear()
                maybe_print(f'键[{key}]')
            elif key == 'n':
                toggles['emergency_stop'] = False
                hold.clear()
                maybe_print(f'键[{key}]')
            elif key == 'h':
                toggles['horn'] = not toggles['horn']
                maybe_print(f'键[{key}]')
            elif key == 't':
                toggles['light'] = not toggles['light']
                maybe_print(f'键[{key}]')
            elif key in ('w', 's', 'a', 'd', 'i', 'k', 'j', 'l', ' '):
                toggles['emergency_stop'] = False
                hold[key] = now
                rivals = {
                    'w': ('s',), 's': ('w',),
                    'a': ('d',), 'd': ('a',),
                    'i': ('k',), 'k': ('i',),
                    'j': ('l',), 'l': ('j',),
                }.get(key, ())
                for rv in rivals:
                    hold.pop(rv, None)
                maybe_print(f'键[{key if key != " " else "空格"}]')
            else:
                # 忽略未映射键，但提示一次便于排查“按了没反应”
                print(f'(忽略按键 code={code!r} ch={ch!r})')
                sys.stdout.flush()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        print('\n已退出交互模式')


def main() -> None:
    parser = argparse.ArgumentParser(description='装载机键盘→CAN 离线测试')
    parser.add_argument('--ros', action='store_true', help='发布到 /controls/teleop')
    parser.add_argument('--interactive', '-i', action='store_true', help='交互键盘')
    args = parser.parse_args()
    if args.interactive:
        run_interactive()
    elif args.ros:
        run_ros_demo()
    else:
        run_demo()


if __name__ == '__main__':
    main()
