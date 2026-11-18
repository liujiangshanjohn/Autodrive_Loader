#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
装载机键盘遥控可视化界面（无需编译 / 无需 CAN）

运行：
  python3 src/teleoptocantransformer/scripts/loader_keyboard_gui.py

默认在 Windows 浏览器打开（WSL 任务栏企鹅图标点不开 Tk 窗口）。
加 --tk 才用本机 Tk。加 --ros 同时发布 /controls/teleop（走 Tk）。
"""

from __future__ import annotations

import argparse
import json
import math
import os
import struct
import subprocess
import sys
import threading
import time
import tkinter as tk
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont, ImageTk

_SCRIPTS = Path(__file__).resolve().parent
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))
from loader_hydro import HydroPlant


def _find_cjk_font_path() -> Path | None:
    """优先使用仓库自带 Noto 简体字体。"""
    here = Path(__file__).resolve()
    candidates = [
        here.parents[3] / 'docs' / 'fonts' / 'NotoSansSC-Regular.ttf',
        here.parents[3] / 'docs' / 'fonts' / 'NotoSansSC-Regular.otf',
        Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'),
        Path('/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc'),
        Path('/usr/share/fonts/truetype/wqy/wqy-microhei.ttc'),
    ]
    for p in candidates:
        if p.is_file():
            return p
    return None


class CjkFonts:
    """用 Pillow 加载 TTF，绕过 WSL/Tk 缺中文字体导致的方框乱码。"""

    def __init__(self) -> None:
        self.path = _find_cjk_font_path()
        self._cache: dict[int, ImageFont.FreeTypeFont | ImageFont.ImageFont] = {}
        self._photo_keep: list[ImageTk.PhotoImage] = []  # 防止被 GC
        if self.path is None:
            print('警告: 未找到中文字体，界面将尽量用英文显示')
        else:
            print(f'使用中文字体: {self.path}')

    def pil(self, size: int):
        if size not in self._cache:
            if self.path is not None:
                self._cache[size] = ImageFont.truetype(str(self.path), size)
            else:
                self._cache[size] = ImageFont.load_default()
        return self._cache[size]

    def text_image(
        self,
        text: str,
        size: int = 14,
        fill: str = '#e8eef7',
        bg: str | None = '#1a2332',
        max_width: int | None = None,
    ) -> ImageTk.PhotoImage:
        font = self.pil(size)
        # 估算尺寸
        tmp = Image.new('RGBA', (4, 4), (0, 0, 0, 0))
        d = ImageDraw.Draw(tmp)
        bbox = d.multiline_textbbox((0, 0), text, font=font, spacing=4)
        tw = max(1, bbox[2] - bbox[0] + 4)
        th = max(1, bbox[3] - bbox[1] + 4)
        if max_width is not None:
            tw = min(tw, max_width)
        if bg is None:
            img = Image.new('RGBA', (tw, th), (0, 0, 0, 0))
        else:
            # hex bg
            img = Image.new('RGB', (tw, th), bg)
        draw = ImageDraw.Draw(img)
        draw.multiline_text((-bbox[0] + 2, -bbox[1] + 2), text, font=font, fill=fill, spacing=4)
        photo = ImageTk.PhotoImage(img)
        self._photo_keep.append(photo)
        # 限制缓存，避免无限增长
        if len(self._photo_keep) > 64:
            self._photo_keep = self._photo_keep[-32:]
        return photo

    def set_label(
        self,
        label: tk.Label,
        text: str,
        size: int = 14,
        fill: str = '#e8eef7',
        bg: str = '#1a2332',
    ) -> None:
        photo = self.text_image(text, size=size, fill=fill, bg=bg)
        label.configure(image=photo, text='')
        label.image = photo  # type: ignore[attr-defined]

    def set_button(
        self,
        button: tk.Button,
        text: str,
        size: int = 13,
        fill: str = '#ffffff',
        bg: str = '#8b2e2e',
    ) -> None:
        photo = self.text_image(text, size=size, fill=fill, bg=bg)
        button.configure(image=photo, compound=tk.CENTER, text='')
        button.image = photo  # type: ignore[attr-defined]


# ---------- 协议编码（与离线脚本一致） ----------

def build_teleop(**kwargs) -> dict:
    return {
        'gear': kwargs.get('gear', 'N'),
        'throttle': float(kwargs.get('throttle', 0.0)),
        'brake': float(kwargs.get('brake', 0.0)),
        'steering': float(kwargs.get('steering', 0.0)),
        'boom': float(kwargs.get('boom', 0.0)),
        'bucket': float(kwargs.get('bucket', 0.0)),
        'horn': bool(kwargs.get('horn', False)),
        'light': bool(kwargs.get('light', False)),
        'emergency_stop': bool(kwargs.get('emergency_stop', False)),
    }


def pct_to_u16(pct_0_100: float) -> int:
    return max(0, min(1000, int(round(pct_0_100 * 10.0))))


def unit_to_valve(unit_0_1: float) -> int:
    if unit_0_1 <= 0.0:
        return 0
    return int(round(400.0 + 300.0 * max(0.0, min(1.0, unit_0_1))))


def axis_to_pair(axis: float) -> tuple[int, int]:
    if axis > 0:
        return unit_to_valve(axis), 0
    if axis < 0:
        return 0, unit_to_valve(-axis)
    return 0, 0


def encode_can(teleop: dict, heartbeat: bool = True) -> dict[str, str]:
    estop = bool(teleop.get('emergency_stop'))
    gear = str(teleop.get('gear', 'N')).upper()
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
    f100 = bytearray(8)
    f100[0] = b0
    if heartbeat:
        f100[7] |= 0x80

    if estop:
        thr, brk, left, right = 0, 1000, 0, 0
        bu = bd = ku = kd = 0
    else:
        thr = pct_to_u16(float(teleop['throttle']) * 100.0)
        brk = pct_to_u16(float(teleop['brake']) * 100.0)
        steer = float(teleop['steering'])
        if steer > 0:
            left, right = 0, pct_to_u16(steer * 100.0)
        elif steer < 0:
            left, right = pct_to_u16((-steer) * 100.0), 0
        else:
            left = right = 0
        if teleop.get('boom_valve_up') is not None:
            bu = int(round(float(teleop['boom_valve_up'])))
            bd = int(round(float(teleop['boom_valve_down'])))
            ku = int(round(float(teleop['bucket_valve_up'])))
            kd = int(round(float(teleop['bucket_valve_down'])))
        else:
            bu, bd = axis_to_pair(float(teleop['boom']))
            ku, kd = axis_to_pair(float(teleop['bucket']))

    f101 = struct.pack('<HHHH', thr, brk, left, right)
    f102 = struct.pack('<HHHH', bu, bd, ku, kd)

    def hx(b: bytes) -> str:
        return ' '.join(f'{x:02X}' for x in b)

    return {
        '0x100': hx(f100),
        '0x101': hx(f101),
        '0x102': hx(f102),
        'explain': (
            f"前进={b0 & 1} 后退={(b0 >> 1) & 1} 油门={thr} 刹车={brk} "
            f"左转={left} 右转={right} 大臂{bu}/{bd} 铲斗{ku}/{kd}"
        ),
    }


class LoaderKeyboardGui:
    def __init__(self, enable_ros: bool = False):
        self.enable_ros = enable_ros
        self.ros_pub = None
        self.ros_node = None
        self._ros_thread = None

        # 控制量
        self.keys_down: set[str] = set()
        self.horn = False
        self.light = False
        self.estop = False
        self.throttle_level = 0.4
        self.steer_level = 0.55
        self.brake_level = 0.6
        self.boom_level = 0.6
        self.bucket_level = 0.6

        # 仿真姿态（俯视图）——先占位，首帧画布就绪后居中
        self.x = 0.0
        self.y = 0.0
        self._centered = False
        self.yaw = -math.pi / 2  # 车头朝上
        self.articulation = 0.0  # 铰接角（转向视觉）
        # 侧视图由油缸行程→关节角驱动
        self.hydro = HydroPlant()
        self.boom_angle = math.radians(self.hydro.boom_angle_deg())
        self.bucket_angle = math.radians(self.hydro.bucket_angle_deg())
        self.wheel_spin = 0.0
        self._hydro_state: dict = {}

        self.root = tk.Tk()
        # 标题栏在部分环境下也不支持中文，用英文避免方框
        self.root.title('Loader Keyboard Teleop 20261102')
        try:
            self.root.wm_class('LoaderTeleop', 'LoaderTeleop')
        except tk.TclError:
            pass
        self.root.geometry('1100x720+80+80')
        self.root.configure(bg='#1a2332')
        self.root.minsize(900, 600)
        self._force_front()

        self.fonts = CjkFonts()
        # 画布静态中文贴图（避免每帧重绘文字）
        self._top_tip_img = None
        self._side_tip_img = None

        self._build_ui()
        self._bind_keys()

        if enable_ros:
            self._start_ros()

        self._last_tick = time.monotonic()
        self._last_status_text = ''
        self._status_pending = False
        # 预热字体，避免第一次按键时卡顿
        self.fonts.pil(12)
        self.fonts.pil(18)
        self.root.after(16, self._tick)
        self.root.after(200, self._force_front)

    def _force_front(self) -> None:
        """WSLg 下 Tk 常被藏在任务栏企鹅图标后，强制置顶一次。"""
        try:
            self.root.deiconify()
            self.root.geometry('1100x720+80+80')
            self.root.lift()
            self.root.attributes('-topmost', True)
            self.root.focus_force()
            self.root.after(1500, lambda: self.root.attributes('-topmost', False))
        except tk.TclError:
            pass

    def _build_ui(self) -> None:
        top = tk.Frame(self.root, bg='#1a2332')
        top.pack(fill=tk.X, padx=16, pady=(12, 4))

        self.title_lbl = tk.Label(top, bg='#1a2332')
        self.title_lbl.pack(side=tk.LEFT)
        self.fonts.set_label(
            self.title_lbl, '无人装载机 · 键盘遥控（协议 20261102）', size=18, fill='#e8eef7', bg='#1a2332'
        )

        self.hint_lbl = tk.Label(top, bg='#1a2332')
        self.hint_lbl.pack(side=tk.RIGHT)
        self.fonts.set_label(
            self.hint_lbl,
            '点击窗口后操作：W/S 进退  A/D 转向  空格刹车  I/K 大臂  J/L 铲斗  H喇叭 T灯 E急停',
            size=11,
            fill='#8fa3bf',
            bg='#1a2332',
        )

        body = tk.Frame(self.root, bg='#1a2332')
        body.pack(fill=tk.BOTH, expand=True, padx=16, pady=8)

        # 左：俯视图
        left = tk.Frame(body, bg='#243044', highlightthickness=1, highlightbackground='#3a4d66')
        left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 8))
        self.top_view_lbl = tk.Label(left, bg='#243044')
        self.top_view_lbl.pack(anchor='w', padx=8, pady=4)
        self.fonts.set_label(
            self.top_view_lbl, '俯视图（前进/转向）', size=13, fill='#b8c7db', bg='#243044'
        )
        self.canvas_top = tk.Canvas(left, bg='#0f1724', highlightthickness=0)
        self.canvas_top.pack(fill=tk.BOTH, expand=True, padx=8, pady=(0, 8))

        # 右栏
        right = tk.Frame(body, bg='#1a2332', width=360)
        right.pack(side=tk.RIGHT, fill=tk.Y)
        right.pack_propagate(False)

        side_box = tk.Frame(right, bg='#243044', highlightthickness=1, highlightbackground='#3a4d66')
        side_box.pack(fill=tk.X, pady=(0, 8))
        self.side_view_lbl = tk.Label(side_box, bg='#243044')
        self.side_view_lbl.pack(anchor='w', padx=8, pady=4)
        self.fonts.set_label(
            self.side_view_lbl, '侧视图（大臂/铲斗）', size=13, fill='#b8c7db', bg='#243044'
        )
        self.canvas_side = tk.Canvas(
            side_box, width=340, height=220, bg='#0f1724', highlightthickness=0
        )
        self.canvas_side.pack(padx=8, pady=(0, 8))

        panel = tk.Frame(right, bg='#243044', highlightthickness=1, highlightbackground='#3a4d66')
        panel.pack(fill=tk.BOTH, expand=True)
        self.panel_title_lbl = tk.Label(panel, bg='#243044')
        self.panel_title_lbl.pack(anchor='w', padx=8, pady=4)
        self.fonts.set_label(
            self.panel_title_lbl, '控制 / CAN 预览', size=13, fill='#b8c7db', bg='#243044'
        )
        self.status_label = tk.Label(panel, bg='#182233', anchor='nw')
        self.status_label.pack(fill=tk.BOTH, expand=True, padx=8, pady=(0, 8))

        bottom = tk.Frame(self.root, bg='#1a2332')
        bottom.pack(fill=tk.X, padx=16, pady=(0, 12))
        self.btn_estop = tk.Button(
            bottom,
            command=self._do_estop,
            bg='#8b2e2e',
            activebackground='#a33',
            relief=tk.FLAT,
            padx=4,
            pady=2,
            borderwidth=0,
            highlightthickness=0,
        )
        self.btn_estop.pack(side=tk.LEFT)
        self.fonts.set_button(self.btn_estop, '急停 (E)', size=13, fill='#ffffff', bg='#8b2e2e')

        self.btn_neutral = tk.Button(
            bottom,
            command=self._do_neutral,
            bg='#2e4a6e',
            activebackground='#3a5f8a',
            relief=tk.FLAT,
            padx=4,
            pady=2,
            borderwidth=0,
            highlightthickness=0,
        )
        self.btn_neutral.pack(side=tk.LEFT, padx=8)
        self.fonts.set_button(
            self.btn_neutral, '空档复位 (N)', size=13, fill='#ffffff', bg='#2e4a6e'
        )

        self.ros_lbl = tk.Label(bottom, bg='#1a2332')
        self.ros_lbl.pack(side=tk.RIGHT)
        ros_text = 'ROS: 关闭' if not self.enable_ros else 'ROS: 发布 /controls/teleop'
        self.fonts.set_label(self.ros_lbl, ros_text, size=11, fill='#8fa3bf', bg='#1a2332')

    def _bind_keys(self) -> None:
        # 绑定到 root，窗口焦点内即可
        for key in (
            'w', 's', 'a', 'd', 'i', 'k', 'j', 'l', 'h', 't', 'e', 'n', 'space',
            'W', 'S', 'A', 'D', 'I', 'K', 'J', 'L', 'H', 'T', 'E', 'N',
        ):
            self.root.bind(f'<KeyPress-{key}>', self._on_key_press)
            self.root.bind(f'<KeyRelease-{key}>', self._on_key_release)
        self.root.bind('<Escape>', lambda e: self._do_estop())
        self.root.focus_force()

    def _norm_key(self, event) -> str:
        if event.keysym.lower() == 'space':
            return ' '
        return event.keysym.lower()

    def _on_key_press(self, event) -> None:
        key = self._norm_key(event)
        if key == 'h' and key not in self.keys_down:
            self.horn = not self.horn
        if key == 't' and key not in self.keys_down:
            self.light = not self.light
        if key == 'e':
            self._do_estop()
            return
        if key == 'n':
            self._do_neutral()
            return
        if key in ('w', 's', 'a', 'd', 'i', 'k', 'j', 'l', ' '):
            self.estop = False
            self.keys_down.add(key)
            # 互斥
            rivals = {
                'w': 's', 's': 'w', 'a': 'd', 'd': 'a',
                'i': 'k', 'k': 'i', 'j': 'l', 'l': 'j',
            }
            if key in rivals:
                self.keys_down.discard(rivals[key])
        # 按键后立刻做一帧，减少“半天才动”的体感延迟
        self._status_pending = True
        self._tick_once(force_draw=True)

    def _on_key_release(self, event) -> None:
        key = self._norm_key(event)
        self.keys_down.discard(key)
        self._status_pending = True
        self._tick_once(force_draw=True)

    def _do_estop(self) -> None:
        self.estop = True
        self.keys_down.clear()
        self._status_pending = True

    def _do_neutral(self) -> None:
        self.estop = False
        self.keys_down.clear()
        self._status_pending = True

    def _compute_teleop(self) -> dict:
        if self.estop:
            return build_teleop(emergency_stop=True, brake=1.0, light=self.light)

        gear, throttle = 'N', 0.0
        if 'w' in self.keys_down:
            gear, throttle = 'D', self.throttle_level
        elif 's' in self.keys_down:
            gear, throttle = 'R', self.throttle_level

        steering = 0.0
        if 'a' in self.keys_down:
            steering = -self.steer_level
        elif 'd' in self.keys_down:
            steering = self.steer_level

        boom = self.boom_level if 'i' in self.keys_down else (
            -self.boom_level if 'k' in self.keys_down else 0.0
        )
        bucket = self.bucket_level if 'j' in self.keys_down else (
            -self.bucket_level if 'l' in self.keys_down else 0.0
        )
        brake = self.brake_level if ' ' in self.keys_down else 0.0

        return build_teleop(
            gear=gear,
            throttle=throttle,
            brake=brake,
            steering=steering,
            boom=boom,
            bucket=bucket,
            horn=self.horn,
            light=self.light,
        )

    def _start_ros(self) -> None:
        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
            from std_msgs.msg import String
        except ImportError:
            self.fonts.set_label(
                self.ros_lbl, 'ROS: 未安装 rclpy，仅本地可视化', size=11, fill='#8fa3bf', bg='#1a2332'
            )
            self.enable_ros = False
            return

        rclpy.init(args=None)
        node = Node('loader_keyboard_gui')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        pub = node.create_publisher(String, '/controls/teleop', qos)
        self.ros_node = node
        self.ros_pub = pub
        self.ros_String = String

        def spin():
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.05)

        self._ros_thread = threading.Thread(target=spin, daemon=True)
        self._ros_thread.start()
        self.fonts.set_label(
            self.ros_lbl, 'ROS: 正在发布 /controls/teleop', size=11, fill='#8fa3bf', bg='#1a2332'
        )

    def _publish_ros(self, teleop: dict) -> None:
        if not self.enable_ros or self.ros_pub is None:
            return
        msg = self.ros_String()
        msg.data = json.dumps(teleop)
        self.ros_pub.publish(msg)

    def _tick(self) -> None:
        self._tick_once(force_draw=False)
        self.root.after(16, self._tick)

    def _tick_once(self, force_draw: bool = False) -> None:
        now = time.monotonic()
        dt = max(0.001, min(0.05, now - self._last_tick))
        self._last_tick = now

        teleop = self._compute_teleop()
        self._publish_ros(teleop)
        self._simulate_motion(teleop, dt)
        self._draw_top()
        self._draw_side()
        # 状态栏 Pillow 较慢：仅按键变化时更新，避免拖慢运动
        if force_draw or self._status_pending:
            self._update_status(teleop)
            self._status_pending = False

    def _ensure_centered(self, w: int, h: int) -> None:
        """画布真实尺寸就绪后再把车放到正中央（修复启动偏左）。"""
        if self._centered:
            return
        if w < 200 or h < 150:
            return
        self.x = w * 0.5
        self.y = h * 0.5
        self._centered = True

    def _simulate_motion(self, teleop: dict, dt: float) -> None:
        w = max(1, self.canvas_top.winfo_width())
        h = max(1, self.canvas_top.winfo_height())
        self._ensure_centered(w, h)

        if teleop['emergency_stop']:
            return

        # 刹车只减速，不完全锁死，避免“按了半天不动”的体感
        brake_factor = 1.0 - min(0.85, teleop['brake'])

        speed = 0.0
        if teleop['gear'] == 'D':
            speed = 220.0 * (teleop['throttle'] / max(0.1, self.throttle_level)) * brake_factor
        elif teleop['gear'] == 'R':
            speed = -180.0 * (teleop['throttle'] / max(0.1, self.throttle_level)) * brake_factor

        steer = teleop['steering']
        self.articulation += (steer * 0.65 - self.articulation) * min(1.0, dt * 10)
        # 原地也可转向，响应更快
        turn_scale = 2.2 if abs(speed) > 1 else 1.6
        self.yaw += steer * turn_scale * dt

        self.x += math.cos(self.yaw) * speed * dt
        self.y += math.sin(self.yaw) * speed * dt
        self.wheel_spin += speed * dt * 0.12

        # 仅在画布尺寸可靠时环绕
        if w >= 200 and h >= 150:
            margin = 40
            if self.x < -margin:
                self.x = w + margin
            elif self.x > w + margin:
                self.x = -margin
            if self.y < -margin:
                self.y = h + margin
            elif self.y > h + margin:
                self.y = -margin

        boom_jog = 1.0 if teleop['boom'] > 0 else (-1.0 if teleop['boom'] < 0 else 0.0)
        bucket_jog = 1.0 if teleop['bucket'] > 0 else (-1.0 if teleop['bucket'] < 0 else 0.0)
        self._hydro_state = self.hydro.step(dt, boom_jog, bucket_jog, bool(teleop['emergency_stop']))
        self.boom_angle = math.radians(self._hydro_state['boom_angle_deg'])
        self.bucket_angle = math.radians(self._hydro_state['bucket_angle_deg'])

        if abs(teleop['steering']) < 0.05:
            self.articulation *= 1.0 - min(1.0, dt * 4)

    def _draw_top(self) -> None:
        c = self.canvas_top
        c.delete('all')
        w = c.winfo_width() or 600
        h = c.winfo_height() or 400

        # 网格地面
        for x in range(0, w, 40):
            c.create_line(x, 0, x, h, fill='#1a2740')
        for y in range(0, h, 40):
            c.create_line(0, y, w, y, fill='#1a2740')

        # 车体：铰接装载机（后车体 + 前车体 + 铲斗示意）
        yaw = self.yaw
        art = self.articulation
        cx, cy = self.x, self.y

        def rot(px, py, ang, ox, oy):
            ca, sa = math.cos(ang), math.sin(ang)
            return ox + px * ca - py * sa, oy + px * sa + py * ca

        # 后车体
        rear = [(-36, -22), (8, -22), (8, 22), (-36, 22)]
        rear_pts = [rot(px, py, yaw, cx, cy) for px, py in rear]
        c.create_polygon(rear_pts, fill='#3d6b9c', outline='#9ec5ef', width=2)

        # 前车体（相对后车体铰接）
        front_ang = yaw + art
        hx, hy = rot(10, 0, yaw, cx, cy)
        front = [(0, -20), (34, -20), (34, 20), (0, 20)]
        front_pts = [rot(px, py, front_ang, hx, hy) for px, py in front]
        c.create_polygon(front_pts, fill='#4a7cae', outline='#b7d6f5', width=2)

        # 铲斗
        tip = rot(40, 0, front_ang, hx, hy)
        bucket = [(34, -16), (52, -10), (52, 10), (34, 16)]
        bucket_pts = [rot(px, py, front_ang, hx, hy) for px, py in bucket]
        c.create_polygon(bucket_pts, fill='#c4a35a', outline='#f0d78c', width=2)

        # 车头箭头
        nose = rot(20, 0, front_ang, hx, hy)
        c.create_oval(nose[0] - 4, nose[1] - 4, nose[0] + 4, nose[1] + 4, fill='#7dffb3', outline='')

        # 灯光
        if self.light:
            c.create_oval(tip[0] - 6, tip[1] - 6, tip[0] + 6, tip[1] + 6, outline='#ffe566', width=2)

        # 轨迹点
        c.create_oval(cx - 2, cy - 2, cx + 2, cy + 2, fill='#6cf', outline='')

        # 画布中文用 Pillow 贴图，避免方框
        if self._top_tip_img is None:
            self._top_tip_img = self.fonts.text_image(
                '俯视 · 黄=铲斗  蓝=车体  绿点=车头', size=12, fill='#6f839e', bg='#0f1724'
            )
        c.create_image(10, 8, anchor='nw', image=self._top_tip_img)

    def _draw_side(self) -> None:
        c = self.canvas_side
        c.delete('all')
        w, h = 340, 220
        # 地面
        c.create_rectangle(0, 170, w, h, fill='#1e2a1c', outline='')
        c.create_line(0, 170, w, 170, fill='#4a6a3a', width=2)

        # 车轮
        c.create_oval(70, 145, 110, 185, fill='#2a2a2a', outline='#888')
        c.create_oval(150, 145, 190, 185, fill='#2a2a2a', outline='#888')
        # 轮辐旋转感
        ang = self.wheel_spin
        for ox in (90, 170):
            x2 = ox + 14 * math.cos(ang)
            y2 = 165 + 14 * math.sin(ang)
            c.create_line(ox, 165, x2, y2, fill='#aaa', width=2)

        # 车身
        c.create_rectangle(60, 110, 200, 155, fill='#3d6b9c', outline='#9ec5ef', width=2)
        c.create_rectangle(150, 80, 195, 115, fill='#2f5478', outline='#9ec5ef')  # 驾驶室

        # 大臂
        base = (185, 125)
        boom_len = 90
        boom_ang = -self.boom_angle
        bx = base[0] + boom_len * math.cos(boom_ang)
        by = base[1] + boom_len * math.sin(boom_ang)
        c.create_line(base[0], base[1], bx, by, fill='#d8b56a', width=8, capstyle=tk.ROUND)

        # 铲斗
        bucket_ang = boom_ang + self.bucket_angle
        tip_x = bx + 40 * math.cos(bucket_ang)
        tip_y = by + 40 * math.sin(bucket_ang)
        # 铲斗多边形
        p1 = (bx + 8 * math.cos(bucket_ang + 1.2), by + 8 * math.sin(bucket_ang + 1.2))
        p2 = (tip_x, tip_y)
        p3 = (bx + 8 * math.cos(bucket_ang - 1.2), by + 8 * math.sin(bucket_ang - 1.2))
        c.create_polygon(p1, p2, p3, fill='#c4a35a', outline='#f0d78c', width=2)
        c.create_line(bx, by, tip_x, tip_y, fill='#e8c878', width=3)

        if self._side_tip_img is None:
            self._side_tip_img = self.fonts.text_image(
                '侧视 · I/K 大臂  J/L 铲斗', size=12, fill='#6f839e', bg='#0f1724'
            )
        c.create_image(10, 8, anchor='nw', image=self._side_tip_img)

    def _update_status(self, teleop: dict) -> None:
        merged = dict(teleop)
        hs = self._hydro_state
        if hs:
            merged.update({
                'boom_valve_up': hs['boom_valve_up'],
                'boom_valve_down': hs['boom_valve_down'],
                'bucket_valve_up': hs['bucket_valve_up'],
                'bucket_valve_down': hs['bucket_valve_down'],
            })
        can = encode_can(merged, heartbeat=True)
        if hs:
            bu = int(round(hs['boom_valve_up']))
            bd = int(round(hs['boom_valve_down']))
            ku = int(round(hs['bucket_valve_up']))
            kd = int(round(hs['bucket_valve_down']))
            hydro_txt = (
                f"倾角→关节角  大臂 {hs['boom_angle_deg']:.1f}°(目标{hs['boom_target_deg']:.1f}°)  "
                f"铲斗 {hs['bucket_angle_deg']:.1f}°(目标{hs['bucket_target_deg']:.1f}°)\n"
                f"油缸 大臂 {hs['boom_stroke_m']:.3f}m {hs['boom_cyl_vel']:+.3f}m/s  "
                f"铲斗 {hs['bucket_stroke_m']:.3f}m {hs['bucket_cyl_vel']:+.3f}m/s\n"
                f"阀400~700  大臂升/落 {bu}/{bd}  铲斗升/落 {ku}/{kd}\n"
            )
        else:
            hydro_txt = ''
        keys = ' '.join(sorted(k if k != ' ' else 'Space' for k in self.keys_down)) or '(无)'
        text = (
            f"按键: {keys}\n"
            f"档位: {teleop['gear']}   油门: {teleop['throttle']:.2f}   刹车: {teleop['brake']:.2f}\n"
            f"转向: {teleop['steering']:+.2f}\n"
            f"{hydro_txt}"
            f"喇叭: {'开' if teleop['horn'] else '关'}   灯: {'开' if teleop['light'] else '关'}   "
            f"急停: {'是' if teleop['emergency_stop'] else '否'}\n"
            f"\nCAN 预览:\n"
            f"  0x100: {can['0x100']}\n"
            f"  0x101: {can['0x101']}\n"
            f"  0x102: {can['0x102']}\n"
            f"  {can['explain']}\n"
        )
        # 仅在文本变化时重绘，减轻 CPU
        if text == self._last_status_text:
            return
        self._last_status_text = text
        photo = self.fonts.text_image(text, size=12, fill='#d7e3f4', bg='#182233', max_width=340)
        self.status_label.configure(image=photo)
        self.status_label.image = photo  # type: ignore[attr-defined]

    def run(self) -> None:
        self.root.protocol('WM_DELETE_WINDOW', self._on_close)
        self.root.mainloop()

    def _on_close(self) -> None:
        self.estop = True
        try:
            if self.enable_ros and self.ros_node is not None:
                import rclpy
                self.ros_node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
        except Exception:
            pass
        self.root.destroy()


def _open_html_in_windows(html: Path) -> None:
    html = html.resolve()
    print(f'在 Windows 浏览器打开仿真: {html}')
    print('请点浏览器页面一下，再按 W/A/S/D 等键。不要点任务栏企鹅图标。')
    try:
        win = subprocess.check_output(['wslpath', '-w', str(html)], text=True).strip()
        uri = 'file:///' + win.replace('\\', '/')
        subprocess.Popen(
            ['cmd.exe', '/c', 'start', '', uri],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except Exception as exc:
        print(f'无法调用 Windows 浏览器: {exc}')
        print(f'请手动用浏览器打开: {html}')


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--ros', action='store_true', help='同时发布 /controls/teleop（Tk）')
    parser.add_argument('--tk', action='store_true', help='使用 WSL Tk 窗口（任务栏企鹅常点不开）')
    args = parser.parse_args()

    html = Path(__file__).with_name('loader_keyboard_gui.html')
    use_tk = args.tk or args.ros
    if not use_tk:
        _open_html_in_windows(html)
        return

    if sys.platform.startswith('linux') and not (
        os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')
    ):
        print('未检测到 DISPLAY，改用浏览器仿真。')
        _open_html_in_windows(html)
        return

    print('Tk 窗口若只有企鹅图标且点不开：关掉后不要加 --tk，改用浏览器。')
    app = LoaderKeyboardGui(enable_ros=args.ros)
    app.run()


if __name__ == '__main__':
    main()
