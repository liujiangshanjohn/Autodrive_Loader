#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Logitech F710（DirectInput / Cordless RumblePad 2）摇杆 -> /vehicle_command

对照 xiaosong_controller.py 的模拟量逻辑：
  摇杆偏置 -> 200~700mA 单向阀电流（回中为 0）
  动臂/铲斗与调试 UI 一样做线号对调后再下发

默认映射（ISO 风格，可 ROS 参数改）：
  左摇杆 Y  大臂 上/下
  右摇杆 Y  小臂 伸/收
  LB(button 4)  铲斗内收：按住缓加电流，松开归零
  LT(button 6)  铲斗外翻：按住缓加电流，松开归零
  X(button 0)   左回转
  B(button 2)   右回转
  Y(button 3)   右履带前进
  A(button 1)   右履带后退
  十字上/下     左履带前进/后退
  RB(button 5)  死人手：大臂/小臂按住才出电流，松开立即清零

先开手柄：
  ros2 run joy joy_node
再开本节点：
  bash src/cannode/scripts/run_f710_joy_controller.sh
"""

import os
import sys
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
WS_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, '../../..'))
INSTALL_DIR = os.path.join(WS_DIR, 'install')


def setup_ros2_environment():
    if os.path.exists(INSTALL_DIR):
        for pkg_dir in os.listdir(INSTALL_DIR):
            pkg_path = os.path.join(
                INSTALL_DIR, pkg_dir, 'local', 'lib', 'python3.10', 'dist-packages')
            if os.path.exists(pkg_path) and pkg_path not in sys.path:
                sys.path.insert(0, pkg_path)
            lib_path = os.path.join(INSTALL_DIR, pkg_dir, 'lib')
            if os.path.exists(lib_path):
                lib_abs = os.path.abspath(lib_path)
                cur = os.environ.get('LD_LIBRARY_PATH', '')
                if lib_abs not in cur:
                    os.environ['LD_LIBRARY_PATH'] = (
                        f'{lib_abs}:{cur}' if cur else lib_abs)

    ros_distro = os.environ.get('ROS_DISTRO', 'humble')
    for p in (
        f'/opt/ros/{ros_distro}/lib/python3.10/site-packages',
        f'/opt/ros/{ros_distro}/local/lib/python3.10/dist-packages',
    ):
        if os.path.exists(p) and p not in sys.path:
            sys.path.insert(0, p)
    ros_lib = f'/opt/ros/{ros_distro}/lib'
    if os.path.exists(ros_lib):
        cur = os.environ.get('LD_LIBRARY_PATH', '')
        if ros_lib not in cur:
            os.environ['LD_LIBRARY_PATH'] = f'{ros_lib}:{cur}' if cur else ros_lib

    protobuf_path = os.path.join(SCRIPT_DIR, '../protobuf/out')
    if os.path.exists(protobuf_path) and protobuf_path not in sys.path:
        sys.path.insert(0, protobuf_path)


setup_ros2_environment()

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy
from std_msgs.msg import String as StringMsg
import cyber_msgs.msg
from control_msgs.control_cmd_pb2 import ControlCommand


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def axis_to_current(value, deadzone, i_min, i_max):
    """[-1,1] -> 电流幅度；死区内返回 0。"""
    mag = abs(value)
    if mag <= deadzone:
        return 0.0
    scale = (mag - deadzone) / max(1e-6, 1.0 - deadzone)
    return i_min + scale * (i_max - i_min)


class F710JoyController(Node):
    def __init__(self):
        super().__init__('f710_joy_controller')

        self.declare_parameter('boom_axis', 1)
        self.declare_parameter('stick_axis', 3)
        self.declare_parameter('invert_boom', True)
        self.declare_parameter('invert_stick', True)
        self.declare_parameter('deadman_button', 5)  # F710 DInput RB
        self.declare_parameter('bucket_close_button', 4)  # LB 铲斗内收
        self.declare_parameter('bucket_dump_button', 6)   # LT 铲斗外翻
        self.declare_parameter('rotate_left_button', 0)   # X
        self.declare_parameter('rotate_right_button', 2)  # B
        self.declare_parameter('right_fwd_button', 3)     # Y
        self.declare_parameter('right_bwd_button', 1)     # A
        self.declare_parameter('hat_y_axis', 5)           # 十字上下
        self.declare_parameter('invert_hat_y', False)     # 十字上 = 左履带前进
        self.declare_parameter('remote_button', 9)   # Start：申请遥控
        self.declare_parameter('deadzone', 0.12)
        self.declare_parameter('hat_deadzone', 0.5)
        self.declare_parameter('current_min', 200.0)
        self.declare_parameter('current_max', 700.0)
        self.declare_parameter('hold_ramp_ma_per_sec', 250.0)
        self.declare_parameter('bucket_ramp_ma_per_sec', 250.0)
        self.declare_parameter('joy_timeout_sec', 0.3)
        self.declare_parameter('swap_arm_like_ui', True)
        self.declare_parameter('swap_bucket_like_ui', True)

        qos_cmd = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.cmd_pub = self.create_publisher(
            cyber_msgs.msg.ProtoAdapter, '/vehicle_command', qos_cmd)
        self.dbg_pub = self.create_publisher(
            StringMsg, '/vehicle_command_debug_text', qos_cmd)
        self.joy_sub = self.create_subscription(
            Joy, '/joy', self.on_joy, qos_profile_sensor_data)

        self.seq = 0
        self.last_joy_time = 0.0
        self._last_ramp_time = time.monotonic()
        self.request_remote = False
        self._remote_btn_prev = False
        self._rb_held = False
        self._hold = {
            'bucket_close': False,
            'bucket_dump': False,
            'rotate_left': False,
            'rotate_right': False,
            'right_fwd': False,
            'right_bwd': False,
            'left_fwd': False,
            'left_bwd': False,
        }
        self.currents = {
            'arm_up': 0.0,
            'arm_down': 0.0,
            'stick_retract': 0.0,
            'stick_extend': 0.0,
            'bucket_close': 0.0,
            'bucket_dump': 0.0,
            'rotate_left': 0.0,
            'rotate_right': 0.0,
            'left_fwd': 0.0,
            'left_bwd': 0.0,
            'right_fwd': 0.0,
            'right_bwd': 0.0,
        }
        self._last_log = ''
        self.create_timer(0.01, self.publish_command)

        self.get_logger().info(
            'F710 摇杆控制已启动\n'
            '  左摇杆上下 = 大臂   右摇杆上下 = 小臂（需按住 RB）\n'
            '  LB/LT = 铲斗内收/外翻   X/B = 左/右回转\n'
            '  Y/A = 右履带前/后   十字上/下 = 左履带前/后（按住缓加，松开归零）\n'
            '  Start 切换申请遥控；请勿同时开调试 UI 抢 /vehicle_command'
        )

    def _axis(self, msg: Joy, idx: int, invert: bool) -> float:
        if idx < 0 or idx >= len(msg.axes):
            return 0.0
        v = float(msg.axes[idx])
        return -v if invert else v

    def _button(self, msg: Joy, idx: int) -> bool:
        if idx < 0 or idx >= len(msg.buttons):
            return False
        return msg.buttons[idx] != 0

    def _ramp_hold_current(self, held: bool, current: float, dt: float) -> float:
        """按住从最小电流缓加到最大；松开立即 0。"""
        if not held:
            return 0.0
        i_min = float(self.get_parameter('current_min').value)
        i_max = float(self.get_parameter('current_max').value)
        rate = float(self.get_parameter('hold_ramp_ma_per_sec').value)
        if current <= 0.0:
            return i_min
        return clamp(current + rate * dt, i_min, i_max)

    def _ramp_exclusive_pair(self, pos_key, neg_key, dt):
        pos_held = self._hold[pos_key] and not self._hold[neg_key]
        neg_held = self._hold[neg_key] and not self._hold[pos_key]
        self.currents[pos_key] = self._ramp_hold_current(
            pos_held, self.currents[pos_key], dt)
        self.currents[neg_key] = self._ramp_hold_current(
            neg_held, self.currents[neg_key], dt)

    def on_joy(self, msg: Joy):
        self.last_joy_time = time.monotonic()
        boom_axis = int(self.get_parameter('boom_axis').value)
        stick_axis = int(self.get_parameter('stick_axis').value)
        invert_boom = bool(self.get_parameter('invert_boom').value)
        invert_stick = bool(self.get_parameter('invert_stick').value)
        deadman = int(self.get_parameter('deadman_button').value)
        close_btn = int(self.get_parameter('bucket_close_button').value)
        dump_btn = int(self.get_parameter('bucket_dump_button').value)
        rot_l_btn = int(self.get_parameter('rotate_left_button').value)
        rot_r_btn = int(self.get_parameter('rotate_right_button').value)
        r_fwd_btn = int(self.get_parameter('right_fwd_button').value)
        r_bwd_btn = int(self.get_parameter('right_bwd_button').value)
        hat_y_axis = int(self.get_parameter('hat_y_axis').value)
        invert_hat_y = bool(self.get_parameter('invert_hat_y').value)
        hat_dz = float(self.get_parameter('hat_deadzone').value)
        remote_btn = int(self.get_parameter('remote_button').value)
        deadzone = float(self.get_parameter('deadzone').value)
        i_min = float(self.get_parameter('current_min').value)
        i_max = float(self.get_parameter('current_max').value)

        remote_now = self._button(msg, remote_btn)
        if remote_now and not self._remote_btn_prev:
            self.request_remote = not self.request_remote
            self.get_logger().info(
                f'申请遥控 request_remote_control={self.request_remote}')
        self._remote_btn_prev = remote_now

        self._rb_held = self._button(msg, deadman)
        hat_y = self._axis(msg, hat_y_axis, invert_hat_y)
        self._hold['bucket_close'] = self._button(msg, close_btn)
        self._hold['bucket_dump'] = self._button(msg, dump_btn)
        self._hold['rotate_left'] = self._button(msg, rot_l_btn)
        self._hold['rotate_right'] = self._button(msg, rot_r_btn)
        self._hold['right_fwd'] = self._button(msg, r_fwd_btn)
        self._hold['right_bwd'] = self._button(msg, r_bwd_btn)
        self._hold['left_fwd'] = hat_y > hat_dz
        self._hold['left_bwd'] = hat_y < -hat_dz

        if not self._rb_held:
            self.currents['arm_up'] = 0.0
            self.currents['arm_down'] = 0.0
            self.currents['stick_extend'] = 0.0
            self.currents['stick_retract'] = 0.0
        else:
            boom = self._axis(msg, boom_axis, invert_boom)
            stick = self._axis(msg, stick_axis, invert_stick)
            self.currents['arm_up'] = (
                axis_to_current(boom, deadzone, i_min, i_max) if boom > 0 else 0.0)
            self.currents['arm_down'] = (
                axis_to_current(boom, deadzone, i_min, i_max) if boom < 0 else 0.0)
            self.currents['stick_extend'] = (
                axis_to_current(stick, deadzone, i_min, i_max) if stick > 0 else 0.0)
            self.currents['stick_retract'] = (
                axis_to_current(stick, deadzone, i_min, i_max) if stick < 0 else 0.0)

    def _to_cmd_current(self, ui_value: float) -> float:
        if ui_value <= 0.0:
            return 0.0
        return clamp(ui_value, 200.0, 700.0)

    def publish_command(self):
        now = time.monotonic()
        dt = max(0.0, min(0.05, now - self._last_ramp_time))
        self._last_ramp_time = now

        timeout = float(self.get_parameter('joy_timeout_sec').value)
        timed_out = (
            self.last_joy_time <= 0.0 or (now - self.last_joy_time) > timeout)
        if timed_out:
            for k in self.currents:
                self.currents[k] = 0.0
            for k in self._hold:
                self._hold[k] = False
        else:
            self._ramp_exclusive_pair('bucket_close', 'bucket_dump', dt)
            self._ramp_exclusive_pair('rotate_left', 'rotate_right', dt)
            self._ramp_exclusive_pair('left_fwd', 'left_bwd', dt)
            self._ramp_exclusive_pair('right_fwd', 'right_bwd', dt)

        swap_arm = bool(self.get_parameter('swap_arm_like_ui').value)
        swap_bucket = bool(self.get_parameter('swap_bucket_like_ui').value)

        arm_up_ui = self.currents['arm_up']
        arm_down_ui = self.currents['arm_down']
        bucket_close_ui = self.currents['bucket_close']
        bucket_dump_ui = self.currents['bucket_dump']

        # 与 xiaosong_controller.publish_command 相同的线号对调
        if swap_arm:
            cmd_arm_up = self._to_cmd_current(arm_down_ui)
            cmd_arm_down = self._to_cmd_current(arm_up_ui)
        else:
            cmd_arm_up = self._to_cmd_current(arm_up_ui)
            cmd_arm_down = self._to_cmd_current(arm_down_ui)
        if swap_bucket:
            cmd_bucket_close = self._to_cmd_current(bucket_dump_ui)
            cmd_bucket_dump = self._to_cmd_current(bucket_close_ui)
        else:
            cmd_bucket_close = self._to_cmd_current(bucket_close_ui)
            cmd_bucket_dump = self._to_cmd_current(bucket_dump_ui)

        cmd = ControlCommand()
        cmd.header.timestamp_sec = self.get_clock().now().nanoseconds / 1e9
        cmd.header.frame_id = 'f710_joy'
        self.seq += 1
        cmd.header.sequence_num = self.seq
        cmd.request_remote_control = bool(self.request_remote)
        cmd.arm_up_current = cmd_arm_up
        cmd.arm_down_current = cmd_arm_down
        cmd.stick_retract_current = self._to_cmd_current(self.currents['stick_retract'])
        cmd.stick_extend_current = self._to_cmd_current(self.currents['stick_extend'])
        cmd.bucket_close_current = cmd_bucket_close
        cmd.bucket_dump_current = cmd_bucket_dump
        cmd.rotate_left_current = self._to_cmd_current(self.currents['rotate_left'])
        cmd.rotate_right_current = self._to_cmd_current(self.currents['rotate_right'])
        cmd.left_track_forward_current = self._to_cmd_current(self.currents['left_fwd'])
        cmd.left_track_backward_current = self._to_cmd_current(self.currents['left_bwd'])
        cmd.right_track_forward_current = self._to_cmd_current(self.currents['right_fwd'])
        cmd.right_track_backward_current = self._to_cmd_current(self.currents['right_bwd'])

        serialized = cmd.SerializeToString()
        ros_msg = cyber_msgs.msg.ProtoAdapter()
        ros_msg.pb = [bytes([b]) for b in serialized]
        self.cmd_pub.publish(ros_msg)

        line = (
            f'remote={int(self.request_remote)} '
            f'boom u/d={arm_up_ui:.0f}/{arm_down_ui:.0f} '
            f'stick e/r={self.currents["stick_extend"]:.0f}/{self.currents["stick_retract"]:.0f} '
            f'bucket c/d={bucket_close_ui:.0f}/{bucket_dump_ui:.0f} '
            f'rot l/r={self.currents["rotate_left"]:.0f}/{self.currents["rotate_right"]:.0f} '
            f'L f/b={self.currents["left_fwd"]:.0f}/{self.currents["left_bwd"]:.0f} '
            f'R f/b={self.currents["right_fwd"]:.0f}/{self.currents["right_bwd"]:.0f} mA'
        )
        if line != self._last_log:
            self._last_log = line
            dbg = StringMsg()
            dbg.data = line
            self.dbg_pub.publish(dbg)
            if any(self.currents.values()) or self.request_remote:
                self.get_logger().info(line, throttle_duration_sec=0.5)


def main():
    os.environ.setdefault('PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION', 'python')
    rclpy.init()
    node = F710JoyController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
