#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
小松挖掘机控制界面
可以控制动臂、斗杆、铲斗等，并实时显示电流值和CAN报文
"""

import sys
import os

# 自动检测并添加工作空间路径
def setup_ros2_environment():
    """自动设置 ROS2 环境路径"""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    ws_dir = os.path.abspath(os.path.join(script_dir, '../../..'))
    install_dir = os.path.join(ws_dir, 'install')
    
    # 添加工作空间的 Python 包路径
    if os.path.exists(install_dir):
        for pkg_dir in os.listdir(install_dir):
            pkg_path = os.path.join(install_dir, pkg_dir, 'local', 'lib', 'python3.10', 'dist-packages')
            if os.path.exists(pkg_path) and pkg_path not in sys.path:
                sys.path.insert(0, pkg_path)
            
            # 同时添加库路径到 LD_LIBRARY_PATH（如果可能）
            lib_path = os.path.join(install_dir, pkg_dir, 'lib')
            if os.path.exists(lib_path):
                lib_path_abs = os.path.abspath(lib_path)
                current_ld_path = os.environ.get('LD_LIBRARY_PATH', '')
                if lib_path_abs not in current_ld_path:
                    os.environ['LD_LIBRARY_PATH'] = f"{lib_path_abs}:{current_ld_path}" if current_ld_path else lib_path_abs
    
    # 尝试从环境变量获取 ROS2 路径
    ros_distro = os.environ.get('ROS_DISTRO', 'humble')
    ros_python_path = f'/opt/ros/{ros_distro}/lib/python3.10/site-packages'
    if os.path.exists(ros_python_path) and ros_python_path not in sys.path:
        sys.path.insert(0, ros_python_path)
    
    ros_local_path = f'/opt/ros/{ros_distro}/local/lib/python3.10/dist-packages'
    if os.path.exists(ros_local_path) and ros_local_path not in sys.path:
        sys.path.insert(0, ros_local_path)
    
    # 添加 ROS2 库路径
    ros_lib_path = f'/opt/ros/{ros_distro}/lib'
    if os.path.exists(ros_lib_path):
        current_ld_path = os.environ.get('LD_LIBRARY_PATH', '')
        if ros_lib_path not in current_ld_path:
            os.environ['LD_LIBRARY_PATH'] = f"{ros_lib_path}:{current_ld_path}" if current_ld_path else ros_lib_path

# 设置环境
setup_ros2_environment()

# 检查 ROS2 环境
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String as StringMsg
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    import cyber_msgs.msg
except ImportError as e:
    # print("=" * 60)
    # print("❌ 错误: 无法导入 ROS2 模块")
    # print("=" * 60)
    # print(f"详细错误: {e}")
    # print("")
    # print("💡 解决方案:")
    # print("  请使用启动脚本运行程序:")
    # print("    bash src/cannode/scripts/run_xiaosong_controller.sh")
    # print("")
    # print("  或者手动设置环境:")
    # print("    source /opt/ros/humble/setup.bash")
    # print("    source ~/cannode_ws/install/setup.bash")
    # print("    python3 src/cannode/scripts/xiaosong_controller.py")
    # print("=" * 60)
    sys.exit(1)

# 尝试导入 protobuf
try:
    # 添加 protobuf 路径（修复路径：从 scripts/ 到 protobuf/out）
    script_dir = os.path.dirname(os.path.abspath(__file__))
    protobuf_path = os.path.join(script_dir, '../protobuf/out')
    if os.path.exists(protobuf_path) and protobuf_path not in sys.path:
        sys.path.insert(0, protobuf_path)
    
    from control_msgs.control_cmd_pb2 import ControlCommand
    from common_msgs.basic_msgs.header_pb2 import Header
    from common_msgs.chassis_msgs.chassis_pb2 import Chassis
    PROTOBUF_AVAILABLE = True
    # print(f"成功导入 protobuf 模块，路径: {protobuf_path}")
except ImportError as e:
    # print(f"警告: 无法导入 protobuf 模块: {e}")
    # print(f"尝试的路径: {protobuf_path if 'protobuf_path' in locals() else '未定义'}")
    # print("控制命令发布功能将被禁用")
    PROTOBUF_AVAILABLE = False

import json
import tkinter as tk
from tkinter import ttk
import threading
from datetime import datetime
import struct

class XiaosongController(Node):
    def __init__(self):
        super().__init__('xiaosong_controller')
        
        # 创建发布者 - 发布控制命令到 /vehicle_command（序列化）
        if PROTOBUF_AVAILABLE:
            qos_profile = QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST
            )
            self.cmd_pub = self.create_publisher(
                cyber_msgs.msg.ProtoAdapter,
                '/vehicle_command',
                qos_profile
            )
            self.get_logger().info('已创建发布者 /vehicle_command')
            
            # 创建调试发布者 - 发布控制命令到 /vehicle_command_debug_text（文本格式，避免与 teleop_converter 的 /vehicle_command_debug 冲突）
            self.cmd_debug_pub = self.create_publisher(
                StringMsg,
                '/vehicle_command_debug_text',
                qos_profile
            )
            self.get_logger().info('已创建发布者 /vehicle_command_debug_text')
            
            # 发动机油门档位（1~7，对应 ControlCommand.pto_cmd，经由 cannode XiaosongCANParser 的 0x18FD0821 报文下发）
            
            # 创建定时器，每10ms自动发布一次控制命令
            self.publish_timer = self.create_timer(0.01, self.publish_command)  # 0.01秒 = 10ms
            self.get_logger().info('已创建定时器，每10ms自动发布控制命令')
        
        # 订阅 CAN 帧信息话题
        qos_profile_can = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )
        self.can_frame_sub = self.create_subscription(
            StringMsg,
            '/can_frame_sent',
            self.can_frame_callback,
            qos_profile_can
        )
        self.get_logger().info('已订阅 /can_frame_sent 话题')
        
        # 存储最新的 CAN 帧信息
        self.latest_can_frames = []
        self.data_lock = threading.Lock()
        
        # 用于跟踪是否已经打印过 protobuf 不可用的警告
        self.protobuf_warning_printed = False
        
        # 存储所有 GUI 控件的变量（用于重置功能）
        self.control_vars = {}
        self.label_widgets = {}  # 存储所有标签控件的引用（显示用户设置的值）
        self.actual_label_widgets = {}  # 存储实际发送电流标签控件的引用（显示从CAN解析的值）
        
        # 当前控制值（所有控制都是直接电流控制，200~700mA，无死区）
        self.controls = {
            'arm_up_current': 0.0,        # 动臂抬升电流 (200~700mA)
            'arm_down_current': 0.0,      # 动臂下降电流 (200~700mA)
            'stick_retract_current': 0.0, # 斗杆收回电流 (200~700mA)
            'stick_extend_current': 0.0,  # 斗杆伸出电流 (200~700mA)
            'bucket_close_current': 0.0,  # 铲斗收斗电流 (200~700mA)
            'bucket_dump_current': 0.0,   # 铲斗翻斗电流 (200~700mA)
            'rotate_left_current': 0.0,   # 回转左电流 (200~700mA)
            'rotate_right_current': 0.0,  # 回转右电流 (200~700mA)
            'left_track_forward_current': 0.0,   # 左履带前进电流 (200~700mA)
            'left_track_backward_current': 0.0,  # 左履带后退电流 (200~700mA)
            'right_track_forward_current': 0.0,  # 右履带前进电流 (200~700mA)
            'right_track_backward_current': 0.0, # 右履带后退电流 (200~700mA)
            'brake': 0.0,            # 刹车 (0~100)
            'estop': False,          # 急停
            'parking_brake': False,  # 驻车制动
            'request_remote_control': False,  # 手/自动（远程申请进入遥控，CAN 0x18210005 bit32）
        }
        
        # 开关量控制（26个开关）
        self.switches = {
            'switch_21': False, 'switch_22': False, 'switch_23': False, 'switch_24': False,
            'switch_25': False, 'switch_26': False, 'switch_27': False, 'switch_28': False,
            'switch_29': False, 'switch_30': False, 'switch_31': False, 'switch_32': False,
            'switch_17': False, 'switch_18': False, 'switch_19': False, 'switch_20': False,
            'switch_38': False, 'switch_39': False, 'switch_40': False, 'switch_42': False,
            'switch_43': False, 'switch_44': False, 'switch_45': False, 'switch_46': False,
            'switch_37': False, 'switch_41': False,
        }

        # 高级功能控制（由 UI 直接选择，最终映射到 0x18210005 对应开关位）
        # bucket_switch: 0/1/2 = 无动作 / 开斗电磁阀(switch_30) / 关斗电磁阀(switch_31)
        # ladder_switch: 0/1/2 = 无动作 / 车梯收起电磁阀(switch_21) / 车梯放下电磁阀(switch_22)
        # horn: false/true = 无动作 / 喇叭(switch_19, bit 11)
        # hydraulic_lock: false/true = 液压锁不工作 / 液压锁工作（先导安全锁定电磁阀 switch_24）
        # light_code: 0/1/2/3 = 关灯 / 开灯1 / 开灯2 / 开灯1+2（灯光1: switch_17, 灯光2: switch_18）
        # power_enable: false/true = 不上电 / 上电（通过 switch_bits bit 26 传递，映射到 switch_ext_bits bit 34）
        # start_up: false/true = 不启动 / 发动机启动（通过 switch_bits bit 27 传递，映射到 switch_ext_bits bit 35）
        # acc: false/true = 关闭 / ACC 指令（通过 switch_bits bit 28 传递，映射到 switch_ext_bits bit 36）
        self.bucket_switch_mode = 0
        self.ladder_switch_mode = 0
        self.horn_active = False
        self.hydraulic_lock_active = False
        self.light_code_mode = 0  # 0:关灯, 1:灯1, 2:灯2, 3:灯1+2
        self.power_enable_active = False
        self.start_up_active = False
        self.acc_active = False
        self.engine_gear_level = 1  # 发动机油门档位 1~7（默认 1 档）
        
        # 开关量对应的线号映射（根据 excavator-can-host2vcu_utf8.csv）
        self.switch_wire_map = {
            'switch_21': '125#',  # bit 0: 车梯收起电磁阀, DOH_OUT17_125(125#)
            'switch_22': '129#',  # bit 1: 车梯放下电磁阀, DOH_OUT18_129(129#)
            'switch_23': 'N/A',   # bit 2: 车梯防落电磁阀
            'switch_24': '103#',  # bit 3: 先导安全锁定电磁阀, OUT_24_DOH(103#)
            'switch_25': 'N/A',  # bit 4: 机器上推电磁阀
            'switch_26': 'N/A',  # bit 5: 直线行走电磁阀
            'switch_27': 'N/A',  # bit 6: 回转优先电磁阀
            'switch_28': 'N/A',  # bit 7: 回转停车制动电磁阀
            'switch_29': 'N/A',  # bit 8: 动臂卸载电磁阀
            'switch_30': '193#', # bit 9: 开斗油缸电磁阀, OUT_26_DOH(193#)
            'switch_31': '183#', # bit 10: 关斗油缸电磁阀, OUT_37_DOL(183#)
            'switch_19': '241#', # bit 11: 喇叭, OUT_28_DOH(241#)
            'switch_17': '128#', # bit 12: 灯光1, OUT_19_DOH(128#)
            'switch_18': '242#', # bit 13: 灯光2, OUT_29_DOH(242#)
            'switch_32': 'N/A',  # bit 14: 备用1
            'switch_20': 'N/A',  # bit 15: 备用2
            'switch_38': 'N/A',  # bit 16: 开关量映射
            'switch_39': 'N/A',  # bit 17: 开关量映射
            'switch_40': 'N/A',  # bit 18: 开关量映射
            'switch_42': 'N/A',  # bit 19: 开关量映射
            'switch_43': 'N/A',  # bit 20: 开关量映射
            'switch_44': 'N/A',  # bit 21: 开关量映射
            'switch_45': 'N/A',  # bit 22: 开关量映射
            'switch_46': 'N/A',  # bit 23: 开关量映射
            'switch_37': 'N/A',  # bit 24: 开关量映射
            'switch_41': 'N/A',  # bit 25: 开关量映射
        }
        
        # 开关量对应的功能名称映射（根据 excavator-can-host2vcu_utf8.csv）
        self.switch_name_map = {
            'switch_21': '车梯收起电磁阀',      # bit 0: DOH_OUT17_125(125#)
            'switch_22': '车梯放下电磁阀',      # bit 1: DOH_OUT18_129(129#)
            'switch_23': '车梯防落电磁阀',      # bit 2
            'switch_24': '先导安全锁定电磁阀',  # bit 3: OUT_24_DOH(103#)
            'switch_25': '机器上推电磁阀',      # bit 4
            'switch_26': '直线行走电磁阀',      # bit 5
            'switch_27': '回转优先电磁阀',      # bit 6
            'switch_28': '回转停车制动电磁阀',  # bit 7
            'switch_29': '动臂卸载电磁阀',      # bit 8
            'switch_30': '开斗油缸电磁阀',      # bit 9: OUT_26_DOH(193#)
            'switch_31': '关斗油缸电磁阀',      # bit 10: OUT_37_DOL(183#)
            'switch_19': '喇叭',                # bit 11: OUT_28_DOH(241#)
            'switch_17': '灯光1',               # bit 12: OUT_19_DOH(128#)
            'switch_18': '灯光2',               # bit 13: OUT_29_DOH(242#)
            'switch_32': '备用1',               # bit 14
            'switch_20': '备用2',               # bit 15
            'switch_38': '开关量映射',          # bit 16
            'switch_39': '开关量映射',          # bit 17
            'switch_40': '开关量映射',          # bit 18
            'switch_42': '开关量映射',          # bit 19
            'switch_43': '开关量映射',          # bit 20
            'switch_44': '开关量映射',          # bit 21
            'switch_45': '开关量映射',          # bit 22
            'switch_46': '开关量映射',          # bit 23
            'switch_37': '开关量映射',          # bit 24
            'switch_41': '开关量映射',          # bit 25
        }
        
        # 功能状态对应的开关和接线信息（用于调试显示，根据 excavator-can-host2vcu_utf8.csv）
        self.function_switch_map = {
            'ladder_switch': {
                'switch_21': {'name': '车梯收起电磁阀', 'bit': 0, 'wire': '125#', 'io': 'DOH_OUT17_125'},
                'switch_22': {'name': '车梯放下电磁阀', 'bit': 1, 'wire': '129#', 'io': 'DOH_OUT18_129'},
                'switch_23': {'name': '车梯防落电磁阀', 'bit': 2, 'wire': 'N/A', 'io': 'N/A'},
            },
            'bucket_switch': {
                'switch_30': {'name': '开斗油缸电磁阀', 'bit': 9, 'wire': '193#', 'io': 'OUT_26_DOH'},
                'switch_31': {'name': '关斗油缸电磁阀', 'bit': 10, 'wire': '183#', 'io': 'OUT_37_DOL'},
            },
            'horn': {
                'switch_19': {'name': '喇叭', 'bit': 11, 'wire': '241#', 'io': 'OUT_28_DOH'},
            },
            'power_enable': {
                'switch_bits_bit26': {'name': '上电', 'bit': 34, 'wire': 'N/A', 'io': 'switch_ext_bits bit34 (1是有效)'},
            },
            'start_up': {
                'switch_bits_bit27': {'name': '发动机启动', 'bit': 35, 'wire': 'N/A', 'io': 'switch_ext_bits bit35 (1是有效)'},
            },
            'acc': {
                'switch_bits_bit28': {'name': 'ACC指令', 'bit': 36, 'wire': '194#', 'io': 'switch_ext_bits bit36 (1是有效, DOH_OUT25_194)'},
            },
            'hydraulic_lock': {
                'switch_24': {'name': '先导安全锁定电磁阀', 'bit': 3, 'wire': '103#', 'io': 'OUT_24_DOH'},
            },
            'light_code': {
                'switch_17': {'name': '灯光1', 'bit': 12, 'wire': '128#', 'io': 'OUT_19_DOH'},
                'switch_18': {'name': '灯光2', 'bit': 13, 'wire': '242#', 'io': 'OUT_29_DOH'},
            },
            'light_code': {
                'switch_17': {'name': '灯光1', 'bit': 12, 'wire': '128#', 'io': 'OUT_19_DOH'},
                'switch_18': {'name': '灯光2', 'bit': 13, 'wire': '242#', 'io': 'OUT_29_DOH'},
            },
            'machine_push': {
                'switch_25': {'name': '机器上推电磁阀', 'bit': 4, 'wire': '194#', 'io': 'OUT_25_DOH'},
            },
            'straight_walk': {
                'switch_26': {'name': '直线行走电磁阀', 'bit': 5, 'wire': '193#', 'io': 'OUT_26_DOH'},
            },
            'rotate_priority': {
                'switch_27': {'name': '回转优先电磁阀', 'bit': 6, 'wire': '243#', 'io': 'OUT_27_DOH'},
            },
            'rotate_brake': {
                'switch_28': {'name': '回转停车制动电磁阀', 'bit': 7, 'wire': '241#', 'io': 'OUT_28_DOH'},
            },
            'arm_unload': {
                'switch_29': {'name': '动臂卸载电磁阀', 'bit': 8, 'wire': '242#', 'io': 'OUT_29_DOH'},
            },
            'spare1': {
                'switch_32': {'name': '备用1', 'bit': 14, 'wire': '257#', 'io': 'OUT_32_DOH'},
            },
            'spare2': {
                'switch_20': {'name': '备用2', 'bit': 15, 'wire': '103#', 'io': 'OUT_20_DOH'},
            },
            'switch_mapping': {
                'switch_38': {'name': '开关量映射', 'bit': 16, 'wire': '184#', 'io': 'OUT_38_DOL'},
                'switch_39': {'name': '开关量映射', 'bit': 17, 'wire': '185#', 'io': 'OUT_39_DOL'},
                'switch_40': {'name': '开关量映射', 'bit': 18, 'wire': '186#', 'io': 'OUT_40_DOL'},
                'switch_42': {'name': '开关量映射', 'bit': 19, 'wire': '180#', 'io': 'OUT_42_DOL'},
                'switch_43': {'name': '开关量映射', 'bit': 20, 'wire': '181#', 'io': 'OUT_43_DOL'},
                'switch_44': {'name': '开关量映射', 'bit': 21, 'wire': '182#', 'io': 'OUT_44_DOL'},
                'switch_45': {'name': '开关量映射', 'bit': 22, 'wire': '188#', 'io': 'OUT_45_DOL'},
                'switch_46': {'name': '开关量映射', 'bit': 23, 'wire': '187#', 'io': 'OUT_46_DOL'},
                'switch_37': {'name': '开关量映射', 'bit': 24, 'wire': '183#', 'io': 'OUT_37_DOL'},
                'switch_41': {'name': '开关量映射', 'bit': 25, 'wire': '179#', 'io': 'OUT_41_DOL'},
            },
        }
        
        # 比例阀对应的线号映射（根据文档定义）
        self.valve_wire_map = {
            'arm_down_current': '153#',         # 动臂下降电磁阀
            'arm_up_current': '177#',           # 动臂抬升电磁阀
            'stick_extend_current': '154#',     # 斗杆伸出电磁阀
            'stick_retract_current': '178#',    # 斗杆收回电磁阀
            'bucket_dump_current': '151#',      # 铲斗翻斗电磁阀
            'bucket_close_current': '175#',     # 铲斗收斗电磁阀
            'rotate_left_current': '152#',     # 回转左电磁阀
            'rotate_right_current': '176#',     # 回转右电磁阀
            'left_track_forward_current': '149#',   # 左前进电磁阀
            'left_track_backward_current': '173#',  # 左后退电磁阀
            'right_track_forward_current': '150#',  # 右前进电磁阀
            'right_track_backward_current': '174#', # 右后退电磁阀
        }
        
        # 从 CAN 解析的状态（0x18210005 bit32/bit33）
        self.can_status = {
            'request_remote_control': False,  # bit32: 远程申请进入遥控操作
            'emergency_stop': False,          # bit33: 远程申请进入急停状态
            'bucket_switch': 0,               # 0/1/2 = 无动作/开斗/关斗（来自 0x18210005）
            'ladder_switch': 0,               # 0/1/2 = 无动作/收起/放下（来自 0x18210005）
            'horn': False,                    # false/true（来自 0x18210005）
            'hydraulic_lock': False,          # 先导安全锁定电磁阀（switch_24）
            'light_code': 0,                  # 0/1/2/3 = 关灯/灯1/灯2/灯1+2（switch_17/18 组合）
            'power_enable': False,            # false/true（来自 switch_ext_bits bit34）
            'start_up': False,                # false/true（来自 switch_ext_bits bit35）
            'acc': False,                     # false/true（来自 switch_ext_bits bit36）
            # 下发 VCU 的真实 CAN 回显（/can_frame_sent）
            'engine_rpm_tx': 0,               # 0x18210005 byte5-6，发动机转速 0~8000 rpm（cannode 注入 J1939）
            'engine_throttle_raw': 0,        # 0x18FD0821 byte6，油门电压原始值（0.05V/bit）
            'engine_throttle_voltage': 0.0,  # 推算电压 V
            'engine_throttle_gear_tx': 0,    # 由电压反算档位 1~7（与下发电压映射一致）
        }
        
        # 计算出的电流值（直接使用控制值，无死区）
        self.calculated_currents = {
            'arm_up': 0,           # 动臂抬升电流 (mA)
            'arm_down': 0,         # 动臂下降电流 (mA)
            'stick_retract': 0,    # 斗杆收回电流 (mA)
            'stick_extend': 0,     # 斗杆伸出电流 (mA)
            'bucket_close': 0,     # 铲斗收斗电流 (mA)
            'bucket_dump': 0,      # 铲斗翻斗电流 (mA)
            'rotate_left': 0,      # 回转左电流 (mA)
            'rotate_right': 0,     # 回转右电流 (mA)
            'left_forward': 0,     # 左前进电流 (mA)
            'left_backward': 0,    # 左后退电流 (mA)
            'right_forward': 0,    # 右前进电流 (mA)
            'right_backward': 0,   # 右后退电流 (mA)
        }
        
        # 实际发送的CAN报文电流值（从CAN反馈中解析）
        self.actual_currents = {
            'arm_up': 0, 'arm_down': 0,
            'stick_retract': 0, 'stick_extend': 0,
            'bucket_close': 0, 'bucket_dump': 0,
            'rotate_left': 0, 'rotate_right': 0,
            'left_forward': 0, 'left_backward': 0,
            'right_forward': 0, 'right_backward': 0,
        }
        
        self.gui_running = True
        
        # 控制是否发送控制数据（默认启用）
        self.enable_send_control = True
        
        # 缩放相关变量
        self.base_width = 1400  # 基准宽度
        self.base_height = 900  # 基准高度
        self.scale_factor = 1.0  # 当前缩放因子
        self.font_cache = {}  # 字体缓存
        self.widget_fonts = {}  # 存储所有控件的字体引用
    
    def can_frame_callback(self, msg):
        """处理接收到的 CAN 帧信息"""
        try:
            can_data = json.loads(msg.data)
            
            # 解析 CAN 帧并更新实际电流值（parse_can_frame 内部会更新 self.actual_currents）
            parsed = self.parse_can_frame(can_data)
            
            # 更新最新的 CAN 帧信息（保留最近20条）
            with self.data_lock:
                self.latest_can_frames.append({
                    'timestamp': datetime.now().strftime('%H:%M:%S.%f')[:-3],
                    'data': can_data
                })
                if len(self.latest_can_frames) > 20:
                    self.latest_can_frames.pop(0)
            
            # 调试：打印接收到的 CAN 帧信息（仅对非零数据）
            can_id_dec = can_data.get('can_id_dec', 0)
            data_array = can_data.get('data', [])
            if any(data_array):  # 如果有非零数据
                self.get_logger().debug(f'收到 CAN 帧: ID=0x{can_id_dec:08X}, 数据={data_array}')
                    
        except json.JSONDecodeError as e:
            self.get_logger().warn(f'CAN 帧 JSON 解析错误: {e}')
        except Exception as e:
            self.get_logger().error(f'处理 CAN 帧消息时出错: {e}')
    
    def calculate_currents(self):
        """根据控制命令计算电流值（直接使用，无死区）"""
        # 动臂控制 - 直接使用电流值
        arm_up = self.controls['arm_up_current']
        arm_down = self.controls['arm_down_current']
        self.calculated_currents['arm_up'] = int(max(0, min(700, arm_up)))
        self.calculated_currents['arm_down'] = int(max(0, min(700, arm_down)))
        
        # 斗杆控制 - 直接使用电流值
        stick_retract = self.controls['stick_retract_current']
        stick_extend = self.controls['stick_extend_current']
        self.calculated_currents['stick_retract'] = int(max(0, min(700, stick_retract)))
        self.calculated_currents['stick_extend'] = int(max(0, min(700, stick_extend)))
        
        # 铲斗控制 - 直接使用电流值
        bucket_close = self.controls['bucket_close_current']
        bucket_dump = self.controls['bucket_dump_current']
        self.calculated_currents['bucket_close'] = int(max(0, min(700, bucket_close)))
        self.calculated_currents['bucket_dump'] = int(max(0, min(700, bucket_dump)))
        
        # 回转控制 - 直接使用电流值
        rotate_left = self.controls['rotate_left_current']
        rotate_right = self.controls['rotate_right_current']
        self.calculated_currents['rotate_left'] = int(max(0, min(700, rotate_left)))
        self.calculated_currents['rotate_right'] = int(max(0, min(700, rotate_right)))
        
        # 行走控制（左右履带独立控制，直接电流值）
        # 左履带前进电流
        left_fwd_current = self.controls['left_track_forward_current']
        self.calculated_currents['left_forward'] = int(max(0, min(700, left_fwd_current)))
        
        # 左履带后退电流
        left_bwd_current = self.controls['left_track_backward_current']
        self.calculated_currents['left_backward'] = int(max(0, min(700, left_bwd_current)))
        
        # 右履带前进电流
        right_fwd_current = self.controls['right_track_forward_current']
        self.calculated_currents['right_forward'] = int(max(0, min(700, right_fwd_current)))
        
        # 右履带后退电流
        right_bwd_current = self.controls['right_track_backward_current']
        self.calculated_currents['right_backward'] = int(max(0, min(700, right_bwd_current)))
    
    def publish_command(self):
        """发布控制命令"""
        # 检查是否允许发送控制数据
        if not self.enable_send_control:
            return
        
        if not PROTOBUF_AVAILABLE:
            # 只在第一次调用时打印警告，避免重复警告
            if not self.protobuf_warning_printed:
                self.get_logger().warn('Protobuf 不可用，无法发布控制命令（后续将静默忽略）')
                self.protobuf_warning_printed = True
            return
        
        try:
            # 创建 ControlCommand 消息
            cmd = ControlCommand()
            
            # 设置 header
            cmd.header.timestamp_sec = self.get_clock().now().nanoseconds / 1e9
            cmd.header.frame_id = "base_link"
            cmd.header.sequence_num = 0
            
            # 设置控制参数
            cmd.brake = self.controls['brake']
            cmd.parking_brake = self.controls['parking_brake']
            cmd.estop = self.controls['estop']
            # 手/自动切换（远程申请进入遥控）
            # -> cannode xiaosong.cc: sendSwitchControl() 会将其映射到 0x18210005 扩展位 bit32
            cmd.request_remote_control = bool(self.controls.get('request_remote_control', False))
            
            # 动臂控制（直接电流控制，200~700mA）
            # 注意：界面上的"抬升"对应CAN的"下降"字段（153#线），"下降"对应CAN的"抬升"字段（177#线）
            # 需要交换映射以匹配实际硬件连接
            cmd.arm_up_current = max(200.0, min(700.0, self.controls['arm_down_current']))  # 界面"抬升" -> CAN下降(153#)
            cmd.arm_down_current = max(200.0, min(700.0, self.controls['arm_up_current']))  # 界面"下降" -> CAN抬升(177#)
            
            # 斗杆控制（直接电流控制，200~700mA）
            cmd.stick_retract_current = max(200.0, min(700.0, self.controls['stick_retract_current']))
            cmd.stick_extend_current = max(200.0, min(700.0, self.controls['stick_extend_current']))
            
            # 铲斗控制（直接电流控制，200~700mA）
            # 注意：界面上的"收斗"对应CAN的"翻斗"字段（151#线），"翻斗"对应CAN的"收斗"字段（175#线）
            # 需要交换映射以匹配实际硬件连接
            cmd.bucket_close_current = max(200.0, min(700.0, self.controls['bucket_dump_current']))  # 界面"收斗" -> CAN翻斗(151#)
            cmd.bucket_dump_current = max(200.0, min(700.0, self.controls['bucket_close_current']))  # 界面"翻斗" -> CAN收斗(175#)
            
            # 回转控制（直接电流控制，200~700mA）
            cmd.rotate_left_current = max(200.0, min(700.0, self.controls['rotate_left_current']))
            cmd.rotate_right_current = max(200.0, min(700.0, self.controls['rotate_right_current']))
            
            # 左右履带独立控制（直接电流控制，200~700mA）
            cmd.left_track_forward_current = max(200.0, min(700.0, self.controls['left_track_forward_current']))
            cmd.left_track_backward_current = max(200.0, min(700.0, self.controls['left_track_backward_current']))
            cmd.right_track_forward_current = max(200.0, min(700.0, self.controls['right_track_forward_current']))
            cmd.right_track_backward_current = max(200.0, min(700.0, self.controls['right_track_backward_current']))
            
            # 开关量控制（使用新扩展的protobuf字段）
            # 将26个开关量打包成位图
            switch_bits = 0
            switch_names = [
                'switch_21', 'switch_22', 'switch_23', 'switch_24',
                'switch_25', 'switch_26', 'switch_27', 'switch_28',
                'switch_29', 'switch_30', 'switch_31', 'switch_32',
                'switch_17', 'switch_18', 'switch_19', 'switch_20',
                'switch_38', 'switch_39', 'switch_40', 'switch_42',
                'switch_43', 'switch_44', 'switch_45', 'switch_46',
                'switch_37', 'switch_41',
            ]
            for i, switch_name in enumerate(switch_names):
                if self.switches.get(switch_name, False):
                    switch_bits |= (1 << i)
            
            # 添加 power_enable / start_up / acc 到 switch_bits 的预留位（bit 26 / 27 / 28）
            # 这些位将在 cannode 的 xiaosong.cc 中映射到 switch_ext_bits 的 bit 34 / 35 / 36
            if self.power_enable_active:
                switch_bits |= (1 << 26)  # bit 26 -> switch_ext_bits bit 34
            if self.start_up_active:
                switch_bits |= (1 << 27)  # bit 27 -> switch_ext_bits bit 35
            if self.acc_active:
                switch_bits |= (1 << 28)  # bit 28 -> switch_ext_bits bit 36
            
            cmd.switch_bits = switch_bits

            # 设置发动机油门档位到 pto_cmd（1~7），供 XiaosongCANParser 映射到 0x18FD0821（4~1V）
            try:
                lvl = int(self.engine_gear_level)
            except Exception:
                lvl = 0
            if lvl < 0:
                lvl = 0
            if lvl > 7:
                lvl = 7
            cmd.pto_cmd = lvl
            
            # 序列化 protobuf 消息
            serialized_data = cmd.SerializeToString()
            
            # 创建 ROS2 消息（序列化）
            ros_msg = cyber_msgs.msg.ProtoAdapter()
            # pb 字段需要 bytes 对象的列表，每个元素是单个字节的 bytes 对象
            ros_msg.pb = [bytes([b]) for b in serialized_data]
            
            # 发布序列化消息到 /vehicle_command
            self.cmd_pub.publish(ros_msg)
            
            # 创建调试消息（非序列化，易读的文本格式）
            # 使用易读的文本格式，方便 ros2 topic echo 查看
            debug_lines = []
            debug_lines.append("=" * 60)
            debug_lines.append(f"Vehicle Command Debug (timestamp: {cmd.header.timestamp_sec:.3f})")
            debug_lines.append("=" * 60)
            debug_lines.append("")
            
            # 安全控制
            debug_lines.append("【安全控制】")
            debug_lines.append(f"  刹车: {cmd.brake:.1f}%")
            debug_lines.append(f"  驻车制动: {'是' if cmd.parking_brake else '否'}")
            debug_lines.append(f"  紧急停止: {'是' if cmd.estop else '否'}")
            debug_lines.append(f"  远程申请进入遥控(手/自动): {'是' if cmd.request_remote_control else '否'}")
            debug_lines.append("")

            # 功能控制（0x18210005 部分位）
            debug_lines.append("【功能控制(0x18210005)】")
            ladder_txt = "无动作"
            if self.ladder_switch_mode == 1:
                ladder_txt = "车梯收起"
            elif self.ladder_switch_mode == 2:
                ladder_txt = "车梯放下"
            bucket_txt = "无动作"
            if self.bucket_switch_mode == 1:
                bucket_txt = "开斗电磁阀"
            elif self.bucket_switch_mode == 2:
                bucket_txt = "关斗电磁阀"
            horn_txt = "是" if self.horn_active else "否"
            power_enable_txt = "是" if self.power_enable_active else "否"
            start_up_txt = "是" if self.start_up_active else "否"
            debug_lines.append(f"  车梯: {ladder_txt}")
            debug_lines.append(f"  斗: {bucket_txt}")
            debug_lines.append(f"  喇叭: {horn_txt}")
            debug_lines.append(f"  上电: {power_enable_txt}")
            debug_lines.append(f"  发动机启动: {start_up_txt}")
            # ACC 指令状态
            acc_txt = "是" if self.acc_active else "否"
            debug_lines.append(f"  ACC: {acc_txt}")
            debug_lines.append("")
            
            # 动臂控制
            debug_lines.append("【动臂控制】")
            debug_lines.append(f"  抬升电流: {cmd.arm_up_current:.0f} mA")
            debug_lines.append(f"  下降电流: {cmd.arm_down_current:.0f} mA")
            debug_lines.append("")
            
            # 斗杆控制
            debug_lines.append("【斗杆控制】")
            debug_lines.append(f"  收回电流: {cmd.stick_retract_current:.0f} mA")
            debug_lines.append(f"  伸出电流: {cmd.stick_extend_current:.0f} mA")
            debug_lines.append("")
            
            # 铲斗控制
            debug_lines.append("【铲斗控制】")
            debug_lines.append(f"  收斗电流: {cmd.bucket_close_current:.0f} mA")
            debug_lines.append(f"  翻斗电流: {cmd.bucket_dump_current:.0f} mA")
            debug_lines.append("")
            
            # 回转控制
            debug_lines.append("【回转控制】")
            debug_lines.append(f"  左转电流: {cmd.rotate_left_current:.0f} mA")
            debug_lines.append(f"  右转电流: {cmd.rotate_right_current:.0f} mA")
            debug_lines.append("")
            
            # 左履带控制
            debug_lines.append("【左履带控制】")
            debug_lines.append(f"  前进电流: {cmd.left_track_forward_current:.0f} mA")
            debug_lines.append(f"  后退电流: {cmd.left_track_backward_current:.0f} mA")
            debug_lines.append("")
            
            # 右履带控制
            debug_lines.append("【右履带控制】")
            debug_lines.append(f"  前进电流: {cmd.right_track_forward_current:.0f} mA")
            debug_lines.append(f"  后退电流: {cmd.right_track_backward_current:.0f} mA")
            debug_lines.append("")
            
            # 开关量控制
            debug_lines.append("【开关量控制】")
            debug_lines.append(f"  开关位图: 0x{cmd.switch_bits:08X} ({cmd.switch_bits})")
            active_switches = [name for name, value in self.switches.items() if value]
            if active_switches:
                debug_lines.append(f"  激活的开关 ({len(active_switches)}个): {', '.join(active_switches)}")
            else:
                debug_lines.append("  激活的开关: 无")
            debug_lines.append("")
            
            # 下发 VCU 的发动机相关（pto→0x18FD0821；转速由 cannode 填 0x18210005）
            debug_lines.append("【下发VCU·发动机】")
            debug_lines.append(f"  pto_cmd(档位): {cmd.pto_cmd} (1~7 → 0x18FD0821)")
            erpm = int(self.can_status.get('engine_rpm_tx', 0) or 0)
            traw = int(self.can_status.get('engine_throttle_raw', 0) or 0)
            tv = float(self.can_status.get('engine_throttle_voltage', 0.0) or 0.0)
            tg = int(self.can_status.get('engine_throttle_gear_tx', 0) or 0)
            debug_lines.append(f"  CAN回显转速: {erpm} rpm (0x18210005 B5-6)")
            debug_lines.append(
                f"  CAN回显油门: raw=0x{traw:02X} {tv:.2f}V ≈{tg}档 (0x18FD0821)"
            )
            debug_lines.append("")
            
            # 发布调试消息到 /vehicle_command_debug（易读文本格式）
            debug_msg = StringMsg()
            debug_msg.data = "\n".join(debug_lines)
            self.cmd_debug_pub.publish(debug_msg)
            
            # 计算电流值
            self.calculate_currents()
            
            # 只在有变化时记录日志（避免日志过多，因为现在是10ms周期发送）
            if not hasattr(self, '_last_log_time'):
                self._last_log_time = 0.0
            
            current_time = self.get_clock().now().nanoseconds / 1e9
            if current_time - self._last_log_time >= 1.0:  # 每秒最多记录一次日志
                # 记录开关量状态
                active_switches = [name for name, value in self.switches.items() if value]
                switch_info = f', 开关量: {len(active_switches)}个激活' if active_switches else ', 开关量: 无激活'
                
                self.get_logger().info(f'已发布控制命令: '
                                      f'动臂: 抬升={self.controls["arm_up_current"]:.0f}mA 下降={self.controls["arm_down_current"]:.0f}mA, '
                                      f'斗杆: 收回={self.controls["stick_retract_current"]:.0f}mA 伸出={self.controls["stick_extend_current"]:.0f}mA, '
                                      f'铲斗: 收斗={self.controls["bucket_close_current"]:.0f}mA 翻斗={self.controls["bucket_dump_current"]:.0f}mA, '
                                      f'回转: 左={self.controls["rotate_left_current"]:.0f}mA 右={self.controls["rotate_right_current"]:.0f}mA, '
                                      f'左履带: 前={self.controls["left_track_forward_current"]:.0f}mA 后={self.controls["left_track_backward_current"]:.0f}mA, '
                                      f'右履带: 前={self.controls["right_track_forward_current"]:.0f}mA 后={self.controls["right_track_backward_current"]:.0f}mA'
                                      f'{switch_info}')
                self._last_log_time = current_time
            
        except Exception as e:
            self.get_logger().error(f'发布控制命令时出错: {e}')
    
    def parse_can_frame(self, can_data):
        """解析 CAN 帧数据，提取电流值"""
        can_id = can_data.get('can_id', '')
        can_id_dec = can_data.get('can_id_dec', 0)
        data_hex = can_data.get('data_hex', '')
        data_array = can_data.get('data', [])
        
        parsed_info = {
            'can_id': can_id,
            'can_id_dec': can_id_dec,
            'data_hex': data_hex,
            'currents': {}
        }
        
        # 根据 CAN ID 解析电流值
        if can_id_dec == 0x18210001:  # 动臂/斗杆控制
            if len(data_array) >= 8:
                # 动臂下降 (byte 0-1, 小端序)
                parsed_info['currents']['arm_down'] = data_array[0] | (data_array[1] << 8)
                # 动臂抬升 (byte 2-3)
                parsed_info['currents']['arm_up'] = data_array[2] | (data_array[3] << 8)
                # 斗杆伸出 (byte 4-5)
                parsed_info['currents']['stick_extend'] = data_array[4] | (data_array[5] << 8)
                # 斗杆收回 (byte 6-7)
                parsed_info['currents']['stick_retract'] = data_array[6] | (data_array[7] << 8)
                
                # 更新实际电流值
                # 注意：CAN报文中byte 0-1是arm_down_current(153#线下降)，byte 2-3是arm_up_current(177#线抬升)
                # 界面标签"下降电流"应该显示CAN的arm_down_current值，"抬升电流"应该显示CAN的arm_up_current值
                self.actual_currents['arm_down'] = parsed_info['currents']['arm_down']  # CAN byte 0-1 -> 界面"下降"标签
                self.actual_currents['arm_up'] = parsed_info['currents']['arm_up']    # CAN byte 2-3 -> 界面"抬升"标签
                self.actual_currents['stick_extend'] = parsed_info['currents']['stick_extend']
                self.actual_currents['stick_retract'] = parsed_info['currents']['stick_retract']
        
        elif can_id_dec == 0x18210002:  # 铲斗/回转控制
            if len(data_array) >= 8:
                # 铲斗翻斗 (byte 0-1)
                parsed_info['currents']['bucket_dump'] = data_array[0] | (data_array[1] << 8)
                # 铲斗收斗 (byte 2-3)
                parsed_info['currents']['bucket_close'] = data_array[2] | (data_array[3] << 8)
                # 回转左 (byte 4-5)
                parsed_info['currents']['rotate_left'] = data_array[4] | (data_array[5] << 8)
                # 回转右 (byte 6-7)
                parsed_info['currents']['rotate_right'] = data_array[6] | (data_array[7] << 8)
                
                # 更新实际电流值
                # 注意：CAN报文中byte 0-1是bucket_dump_current(151#线翻斗)，byte 2-3是bucket_close_current(175#线收斗)
                # 界面标签"翻斗电流"应该显示CAN的bucket_dump_current值，"收斗电流"应该显示CAN的bucket_close_current值
                self.actual_currents['bucket_dump'] = parsed_info['currents']['bucket_dump']    # CAN byte 0-1 -> 界面"翻斗"标签
                self.actual_currents['bucket_close'] = parsed_info['currents']['bucket_close']  # CAN byte 2-3 -> 界面"收斗"标签
                self.actual_currents['rotate_left'] = parsed_info['currents']['rotate_left']
                self.actual_currents['rotate_right'] = parsed_info['currents']['rotate_right']
        
        elif can_id_dec == 0x18210003:  # 行走控制
            if len(data_array) >= 8:
                # 左前进 (byte 0-1)
                parsed_info['currents']['left_forward'] = data_array[0] | (data_array[1] << 8)
                # 左后退 (byte 2-3)
                parsed_info['currents']['left_backward'] = data_array[2] | (data_array[3] << 8)
                # 右前进 (byte 4-5)
                parsed_info['currents']['right_forward'] = data_array[4] | (data_array[5] << 8)
                # 右后退 (byte 6-7)
                parsed_info['currents']['right_backward'] = data_array[6] | (data_array[7] << 8)
                
                # 更新实际电流值
                self.actual_currents['left_forward'] = parsed_info['currents']['left_forward']
                self.actual_currents['left_backward'] = parsed_info['currents']['left_backward']
                self.actual_currents['right_forward'] = parsed_info['currents']['right_forward']
                self.actual_currents['right_backward'] = parsed_info['currents']['right_backward']
        
        elif can_id_dec == 0x18210005:  # 开关量控制
            if len(data_array) >= 8:
                # 解析开关量（bit字段）
                switch_byte0 = data_array[0]
                switch_byte1 = data_array[1]
                switch_byte2 = data_array[2]
                switch_byte3 = data_array[3]
                
                parsed_info['switches'] = {
                    'switch_21': bool(switch_byte0 & 0x01),
                    'switch_22': bool(switch_byte0 & 0x02),
                    'switch_23': bool(switch_byte0 & 0x04),
                    'switch_24': bool(switch_byte0 & 0x08),
                    'switch_25': bool(switch_byte0 & 0x10),
                    'switch_26': bool(switch_byte0 & 0x20),
                    'switch_27': bool(switch_byte0 & 0x40),
                    'switch_28': bool(switch_byte0 & 0x80),
                    'switch_29': bool(switch_byte1 & 0x01),  # bit 8
                    'switch_30': bool(switch_byte1 & 0x02),  # bit 9
                    'switch_31': bool(switch_byte1 & 0x04),  # bit 10
                    'switch_19': bool(switch_byte1 & 0x08),  # bit 11: 喇叭 (OUT_19_DOH(128#线))
                    'switch_17': bool(switch_byte1 & 0x10),  # bit 12: 灯光1 (OUT_17_DOH(125#线))
                    'switch_18': bool(switch_byte1 & 0x20),  # bit 13: 灯光2 (OUT_18_DOH(129#线))
                    'switch_32': bool(switch_byte1 & 0x40),  # bit 14: 备用1 (OUT_32_DOH(257#线))
                    'switch_20': bool(switch_byte1 & 0x80),  # bit 15: 备用2 (OUT_20_DOH(103#线))
                    'switch_38': bool(switch_byte2 & 0x01),
                    'switch_39': bool(switch_byte2 & 0x02),
                    'switch_40': bool(switch_byte2 & 0x04),
                    'switch_42': bool(switch_byte2 & 0x08),
                    'switch_43': bool(switch_byte2 & 0x10),
                    'switch_44': bool(switch_byte2 & 0x20),
                    'switch_45': bool(switch_byte2 & 0x40),
                    'switch_46': bool(switch_byte2 & 0x80),
                    'switch_37': bool(switch_byte3 & 0x01),
                    'switch_41': bool(switch_byte3 & 0x02),
                }

                # 解析高级功能状态（由部分开关量位组合得到）
                # ladder_switch: switch_21 / switch_22
                ladder_switch = 0
                if parsed_info['switches'].get('switch_21', False):
                    ladder_switch = 1  # 收起
                elif parsed_info['switches'].get('switch_22', False):
                    ladder_switch = 2  # 放下

                # bucket_switch: switch_30 / switch_31
                bucket_switch = 0
                if parsed_info['switches'].get('switch_30', False):
                    bucket_switch = 1  # 开斗
                elif parsed_info['switches'].get('switch_31', False):
                    bucket_switch = 2  # 关斗

                # 喇叭：switch_19 (bit 11)
                horn = bool(parsed_info['switches'].get('switch_19', False))

                # 先导安全锁定：switch_24
                hydraulic_lock = bool(parsed_info['switches'].get('switch_24', False))

                # 灯光1/2：switch_17 / switch_18 组合为 light_code
                light_code = 0
                if parsed_info['switches'].get('switch_17', False):
                    light_code |= 0x1  # 灯光1
                if parsed_info['switches'].get('switch_18', False):
                    light_code |= 0x2  # 灯光2

                parsed_info['function_status'] = {
                    'ladder_switch': ladder_switch,
                    'bucket_switch': bucket_switch,
                    'horn': horn,
                    'hydraulic_lock': hydraulic_lock,
                    'light_code': light_code,
                }

                # 更新状态变量（用于 UI 显示）
                self.can_status['ladder_switch'] = ladder_switch
                self.can_status['bucket_switch'] = bucket_switch
                self.can_status['horn'] = horn
                self.can_status['hydraulic_lock'] = hydraulic_lock
                self.can_status['light_code'] = light_code

                # byte5-6：发动机转速 0~8000 rpm（小端），sendSwitchControl 末尾覆盖（见 xiaosong.cc）
                engine_rpm_tx = int(data_array[5]) | (int(data_array[6]) << 8)
                if engine_rpm_tx > 8000:
                    engine_rpm_tx = 8000
                parsed_info['engine_rpm_tx'] = engine_rpm_tx
                self.can_status['engine_rpm_tx'] = engine_rpm_tx
                
                # 解析扩展位 switch_ext_bits：低 5 个标志位在 byte4（bit32~bit36 对应 0x1~0x10），
                # byte5-6 已被转速覆盖，不可用 uint32 拼整帧。
                if len(data_array) >= 5:
                    ext_low = data_array[4]
                    request_remote = bool(ext_low & 0x01)   # bit32
                    emergency_stop = bool(ext_low & 0x02)   # bit33
                    power_enable = bool(ext_low & 0x04)     # bit34
                    start_up = bool(ext_low & 0x08)         # bit35
                    acc = bool(ext_low & 0x10)              # bit36
                    
                    parsed_info['can_status'] = {
                        'request_remote_control': request_remote,
                        'emergency_stop': emergency_stop,
                        'power_enable': power_enable,
                        'start_up': start_up,
                        'acc': acc,
                    }
                    
                    self.can_status['request_remote_control'] = request_remote
                    self.can_status['emergency_stop'] = emergency_stop
                    self.can_status['power_enable'] = power_enable
                    self.can_status['start_up'] = start_up
                    self.can_status['acc'] = acc
        
        elif can_id_dec == 0x18FD0821:  # 发动机油门控制器（pto_cmd→电压，与 publish_command 一致）
            if len(data_array) >= 8:
                raw_byte = int(data_array[6])
                voltage = raw_byte * 0.05
                # 与 cannode sendEngineThrottleControl：1~7 档 → 4V~1V，raw = voltage/0.05
                gear_tx = 1
                if raw_byte > 0:
                    try:
                        g = 1 + round((4.0 - voltage) / 3.0 * 6.0)
                        gear_tx = max(1, min(7, int(g)))
                    except Exception:
                        gear_tx = 1
                parsed_info['engine_throttle'] = {'raw': raw_byte, 'voltage': voltage, 'gear_tx': gear_tx}
                self.can_status['engine_throttle_raw'] = raw_byte
                self.can_status['engine_throttle_voltage'] = voltage
                self.can_status['engine_throttle_gear_tx'] = gear_tx
        
        return parsed_info
    
    def get_latest_can_frames(self):
        """获取最新的 CAN 帧信息（线程安全）"""
        with self.data_lock:
            return self.latest_can_frames.copy()
    
    def run_gui(self):
        """运行 GUI 界面"""
        root = tk.Tk()
        root.title('小松挖掘机控制界面')
        
        # 设置默认窗口大小（原来的大小）
        root.geometry('1400x900')
        
        # 初始化缩放因子为1.0（默认大小）
        self.scale_factor = 1.0
        
        # 深色科技感主题
        bg_color = '#0a0e27'
        fg_color = '#00ffff'
        accent_color = '#00ff88'
        danger_color = '#ff0044'
        panel_bg = '#1a1f3a'
        
        # 存储颜色变量为类属性，以便在其他方法中使用
        self.fg_color = fg_color
        self.accent_color = accent_color
        
        root.configure(bg=bg_color)
        
        # 存储根窗口引用
        self.root_window = root
        
        # 绑定窗口关闭事件
        def on_closing():
            self.gui_running = False
            root.quit()
            root.destroy()
            rclpy.shutdown()
            sys.exit(0)
        
        root.protocol("WM_DELETE_WINDOW", on_closing)
        
        # 绑定窗口大小变化事件
        def on_window_resize(event=None):
            """窗口大小变化时的回调函数"""
            if event and event.widget == root:
                new_width = root.winfo_width()
                new_height = root.winfo_height()
                
                # 计算新的缩放因子
                width_scale = new_width / self.base_width
                height_scale = new_height / self.base_height
                new_scale = min(width_scale, height_scale)
                
                # 如果缩放因子变化超过5%，更新所有字体
                if abs(new_scale - self.scale_factor) > 0.05:
                    self.scale_factor = new_scale
                    self.update_all_fonts()
        
        root.bind('<Configure>', on_window_resize)
        
        # 创建主容器
        main_container = tk.Frame(root, bg=bg_color)
        main_container.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # 确保主容器可以接收焦点
        main_container.focus_set()
        
        # 标题栏
        title_frame = tk.Frame(main_container, bg=bg_color)
        title_frame.pack(fill=tk.X, pady=(0, 20))
        
        title_label = tk.Label(
            title_frame,
            text='小松挖掘机控制界面',
            font=self.get_scaled_font('Arial', 24, 'bold'),
            bg=bg_color,
            fg=fg_color
        )
        title_label.pack()
        self.widget_fonts[title_label] = ('Arial', 24, 'bold')
        
        # 创建左中右布局（左侧和右侧自动填充，中间面板固定较小宽度）
        left_panel = tk.Frame(main_container, bg=panel_bg, relief=tk.RAISED, bd=2)
        left_panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 10))
        
        middle_panel = tk.Frame(main_container, bg=panel_bg, relief=tk.RAISED, bd=2, width=380)
        middle_panel.pack_propagate(False)  # 防止子组件改变父组件大小
        middle_panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=False, padx=(0, 10))
        
        right_panel = tk.Frame(main_container, bg=panel_bg, relief=tk.RAISED, bd=2)
        right_panel.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(10, 0))
        
        # 左侧：控制面板
        self.create_control_panel(left_panel, bg_color, fg_color, accent_color, danger_color, panel_bg)
        
        # 中间：安全控制和开关量控制
        self.create_middle_panel(middle_panel, bg_color, fg_color, accent_color, danger_color, panel_bg)
        
        # 右侧：数据显示面板
        self.create_data_panel(right_panel, bg_color, fg_color, accent_color, panel_bg)
        
        # 存储根窗口
        self.root_window = root
        
        # 设置键盘增量步长（mA）
        self.keyboard_step = 10.0  # 每次按键增加/减少 10mA
        
        # 绑定键盘事件
        self.setup_keyboard_controls(root)
        
        # 启动时重置所有控制值（在 GUI 创建完成后）
        self.reset_controls()
        
        # 更新函数
        def update_gui():
            if not self.gui_running:
                return
            try:
                self.update_data_panel()
                self.update_actual_current_labels()  # 更新实际发送电流标签
                self.update_status_labels()  # 更新状态标签
                root.after(100, update_gui)  # 每100ms更新一次
            except tk.TclError:
                pass
        
        # 开始更新循环
        update_gui()
        
        # 运行 GUI
        root.mainloop()
    
    def get_scaled_font(self, family, size, style='', widget=None):
        """获取缩放后的字体，并可选择性地注册到字体更新系统"""
        scaled_size = int(size * self.scale_factor)
        font_key = (family, scaled_size, style)
        
        if font_key not in self.font_cache:
            if style:
                self.font_cache[font_key] = (family, scaled_size, style)
            else:
                self.font_cache[font_key] = (family, scaled_size)
        
        # 如果提供了 widget，自动注册到字体更新系统
        if widget is not None:
            if style:
                self.widget_fonts[widget] = (family, size, style)
            else:
                self.widget_fonts[widget] = (family, size)
        
        return self.font_cache[font_key]
    
    def update_all_fonts(self):
        """更新所有控件的字体"""
        for widget, font_info in self.widget_fonts.items():
            try:
                if isinstance(font_info, tuple) and len(font_info) == 3:
                    family, base_size, style = font_info
                    scaled_size = int(base_size * self.scale_factor)
                    widget.config(font=(family, scaled_size, style))
                elif isinstance(font_info, tuple) and len(font_info) == 2:
                    family, base_size = font_info
                    scaled_size = int(base_size * self.scale_factor)
                    widget.config(font=(family, scaled_size))
            except tk.TclError:
                pass  # 忽略已销毁的控件
    
    def create_control_panel(self, parent, bg_color, fg_color, accent_color, danger_color, panel_bg):
        """创建控制面板"""
        # 标题
        title = tk.Label(
            parent,
            text='控制面板',
            font=self.get_scaled_font('Arial', 16, 'bold'),
            bg=panel_bg,
            fg=fg_color
        )
        title.pack(pady=10)
        self.widget_fonts[title] = ('Arial', 16, 'bold')
        
        # 创建滚动区域
        canvas = tk.Canvas(parent, bg=panel_bg, highlightthickness=0)
        scrollbar = ttk.Scrollbar(parent, orient="vertical", command=canvas.yview)
        scrollable_frame = tk.Frame(canvas, bg=panel_bg)
        
        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        
        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        canvas.pack(side="left", fill="both", expand=True, padx=10, pady=10)
        scrollbar.pack(side="right", fill="y")
        
        # 动臂控制（直接电流控制）
        arm_frame = tk.LabelFrame(
            scrollable_frame,
            text='动臂控制（电流控制 200~700mA）',
            font=self.get_scaled_font('Arial', 12, 'bold'),
            bg=panel_bg,
            fg=fg_color,
            padx=10,
            pady=10
        )
        arm_frame.pack(fill=tk.X, padx=10, pady=5)
        self.widget_fonts[arm_frame] = ('Arial', 12, 'bold')
        
        # 动臂抬升电流
        arm_up_frame = tk.Frame(arm_frame, bg=panel_bg)
        arm_up_frame.pack(fill=tk.X, pady=3)
        
        # 获取对应的线号
        arm_up_wire = self.valve_wire_map.get('arm_up_current', '')
        tk.Label(
            arm_up_frame,
            text=f'抬升电流 ({arm_up_wire}线):',
            bg=panel_bg,
            fg=fg_color,
            font=('Arial', 10),
            width=15
        ).pack(side=tk.LEFT)
        
        # 键盘快捷键提示
        tk.Label(
            arm_up_frame,
            text='[Q/q]',
            bg=panel_bg,
            fg=accent_color,
            font=('Arial', 8),
            width=4
        ).pack(side=tk.LEFT, padx=(0, 3))
        
        arm_up_var = tk.DoubleVar(value=self.controls['arm_up_current'])
        self.control_vars['arm_up_current'] = arm_up_var  # 存储变量用于重置
        arm_up_scale = tk.Scale(
            arm_up_frame,
            from_=0,
            to=700,
            orient=tk.HORIZONTAL,
            variable=arm_up_var,
            bg=panel_bg,
            fg=fg_color,
            troughcolor='#2a3f5a',
            activebackground=accent_color,
            length=130,
            command=lambda v: self.update_control('arm_up_current', float(v))
        )
        arm_up_scale.pack(side=tk.LEFT, fill=tk.NONE, expand=False)
        
        arm_up_label = tk.Label(
            arm_up_frame,
            text=f'{self.controls["arm_up_current"]:.0f}mA',
            bg=panel_bg,
            fg=accent_color,
            font=('Arial', 10, 'bold'),
            width=7
        )
        arm_up_label.pack(side=tk.LEFT, padx=3)
        self.label_widgets['arm_up_current'] = arm_up_label  # 存储标签引用
        
        # 显示实际发送的电流值（从CAN解析）
        arm_up_actual_label = tk.Label(
            arm_up_frame,
            text='[CAN:0mA]',
            bg=panel_bg,
            fg=fg_color,
            font=('Arial', 11),
            width=9
        )
        arm_up_actual_label.pack(side=tk.LEFT, padx=2)
        self.actual_label_widgets['arm_up'] = arm_up_actual_label  # 存储实际电流标签引用
        
        def update_arm_up_label(v):
            arm_up_label.config(text=f'{float(v):.0f}mA')
            self.update_control('arm_up_current', float(v))
        
        arm_up_scale.config(command=update_arm_up_label)
        
        # 动臂下降电流
        arm_down_frame = tk.Frame(arm_frame, bg=panel_bg)
        arm_down_frame.pack(fill=tk.X, pady=3)
        
        # 获取对应的线号
        arm_down_wire = self.valve_wire_map.get('arm_down_current', '')
        tk.Label(
            arm_down_frame,
            text=f'下降电流 ({arm_down_wire}线):',
            bg=panel_bg,
            fg=fg_color,
            font=('Arial', 10),
            width=15
        ).pack(side=tk.LEFT)
        
        # 键盘快捷键提示
        tk.Label(
            arm_down_frame,
            text='[A/a]',
            bg=panel_bg,
            fg=accent_color,
            font=('Arial', 8),
            width=4
        ).pack(side=tk.LEFT, padx=(0, 3))
        
        arm_down_var = tk.DoubleVar(value=self.controls['arm_down_current'])
        self.control_vars['arm_down_current'] = arm_down_var
        arm_down_scale = tk.Scale(
            arm_down_frame,
            from_=0,
            to=700,
            orient=tk.HORIZONTAL,
            variable=arm_down_var,
            bg=panel_bg,
            fg=fg_color,
            troughcolor='#2a3f5a',
            activebackground=danger_color,
            length=130,
            command=lambda v: self.update_control('arm_down_current', float(v))
        )
        arm_down_scale.pack(side=tk.LEFT, fill=tk.NONE, expand=False)
        
        arm_down_label = tk.Label(
            arm_down_frame,
            text=f'{self.controls["arm_down_current"]:.0f}mA',
            bg=panel_bg,
            fg=danger_color,
            font=('Arial', 10, 'bold'),
            width=7
        )
        arm_down_label.pack(side=tk.LEFT, padx=3)
        self.label_widgets['arm_down_current'] = arm_down_label  # 存储标签引用
        
        # 显示实际发送的电流值（从CAN解析）
        arm_down_actual_label = tk.Label(
            arm_down_frame,
            text='[CAN:0mA]',
            bg=panel_bg,
            fg=fg_color,
            font=('Arial', 11),
            width=9
        )
        arm_down_actual_label.pack(side=tk.LEFT, padx=2)
        self.actual_label_widgets['arm_down'] = arm_down_actual_label  # 存储实际电流标签引用
        
        def update_arm_down_label(v):
            arm_down_label.config(text=f'{float(v):.0f}mA')
            self.update_control('arm_down_current', float(v))
        
        arm_down_scale.config(command=update_arm_down_label)
        
        # 铲斗控制（直接电流控制）
        shovel_frame = tk.LabelFrame(
            scrollable_frame,
            text='铲斗控制（电流控制 200~700mA）',
            font=self.get_scaled_font('Arial', 12, 'bold'),
            bg=panel_bg,
            fg=fg_color,
            padx=10,
            pady=10
        )
        shovel_frame.pack(fill=tk.X, padx=10, pady=5)
        self.widget_fonts[shovel_frame] = ('Arial', 12, 'bold')
        
        # 铲斗收斗电流
        bucket_close_frame = tk.Frame(shovel_frame, bg=panel_bg)
        bucket_close_frame.pack(fill=tk.X, pady=3)
        
        # 获取对应的线号
        bucket_close_wire = self.valve_wire_map.get('bucket_close_current', '')
        tk.Label(
            bucket_close_frame,
            text=f'收斗电流 ({bucket_close_wire}线):',
            bg=panel_bg,
            fg=fg_color,
            font=('Arial', 10),
            width=15
        ).pack(side=tk.LEFT)
        
        # 键盘快捷键提示
        tk.Label(
            bucket_close_frame,
            text='[W/w]',
            bg=panel_bg,
            fg=accent_color,
            font=('Arial', 8),
            width=4
        ).pack(side=tk.LEFT, padx=(0, 3))
        
        bucket_close_var = tk.DoubleVar(value=self.controls['bucket_close_current'])
        self.control_vars['bucket_close_current'] = bucket_close_var
        bucket_close_scale = tk.Scale(
            bucket_close_frame,
            from_=0,
            to=700,
            orient=tk.HORIZONTAL,
            variable=bucket_close_var,
            bg=panel_bg,
            fg=fg_color,
            troughcolor='#2a3f5a',
            activebackground=accent_color,
            length=130,
            command=lambda v: self.update_control('bucket_close_current', float(v))
        )
        bucket_close_scale.pack(side=tk.LEFT, fill=tk.NONE, expand=False)
        
        bucket_close_label = tk.Label(
            bucket_close_frame,
            text=f'{self.controls["bucket_close_current"]:.0f}mA',
            bg=panel_bg,
            fg=accent_color,
            font=('Arial', 10, 'bold'),
            width=7
        )
        bucket_close_label.pack(side=tk.LEFT, padx=3)
        self.label_widgets['bucket_close_current'] = bucket_close_label  # 存储标签引用
        
        # 显示实际发送的电流值（从CAN解析）
        bucket_close_actual_label = tk.Label(
            bucket_close_frame,
            text='[CAN:0mA]',
            bg=panel_bg,
            fg=fg_color,
            font=('Arial', 11),
            width=9
        )
        bucket_close_actual_label.pack(side=tk.LEFT, padx=2)
        self.actual_label_widgets['bucket_close'] = bucket_close_actual_label  # 存储实际电流标签引用
        
        def update_bucket_close_label(v):
            bucket_close_label.config(text=f'{float(v):.0f}mA')
            self.update_control('bucket_close_current', float(v))
        
        bucket_close_scale.config(command=update_bucket_close_label)
        
        # 铲斗翻斗电流
        bucket_dump_frame = tk.Frame(shovel_frame, bg=panel_bg)
        bucket_dump_frame.pack(fill=tk.X, pady=3)
        
        # 获取对应的线号
        bucket_dump_wire = self.valve_wire_map.get('bucket_dump_current', '')
        tk.Label(
            bucket_dump_frame,
            text=f'翻斗电流 ({bucket_dump_wire}线):',
            bg=panel_bg,
            fg=fg_color,
            font=('Arial', 10),
            width=15
        ).pack(side=tk.LEFT)
        
        # 键盘快捷键提示
        tk.Label(
            bucket_dump_frame,
            text='[S/s]',
            bg=panel_bg,
            fg=accent_color,
            font=('Arial', 8),
            width=4
        ).pack(side=tk.LEFT, padx=(0, 3))
        
        bucket_dump_var = tk.DoubleVar(value=self.controls['bucket_dump_current'])
        self.control_vars['bucket_dump_current'] = bucket_dump_var
        bucket_dump_scale = tk.Scale(
            bucket_dump_frame,
            from_=0,
            to=700,
            orient=tk.HORIZONTAL,
            variable=bucket_dump_var,
            bg=panel_bg,
            fg=fg_color,
            troughcolor='#2a3f5a',
            activebackground=danger_color,
            length=130,
            command=lambda v: self.update_control('bucket_dump_current', float(v))
        )
        bucket_dump_scale.pack(side=tk.LEFT, fill=tk.NONE, expand=False)
        
        bucket_dump_label = tk.Label(
            bucket_dump_frame,
            text=f'{self.controls["bucket_dump_current"]:.0f}mA',
            bg=panel_bg,
            fg=danger_color,
            font=('Arial', 10, 'bold'),
            width=7
        )
        bucket_dump_label.pack(side=tk.LEFT, padx=3)
        self.label_widgets['bucket_dump_current'] = bucket_dump_label  # 存储标签引用
        
        # 显示实际发送的电流值（从CAN解析）
        bucket_dump_actual_label = tk.Label(
            bucket_dump_frame,
            text='[CAN:0mA]',
            bg=panel_bg,
            fg=fg_color,
            font=('Arial', 11),
            width=9
        )
        bucket_dump_actual_label.pack(side=tk.LEFT, padx=2)
        self.actual_label_widgets['bucket_dump'] = bucket_dump_actual_label  # 存储实际电流标签引用
        
        def update_bucket_dump_label(v):
            bucket_dump_label.config(text=f'{float(v):.0f}mA')
            self.update_control('bucket_dump_current', float(v))
        
        bucket_dump_scale.config(command=update_bucket_dump_label)
        
        # 斗杆控制（直接电流控制）
        stick_frame = tk.LabelFrame(
            scrollable_frame,
            text='斗杆控制（电流控制 200~700mA）',
            font=self.get_scaled_font('Arial', 12, 'bold'),
            bg=panel_bg,
            fg=fg_color,
            padx=10,
            pady=10
        )
        stick_frame.pack(fill=tk.X, padx=10, pady=5)
        self.widget_fonts[stick_frame] = ('Arial', 12, 'bold')
        
        # 斗杆收回电流
        stick_retract_frame = tk.Frame(stick_frame, bg=panel_bg)
        stick_retract_frame.pack(fill=tk.X, pady=3)
        
        # 获取对应的线号
        stick_retract_wire = self.valve_wire_map.get('stick_retract_current', '')
        tk.Label(
            stick_retract_frame,
            text=f'收回电流 ({stick_retract_wire}线):',
            bg=panel_bg,
            fg=fg_color,
            font=('Arial', 10),
            width=15
        ).pack(side=tk.LEFT)
        
        # 键盘快捷键提示
        tk.Label(
            stick_retract_frame,
            text='[E/e]',
            bg=panel_bg,
            fg=accent_color,
            font=('Arial', 8),
            width=4
        ).pack(side=tk.LEFT, padx=(0, 3))
        
        stick_retract_var = tk.DoubleVar(value=self.controls['stick_retract_current'])
        self.control_vars['stick_retract_current'] = stick_retract_var
        stick_retract_scale = tk.Scale(
            stick_retract_frame,
            from_=0,
            to=700,
            orient=tk.HORIZONTAL,
            variable=stick_retract_var,
            bg=panel_bg,
            fg=fg_color,
            troughcolor='#2a3f5a',
            activebackground=danger_color,
            length=130,
            command=lambda v: self.update_control('stick_retract_current', float(v))
        )
        stick_retract_scale.pack(side=tk.LEFT, fill=tk.NONE, expand=False)
        
        stick_retract_label = tk.Label(
            stick_retract_frame,
            text=f'{self.controls["stick_retract_current"]:.0f}mA',
            bg=panel_bg,
            fg=danger_color,
            font=('Arial', 10, 'bold'),
            width=7
        )
        stick_retract_label.pack(side=tk.LEFT, padx=3)
        self.label_widgets['stick_retract_current'] = stick_retract_label  # 存储标签引用
        
        # 显示实际发送的电流值（从CAN解析）
        stick_retract_actual_label = tk.Label(
            stick_retract_frame,
            text='[CAN:0mA]',
            bg=panel_bg,
            fg=fg_color,
            font=('Arial', 11),
            width=9
        )
        stick_retract_actual_label.pack(side=tk.LEFT, padx=2)
        self.actual_label_widgets['stick_retract'] = stick_retract_actual_label  # 存储实际电流标签引用
        
        def update_stick_retract_label(v):
            stick_retract_label.config(text=f'{float(v):.0f}mA')
            self.update_control('stick_retract_current', float(v))
        
        stick_retract_scale.config(command=update_stick_retract_label)
        
        # 斗杆伸出电流
        stick_extend_frame = tk.Frame(stick_frame, bg=panel_bg)
        stick_extend_frame.pack(fill=tk.X, pady=3)
        
        # 获取对应的线号
        stick_extend_wire = self.valve_wire_map.get('stick_extend_current', '')
        tk.Label(
            stick_extend_frame,
            text=f'伸出电流 ({stick_extend_wire}线):',
            bg=panel_bg,
            fg=fg_color,
            font=('Arial', 10),
            width=15
        ).pack(side=tk.LEFT)
        
        # 键盘快捷键提示
        tk.Label(
            stick_extend_frame,
            text='[D/d]',
            bg=panel_bg,
            fg=accent_color,
            font=('Arial', 8),
            width=4
        ).pack(side=tk.LEFT, padx=(0, 3))
        
        stick_extend_var = tk.DoubleVar(value=self.controls['stick_extend_current'])
        self.control_vars['stick_extend_current'] = stick_extend_var
        stick_extend_scale = tk.Scale(
            stick_extend_frame,
            from_=0,
            to=700,
            orient=tk.HORIZONTAL,
            variable=stick_extend_var,
            bg=panel_bg,
            fg=fg_color,
            troughcolor='#2a3f5a',
            activebackground=accent_color,
            length=130,
            command=lambda v: self.update_control('stick_extend_current', float(v))
        )
        stick_extend_scale.pack(side=tk.LEFT, fill=tk.NONE, expand=False)
        
        stick_extend_label = tk.Label(
            stick_extend_frame,
            text=f'{self.controls["stick_extend_current"]:.0f}mA',
            bg=panel_bg,
            fg=accent_color,
            font=('Arial', 10, 'bold'),
            width=7
        )
        stick_extend_label.pack(side=tk.LEFT, padx=3)
        self.label_widgets['stick_extend_current'] = stick_extend_label  # 存储标签引用
        
        # 显示实际发送的电流值（从CAN解析）
        stick_extend_actual_label = tk.Label(
            stick_extend_frame,
            text='[CAN:0mA]',
            bg=panel_bg,
            fg=fg_color,
            font=('Arial', 11),
            width=9
        )
        stick_extend_actual_label.pack(side=tk.LEFT, padx=2)
        self.actual_label_widgets['stick_extend'] = stick_extend_actual_label  # 存储实际电流标签引用
        
        def update_stick_extend_label(v):
            stick_extend_label.config(text=f'{float(v):.0f}mA')
            self.update_control('stick_extend_current', float(v))
        
        stick_extend_scale.config(command=update_stick_extend_label)
        
        # 回转控制（直接电流控制）
        rotate_frame = tk.LabelFrame(
            scrollable_frame,
            text='回转控制（电流控制 200~700mA）',
            font=self.get_scaled_font('Arial', 12, 'bold'),
            bg=panel_bg,
            fg=fg_color,
            padx=10,
            pady=10
        )
        rotate_frame.pack(fill=tk.X, padx=10, pady=5)
        self.widget_fonts[rotate_frame] = ('Arial', 12, 'bold')
        
        # 回转左电流
        rotate_left_frame = tk.Frame(rotate_frame, bg=panel_bg)
        rotate_left_frame.pack(fill=tk.X, pady=3)
        
        # 获取对应的线号
        rotate_left_wire = self.valve_wire_map.get('rotate_left_current', '')
        tk.Label(
            rotate_left_frame,
            text=f'左转电流 ({rotate_left_wire}线):',
            bg=panel_bg,
            fg=fg_color,
            font=('Arial', 10),
            width=15
        ).pack(side=tk.LEFT)
        
        # 键盘快捷键提示
        tk.Label(
            rotate_left_frame,
            text='[R/r]',
            bg=panel_bg,
            fg=accent_color,
            font=('Arial', 8),
            width=4
        ).pack(side=tk.LEFT, padx=(0, 3))
        
        rotate_left_var = tk.DoubleVar(value=self.controls['rotate_left_current'])
        self.control_vars['rotate_left_current'] = rotate_left_var
        rotate_left_scale = tk.Scale(
            rotate_left_frame,
            from_=0,
            to=700,
            orient=tk.HORIZONTAL,
            variable=rotate_left_var,
            bg=panel_bg,
            fg=fg_color,
            troughcolor='#2a3f5a',
            activebackground=danger_color,
            length=130,
            command=lambda v: self.update_control('rotate_left_current', float(v))
        )
        rotate_left_scale.pack(side=tk.LEFT, fill=tk.NONE, expand=False)
        
        rotate_left_label = tk.Label(
            rotate_left_frame,
            text=f'{self.controls["rotate_left_current"]:.0f}mA',
            bg=panel_bg,
            fg=danger_color,
            font=('Arial', 10, 'bold'),
            width=7
        )
        rotate_left_label.pack(side=tk.LEFT, padx=3)
        self.label_widgets['rotate_left_current'] = rotate_left_label  # 存储标签引用
        
        # 显示实际发送的电流值（从CAN解析）
        rotate_left_actual_label = tk.Label(
            rotate_left_frame,
            text='[CAN:0mA]',
            bg=panel_bg,
            fg=fg_color,
            font=('Arial', 11),
            width=9
        )
        rotate_left_actual_label.pack(side=tk.LEFT, padx=2)
        self.actual_label_widgets['rotate_left'] = rotate_left_actual_label  # 存储实际电流标签引用
        
        def update_rotate_left_label(v):
            rotate_left_label.config(text=f'{float(v):.0f}mA')
            self.update_control('rotate_left_current', float(v))
        
        rotate_left_scale.config(command=update_rotate_left_label)
        
        # 回转右电流
        rotate_right_frame = tk.Frame(rotate_frame, bg=panel_bg)
        rotate_right_frame.pack(fill=tk.X, pady=3)
        
        # 获取对应的线号
        rotate_right_wire = self.valve_wire_map.get('rotate_right_current', '')
        tk.Label(
            rotate_right_frame,
            text=f'右转电流 ({rotate_right_wire}线):',
            bg=panel_bg,
            fg=fg_color,
            font=('Arial', 10),
            width=15
        ).pack(side=tk.LEFT)
        
        # 键盘快捷键提示
        tk.Label(
            rotate_right_frame,
            text='[F/f]',
            bg=panel_bg,
            fg=accent_color,
            font=('Arial', 8),
            width=4
        ).pack(side=tk.LEFT, padx=(0, 3))
        
        rotate_right_var = tk.DoubleVar(value=self.controls['rotate_right_current'])
        self.control_vars['rotate_right_current'] = rotate_right_var
        rotate_right_scale = tk.Scale(
            rotate_right_frame,
            from_=0,
            to=700,
            orient=tk.HORIZONTAL,
            variable=rotate_right_var,
            bg=panel_bg,
            fg=fg_color,
            troughcolor='#2a3f5a',
            activebackground=accent_color,
            length=130,
            command=lambda v: self.update_control('rotate_right_current', float(v))
        )
        rotate_right_scale.pack(side=tk.LEFT, fill=tk.NONE, expand=False)
        
        rotate_right_label = tk.Label(
            rotate_right_frame,
            text=f'{self.controls["rotate_right_current"]:.0f}mA',
            bg=panel_bg,
            fg=accent_color,
            font=('Arial', 10, 'bold'),
            width=7
        )
        rotate_right_label.pack(side=tk.LEFT, padx=3)
        self.label_widgets['rotate_right_current'] = rotate_right_label  # 存储标签引用
        
        # 显示实际发送的电流值（从CAN解析）
        rotate_right_actual_label = tk.Label(
            rotate_right_frame,
            text='[CAN:0mA]',
            bg=panel_bg,
            fg=fg_color,
            font=('Arial', 11),
            width=9
        )
        rotate_right_actual_label.pack(side=tk.LEFT, padx=2)
        self.actual_label_widgets['rotate_right'] = rotate_right_actual_label  # 存储实际电流标签引用
        
        def update_rotate_right_label(v):
            rotate_right_label.config(text=f'{float(v):.0f}mA')
            self.update_control('rotate_right_current', float(v))
        
        rotate_right_scale.config(command=update_rotate_right_label)
        
        # 行走控制（左右履带独立控制，直接电流控制）
        walking_frame = tk.LabelFrame(
            scrollable_frame,
            text='行走控制（左右履带独立，电流控制 200~700mA）',
            font=self.get_scaled_font('Arial', 12, 'bold'),
            bg=panel_bg,
            fg=fg_color,
            padx=10,
            pady=10
        )
        walking_frame.pack(fill=tk.X, padx=10, pady=5)
        self.widget_fonts[walking_frame] = ('Arial', 12, 'bold')
        
        # 左履带前进
        left_fwd_frame = tk.Frame(walking_frame, bg=panel_bg)
        left_fwd_frame.pack(fill=tk.X, pady=3)
        
        # 获取对应的线号
        left_fwd_wire = self.valve_wire_map.get('left_track_forward_current', '')
        tk.Label(
            left_fwd_frame,
            text=f'左履带前进 ({left_fwd_wire}线):',
            bg=panel_bg,
            fg=fg_color,
            font=('Arial', 10),
            width=18
        ).pack(side=tk.LEFT)
        
        # 键盘快捷键提示
        tk.Label(
            left_fwd_frame,
            text='[T/t]',
            bg=panel_bg,
            fg=accent_color,
            font=('Arial', 8),
            width=4
        ).pack(side=tk.LEFT, padx=(0, 3))
        
        left_fwd_var = tk.DoubleVar(value=self.controls['left_track_forward_current'])
        self.control_vars['left_track_forward_current'] = left_fwd_var
        left_fwd_scale = tk.Scale(
            left_fwd_frame,
            from_=0,
            to=700,
            orient=tk.HORIZONTAL,
            variable=left_fwd_var,
            bg=panel_bg,
            fg=fg_color,
            troughcolor='#2a3f5a',
            activebackground=accent_color,
            length=130,
            command=lambda v: self.update_control('left_track_forward_current', float(v))
        )
        left_fwd_scale.pack(side=tk.LEFT, fill=tk.NONE, expand=False)
        
        left_fwd_label = tk.Label(
            left_fwd_frame,
            text=f'{self.controls["left_track_forward_current"]:.0f}mA',
            bg=panel_bg,
            fg=accent_color,
            font=('Arial', 10, 'bold'),
            width=7
        )
        left_fwd_label.pack(side=tk.LEFT, padx=3)
        self.label_widgets['left_track_forward_current'] = left_fwd_label  # 存储标签引用
        
        # 显示实际发送的电流值（从CAN解析）
        left_fwd_actual_label = tk.Label(
            left_fwd_frame,
            text='[CAN:0mA]',
            bg=panel_bg,
            fg=fg_color,
            font=('Arial', 11),
            width=9
        )
        left_fwd_actual_label.pack(side=tk.LEFT, padx=2)
        self.actual_label_widgets['left_forward'] = left_fwd_actual_label  # 存储实际电流标签引用
        
        def update_left_fwd_label(v):
            left_fwd_label.config(text=f'{float(v):.0f}mA')
            self.update_control('left_track_forward_current', float(v))
        
        left_fwd_scale.config(command=update_left_fwd_label)
        
        # 左履带后退
        left_bwd_frame = tk.Frame(walking_frame, bg=panel_bg)
        left_bwd_frame.pack(fill=tk.X, pady=3)
        
        # 获取对应的线号
        left_bwd_wire = self.valve_wire_map.get('left_track_backward_current', '')
        tk.Label(
            left_bwd_frame,
            text=f'左履带后退 ({left_bwd_wire}线):',
            bg=panel_bg,
            fg=fg_color,
            font=('Arial', 10),
            width=18
        ).pack(side=tk.LEFT)
        
        # 键盘快捷键提示
        tk.Label(
            left_bwd_frame,
            text='[G/g]',
            bg=panel_bg,
            fg=accent_color,
            font=('Arial', 8),
            width=4
        ).pack(side=tk.LEFT, padx=(0, 3))
        
        left_bwd_var = tk.DoubleVar(value=self.controls['left_track_backward_current'])
        self.control_vars['left_track_backward_current'] = left_bwd_var
        left_bwd_scale = tk.Scale(
            left_bwd_frame,
            from_=0,
            to=700,
            orient=tk.HORIZONTAL,
            variable=left_bwd_var,
            bg=panel_bg,
            fg=fg_color,
            troughcolor='#2a3f5a',
            activebackground=danger_color,
            length=130,
            command=lambda v: self.update_control('left_track_backward_current', float(v))
        )
        left_bwd_scale.pack(side=tk.LEFT, fill=tk.NONE, expand=False)
        
        left_bwd_label = tk.Label(
            left_bwd_frame,
            text=f'{self.controls["left_track_backward_current"]:.0f}mA',
            bg=panel_bg,
            fg=danger_color,
            font=('Arial', 10, 'bold'),
            width=7
        )
        left_bwd_label.pack(side=tk.LEFT, padx=3)
        self.label_widgets['left_track_backward_current'] = left_bwd_label  # 存储标签引用
        
        # 显示实际发送的电流值（从CAN解析）
        left_bwd_actual_label = tk.Label(
            left_bwd_frame,
            text='[CAN:0mA]',
            bg=panel_bg,
            fg=fg_color,
            font=('Arial', 11),
            width=9
        )
        left_bwd_actual_label.pack(side=tk.LEFT, padx=2)
        self.actual_label_widgets['left_backward'] = left_bwd_actual_label  # 存储实际电流标签引用
        
        def update_left_bwd_label(v):
            left_bwd_label.config(text=f'{float(v):.0f}mA')
            self.update_control('left_track_backward_current', float(v))
        
        left_bwd_scale.config(command=update_left_bwd_label)
        
        # 右履带前进
        right_fwd_frame = tk.Frame(walking_frame, bg=panel_bg)
        right_fwd_frame.pack(fill=tk.X, pady=3)
        
        # 获取对应的线号
        right_fwd_wire = self.valve_wire_map.get('right_track_forward_current', '')
        tk.Label(
            right_fwd_frame,
            text=f'右履带前进 ({right_fwd_wire}线):',
            bg=panel_bg,
            fg=fg_color,
            font=('Arial', 10),
            width=18
        ).pack(side=tk.LEFT)
        
        # 键盘快捷键提示
        tk.Label(
            right_fwd_frame,
            text='[Y/y]',
            bg=panel_bg,
            fg=accent_color,
            font=('Arial', 8),
            width=4
        ).pack(side=tk.LEFT, padx=(0, 3))
        
        right_fwd_var = tk.DoubleVar(value=self.controls['right_track_forward_current'])
        self.control_vars['right_track_forward_current'] = right_fwd_var
        right_fwd_scale = tk.Scale(
            right_fwd_frame,
            from_=0,
            to=700,
            orient=tk.HORIZONTAL,
            variable=right_fwd_var,
            bg=panel_bg,
            fg=fg_color,
            troughcolor='#2a3f5a',
            activebackground=accent_color,
            length=130,
            command=lambda v: self.update_control('right_track_forward_current', float(v))
        )
        right_fwd_scale.pack(side=tk.LEFT, fill=tk.NONE, expand=False)
        
        right_fwd_label = tk.Label(
            right_fwd_frame,
            text=f'{self.controls["right_track_forward_current"]:.0f}mA',
            bg=panel_bg,
            fg=accent_color,
            font=('Arial', 10, 'bold'),
            width=7
        )
        right_fwd_label.pack(side=tk.LEFT, padx=3)
        self.label_widgets['right_track_forward_current'] = right_fwd_label  # 存储标签引用
        
        # 显示实际发送的电流值（从CAN解析）
        right_fwd_actual_label = tk.Label(
            right_fwd_frame,
            text='[CAN:0mA]',
            bg=panel_bg,
            fg=fg_color,
            font=('Arial', 11),
            width=9
        )
        right_fwd_actual_label.pack(side=tk.LEFT, padx=2)
        self.actual_label_widgets['right_forward'] = right_fwd_actual_label  # 存储实际电流标签引用
        
        def update_right_fwd_label(v):
            right_fwd_label.config(text=f'{float(v):.0f}mA')
            self.update_control('right_track_forward_current', float(v))
        
        right_fwd_scale.config(command=update_right_fwd_label)
        
        # 右履带后退
        right_bwd_frame = tk.Frame(walking_frame, bg=panel_bg)
        right_bwd_frame.pack(fill=tk.X, pady=3)
        
        # 获取对应的线号
        right_bwd_wire = self.valve_wire_map.get('right_track_backward_current', '')
        tk.Label(
            right_bwd_frame,
            text=f'右履带后退 ({right_bwd_wire}线):',
            bg=panel_bg,
            fg=fg_color,
            font=('Arial', 10),
            width=18
        ).pack(side=tk.LEFT)
        
        # 键盘快捷键提示
        tk.Label(
            right_bwd_frame,
            text='[H/h]',
            bg=panel_bg,
            fg=accent_color,
            font=('Arial', 8),
            width=4
        ).pack(side=tk.LEFT, padx=(0, 3))
        
        right_bwd_var = tk.DoubleVar(value=self.controls['right_track_backward_current'])
        self.control_vars['right_track_backward_current'] = right_bwd_var
        right_bwd_scale = tk.Scale(
            right_bwd_frame,
            from_=0,
            to=700,
            orient=tk.HORIZONTAL,
            variable=right_bwd_var,
            bg=panel_bg,
            fg=fg_color,
            troughcolor='#2a3f5a',
            activebackground=danger_color,
            length=130,
            command=lambda v: self.update_control('right_track_backward_current', float(v))
        )
        right_bwd_scale.pack(side=tk.LEFT, fill=tk.NONE, expand=False)
        
        right_bwd_label = tk.Label(
            right_bwd_frame,
            text=f'{self.controls["right_track_backward_current"]:.0f}mA',
            bg=panel_bg,
            fg=danger_color,
            font=('Arial', 10, 'bold'),
            width=7
        )
        right_bwd_label.pack(side=tk.LEFT, padx=3)
        self.label_widgets['right_track_backward_current'] = right_bwd_label  # 存储标签引用
        
        # 显示实际发送的电流值（从CAN解析）
        right_bwd_actual_label = tk.Label(
            right_bwd_frame,
            text='[CAN:0mA]',
            bg=panel_bg,
            fg=fg_color,
            font=('Arial', 11),
            width=9
        )
        right_bwd_actual_label.pack(side=tk.LEFT, padx=2)
        self.actual_label_widgets['right_backward'] = right_bwd_actual_label  # 存储实际电流标签引用
        
        def update_right_bwd_label(v):
            right_bwd_label.config(text=f'{float(v):.0f}mA')
            self.update_control('right_track_backward_current', float(v))
        
        right_bwd_scale.config(command=update_right_bwd_label)
        
        # 刹车滑块
        brake_frame = tk.Frame(walking_frame, bg=panel_bg)
        brake_frame.pack(fill=tk.X, pady=5)
        
        tk.Label(
            brake_frame,
            text='刹车 (0~100%):',
            bg=panel_bg,
            fg=fg_color,
            font=('Arial', 10),
            width=10
        ).pack(side=tk.LEFT)
        
        # 键盘快捷键提示
        tk.Label(
            brake_frame,
            text='[B/b]',
            bg=panel_bg,
            fg=accent_color,
            font=('Arial', 8),
            width=4
        ).pack(side=tk.LEFT, padx=(0, 3))
        
        brake_var = tk.DoubleVar(value=self.controls['brake'])
        self.control_vars['brake'] = brake_var
        brake_scale = tk.Scale(
            brake_frame,
            from_=0,
            to=100,
            orient=tk.HORIZONTAL,
            variable=brake_var,
            bg=panel_bg,
            fg=fg_color,
            troughcolor='#2a3f5a',
            activebackground=danger_color,
            length=130,
            command=lambda v: self.update_control('brake', float(v))
        )
        brake_scale.pack(side=tk.LEFT, fill=tk.NONE, expand=False)
        
        brake_label = tk.Label(
            brake_frame,
            text=f'{self.controls["brake"]:.1f}%',
            bg=panel_bg,
            fg=danger_color,
            font=('Arial', 10, 'bold'),
            width=8
        )
        brake_label.pack(side=tk.LEFT, padx=5)
        self.label_widgets['brake'] = brake_label  # 存储标签引用
        
        def update_brake_label(v):
            brake_label.config(text=f'{float(v):.1f}%')
            self.update_control('brake', float(v))
        
        brake_scale.config(command=update_brake_label)
    
    def create_middle_panel(self, parent, bg_color, fg_color, accent_color, danger_color, panel_bg):
        """创建中间面板（安全控制和开关量控制）"""
        # 标题
        title = tk.Label(
            parent,
            text='安全与开关控制',
            font=self.get_scaled_font('Arial', 16, 'bold'),
            bg=panel_bg,
            fg=fg_color
        )
        title.pack(pady=10)
        self.widget_fonts[title] = ('Arial', 16, 'bold')
        
        # 创建滚动区域
        canvas = tk.Canvas(parent, bg=panel_bg, highlightthickness=0)
        scrollbar = ttk.Scrollbar(parent, orient="vertical", command=canvas.yview)
        scrollable_frame = tk.Frame(canvas, bg=panel_bg)
        
        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        
        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        canvas.pack(side="left", fill="both", expand=True, padx=10, pady=10)
        scrollbar.pack(side="right", fill="y")
        
        # 安全控制
        safety_frame = tk.LabelFrame(
            scrollable_frame,
            text='安全控制',
            font=self.get_scaled_font('Arial', 12, 'bold'),
            bg=panel_bg,
            fg=danger_color,
            padx=10,
            pady=10
        )
        safety_frame.pack(fill=tk.X, padx=10, pady=5)
        self.widget_fonts[safety_frame] = ('Arial', 12, 'bold')
        
        # 急停
        estop_var = tk.BooleanVar(value=self.controls['estop'])
        self.control_vars['estop'] = estop_var
        estop_check = tk.Checkbutton(
            safety_frame,
            text='紧急停止',
            variable=estop_var,
            bg=panel_bg,
            fg=danger_color,
            selectcolor=panel_bg,
            activebackground=panel_bg,
            activeforeground=danger_color,
            font=('Arial', 10, 'bold'),
            command=lambda: self.update_control('estop', estop_var.get())
        )
        estop_check.pack(anchor=tk.W, pady=5)
        
        # 驻车制动
        parking_var = tk.BooleanVar(value=self.controls['parking_brake'])
        self.control_vars['parking_brake'] = parking_var
        parking_check = tk.Checkbutton(
            safety_frame,
            text='驻车制动',
            variable=parking_var,
            bg=panel_bg,
            fg=fg_color,
            selectcolor=panel_bg,
            activebackground=panel_bg,
            activeforeground=accent_color,
            font=('Arial', 10),
            command=lambda: self.update_control('parking_brake', parking_var.get())
        )
        parking_check.pack(anchor=tk.W, pady=5)

        # 手/自动切换（远程申请进入遥控）
        request_remote_var = tk.BooleanVar(value=self.controls.get('request_remote_control', False))
        self.control_vars['request_remote_control'] = request_remote_var
        request_remote_check = tk.Checkbutton(
            safety_frame,
            text='申请进入遥控(手/自动)',
            variable=request_remote_var,
            bg=panel_bg,
            fg=accent_color,
            selectcolor=panel_bg,
            activebackground=panel_bg,
            activeforeground=accent_color,
            font=('Arial', 10, 'bold'),
            command=lambda: self.update_control('request_remote_control', request_remote_var.get())
        )
        request_remote_check.pack(anchor=tk.W, pady=5)
        
        # 分隔线
        separator = tk.Frame(safety_frame, bg='#333333', height=1)
        separator.pack(fill=tk.X, padx=5, pady=10)
        
        # CAN 状态显示区域
        status_label_frame = tk.Frame(safety_frame, bg=panel_bg)
        status_label_frame.pack(fill=tk.X, pady=5)
        
        tk.Label(
            status_label_frame,
            text='CAN 状态 (0x18210005):',
            bg=panel_bg,
            fg=fg_color,
            font=('Arial', 10, 'bold')
        ).pack(anchor=tk.W, pady=(0, 5))
        
        # 远程申请进入遥控状态显示
        remote_status_frame = tk.Frame(status_label_frame, bg=panel_bg)
        remote_status_frame.pack(fill=tk.X, pady=2)
        tk.Label(
            remote_status_frame,
            text='远程申请进入遥控:',
            bg=panel_bg,
            fg=fg_color,
            font=('Arial', 9),
            width=18,
            anchor=tk.W
        ).pack(side=tk.LEFT)
        self.remote_control_status_label = tk.Label(
            remote_status_frame,
            text='否',
            bg=panel_bg,
            fg='#888888',
            font=('Arial', 9, 'bold'),
            width=8,
            anchor=tk.W
        )
        self.remote_control_status_label.pack(side=tk.LEFT)
        
        # 远程申请进入急停状态显示
        estop_status_frame = tk.Frame(status_label_frame, bg=panel_bg)
        estop_status_frame.pack(fill=tk.X, pady=2)
        tk.Label(
            estop_status_frame,
            text='远程申请进入急停:',
            bg=panel_bg,
            fg=fg_color,
            font=('Arial', 9),
            width=18,
            anchor=tk.W
        ).pack(side=tk.LEFT)
        self.emergency_stop_status_label = tk.Label(
            estop_status_frame,
            text='否',
            bg=panel_bg,
            fg='#888888',
            font=('Arial', 9, 'bold'),
            width=8,
            anchor=tk.W
        )
        self.emergency_stop_status_label.pack(side=tk.LEFT)

        # 常用功能状态显示（从 0x18210005 解析）
        func_status_sep = tk.Frame(status_label_frame, bg='#333333', height=1)
        func_status_sep.pack(fill=tk.X, padx=0, pady=8)

        tk.Label(
            status_label_frame,
            text='功能状态 (0x18210005):',
            bg=panel_bg,
            fg=fg_color,
            font=('Arial', 10, 'bold')
        ).pack(anchor=tk.W, pady=(0, 5))

        # 车梯状态
        ladder_status_frame = tk.Frame(status_label_frame, bg=panel_bg)
        ladder_status_frame.pack(fill=tk.X, pady=2)
        tk.Label(
            ladder_status_frame,
            text='车梯:',
            bg=panel_bg,
            fg=fg_color,
            font=('Arial', 9),
            width=8,
            anchor=tk.W
        ).pack(side=tk.LEFT)
        self.ladder_status_label = tk.Label(
            ladder_status_frame,
            text='无动作',
            bg=panel_bg,
            fg='#888888',
            font=('Arial', 9, 'bold'),
            width=40,
            anchor=tk.W
        )
        self.ladder_status_label.pack(side=tk.LEFT)

        # 斗开关状态
        bucket_status_frame = tk.Frame(status_label_frame, bg=panel_bg)
        bucket_status_frame.pack(fill=tk.X, pady=2)
        tk.Label(
            bucket_status_frame,
            text='斗:',
            bg=panel_bg,
            fg=fg_color,
            font=('Arial', 9),
            width=8,
            anchor=tk.W
        ).pack(side=tk.LEFT)
        self.bucket_status_label = tk.Label(
            bucket_status_frame,
            text='无动作',
            bg=panel_bg,
            fg='#888888',
            font=('Arial', 9, 'bold'),
            width=40,
            anchor=tk.W
        )
        self.bucket_status_label.pack(side=tk.LEFT)

        # 喇叭状态
        horn_status_frame = tk.Frame(status_label_frame, bg=panel_bg)
        horn_status_frame.pack(fill=tk.X, pady=2)
        tk.Label(
            horn_status_frame,
            text='喇叭:',
            bg=panel_bg,
            fg=fg_color,
            font=('Arial', 9),
            width=8,
            anchor=tk.W
        ).pack(side=tk.LEFT)
        self.horn_status_label = tk.Label(
            horn_status_frame,
            text='否',
            bg=panel_bg,
            fg='#888888',
            font=('Arial', 9, 'bold'),
            width=40,
            anchor=tk.W
        )
        self.horn_status_label.pack(side=tk.LEFT)

        # 先导安全锁定状态
        hydraulic_status_frame = tk.Frame(status_label_frame, bg=panel_bg)
        hydraulic_status_frame.pack(fill=tk.X, pady=2)
        tk.Label(
            hydraulic_status_frame,
            text='先导锁:',
            bg=panel_bg,
            fg=fg_color,
            font=('Arial', 9),
            width=8,
            anchor=tk.W
        ).pack(side=tk.LEFT)
        self.hydraulic_status_label = tk.Label(
            hydraulic_status_frame,
            text='关',
            bg=panel_bg,
            fg='#888888',
            font=('Arial', 9, 'bold'),
            width=40,
            anchor=tk.W
        )
        self.hydraulic_status_label.pack(side=tk.LEFT)

        # 灯光状态
        light_status_frame = tk.Frame(status_label_frame, bg=panel_bg)
        light_status_frame.pack(fill=tk.X, pady=2)
        tk.Label(
            light_status_frame,
            text='灯光:',
            bg=panel_bg,
            fg=fg_color,
            font=('Arial', 9),
            width=8,
            anchor=tk.W
        ).pack(side=tk.LEFT)
        self.light_status_label = tk.Label(
            light_status_frame,
            text='全关',
            bg=panel_bg,
            fg='#888888',
            font=('Arial', 9, 'bold'),
            width=40,
            anchor=tk.W
        )
        self.light_status_label.pack(side=tk.LEFT)

        # 上电状态
        power_enable_status_frame = tk.Frame(status_label_frame, bg=panel_bg)
        power_enable_status_frame.pack(fill=tk.X, pady=2)
        tk.Label(
            power_enable_status_frame,
            text='上电:',
            bg=panel_bg,
            fg=fg_color,
            font=('Arial', 9),
            width=8,
            anchor=tk.W
        ).pack(side=tk.LEFT)
        self.power_enable_status_label = tk.Label(
            power_enable_status_frame,
            text='否',
            bg=panel_bg,
            fg='#888888',
            font=('Arial', 9, 'bold'),
            width=40,
            anchor=tk.W
        )
        self.power_enable_status_label.pack(side=tk.LEFT)

        # 发动机启动状态
        start_up_status_frame = tk.Frame(status_label_frame, bg=panel_bg)
        start_up_status_frame.pack(fill=tk.X, pady=2)
        tk.Label(
            start_up_status_frame,
            text='发动机:',
            bg=panel_bg,
            fg=fg_color,
            font=('Arial', 9),
            width=8,
            anchor=tk.W
        ).pack(side=tk.LEFT)
        self.start_up_status_label = tk.Label(
            start_up_status_frame,
            text='否',
            bg=panel_bg,
            fg='#888888',
            font=('Arial', 9, 'bold'),
            width=40,
            anchor=tk.W
        )
        self.start_up_status_label.pack(side=tk.LEFT)

        # 发动机油门档位状态（1~7，经 0x18FD0821 报文控制）
        engine_gear_status_frame = tk.Frame(status_label_frame, bg=panel_bg)
        engine_gear_status_frame.pack(fill=tk.X, pady=2)
        tk.Label(
            engine_gear_status_frame,
            text='油门档位:',
            bg=panel_bg,
            fg=fg_color,
            font=('Arial', 9),
            width=8,
            anchor=tk.W
        ).pack(side=tk.LEFT)
        self.engine_gear_status_label = tk.Label(
            engine_gear_status_frame,
            text='0',
            bg=panel_bg,
            fg='#888888',
            font=('Arial', 9, 'bold'),
            width=40,
            anchor=tk.W
        )
        self.engine_gear_status_label.pack(side=tk.LEFT)

        # 下发 VCU 的发动机转速（CAN 回显 0x18210005 byte5-6，由 cannode 注入 J1939 转速）
        engine_rpm_tx_frame = tk.Frame(status_label_frame, bg=panel_bg)
        engine_rpm_tx_frame.pack(fill=tk.X, pady=2)
        tk.Label(
            engine_rpm_tx_frame,
            text='下发转速:',
            bg=panel_bg,
            fg=fg_color,
            font=('Arial', 9),
            width=8,
            anchor=tk.W
        ).pack(side=tk.LEFT)
        self.engine_rpm_tx_status_label = tk.Label(
            engine_rpm_tx_frame,
            text='-- rpm',
            bg=panel_bg,
            fg='#888888',
            font=('Arial', 9, 'bold'),
            width=40,
            anchor=tk.W
        )
        self.engine_rpm_tx_status_label.pack(side=tk.LEFT)

        # 下发 VCU 的油门模拟电压（CAN 回显 0x18FD0821，对应 ControlCommand.pto_cmd）
        engine_throttle_tx_frame = tk.Frame(status_label_frame, bg=panel_bg)
        engine_throttle_tx_frame.pack(fill=tk.X, pady=2)
        tk.Label(
            engine_throttle_tx_frame,
            text='油门下发:',
            bg=panel_bg,
            fg=fg_color,
            font=('Arial', 9),
            width=8,
            anchor=tk.W
        ).pack(side=tk.LEFT)
        self.engine_throttle_tx_status_label = tk.Label(
            engine_throttle_tx_frame,
            text='--',
            bg=panel_bg,
            fg='#888888',
            font=('Arial', 9, 'bold'),
            width=40,
            anchor=tk.W
        )
        self.engine_throttle_tx_status_label.pack(side=tk.LEFT)
        
        # ACC 状态
        acc_status_frame = tk.Frame(status_label_frame, bg=panel_bg)
        acc_status_frame.pack(fill=tk.X, pady=2)
        tk.Label(
            acc_status_frame,
            text='ACC:',
            bg=panel_bg,
            fg=fg_color,
            font=('Arial', 9),
            width=8,
            anchor=tk.W
        ).pack(side=tk.LEFT)
        self.acc_status_label = tk.Label(
            acc_status_frame,
            text='否',
            bg=panel_bg,
            fg='#888888',
            font=('Arial', 9, 'bold'),
            width=40,
            anchor=tk.W
        )
        self.acc_status_label.pack(side=tk.LEFT)
        
        # 控制发送开关按钮
        self.send_control_button = tk.Button(
            safety_frame,
            text='✓ 允许发送控制数据',
            font=('Arial', 11, 'bold'),
            bg='#4CAF50',  # 绿色表示启用
            fg='#ffffff',
            activebackground='#45a049',
            activeforeground='#ffffff',
            relief=tk.RAISED,
            bd=2,
            padx=15,
            pady=8,
            command=self.toggle_send_control
        )
        self.send_control_button.pack(anchor=tk.W, pady=10, padx=5)
        
        # 开关量控制
        switch_frame = tk.LabelFrame(
            scrollable_frame,
            text='开关量控制 (26个开关)',
            font=self.get_scaled_font('Arial', 12, 'bold'),
            bg=panel_bg,
            fg=fg_color,
            padx=10,
            pady=10
        )
        switch_frame.pack(fill=tk.X, padx=10, pady=5)
        self.widget_fonts[switch_frame] = ('Arial', 12, 'bold')

        # 常用功能控制（映射到 0x18210005 的部分开关位）
        func_ctrl_frame = tk.LabelFrame(
            switch_frame,
            text='常用功能控制(映射到0x18210005)',
            font=self.get_scaled_font('Arial', 10, 'bold'),
            bg=panel_bg,
            fg=accent_color,
            padx=10,
            pady=8
        )
        func_ctrl_frame.pack(fill=tk.X, padx=5, pady=(0, 10))

        # 车梯控制：0/1/2 = 无动作/收起/放下
        ladder_row = tk.Frame(func_ctrl_frame, bg=panel_bg)
        ladder_row.pack(fill=tk.X, pady=2)
        tk.Label(ladder_row, text='车梯:', bg=panel_bg, fg=fg_color, font=('Arial', 9), width=6, anchor=tk.W).pack(side=tk.LEFT)
        self.ladder_switch_var = tk.IntVar(value=self.ladder_switch_mode)
        tk.Radiobutton(ladder_row, text='无', value=0, variable=self.ladder_switch_var,
                       bg=panel_bg, fg=fg_color, selectcolor=panel_bg,
                       activebackground=panel_bg, activeforeground=accent_color,
                       command=lambda: self.set_ladder_switch_mode(self.ladder_switch_var.get())
                       ).pack(side=tk.LEFT, padx=4)
        tk.Radiobutton(ladder_row, text='收起', value=1, variable=self.ladder_switch_var,
                       bg=panel_bg, fg=fg_color, selectcolor=panel_bg,
                       activebackground=panel_bg, activeforeground=accent_color,
                       command=lambda: self.set_ladder_switch_mode(self.ladder_switch_var.get())
                       ).pack(side=tk.LEFT, padx=4)
        tk.Radiobutton(ladder_row, text='放下', value=2, variable=self.ladder_switch_var,
                       bg=panel_bg, fg=fg_color, selectcolor=panel_bg,
                       activebackground=panel_bg, activeforeground=accent_color,
                       command=lambda: self.set_ladder_switch_mode(self.ladder_switch_var.get())
                       ).pack(side=tk.LEFT, padx=4)

        # 斗开关控制：0/1/2 = 无动作/开斗/关斗
        bucket_row = tk.Frame(func_ctrl_frame, bg=panel_bg)
        bucket_row.pack(fill=tk.X, pady=2)
        tk.Label(bucket_row, text='斗:', bg=panel_bg, fg=fg_color, font=('Arial', 9), width=6, anchor=tk.W).pack(side=tk.LEFT)
        self.bucket_switch_var = tk.IntVar(value=self.bucket_switch_mode)
        tk.Radiobutton(bucket_row, text='无', value=0, variable=self.bucket_switch_var,
                       bg=panel_bg, fg=fg_color, selectcolor=panel_bg,
                       activebackground=panel_bg, activeforeground=accent_color,
                       command=lambda: self.set_bucket_switch_mode(self.bucket_switch_var.get())
                       ).pack(side=tk.LEFT, padx=4)
        tk.Radiobutton(bucket_row, text='开斗', value=1, variable=self.bucket_switch_var,
                       bg=panel_bg, fg=fg_color, selectcolor=panel_bg,
                       activebackground=panel_bg, activeforeground=accent_color,
                       command=lambda: self.set_bucket_switch_mode(self.bucket_switch_var.get())
                       ).pack(side=tk.LEFT, padx=4)
        tk.Radiobutton(bucket_row, text='关斗', value=2, variable=self.bucket_switch_var,
                       bg=panel_bg, fg=fg_color, selectcolor=panel_bg,
                       activebackground=panel_bg, activeforeground=accent_color,
                       command=lambda: self.set_bucket_switch_mode(self.bucket_switch_var.get())
                       ).pack(side=tk.LEFT, padx=4)

        # 先导安全锁定：开/关
        hydraulic_row = tk.Frame(func_ctrl_frame, bg=panel_bg)
        hydraulic_row.pack(fill=tk.X, pady=2)
        tk.Label(hydraulic_row, text='先导锁:', bg=panel_bg, fg=fg_color, font=('Arial', 9), width=6, anchor=tk.W).pack(side=tk.LEFT)
        self.hydraulic_lock_var = tk.BooleanVar(value=self.hydraulic_lock_active)
        hydraulic_check = tk.Checkbutton(
            hydraulic_row,
            text='液压锁工作',
            variable=self.hydraulic_lock_var,
            bg=panel_bg,
            fg=fg_color,
            selectcolor=panel_bg,
            activebackground=panel_bg,
            activeforeground=accent_color,
            font=('Arial', 9),
            command=lambda: self.set_hydraulic_lock_active(self.hydraulic_lock_var.get())
        )
        hydraulic_check.pack(side=tk.LEFT, padx=4)

        # 灯光控制：0/1/2/3 = 关灯/灯1/灯2/灯1+2
        light_row = tk.Frame(func_ctrl_frame, bg=panel_bg)
        light_row.pack(fill=tk.X, pady=2)
        tk.Label(light_row, text='灯光:', bg=panel_bg, fg=fg_color, font=('Arial', 9), width=6, anchor=tk.W).pack(side=tk.LEFT)
        self.light_code_var = tk.IntVar(value=self.light_code_mode)
        tk.Radiobutton(light_row, text='关', value=0, variable=self.light_code_var,
                       bg=panel_bg, fg=fg_color, selectcolor=panel_bg,
                       activebackground=panel_bg, activeforeground=accent_color,
                       command=lambda: self.set_light_code_mode(self.light_code_var.get())
                       ).pack(side=tk.LEFT, padx=2)
        tk.Radiobutton(light_row, text='灯1', value=1, variable=self.light_code_var,
                       bg=panel_bg, fg=fg_color, selectcolor=panel_bg,
                       activebackground=panel_bg, activeforeground=accent_color,
                       command=lambda: self.set_light_code_mode(self.light_code_var.get())
                       ).pack(side=tk.LEFT, padx=2)
        tk.Radiobutton(light_row, text='灯2', value=2, variable=self.light_code_var,
                       bg=panel_bg, fg=fg_color, selectcolor=panel_bg,
                       activebackground=panel_bg, activeforeground=accent_color,
                       command=lambda: self.set_light_code_mode(self.light_code_var.get())
                       ).pack(side=tk.LEFT, padx=2)
        tk.Radiobutton(light_row, text='灯1+2', value=3, variable=self.light_code_var,
                       bg=panel_bg, fg=fg_color, selectcolor=panel_bg,
                       activebackground=panel_bg, activeforeground=accent_color,
                       command=lambda: self.set_light_code_mode(self.light_code_var.get())
                       ).pack(side=tk.LEFT, padx=2)

        # 喇叭：按下有效（按钮按下/松开）
        horn_row = tk.Frame(func_ctrl_frame, bg=panel_bg)
        horn_row.pack(fill=tk.X, pady=2)
        tk.Label(horn_row, text='喇叭:', bg=panel_bg, fg=fg_color, font=('Arial', 9), width=6, anchor=tk.W).pack(side=tk.LEFT)
        horn_btn = tk.Button(
            horn_row,
            text='按住鸣笛',
            bg='#2a3f5a',
            fg=fg_color,
            activebackground=accent_color,
            activeforeground='#000000',
            font=('Arial', 9, 'bold'),
            relief=tk.RAISED,
            bd=2,
            padx=10,
            pady=2
        )
        horn_btn.pack(side=tk.LEFT, padx=4)
        horn_btn.bind('<ButtonPress-1>', lambda _e: self.set_horn_active(True))
        horn_btn.bind('<ButtonRelease-1>', lambda _e: self.set_horn_active(False))

        # 上电控制（复选框）
        power_enable_row = tk.Frame(func_ctrl_frame, bg=panel_bg)
        power_enable_row.pack(fill=tk.X, pady=2)
        self.power_enable_var = tk.BooleanVar(value=False)
        power_enable_check = tk.Checkbutton(
            power_enable_row,
            text='上电',
            variable=self.power_enable_var,
            bg=panel_bg,
            fg=fg_color,
            activebackground=panel_bg,
            activeforeground=accent_color,
            selectcolor=panel_bg,
            font=('Arial', 9),
            command=lambda: self.set_power_enable_active(self.power_enable_var.get())
        )
        power_enable_check.pack(side=tk.LEFT, padx=4)

        # 发动机启动控制（复选框）
        start_up_row = tk.Frame(func_ctrl_frame, bg=panel_bg)
        start_up_row.pack(fill=tk.X, pady=2)
        self.start_up_var = tk.BooleanVar(value=False)
        start_up_check = tk.Checkbutton(
            start_up_row,
            text='发动机启动',
            variable=self.start_up_var,
            bg=panel_bg,
            fg=fg_color,
            activebackground=panel_bg,
            activeforeground=accent_color,
            selectcolor=panel_bg,
            font=('Arial', 9),
            command=lambda: self.set_start_up_active(self.start_up_var.get())
        )
        start_up_check.pack(side=tk.LEFT, padx=4)

        # 发动机油门档位（1~7，对应 0x18FD0821 电压控制）
        engine_gear_row = tk.Frame(func_ctrl_frame, bg=panel_bg)
        engine_gear_row.pack(fill=tk.X, pady=2)
        tk.Label(
            engine_gear_row,
            text='油门档:',
            bg=panel_bg,
            fg=fg_color,
            font=('Arial', 9),
            width=6,
            anchor=tk.W
        ).pack(side=tk.LEFT)
        self.engine_gear_var = tk.IntVar(value=self.engine_gear_level)
        # 使用 0~7 的单选按钮（0 表示关闭）
        for val in range(0, 8):
            txt = str(val)
            tk.Radiobutton(
                engine_gear_row,
                text=txt,
                value=val,
                variable=self.engine_gear_var,
                bg=panel_bg,
                fg=fg_color,
                selectcolor=panel_bg,
                activebackground=panel_bg,
                activeforeground=accent_color,
                font=('Arial', 9),
                command=lambda v=val: self.set_engine_gear_level(v)
            ).pack(side=tk.LEFT, padx=2)

        # ACC 控制（复选框）
        acc_row = tk.Frame(func_ctrl_frame, bg=panel_bg)
        acc_row.pack(fill=tk.X, pady=2)
        self.acc_var = tk.BooleanVar(value=False)
        acc_check = tk.Checkbutton(
            acc_row,
            text='ACC',
            variable=self.acc_var,
            bg=panel_bg,
            fg=fg_color,
            activebackground=panel_bg,
            activeforeground=accent_color,
            selectcolor=panel_bg,
            font=('Arial', 9),
            command=lambda: self.set_acc_active(self.acc_var.get())
        )
        acc_check.pack(side=tk.LEFT, padx=4)
        
        # 创建开关量网格布局
        switch_grid = tk.Frame(switch_frame, bg=panel_bg)
        switch_grid.pack(fill=tk.BOTH, expand=True)
        
        # 开关量名称列表（按CSV顺序）
        switch_names = [
            'switch_21', 'switch_22', 'switch_23', 'switch_24',
            'switch_25', 'switch_26', 'switch_27', 'switch_28',
            'switch_29', 'switch_30', 'switch_31', 'switch_32',
            'switch_17', 'switch_18', 'switch_19', 'switch_20',
            'switch_38', 'switch_39', 'switch_40', 'switch_42',
            'switch_43', 'switch_44', 'switch_45', 'switch_46',
            'switch_37', 'switch_41',
        ]
        
        # 创建开关量复选框（每行3个，因为中间面板较窄）
        if not hasattr(self, 'switch_vars'):
            self.switch_vars = {}
        for i, switch_name in enumerate(switch_names):
            row = i // 3
            col = i % 3
            
            var = tk.BooleanVar(value=self.switches[switch_name])
            self.switch_vars[switch_name] = var
            
            # 获取对应的线号
            wire_num = self.switch_wire_map.get(switch_name, '')
            # 显示格式：SW21 (107#线)
            display_text = f"{switch_name.replace('switch_', 'SW')} ({wire_num}线)" if wire_num else switch_name.replace('switch_', 'SW')
            
            check = tk.Checkbutton(
                switch_grid,
                text=display_text,
                variable=var,
                bg=panel_bg,
                fg=fg_color,
                selectcolor=panel_bg,
                activebackground=panel_bg,
                activeforeground=accent_color,
                font=('Arial', 9),
                command=lambda s=switch_name, v=var: self.update_switch(s, v.get())
            )
            check.grid(row=row, column=col, padx=5, pady=3, sticky=tk.W)
        
        # 重置按钮（移到中间面板）
        reset_button = tk.Button(
            scrollable_frame,
            text='重置所有控制',
            font=('Arial', 12, 'bold'),
            bg=danger_color,
            fg='#ffffff',
            activebackground='#ff3366',
            activeforeground='#ffffff',
            relief=tk.RAISED,
            bd=3,
            padx=20,
            pady=10,
            command=self.reset_controls
        )
        reset_button.pack(pady=20)
        
        # 键盘快捷键提示
        help_frame = tk.LabelFrame(
            scrollable_frame,
            text='键盘快捷键',
            font=self.get_scaled_font('Arial', 10, 'bold'),
            bg=panel_bg,
            fg=accent_color,
            padx=10,
            pady=10
        )
        help_frame.pack(fill=tk.X, padx=10, pady=5)
        self.widget_fonts[help_frame] = ('Arial', 10, 'bold')
        
        help_label = tk.Label(
            help_frame,
            text=(
                "大写=增加, 小写=减少 (步长: 10mA)\n"
                "Q/q: 动臂抬升 +/-  |  A/a: 动臂下降 +/-\n"
                "W/w: 铲斗收斗 +/-  |  S/s: 铲斗翻斗 +/-\n"
                "E/e: 斗杆收回 +/-  |  D/d: 斗杆伸出 +/-\n"
                "R/r: 回转左 +/-    |  F/f: 回转右 +/-\n"
                "T/t: 左履带前进 +/- |  G/g: 左履带后退 +/-\n"
                "Y/y: 右履带前进 +/- |  H/h: 右履带后退 +/-\n"
                "B/b: 刹车 +/-"
            ),
            bg=panel_bg,
            fg=fg_color,
            font=self.get_scaled_font('Courier', 8),
            justify=tk.LEFT,
            anchor='w'
        )
        help_label.pack(anchor=tk.W, padx=5, pady=5)
        self.widget_fonts[help_label] = ('Courier', 8)
    
    def create_data_panel(self, parent, bg_color, fg_color, accent_color, panel_bg):
        """创建数据显示面板"""
        # 标题
        title = tk.Label(
            parent,
            text='CAN 报文',
            font=self.get_scaled_font('Arial', 16, 'bold'),
            bg=panel_bg,
            fg=fg_color
        )
        title.pack(pady=10)
        self.widget_fonts[title] = ('Arial', 16, 'bold')
        
        # 创建滚动文本框显示 CAN 报文
        can_text = tk.Text(
            parent,
            bg='#0a0e27',
            fg=fg_color,
            font=self.get_scaled_font('Courier', 9),
            wrap=tk.WORD,
            relief=tk.FLAT,
            padx=10,
            pady=10
        )
        can_text.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # 存储文本框引用
        self.can_text = can_text
        self.widget_fonts[can_text] = ('Courier', 9)
    
    def update_control(self, key, value):
        """更新控制值并发布"""
        # 只对电流类key做200~700mA限制，其他key不变
        current_keys = [
            'arm_up_current', 'arm_down_current', 'stick_retract_current', 'stick_extend_current',
            'bucket_close_current', 'bucket_dump_current', 'rotate_left_current', 'rotate_right_current',
            'left_track_forward_current', 'left_track_backward_current', 'right_track_forward_current', 'right_track_backward_current'
        ]
        if key in current_keys:
            if value != 0:
                value = max(200, min(700, float(value)))
            else:
                value = 0.0
        self.controls[key] = value
        # 同时更新对应的 GUI 控件变量（滑块）
        if key in self.control_vars:
            var = self.control_vars[key]
            if isinstance(var, tk.DoubleVar):
                var.set(float(value))
        # 同时更新对应的标签文本
        if key in self.label_widgets:
            label = self.label_widgets[key]
            if key == 'brake':
                label.config(text=f'{float(value):.1f}%')
            else:
                label.config(text=f'{float(value):.0f}mA')
        self.publish_command()
    
    def adjust_current(self, key, delta):
        """通过键盘调整电流值（增量控制）"""
        if key not in self.controls:
            return
        # 只对电流类key做200~700mA限制，允许0
        current_keys = [
            'arm_up_current', 'arm_down_current', 'stick_retract_current', 'stick_extend_current',
            'bucket_close_current', 'bucket_dump_current', 'rotate_left_current', 'rotate_right_current',
            'left_track_forward_current', 'left_track_backward_current', 'right_track_forward_current', 'right_track_backward_current'
        ]
        value = self.controls[key] + delta
        if key in current_keys:
            if value != 0:
                value = max(200, min(700, value))
            else:
                value = 0.0
        self.update_control(key, value)
        
        current_value = self.controls[key]
        new_value = max(0.0, min(700.0, current_value + delta))
        
        if abs(new_value - current_value) > 0.01:  # 只有变化超过0.01才更新
            self.update_control(key, new_value)
    
    def setup_keyboard_controls(self, root):
        """设置键盘快捷键控制"""
        # 键盘映射：按键 -> (控制键名, 增量)
        # 使用 keysym 来匹配，支持大小写
        keyboard_map = {
            # 动臂控制
            'q': ('arm_up_current', -self.keyboard_step),      # q: 动臂抬升减少
            'Q': ('arm_up_current', self.keyboard_step),       # Q: 动臂抬升增加
            'a': ('arm_down_current', -self.keyboard_step),    # a: 动臂下降减少
            'A': ('arm_down_current', self.keyboard_step),     # A: 动臂下降增加
            
            # 铲斗控制
            'w': ('bucket_close_current', -self.keyboard_step),  # w: 铲斗收斗减少
            'W': ('bucket_close_current', self.keyboard_step),   # W: 铲斗收斗增加
            's': ('bucket_dump_current', -self.keyboard_step),   # s: 铲斗翻斗减少
            'S': ('bucket_dump_current', self.keyboard_step),   # S: 铲斗翻斗增加
            
            # 斗杆控制
            'e': ('stick_retract_current', -self.keyboard_step),  # e: 斗杆收回减少
            'E': ('stick_retract_current', self.keyboard_step),  # E: 斗杆收回增加
            'd': ('stick_extend_current', -self.keyboard_step),   # d: 斗杆伸出减少
            'D': ('stick_extend_current', self.keyboard_step),   # D: 斗杆伸出增加
            
            # 回转控制
            'r': ('rotate_left_current', -self.keyboard_step),    # r: 回转左减少
            'R': ('rotate_left_current', self.keyboard_step),     # R: 回转左增加
            'f': ('rotate_right_current', -self.keyboard_step),  # f: 回转右减少
            'F': ('rotate_right_current', self.keyboard_step),   # F: 回转右增加
            
            # 左履带控制
            't': ('left_track_forward_current', -self.keyboard_step),   # t: 左履带前进减少
            'T': ('left_track_forward_current', self.keyboard_step),    # T: 左履带前进增加
            'g': ('left_track_backward_current', -self.keyboard_step),  # g: 左履带后退减少
            'G': ('left_track_backward_current', self.keyboard_step),   # G: 左履带后退增加
            
            # 右履带控制
            'y': ('right_track_forward_current', -self.keyboard_step),  # y: 右履带前进减少
            'Y': ('right_track_forward_current', self.keyboard_step),   # Y: 右履带前进增加
            'h': ('right_track_backward_current', -self.keyboard_step),  # h: 右履带后退减少
            'H': ('right_track_backward_current', self.keyboard_step),  # H: 右履带后退增加
            
            # 刹车控制
            'b': ('brake', -self.keyboard_step),   # b: 刹车减少
            'B': ('brake', self.keyboard_step),     # B: 刹车增加
        }
        
        def on_key_press(event):
            """处理键盘按键事件"""
            # 尝试多种方式获取按键
            key = None
            
            # 优先使用 event.char（对于可打印字符）
            if event.char and len(event.char) == 1 and event.char.isprintable():
                key = event.char
            # 如果 event.char 为空，尝试使用 event.keysym
            elif event.keysym and len(event.keysym) == 1:
                key = event.keysym
            # 如果 keysym 是字母，检查是否有 Shift 键
            elif event.keysym and event.keysym.isalpha() and len(event.keysym) == 1:
                # 检查是否按下了 Shift 键
                if event.state & 0x1:  # Shift 键被按下
                    key = event.keysym.upper()
                else:
                    key = event.keysym.lower()
            
            # 调试输出
            # if key:
            #     print(f"按键检测: char='{event.char}', keysym='{event.keysym}', state={event.state}, key='{key}'")
            
            if key and key in keyboard_map:
                control_key, delta = keyboard_map[key]
                try:
                    self.adjust_current(control_key, delta)
                    # print(f"✓ 键盘控制成功: {key} -> {control_key} {delta:+}mA (当前值: {self.controls[control_key]:.1f}mA)")
                except Exception as e:
                    # print(f"✗ 键盘控制错误: {e}")
                    pass
                return 'break'  # 阻止事件继续传播
            elif key:
                # print(f"未映射的按键: '{key}' (char='{event.char}', keysym='{event.keysym}')")
                pass
        
        # 绑定键盘事件（使用 bind_all 确保所有控件都能接收）
        root.bind_all('<KeyPress>', on_key_press)
        root.bind_all('<Key>', on_key_press)
        
        # 设置窗口属性，确保可以接收键盘事件
        root.focus_set()
        root.focus_force()  # 强制获取焦点
        
        # 确保窗口始终可以接收焦点
        def on_focus_in(event):
            root.focus_set()
        
        def on_click(event):
            root.focus_set()
            root.focus_force()
            return True
        
        root.bind('<FocusIn>', on_focus_in)
        root.bind('<Button-1>', on_click)
        
        # 为所有子控件绑定点击事件以获取焦点
        def bind_focus_to_children(widget):
            try:
                widget.bind('<Button-1>', lambda e: (root.focus_set(), root.focus_force()))
                for child in widget.winfo_children():
                    bind_focus_to_children(child)
            except:
                pass
        
        # 延迟绑定，确保所有控件都已创建
        root.after(200, lambda: bind_focus_to_children(root))
        
        # 添加提示信息
        # print("键盘控制已启用，请点击窗口后使用键盘快捷键")
    
    def toggle_send_control(self):
        """切换是否允许发送控制数据"""
        self.enable_send_control = not self.enable_send_control
        
        # 更新按钮显示
        if hasattr(self, 'send_control_button'):
            if self.enable_send_control:
                self.send_control_button.config(
                    text='✓ 允许发送控制数据',
                    bg='#4CAF50',  # 绿色表示启用
                    activebackground='#45a049'
                )
                self.get_logger().info('已启用控制数据发送')
            else:
                self.send_control_button.config(
                    text='✗ 禁止发送控制数据',
                    bg='#f44336',  # 红色表示禁用
                    activebackground='#da190b'
                )
                self.get_logger().info('已禁用控制数据发送')
    
    def update_switch(self, switch_name, value):
        """更新开关量并发布"""
        self.switches[switch_name] = value
        self.publish_command()

    def _set_switch_quiet(self, switch_name: str, value: bool):
        """设置开关量（不触发 publish_command），并同步 UI 复选框变量。"""
        self.switches[switch_name] = bool(value)
        if hasattr(self, 'switch_vars') and switch_name in self.switch_vars:
            try:
                self.switch_vars[switch_name].set(bool(value))
            except Exception:
                pass

    def set_ladder_switch_mode(self, mode: int):
        """
        ladder_switch: 0/1/2 = 无动作 / 车梯收起电磁阀 / 车梯放下电磁阀
        映射：switch_21(bit0) 车梯收起电磁阀, DOH_OUT17_125(125#线)
              switch_22(bit1) 车梯放下电磁阀, DOH_OUT18_129(129#线)
        根据 excavator-can-host2vcu_utf8.csv
        """
        try:
            mode_i = int(mode)
        except Exception:
            mode_i = 0
        if mode_i not in (0, 1, 2):
            mode_i = 0
        self.ladder_switch_mode = mode_i

        # 互斥：先清零再置位
        self._set_switch_quiet('switch_21', mode_i == 1)
        self._set_switch_quiet('switch_22', mode_i == 2)
        self.publish_command()

    def set_bucket_switch_mode(self, mode: int):
        """
        bucket_switch: 0/1/2 = 无动作 / 开斗电磁阀 / 关斗电磁阀
        映射：switch_30(bit9) 开斗油缸电磁阀, OUT_26_DOH(193#线)
              switch_31(bit10) 关斗油缸电磁阀, OUT_37_DOL(183#线)
        根据 excavator-can-host2vcu_utf8.csv
        """
        try:
            mode_i = int(mode)
        except Exception:
            mode_i = 0
        if mode_i not in (0, 1, 2):
            mode_i = 0
        self.bucket_switch_mode = mode_i

        # 互斥：先清零再置位
        self._set_switch_quiet('switch_30', mode_i == 1)
        self._set_switch_quiet('switch_31', mode_i == 2)
        self.publish_command()

    def set_horn_active(self, active: bool):
        """
        horn: false/true = 无动作 / 按喇叭
        映射：switch_19(bit 11) - 喇叭, OUT_28_DOH(241#线)
        根据 excavator-can-host2vcu_utf8.csv
        """
        self.horn_active = bool(active)
        self._set_switch_quiet('switch_19', self.horn_active)
        self.publish_command()

    def set_hydraulic_lock_active(self, active: bool):
        """
        hydraulic_lock: false/true = 液压锁不工作 / 液压锁工作
        映射：switch_24(bit3) 先导安全锁定电磁阀, OUT_24_DOH(103#线)
        根据 excavator-can-host2vcu_utf8.csv
        """
        self.hydraulic_lock_active = bool(active)
        self._set_switch_quiet('switch_24', self.hydraulic_lock_active)
        self.publish_command()

    def set_light_code_mode(self, mode: int):
        """
        light_code: 0/1/2/3 = 关灯 / 开灯1 / 开灯2 / 开灯1+2
        映射：switch_17(bit12) 灯光1, OUT_19_DOH(128#线)
              switch_18(bit13) 灯光2, OUT_29_DOH(242#线)
        根据 excavator-can-host2vcu_utf8.csv
        """
        try:
            mode_i = int(mode)
        except Exception:
            mode_i = 0
        if mode_i not in (0, 1, 2, 3):
            mode_i = 0
        self.light_code_mode = mode_i

        # 根据 light_code 设置灯光1/2 开关
        self._set_switch_quiet('switch_17', (mode_i & 0x1) != 0)
        self._set_switch_quiet('switch_18', (mode_i & 0x2) != 0)
        self.publish_command()

    def set_power_enable_active(self, active: bool):
        """
        power_enable: false/true = 不上电 / 上电
        映射：通过 switch_bits bit 26 传递，在 cannode 中映射到 switch_ext_bits bit 34
        根据 excavator-can-host2vcu_utf8.csv: bit 34: 上电 (1是有效)
        """
        self.power_enable_active = bool(active)
        self.publish_command()

    def set_start_up_active(self, active: bool):
        """
        start_up: false/true = 不启动 / 发动机启动
        映射：通过 switch_bits bit 27 传递，在 cannode 中映射到 switch_ext_bits bit 35
        根据 excavator-can-host2vcu_utf8.csv: bit 35: 发动机启动 (1是有效)
        """
        self.start_up_active = bool(active)
        self.publish_command()

    def set_engine_gear_level(self, level: int):
        """
        设置发动机油门档位（0~7）
        - 通过 ControlCommand.pto_cmd 下发（在 publish_command 中处理）
        - 在 cannode 的 XiaosongCANParser 中映射到 0x18FD0821 报文（起始位48，长度8bit，0.05V/bit；1~7档对应4~1V）
        """
        try:
            lvl = int(level)
        except Exception:
            lvl = 0
        if lvl < 0:
            lvl = 0
        if lvl > 7:
            lvl = 7
        self.engine_gear_level = lvl
        # UI 状态标签
        if hasattr(self, 'engine_gear_status_label'):
            self.engine_gear_status_label.config(
                text=str(lvl),
                fg=self.accent_color if lvl > 0 else '#888888'
            )
        # 立即下发一次控制，便于快速生效
        self.publish_command()

    def set_acc_active(self, active: bool):
        """
        acc: false/true = 关闭 / ACC 指令
        映射：通过 switch_bits bit 28 传递，在 cannode 中映射到 switch_ext_bits bit 36
        根据 excavator-can-host2vcu_utf8.csv: bit 36: acc指令(DOH_OUT25_194), 1是有效
        """
        self.acc_active = bool(active)
        self.publish_command()

    def reset_controls(self):
        """重置所有控制值"""
        self.controls = {
            'arm_up_current': 0.0,
            'arm_down_current': 0.0,
            'stick_retract_current': 0.0,
            'stick_extend_current': 0.0,
            'bucket_close_current': 0.0,
            'bucket_dump_current': 0.0,
            'rotate_left_current': 0.0,
            'rotate_right_current': 0.0,
            'left_track_forward_current': 0.0,
            'left_track_backward_current': 0.0,
            'right_track_forward_current': 0.0,
            'right_track_backward_current': 0.0,
            'brake': 0.0,
            'estop': False,
            'parking_brake': False,
            'request_remote_control': False,
        }

        # 重置高级功能控制
        self.bucket_switch_mode = 0
        self.ladder_switch_mode = 0
        self.horn_active = False
        self.hydraulic_lock_active = False
        self.light_code_mode = 0
        self.power_enable_active = False
        self.start_up_active = False
        self.acc_active = False
        self.engine_gear_level = 1  # 重置时默认 1 档
        if hasattr(self, 'power_enable_var'):
            try:
                self.power_enable_var.set(False)
            except Exception:
                pass
        if hasattr(self, 'start_up_var'):
            try:
                self.start_up_var.set(False)
            except Exception:
                pass
        if hasattr(self, 'acc_var'):
            try:
                self.acc_var.set(False)
            except Exception:
                pass
        if hasattr(self, 'bucket_switch_var'):
            try:
                self.bucket_switch_var.set(0)
            except Exception:
                pass
        if hasattr(self, 'ladder_switch_var'):
            try:
                self.ladder_switch_var.set(0)
            except Exception:
                pass
        if hasattr(self, 'hydraulic_lock_var'):
            try:
                self.hydraulic_lock_var.set(False)
            except Exception:
                pass
        if hasattr(self, 'light_code_var'):
            try:
                self.light_code_var.set(0)
            except Exception:
                pass
        if hasattr(self, 'engine_gear_var'):
            try:
                self.engine_gear_var.set(1)
            except Exception:
                pass
        
        # 更新所有 GUI 控件的变量
        for key, value in self.controls.items():
            if key in self.control_vars:
                var = self.control_vars[key]
                if isinstance(var, tk.DoubleVar):
                    var.set(float(value))
                elif isinstance(var, tk.BooleanVar):
                    var.set(bool(value))
                elif isinstance(var, tk.IntVar):
                    var.set(int(value))
        
        # 重置所有开关量
        for switch_name in self.switches:
            self.switches[switch_name] = False
            if switch_name in self.switch_vars:
                self.switch_vars[switch_name].set(False)
        
        self.publish_command()
        self.get_logger().info('已重置所有控制值')
    
    def update_actual_current_labels(self):
        """更新实际发送电流标签（从CAN解析的值）"""
        # 更新所有实际发送电流标签
        if 'arm_up' in self.actual_label_widgets:
            self.actual_label_widgets['arm_up'].config(text=f'[CAN:{self.actual_currents["arm_up"]}mA]')
        if 'arm_down' in self.actual_label_widgets:
            self.actual_label_widgets['arm_down'].config(text=f'[CAN:{self.actual_currents["arm_down"]}mA]')
        if 'stick_retract' in self.actual_label_widgets:
            self.actual_label_widgets['stick_retract'].config(text=f'[CAN:{self.actual_currents["stick_retract"]}mA]')
        if 'stick_extend' in self.actual_label_widgets:
            self.actual_label_widgets['stick_extend'].config(text=f'[CAN:{self.actual_currents["stick_extend"]}mA]')
        if 'bucket_close' in self.actual_label_widgets:
            self.actual_label_widgets['bucket_close'].config(text=f'[CAN:{self.actual_currents["bucket_close"]}mA]')
        if 'bucket_dump' in self.actual_label_widgets:
            self.actual_label_widgets['bucket_dump'].config(text=f'[CAN:{self.actual_currents["bucket_dump"]}mA]')
        if 'rotate_left' in self.actual_label_widgets:
            self.actual_label_widgets['rotate_left'].config(text=f'[CAN:{self.actual_currents["rotate_left"]}mA]')
        if 'rotate_right' in self.actual_label_widgets:
            self.actual_label_widgets['rotate_right'].config(text=f'[CAN:{self.actual_currents["rotate_right"]}mA]')
        if 'left_forward' in self.actual_label_widgets:
            self.actual_label_widgets['left_forward'].config(text=f'[CAN:{self.actual_currents["left_forward"]}mA]')
        if 'left_backward' in self.actual_label_widgets:
            self.actual_label_widgets['left_backward'].config(text=f'[CAN:{self.actual_currents["left_backward"]}mA]')
        if 'right_forward' in self.actual_label_widgets:
            self.actual_label_widgets['right_forward'].config(text=f'[CAN:{self.actual_currents["right_forward"]}mA]')
        if 'right_backward' in self.actual_label_widgets:
            self.actual_label_widgets['right_backward'].config(text=f'[CAN:{self.actual_currents["right_backward"]}mA]')
    
    def update_status_labels(self):
        """更新状态标签显示（从CAN解析的值）"""
        if hasattr(self, 'remote_control_status_label'):
            remote_status = self.can_status.get('request_remote_control', False)
            self.remote_control_status_label.config(
                text='是' if remote_status else '否',
                fg=self.accent_color if remote_status else '#888888'
            )
        if hasattr(self, 'emergency_stop_status_label'):
            estop_status = self.can_status.get('emergency_stop', False)
            danger_color = '#ff0044'  # 危险颜色（红色）
            self.emergency_stop_status_label.config(
                text='是' if estop_status else '否',
                fg=danger_color if estop_status else '#888888'
            )

        # 功能状态（车梯/斗/喇叭）
        if hasattr(self, 'ladder_status_label'):
            ladder = int(self.can_status.get('ladder_switch', 0) or 0)
            ladder_txt = '无动作'
            if ladder == 1:
                switch_info = self.function_switch_map['ladder_switch']['switch_21']
                ladder_txt = f'收起 (bit{switch_info["bit"]}, {switch_info["wire"]}线, {switch_info["io"]})'
            elif ladder == 2:
                switch_info = self.function_switch_map['ladder_switch']['switch_22']
                ladder_txt = f'放下 (bit{switch_info["bit"]}, {switch_info["wire"]}线, {switch_info["io"]})'
            self.ladder_status_label.config(
                text=ladder_txt,
                fg=self.accent_color if ladder != 0 else '#888888'
            )
        if hasattr(self, 'bucket_status_label'):
            bucket = int(self.can_status.get('bucket_switch', 0) or 0)
            bucket_txt = '无动作'
            if bucket == 1:
                switch_info = self.function_switch_map['bucket_switch']['switch_30']
                bucket_txt = f'开斗 (bit{switch_info["bit"]}, {switch_info["wire"]}线, {switch_info["io"]})'
            elif bucket == 2:
                switch_info = self.function_switch_map['bucket_switch']['switch_31']
                bucket_txt = f'关斗 (bit{switch_info["bit"]}, {switch_info["wire"]}线, {switch_info["io"]})'
            self.bucket_status_label.config(
                text=bucket_txt,
                fg=self.accent_color if bucket != 0 else '#888888'
            )
        if hasattr(self, 'horn_status_label'):
            horn = bool(self.can_status.get('horn', False))
            if horn:
                switch_info = self.function_switch_map['horn']['switch_19']
                horn_txt = f'是 (bit{switch_info["bit"]}, {switch_info["wire"]}线, {switch_info["io"]})'
            else:
                horn_txt = '否'
            self.horn_status_label.config(
                text=horn_txt,
                fg=self.accent_color if horn else '#888888'
            )

        # 上电状态
        if hasattr(self, 'power_enable_status_label'):
            power_enable = bool(self.can_status.get('power_enable', False))
            if power_enable:
                switch_info = self.function_switch_map['power_enable']['switch_bits_bit26']
                power_enable_txt = f'是 (bit{switch_info["bit"]}, {switch_info["io"]})'
            else:
                power_enable_txt = '否'
            self.power_enable_status_label.config(
                text=power_enable_txt,
                fg=self.accent_color if power_enable else '#888888'
            )

        # 发动机启动状态
        if hasattr(self, 'start_up_status_label'):
            start_up = bool(self.can_status.get('start_up', False))
            if start_up:
                switch_info = self.function_switch_map['start_up']['switch_bits_bit27']
                start_up_txt = f'是 (bit{switch_info["bit"]}, {switch_info["io"]})'
            else:
                start_up_txt = '否'
            self.start_up_status_label.config(
                text=start_up_txt,
                fg=self.accent_color if start_up else '#888888'
            )

        # ACC 状态
        if hasattr(self, 'acc_status_label'):
            acc = bool(self.can_status.get('acc', False))
            if acc:
                switch_info = self.function_switch_map['acc']['switch_bits_bit28']
                acc_txt = f'是 (bit{switch_info["bit"]}, {switch_info["io"]})'
            else:
                acc_txt = '否'
            self.acc_status_label.config(
                text=acc_txt,
                fg=self.accent_color if acc else '#888888'
            )

        # 下发 VCU 转速 / 油门 CAN 回显
        if hasattr(self, 'engine_rpm_tx_status_label'):
            erpm = int(self.can_status.get('engine_rpm_tx', 0) or 0)
            self.engine_rpm_tx_status_label.config(
                text=f'{erpm} rpm (0x18210005)',
                fg=self.accent_color if erpm > 0 else '#888888'
            )
        if hasattr(self, 'engine_throttle_tx_status_label'):
            traw = int(self.can_status.get('engine_throttle_raw', 0) or 0)
            tv = float(self.can_status.get('engine_throttle_voltage', 0.0) or 0.0)
            tg = int(self.can_status.get('engine_throttle_gear_tx', 0) or 0)
            self.engine_throttle_tx_status_label.config(
                text=f'raw 0x{traw:02X}  {tv:.2f}V  ≈{tg}档 (0x18FD0821)',
                fg=self.accent_color if traw > 0 else '#888888'
            )

        # 先导安全锁定状态
        if hasattr(self, 'hydraulic_status_label'):
            hyd = bool(self.can_status.get('hydraulic_lock', False))
            if hyd:
                switch_info = self.function_switch_map['hydraulic_lock']['switch_24']
                hyd_txt = f'开 (bit{switch_info["bit"]}, {switch_info["wire"]}线, {switch_info["io"]})'
            else:
                hyd_txt = '关'
            self.hydraulic_status_label.config(
                text=hyd_txt,
                fg=self.accent_color if hyd else '#888888'
            )

        # 灯光状态
        if hasattr(self, 'light_status_label'):
            light_code = int(self.can_status.get('light_code', 0) or 0)
            if light_code == 0:
                txt = '全关'
            elif light_code == 1:
                switch_info = self.function_switch_map['light_code']['switch_17']
                txt = f'灯1 (bit{switch_info["bit"]}, {switch_info["wire"]}线, {switch_info["io"]})'
            elif light_code == 2:
                switch_info = self.function_switch_map['light_code']['switch_18']
                txt = f'灯2 (bit{switch_info["bit"]}, {switch_info["wire"]}线, {switch_info["io"]})'
            elif light_code == 3:
                switch_info1 = self.function_switch_map['light_code']['switch_17']
                switch_info2 = self.function_switch_map['light_code']['switch_18']
                txt = f'灯1+2 (bit{switch_info1["bit"]}/{switch_info1["wire"]}线, bit{switch_info2["bit"]}/{switch_info2["wire"]}线)'
            else:
                txt = f'未知({light_code})'
            self.light_status_label.config(
                text=txt,
                fg=self.accent_color if light_code != 0 else '#888888'
            )
    
    def update_data_panel(self):
        """更新数据显示面板（只显示 CAN 报文）"""
        # 更新 CAN 报文显示
        can_frames = self.get_latest_can_frames()
        self.can_text.delete('1.0', tk.END)
        
        if can_frames:
            self.can_text.insert('1.0', '=' * 60 + '\n')
            self.can_text.insert(tk.END, '最新 CAN 报文 (最近5条)\n')
            self.can_text.insert(tk.END, '=' * 60 + '\n\n')
            
            # 显示最近的 CAN 帧（倒序，最新的在前）
            for frame_info in reversed(can_frames[-5:]):  # 只显示最近5条
                timestamp = frame_info['timestamp']
                can_data = frame_info['data']
                
                # 解析 CAN 帧
                parsed = self.parse_can_frame(can_data)
                
                self.can_text.insert(tk.END, f'[{timestamp}] ', 'timestamp')
                self.can_text.insert(tk.END, f'CAN ID: {parsed["can_id"]} ({parsed["can_id_dec"]})\n', 'can_id')
                self.can_text.insert(tk.END, f'  DLC: {can_data.get("dlc", "N/A")}\n')
                self.can_text.insert(tk.END, f'  数据(HEX): {parsed["data_hex"]}\n')
                self.can_text.insert(tk.END, f'  数据(数组): {can_data.get("data", [])}\n')
                
                # 显示解析出的电流值
                if parsed.get('currents'):
                    self.can_text.insert(tk.END, '  解析的电流值:\n', 'current_header')
                    for name, value in parsed['currents'].items():
                        self.can_text.insert(tk.END, f'    {name}: {value} mA\n', 'current_value')
                
                # 显示解析出的开关量（带接线信息）
                if parsed.get('switches'):
                    self.can_text.insert(tk.END, '  解析的开关量:\n', 'switch_header')
                    active_switches = [name for name, value in parsed['switches'].items() if value]
                    if active_switches:
                        switch_details = []
                        for switch_name in active_switches:
                            func_name = self.switch_name_map.get(switch_name, switch_name)
                            wire_num = self.switch_wire_map.get(switch_name, '')
                            if wire_num:
                                switch_details.append(f'{switch_name}({func_name}, {wire_num}线)')
                            else:
                                switch_details.append(f'{switch_name}({func_name})')
                        self.can_text.insert(tk.END, f'    激活: {", ".join(switch_details)}\n', 'switch_value')
                    else:
                        self.can_text.insert(tk.END, '    无激活开关\n', 'switch_value')
                
                # 显示 CAN 状态（bit32/bit33）
                if parsed.get('can_status'):
                    self.can_text.insert(tk.END, '  CAN 状态 (bit32/bit33):\n', 'status_header')
                    status = parsed['can_status']
                    remote_text = '是' if status.get('request_remote_control', False) else '否'
                    estop_text = '是' if status.get('emergency_stop', False) else '否'
                    self.can_text.insert(tk.END, f'    远程申请进入遥控: {remote_text}\n', 'status_value')
                    self.can_text.insert(tk.END, f'    远程申请进入急停: {estop_text}\n', 'status_value')

                # 显示功能状态（车梯/斗/喇叭/先导锁/灯光）- 带接线信息
                if parsed.get('function_status'):
                    self.can_text.insert(tk.END, '  功能状态:\n', 'func_header')
                    fs = parsed['function_status']
                    ladder = int(fs.get('ladder_switch', 0) or 0)
                    bucket = int(fs.get('bucket_switch', 0) or 0)
                    horn = bool(fs.get('horn', False))
                    hydraulic = bool(fs.get('hydraulic_lock', False))
                    light_code = int(fs.get('light_code', 0) or 0)
                    
                    # 车梯状态（带bit和接线信息）
                    if ladder == 0:
                        ladder_txt = '无动作'
                    elif ladder == 1:
                        switch_info = self.function_switch_map['ladder_switch']['switch_21']
                        ladder_txt = f'车梯收起 (bit{switch_info["bit"]}, switch_21, {switch_info["wire"]}线, {switch_info["io"]})'
                    else:
                        switch_info = self.function_switch_map['ladder_switch']['switch_22']
                        ladder_txt = f'车梯放下 (bit{switch_info["bit"]}, switch_22, {switch_info["wire"]}线, {switch_info["io"]})'
                    
                    # 斗状态（带bit和接线信息）
                    if bucket == 0:
                        bucket_txt = '无动作'
                    elif bucket == 1:
                        switch_info = self.function_switch_map['bucket_switch']['switch_30']
                        bucket_txt = f'开斗 (bit{switch_info["bit"]}, switch_30, {switch_info["wire"]}线, {switch_info["io"]})'
                    else:
                        switch_info = self.function_switch_map['bucket_switch']['switch_31']
                        bucket_txt = f'关斗 (bit{switch_info["bit"]}, switch_31, {switch_info["wire"]}线, {switch_info["io"]})'
                    
                    # 喇叭状态（带bit和接线信息）
                    horn_txt = '否'
                    if horn:
                        switch_info = self.function_switch_map['horn']['switch_19']
                        horn_txt = f'是 (bit{switch_info["bit"]}, switch_19, {switch_info["wire"]}线, {switch_info["io"]})'
                    
                    # 先导锁状态（带bit和接线信息）
                    hydraulic_txt = '关'
                    if hydraulic:
                        switch_info = self.function_switch_map['hydraulic_lock']['switch_24']
                        hydraulic_txt = f'开 (bit{switch_info["bit"]}, switch_24, {switch_info["wire"]}线, {switch_info["io"]})'
                    
                    # 灯光状态（带bit和接线信息）
                    if light_code == 0:
                        light_txt = '全关'
                    elif light_code == 1:
                        switch_info = self.function_switch_map['light_code']['switch_17']
                        light_txt = f'灯1 (bit{switch_info["bit"]}, switch_17, {switch_info["wire"]}线, {switch_info["io"]})'
                    elif light_code == 2:
                        switch_info = self.function_switch_map['light_code']['switch_18']
                        light_txt = f'灯2 (bit{switch_info["bit"]}, switch_18, {switch_info["wire"]}线, {switch_info["io"]})'
                    elif light_code == 3:
                        switch_info1 = self.function_switch_map['light_code']['switch_17']
                        switch_info2 = self.function_switch_map['light_code']['switch_18']
                        light_txt = f'灯1+2 (bit{switch_info1["bit"]}/switch_17/{switch_info1["wire"]}线, bit{switch_info2["bit"]}/switch_18/{switch_info2["wire"]}线)'
                    else:
                        light_txt = f'未知({light_code})'
                    
                    self.can_text.insert(tk.END, f'    车梯: {ladder_txt}\n', 'func_value')
                    self.can_text.insert(tk.END, f'    斗: {bucket_txt}\n', 'func_value')
                    self.can_text.insert(tk.END, f'    喇叭: {horn_txt}\n', 'func_value')
                    self.can_text.insert(tk.END, f'    先导锁: {hydraulic_txt}\n', 'func_value')
                    self.can_text.insert(tk.END, f'    灯光: {light_txt}\n', 'func_value')
                
                self.can_text.insert(tk.END, '\n' + '-' * 60 + '\n\n')
        else:
            self.can_text.insert('1.0', '等待 CAN 报文...\n')
        
        # 配置文本样式
        self.can_text.tag_config('timestamp', foreground=self.accent_color)
        self.can_text.tag_config('can_id', foreground=self.fg_color, font=('Courier', 9, 'bold'))
        self.can_text.tag_config('current_header', foreground=self.accent_color, font=('Courier', 9, 'bold'))
        self.can_text.tag_config('current_value', foreground='#00ff88')
        self.can_text.tag_config('switch_header', foreground=self.accent_color, font=('Courier', 9, 'bold'))
        self.can_text.tag_config('switch_value', foreground='#ffaa00')
        self.can_text.tag_config('status_header', foreground=self.accent_color, font=('Courier', 9, 'bold'))
        self.can_text.tag_config('status_value', foreground='#ffaa00')
        self.can_text.tag_config('func_header', foreground=self.accent_color, font=('Courier', 9, 'bold'))
        self.can_text.tag_config('func_value', foreground='#ffaa00')
        
        # 更新状态标签显示
        self.update_status_labels()


def main(args=None):
    rclpy.init(args=args)
    
    controller = XiaosongController()
    
    # 在单独线程中运行 ROS2 节点
    def spin_node():
        rclpy.spin(controller)
    
    spin_thread = threading.Thread(target=spin_node, daemon=True)
    spin_thread.start()
    
    # 运行 GUI（主线程）
    controller.run_gui()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()

