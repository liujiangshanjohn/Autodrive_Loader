#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
挖掘机控制可视化界面 - 科技感版本
订阅 /controls/teleop 话题，实时显示挖掘机的控制指令和动作状态
包含动画模拟和科技感界面设计
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
    
    # 尝试从环境变量获取 ROS2 路径
    ros_distro = os.environ.get('ROS_DISTRO', 'humble')
    ros_python_path = f'/opt/ros/{ros_distro}/lib/python3.10/site-packages'
    if os.path.exists(ros_python_path) and ros_python_path not in sys.path:
        sys.path.insert(0, ros_python_path)
    
    ros_local_path = f'/opt/ros/{ros_distro}/local/lib/python3.10/dist-packages'
    if os.path.exists(ros_local_path) and ros_local_path not in sys.path:
        sys.path.insert(0, ros_local_path)

# 设置环境
setup_ros2_environment()

# 检查 ROS2 环境
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String as StringMsg
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
except ImportError as e:
    # print("错误: 无法导入 ROS2 模块。")
    # print("请运行以下命令设置环境:")
    # print("  source /opt/ros/humble/setup.bash  # 根据你的 ROS2 版本调整")
    # print("  source ~/cannode_ws/install/setup.bash")
    # print("或者使用启动脚本: ./run_visualizer.sh")
    # print(f"详细错误: {e}")
    sys.exit(1)

# 尝试导入自定义消息（可选，如果失败则输出信息功能将被禁用）
VehicleCommand = None
try:
    from teleoptocantransformer.msg import VehicleCommand
except (ImportError, Exception) as e:
    # print("警告: 无法导入 teleoptocantransformer 消息包。")
    # print("输出信息显示功能将被禁用，但输入信息显示和动画仍可正常工作。")
    # print("如果需要输出信息，请确保:")
    # print("  1. 已编译工作空间: colcon build")
    # print("  2. 已 source 工作空间: source ~/cannode_ws/install/setup.bash")
    # print("或者使用启动脚本: ./run_visualizer.sh")
    # print(f"详细错误: {e}")
    VehicleCommand = None  # 设置为 None，表示不可用

import json
import tkinter as tk
from tkinter import ttk
import threading
import math
from datetime import datetime

class TeleopVisualizer(Node):
    def __init__(self):
        super().__init__('teleop_visualizer')
        
        # 订阅 /controls/teleop 话题
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )
        self.teleop_sub = self.create_subscription(
            StringMsg,
            '/controls/teleop',
            self.teleop_callback,
            qos_profile
        )
        self.get_logger().info('已订阅 /controls/teleop 话题')
        
        # 订阅输出信息 /vehicle_command_debug（可选，如果类型支持不可用则跳过）
        self.output_available = False
        if VehicleCommand is not None:
            try:
                qos_profile_reliable = QoSProfile(
                    depth=10,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.VOLATILE,
                    history=HistoryPolicy.KEEP_LAST
                )
                self.vehicle_cmd_sub = self.create_subscription(
                    VehicleCommand,
                    '/vehicle_command_debug',
                    self.vehicle_command_callback,
                    qos_profile_reliable
                )
                self.get_logger().info('已订阅 /vehicle_command_debug 话题')
                self.output_available = True
            except Exception as e:
                self.get_logger().warn(f'无法订阅 /vehicle_command_debug 话题: {e}')
                self.get_logger().warn('输出信息显示将被禁用。如果需要，请重新编译工作空间: colcon build')
                self.output_available = False
        else:
            self.get_logger().warn('VehicleCommand 消息类型不可用，输出信息显示将被禁用')
            self.output_available = False
        
        # 订阅 CAN 帧信息话题
        self.can_frame_sub = self.create_subscription(
            StringMsg,
            '/can_frame_sent',
            self.can_frame_callback,
            qos_profile
        )
        self.get_logger().info('已订阅 /can_frame_sent 话题')
        
        # 存储最新的控制数据（输入）
        self.latest_data = {
            'leftTrack': 0.0,
            'rightTrack': 0.0,
            'swing': 0.0,
            'boom': 0.0,
            'stick': 0.0,
            'bucket': 0.0,
            'steering': 0.0,
            'throttle': 0.0,
            'brake': 0.0,
            'gear': 'N',
            'speed_mode': 'turtle',
            'emergency_stop': False,
            'parking_brake': False,
            'power_enable': False
        }
        
        # 存储最新的输出数据
        self.latest_output = {
            'steering_target': 0.0,
            'throttle': 0.0,
            'brake': 0.0,
            'gear_location': 0,
            'speed': 0.0,
            'arm_angle': 0.0,
            'arm_enable': False,
            'shovel_angle': 0.0,
            'shovel_enable': False,
            'estop': False,
            'parking_brake': False,
            'engine_on_off': False
        }
        
        # 存储最新的 CAN 帧信息
        self.latest_can_frames = []  # 存储最近发送的 CAN 帧（最多保留最近10条）
        
        # 线程锁
        self.data_lock = threading.Lock()
        self.gui_running = True
        
        # 启动 GUI
        self.gui_thread = threading.Thread(target=self.run_gui, daemon=False)
        self.gui_thread.start()
        
    def teleop_callback(self, msg):
        """处理接收到的控制指令（输入）"""
        try:
            data = json.loads(msg.data)
            
            # 打印输入信息到终端
            timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
            # print(f'\n[{timestamp}] 📥 输入信息:')
            # print('=' * 60)
            for key, value in sorted(data.items()):
                if isinstance(value, float):
                    # print(f'  {key:20s}: {value:8.3f}')
                elif isinstance(value, bool):
                    # print(f'  {key:20s}: {value}')
                else:
                    # print(f'  {key:20s}: {value}')
            # print('=' * 60)
            
            with self.data_lock:
                # 更新数据（支持两种命名方式：驼峰和下划线）
                for key in self.latest_data:
                    if key in data:
                        self.latest_data[key] = data[key]
                    # 同时支持下划线命名方式（left_track -> leftTrack, right_track -> rightTrack）
                    elif key == 'leftTrack' and 'left_track' in data:
                        self.latest_data[key] = data['left_track']
                    elif key == 'rightTrack' and 'right_track' in data:
                        self.latest_data[key] = data['right_track']
                        
        except json.JSONDecodeError as e:
            self.get_logger().warn(f'JSON 解析错误: {e}')
        except Exception as e:
            self.get_logger().error(f'处理消息时出错: {e}')
    
    def vehicle_command_callback(self, msg):
        """处理接收到的车辆控制命令（输出）"""
        if not self.output_available:
            return
            
        try:
            # 更新输出数据
            with self.data_lock:
                self.latest_output = {
                    'steering_target': msg.steering_target,
                    'throttle': msg.throttle,
                    'brake': msg.brake,
                    'gear_location': msg.gear_location,
                    'speed': msg.speed,
                    'arm_angle': msg.arm_angle,
                    'arm_enable': msg.arm_enable,
                    'shovel_angle': msg.shovel_angle,
                    'shovel_enable': msg.shovel_enable,
                    'estop': msg.estop,
                    'parking_brake': msg.parking_brake,
                    'engine_on_off': msg.engine_on_off
                }
            
            # 打印输出信息到终端
            timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
            gear_map = {0: 'N(空档)', 1: 'D(前进)', 2: 'R(后退)'}
            # print(f'\n[{timestamp}] 📤 输出信息:')
            # print('=' * 60)
            # print(f'  {"转向目标":20s}: {msg.steering_target:8.2f} (范围: -800~800)')
            # print(f'  {"油门":20s}: {msg.throttle:8.2f} (范围: 0~200)')
            # print(f'  {"刹车":20s}: {msg.brake:8.2f} (范围: 350~3900)')
            # print(f'  {"档位":20s}: {gear_map.get(msg.gear_location, "未知")} ({msg.gear_location})')
            # print(f'  {"目标速度":20s}: {msg.speed:8.2f} m/s')
            # print(f'  {"大臂角度":20s}: {msg.arm_angle:8.2f}° (范围: -800~800) {"[启用]" if msg.arm_enable else "[禁用]"}')
            # print(f'  {"铲斗角度":20s}: {msg.shovel_angle:8.2f}° (范围: -800~800) {"[启用]" if msg.shovel_enable else "[禁用]"}')
            # print(f'  {"紧急停止":20s}: {"是" if msg.estop else "否"}')
            # print(f'  {"驻车制动":20s}: {"是" if msg.parking_brake else "否"}')
            # print(f'  {"发动机":20s}: {"开启" if msg.engine_on_off else "关闭"}')
            # print('=' * 60)
            
        except Exception as e:
            self.get_logger().error(f'处理输出消息时出错: {e}')
    
    def can_frame_callback(self, msg):
        """处理接收到的 CAN 帧信息"""
        try:
            can_data = json.loads(msg.data)
            
            # 打印 CAN 帧信息到终端
            timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
            # print(f'\n[{timestamp}] 🔌 CAN 帧发送:')
            # print('=' * 60)
            # print(f'  {"CAN ID":20s}: {can_data.get("can_id", "N/A")} ({can_data.get("can_id_dec", "N/A")})')
            # print(f'  {"DLC":20s}: {can_data.get("dlc", "N/A")}')
            # print(f'  {"数据(HEX)":20s}: {can_data.get("data_hex", "N/A")}')
            # print(f'  {"数据(数组)":20s}: {can_data.get("data", [])}')
            # print('=' * 60)
            
            # 更新最新的 CAN 帧信息（保留最近10条）
            with self.data_lock:
                self.latest_can_frames.append(can_data)
                if len(self.latest_can_frames) > 10:
                    self.latest_can_frames.pop(0)
                    
        except json.JSONDecodeError as e:
            self.get_logger().warn(f'CAN 帧 JSON 解析错误: {e}')
        except Exception as e:
            self.get_logger().error(f'处理 CAN 帧消息时出错: {e}')
    
    def get_latest_data(self):
        """获取最新的控制数据（线程安全）"""
        with self.data_lock:
            return self.latest_data.copy()
    
    def get_latest_output(self):
        """获取最新的输出数据（线程安全）"""
        with self.data_lock:
            return self.latest_output.copy()
    
    def run_gui(self):
        """运行 GUI 界面"""
        root = tk.Tk()
        root.title('挖掘机控制可视化系统')
        root.geometry('1200x800')
        
        # 深色科技感主题
        bg_color = '#0a0e27'  # 深蓝黑色背景
        fg_color = '#00ffff'  # 青色文字
        accent_color = '#00ff88'  # 绿色强调
        danger_color = '#ff0044'  # 红色警告
        panel_bg = '#1a1f3a'  # 面板背景
        
        root.configure(bg=bg_color)
        
        # 绑定窗口关闭事件
        def on_closing():
            self.gui_running = False
            root.quit()
            root.destroy()
            # 通知 ROS2 节点退出
            rclpy.shutdown()
            sys.exit(0)
        
        root.protocol("WM_DELETE_WINDOW", on_closing)
        
        # 创建主容器
        main_container = tk.Frame(root, bg=bg_color)
        main_container.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # 标题栏
        title_frame = tk.Frame(main_container, bg=bg_color)
        title_frame.pack(fill=tk.X, pady=(0, 20))
        
        title_label = tk.Label(
            title_frame,
            text='挖掘机控制可视化系统',
            font=('Arial', 24, 'bold'),
            bg=bg_color,
            fg=fg_color
        )
        title_label.pack()
        
        subtitle_label = tk.Label(
            title_frame,
            text='EXCAVATOR CONTROL VISUALIZATION SYSTEM',
            font=('Arial', 10),
            bg=bg_color,
            fg=accent_color
        )
        subtitle_label.pack()
        
        # 创建左右布局
        left_panel = tk.Frame(main_container, bg=panel_bg, relief=tk.RAISED, bd=2)
        left_panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 10))
        
        right_panel = tk.Frame(main_container, bg=panel_bg, relief=tk.RAISED, bd=2)
        right_panel.pack(side=tk.RIGHT, fill=tk.BOTH, expand=False, padx=(10, 0))
        
        # 左侧：挖掘机动画
        self.create_excavator_animation(left_panel, bg_color, fg_color, accent_color)
        
        # 右侧：控制数据显示
        self.create_control_panel(right_panel, bg_color, fg_color, accent_color, danger_color, panel_bg)
        
        # 存储根窗口和颜色，供重置函数使用
        self.root_window = root
        self.bg_color = bg_color
        self.fg_color = fg_color
        self.accent_color = accent_color
        
        # 更新函数
        def update_gui():
            if not self.gui_running:
                return
            try:
                data = self.get_latest_data()
                output_data = self.get_latest_output()
                self.update_excavator_animation(data)
                self.update_control_panel(data)
                self.update_output_panel(output_data)
                self.update_can_frame_panel()
                root.after(30, update_gui)  # 每30ms更新一次，更流畅
            except tk.TclError:
                # 窗口已关闭
                pass
        
        # 开始更新循环
        update_gui()
        
        # 运行 GUI
        root.mainloop()
    
    def create_excavator_animation(self, parent, bg_color, fg_color, accent_color):
        """创建挖掘机动画画布"""
        # 标题
        title = tk.Label(
            parent,
            text='挖掘机动作模拟',
            font=('Arial', 14, 'bold'),
            bg=parent['bg'],
            fg=fg_color
        )
        title.pack(pady=10)
        
        # 创建画布
        canvas_width = 600
        canvas_height = 500
        self.canvas = tk.Canvas(
            parent,
            width=canvas_width,
            height=canvas_height,
            bg=bg_color,
            highlightthickness=2,
            highlightbackground=accent_color
        )
        self.canvas.pack(padx=20, pady=(20, 10))
        
        # 添加重置按钮
        reset_button = tk.Button(
            parent,
            text='重置位置 (Reset)',
            font=('Arial', 12, 'bold'),
            bg='#ff0044',
            fg='#ffffff',
            activebackground='#ff3366',
            activeforeground='#ffffff',
            relief=tk.RAISED,
            bd=3,
            padx=20,
            pady=10,
            command=self.reset_excavator_position
        )
        reset_button.pack(pady=10)
        
        # 初始化动画参数
        self.canvas_width = canvas_width
        self.canvas_height = canvas_height
        self.center_x = canvas_width // 2
        self.center_y = canvas_height // 2
        
        # 绘制网格背景（科技感）
        self.draw_grid()
        
        # 绘制坐标轴
        self.draw_axes()
    
    def draw_grid(self):
        """绘制网格背景"""
        grid_color = '#1a2a4a'
        step = 20
        for x in range(0, self.canvas_width, step):
            self.canvas.create_line(x, 0, x, self.canvas_height, fill=grid_color, width=1, tags='grid')
        for y in range(0, self.canvas_height, step):
            self.canvas.create_line(0, y, self.canvas_width, y, fill=grid_color, width=1, tags='grid')
    
    def draw_axes(self):
        """绘制坐标轴"""
        axis_color = '#00ffff'
        # X轴
        self.canvas.create_line(
            0, self.center_y,
            self.canvas_width, self.center_y,
            fill=axis_color, width=2, tags='axis'
        )
        # Y轴
        self.canvas.create_line(
            self.center_x, 0,
            self.center_x, self.canvas_height,
            fill=axis_color, width=2, tags='axis'
        )
    
    def get_track_value(self, data, track_name):
        """获取履带值，支持两种命名方式：驼峰（leftTrack/rightTrack）和下划线（left_track/right_track）"""
        # 优先使用驼峰命名
        camel_name = track_name
        snake_name = track_name.replace('Track', '_track')
        if camel_name in data:
            return data[camel_name]
        elif snake_name in data:
            return data[snake_name]
        else:
            return 0.0
    
    def apply_track_deadzone(self, value, deadzone=0.3):
        """应用履带死区：-deadzone 到 deadzone 范围内不动作，剩余部分映射到 [-1, 1]"""
        if abs(value) <= deadzone:
            return 0.0
        
        # 将有效范围映射到 [-1, 1]
        if value > deadzone:
            # 正方向：将 [deadzone, 1] 映射到 [0, 1]
            mapped = (value - deadzone) / (1.0 - deadzone)
        else:
            # 负方向：将 [-1, -deadzone] 映射到 [-1, 0]
            mapped = (value + deadzone) / (1.0 - deadzone)
        
        return mapped
    
    def update_excavator_animation(self, data):
        """更新挖掘机动画"""
        # 清除之前的绘制（保留网格和坐标轴）
        items_to_keep = []
        for item in self.canvas.find_all():
            tags = self.canvas.gettags(item)
            if 'grid' in tags or 'axis' in tags:
                items_to_keep.append(item)
        
        # 删除非保留项
        for item in self.canvas.find_all():
            if item not in items_to_keep:
                self.canvas.delete(item)
        
        # 获取控制数据（支持两种命名方式）
        leftTrack_raw = self.get_track_value(data, 'leftTrack')  # -1 到 1，后退到前进
        rightTrack_raw = self.get_track_value(data, 'rightTrack')  # -1 到 1，后退到前进
        
        # 应用死区：-0.3 到 0.3 不动作，剩余部分映射到 [-1, 1]
        leftTrack = self.apply_track_deadzone(leftTrack_raw, deadzone=0.3)
        rightTrack = self.apply_track_deadzone(rightTrack_raw, deadzone=0.3)
        
        swing = data.get('swing', 0.0)  # -1 到 1，左转到右转
        boom = data.get('boom', 0.0)  # -1 到 1，下降到提升
        stick = data.get('stick', 0.0)  # -1 到 1，收回到伸出
        bucket = data.get('bucket', 0.0)  # -1 到 1，收回到翻转
        
        # 计算底盘运动（根据左右履带差速）
        # 平均速度（前进/后退）
        avg_speed = (leftTrack + rightTrack) / 2.0
        # 转向速度（差速转向）
        turn_speed = (rightTrack - leftTrack) / 2.0
        
        # 底盘位置（根据履带运动计算，累积位置）
        if not hasattr(self, 'base_offset_x'):
            self.base_offset_x = 0.0
            self.base_offset_y = 0.0
            self.base_heading = 0.0  # 底盘朝向角度（度）
        
        # 重置标志（用于重置位置）
        if not hasattr(self, 'reset_position_flag'):
            self.reset_position_flag = False
        
        # 检查是否需要重置位置
        if hasattr(self, 'reset_position_flag') and self.reset_position_flag:
            self.base_offset_x = 0.0
            self.base_offset_y = 0.0
            self.base_heading = 0.0
            self.reset_position_flag = False
        
        # 更新底盘位置和朝向（简化模型：每帧移动）
        # 使用较小的速度缩放，使运动更平滑
        move_speed = avg_speed * 1.5  # 移动速度缩放
        turn_rate = turn_speed * 2.0  # 转向速度缩放（度/帧）
        
        # 添加阻尼，使运动更平滑
        if abs(avg_speed) < 0.01:
            move_speed *= 0.9  # 减速
        if abs(turn_speed) < 0.01:
            turn_rate *= 0.9  # 减转向
        
        # 更新朝向
        self.base_heading += turn_rate
        self.base_heading = self.base_heading % 360  # 归一化到 0-360
        
        # 更新位置（根据朝向和速度）
        heading_rad = math.radians(self.base_heading)
        self.base_offset_x += move_speed * math.sin(heading_rad)
        self.base_offset_y -= move_speed * math.cos(heading_rad)  # Y轴向上为负
        
        # 限制底盘位置在画布范围内（允许更大的移动范围）
        max_offset = 250
        self.base_offset_x = max(-max_offset, min(max_offset, self.base_offset_x))
        self.base_offset_y = max(-max_offset, min(max_offset, self.base_offset_y))
        
        # 底盘中心（可随履带移动和回转）
        swing_angle = swing * 60  # 最大回转角度 ±60度，更明显
        base_x = self.center_x + self.base_offset_x
        base_y = self.center_y + 200 + self.base_offset_y  # 底盘在下方，往下移更多空间给铲斗
        
        # 绘制底盘（矩形，根据朝向旋转）
        base_width = 120
        base_height = 40
        # 计算旋转后的底盘四个角点
        base_heading_rad = math.radians(self.base_heading)
        cos_h = math.cos(base_heading_rad)
        sin_h = math.sin(base_heading_rad)
        
        # 底盘四个角的相对坐标
        corners = [
            (-base_width//2, -base_height//2),
            (base_width//2, -base_height//2),
            (base_width//2, base_height//2),
            (-base_width//2, base_height//2)
        ]
        
        # 旋转并平移
        base_points = []
        for cx, cy in corners:
            # 旋转
            rx = cx * cos_h - cy * sin_h
            ry = cx * sin_h + cy * cos_h
            # 平移
            base_points.extend([base_x + rx, base_y + ry])
        
        self.canvas.create_polygon(base_points, fill='#333366', outline='#00ffff', width=2, tags='excavator')
        
        # 绘制左右履带指示（在底盘两侧）
        track_indicator_length = 15
        # 左履带指示
        left_track_y = base_y - base_height//2 - 5
        left_track_color = '#00ff88' if leftTrack > 0.01 else '#ff8800' if leftTrack < -0.01 else '#666666'
        self.canvas.create_line(
            base_x - base_width//2 - 10, left_track_y,
            base_x - base_width//2 - 10 - track_indicator_length * abs(leftTrack), left_track_y,
            fill=left_track_color, width=3, tags='excavator', arrow=tk.LAST if abs(leftTrack) > 0.01 else tk.NONE
        )
        # 右履带指示
        right_track_y = base_y - base_height//2 - 5
        right_track_color = '#00ff88' if rightTrack > 0.01 else '#ff8800' if rightTrack < -0.01 else '#666666'
        self.canvas.create_line(
            base_x + base_width//2 + 10, right_track_y,
            base_x + base_width//2 + 10 + track_indicator_length * abs(rightTrack), right_track_y,
            fill=right_track_color, width=3, tags='excavator', arrow=tk.LAST if abs(rightTrack) > 0.01 else tk.NONE
        )
        
        # 回转中心（在底盘上）
        swing_center_x = base_x
        swing_center_y = base_y - base_height//2
        
        # 计算回转后的坐标
        swing_rad = math.radians(swing_angle)
        cos_swing = math.cos(swing_rad)
        sin_swing = math.sin(swing_rad)
        
        # 大臂基座（在回转中心上方）
        boom_base_x = swing_center_x
        boom_base_y = swing_center_y - 20
        
        # 大臂角度（-1 到 1 映射到 -80 到 80 度，相对于垂直方向，更明显）
        boom_angle = boom * 80  # 提升为正，下降为负
        boom_length = 90  # 长度缩小一半（原来180）
        
        # 大臂末端坐标（相对于大臂基座）
        boom_rad = math.radians(boom_angle + swing_angle)
        boom_end_x = boom_base_x + boom_length * math.sin(boom_rad)
        boom_end_y = boom_base_y - boom_length * math.cos(boom_rad)
        
        # 绘制大臂（更粗的线条，更明显）
        self.canvas.create_line(
            boom_base_x, boom_base_y,
            boom_end_x, boom_end_y,
            fill='#00ff88', width=12, tags='excavator'
        )
        self.canvas.create_oval(
            boom_base_x - 8, boom_base_y - 8,
            boom_base_x + 8, boom_base_y + 8,
            fill='#00ffff', outline='#00ff88', width=2, tags='excavator'
        )
        
        # 斗杆（连接在大臂末端）
        stick_angle = stick * 60  # -1 到 1 映射到 -60 到 60 度，更明显
        stick_length = 75  # 长度缩小一半（原来150）
        stick_rad = math.radians(boom_angle + stick_angle + swing_angle)
        stick_end_x = boom_end_x + stick_length * math.sin(stick_rad)
        stick_end_y = boom_end_y - stick_length * math.cos(stick_rad)
        
        # 绘制斗杆（更粗的线条，更明显）
        self.canvas.create_line(
            boom_end_x, boom_end_y,
            stick_end_x, stick_end_y,
            fill='#ff8800', width=10, tags='excavator'
        )
        self.canvas.create_oval(
            boom_end_x - 6, boom_end_y - 6,
            boom_end_x + 6, boom_end_y + 6,
            fill='#00ffff', outline='#ff8800', width=2, tags='excavator'
        )
        
        # 铲斗（连接在斗杆末端）
        bucket_angle = bucket * 70  # -1 到 1 映射到 -70 到 70 度，更明显
        bucket_length = 65  # 长度缩小一半（原来130）
        bucket_rad = math.radians(boom_angle + stick_angle + bucket_angle + swing_angle)
        bucket_end_x = stick_end_x + bucket_length * math.sin(bucket_rad)
        bucket_end_y = stick_end_y - bucket_length * math.cos(bucket_rad)
        
        # 绘制铲斗（更粗的线条，更明显）
        self.canvas.create_line(
            stick_end_x, stick_end_y,
            bucket_end_x, bucket_end_y,
            fill='#ff0044', width=12, tags='excavator'  # 增加线条粗细
        )
        
        # 铲斗形状（更大的三角形，更突出）
        bucket_size = 35  # 增大铲斗尺寸
        # 计算铲斗的宽度（垂直于铲斗方向）
        perp_rad = bucket_rad + math.pi / 2
        # 铲斗的四个点，形成更完整的形状
        bucket_points = [
            stick_end_x, stick_end_y,  # 连接点
            bucket_end_x + bucket_size * math.cos(perp_rad), bucket_end_y + bucket_size * math.sin(perp_rad),  # 上角
            bucket_end_x + bucket_size * 0.6 * math.cos(bucket_rad), bucket_end_y + bucket_size * 0.6 * math.sin(bucket_rad),  # 前端点
            bucket_end_x - bucket_size * math.cos(perp_rad), bucket_end_y - bucket_size * math.sin(perp_rad),  # 下角
        ]
        self.canvas.create_polygon(bucket_points, fill='#ff0044', outline='#ffaa00', width=3, tags='excavator')
        
        # 在铲斗中心添加高光效果，使其更突出
        highlight_x = bucket_end_x + bucket_size * 0.3 * math.cos(bucket_rad)
        highlight_y = bucket_end_y + bucket_size * 0.3 * math.sin(bucket_rad)
        self.canvas.create_oval(
            highlight_x - 8, highlight_y - 8,
            highlight_x + 8, highlight_y + 8,
            fill='#ff6666', outline='#ffaa00', width=2, tags='excavator'
        )
        
        # 绘制回转指示器
        if abs(swing) > 0.01:
            indicator_color = '#00ff88' if swing > 0 else '#ff8800'
            self.canvas.create_arc(
                swing_center_x - 30, swing_center_y - 30,
                swing_center_x + 30, swing_center_y + 30,
                start=0, extent=swing_angle,
                outline=indicator_color, width=3, style=tk.ARC, tags='excavator'
            )
        
        # 添加标签
        self.canvas.create_text(
            base_x, base_y + 30,
            text='底盘', fill='#00ffff', font=('Arial', 10), tags='excavator'
        )
        # 显示履带速度
        if abs(leftTrack) > 0.01 or abs(rightTrack) > 0.01:
            track_text = f'L:{leftTrack:.2f} R:{rightTrack:.2f}'
            self.canvas.create_text(
                base_x, base_y + 45,
                text=track_text, fill='#00ff88', font=('Arial', 8), tags='excavator'
            )
        # 大臂标签（在大臂右侧，靠近部件）
        boom_mid_x = (boom_base_x + boom_end_x) / 2
        boom_mid_y = (boom_base_y + boom_end_y) / 2
        # 计算垂直于大臂的方向（右侧）
        # 在画布坐标系中，右侧是 X 正方向，所以需要根据部件角度计算
        boom_perp_rad = boom_rad - math.pi / 2  # 顺时针旋转90度得到右侧方向
        boom_label_offset = 25  # 偏移距离，让标签在部件右侧外部但不太远
        boom_label_x = boom_mid_x + boom_label_offset * math.cos(boom_perp_rad)
        boom_label_y = boom_mid_y + boom_label_offset * math.sin(boom_perp_rad)
        self.canvas.create_text(
            boom_label_x, boom_label_y,
            text='大臂', fill='#ffffff', font=('Arial', 10, 'bold'), tags='excavator'
        )
        
        # 斗杆标签（在斗杆右侧，靠近部件）
        stick_mid_x = (boom_end_x + stick_end_x) / 2
        stick_mid_y = (boom_end_y + stick_end_y) / 2
        # 计算垂直于斗杆的方向（右侧）
        stick_perp_rad = stick_rad - math.pi / 2  # 顺时针旋转90度得到右侧方向
        stick_label_offset = 25  # 偏移距离
        stick_label_x = stick_mid_x + stick_label_offset * math.cos(stick_perp_rad)
        stick_label_y = stick_mid_y + stick_label_offset * math.sin(stick_perp_rad)
        self.canvas.create_text(
            stick_label_x, stick_label_y,
            text='斗杆', fill='#ffffff', font=('Arial', 10, 'bold'), tags='excavator'
        )
        
        # 铲斗标签（在铲斗中间位置的右侧，白色）
        # 使用铲斗的中点位置
        bucket_mid_x = (stick_end_x + bucket_end_x) / 2
        bucket_mid_y = (stick_end_y + bucket_end_y) / 2
        # 计算垂直于铲斗的方向（右侧）
        bucket_perp_rad = bucket_rad - math.pi / 2  # 顺时针旋转90度得到右侧方向
        bucket_label_offset = 25  # 与大臂、斗杆使用相同的偏移距离
        bucket_label_x = bucket_mid_x + bucket_label_offset * math.cos(bucket_perp_rad)
        bucket_label_y = bucket_mid_y + bucket_label_offset * math.sin(bucket_perp_rad)
        # 显示铲斗标签（白色）
        self.canvas.create_text(
            bucket_label_x, bucket_label_y,
            text='铲斗', fill='#ffffff', font=('Arial', 11, 'bold'), tags='excavator'
        )
        # 显示铲斗角度值（在铲斗标签下方，白色）
        bucket_value_text = f'{bucket:.2f}'
        self.canvas.create_text(
            bucket_label_x, bucket_label_y + 15,
            text=bucket_value_text, fill='#ffffff', font=('Arial', 9), tags='excavator'
        )
    
    def reset_excavator_position(self):
        """重置挖掘机位置到初始状态"""
        if hasattr(self, 'base_offset_x'):
            self.reset_position_flag = True
            self.get_logger().info('重置挖掘机位置到初始状态')
    
    def reset_excavator_position(self):
        """重置挖掘机位置到初始状态"""
        if hasattr(self, 'base_offset_x'):
            self.reset_position_flag = True
            self.get_logger().info('重置挖掘机位置到初始状态')
    
    def create_control_panel(self, parent, bg_color, fg_color, accent_color, danger_color, panel_bg):
        """创建控制面板"""
        # 标题
        title = tk.Label(
            parent,
            text='控制数据',
            font=('Arial', 14, 'bold'),
            bg=panel_bg,
            fg=fg_color
        )
        title.pack(pady=10)
        
        # 创建滚动框架
        canvas = tk.Canvas(parent, bg=panel_bg, highlightthickness=0)
        scrollbar = ttk.Scrollbar(parent, orient="vertical", command=canvas.yview)
        scrollable_frame = tk.Frame(canvas, bg=panel_bg)
        
        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        
        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        
        # 创建各个控制项
        self.control_widgets = {}
        
        # 左右履带
        self.create_control_item(scrollable_frame, '左履带', 'leftTrack', -1.0, 1.0, fg_color, accent_color, panel_bg)
        self.create_control_item(scrollable_frame, '右履带', 'rightTrack', -1.0, 1.0, fg_color, accent_color, panel_bg)
        
        # 车身回转
        self.create_control_item(scrollable_frame, '车身回转', 'swing', -1.0, 1.0, fg_color, accent_color, panel_bg)
        
        # 大臂
        self.create_control_item(scrollable_frame, '大臂', 'boom', -1.0, 1.0, fg_color, accent_color, panel_bg)
        
        # 斗杆
        self.create_control_item(scrollable_frame, '斗杆', 'stick', -1.0, 1.0, fg_color, accent_color, panel_bg)
        
        # 铲斗
        self.create_control_item(scrollable_frame, '铲斗', 'bucket', -1.0, 1.0, fg_color, accent_color, panel_bg)
        
        # 状态信息
        self.create_status_section(scrollable_frame, fg_color, accent_color, danger_color, panel_bg)
        
        # 输出信息
        self.create_output_section(scrollable_frame, fg_color, accent_color, danger_color, panel_bg)
        
        # CAN 帧信息
        self.create_can_frame_section(scrollable_frame, fg_color, accent_color, danger_color, panel_bg)
    
    def create_control_item(self, parent, label, key, min_val, max_val, fg_color, accent_color, panel_bg):
        """创建单个控制项"""
        frame = tk.Frame(parent, bg=panel_bg, relief=tk.RAISED, bd=1)
        frame.pack(fill=tk.X, padx=10, pady=8)
        
        # 标签
        label_widget = tk.Label(
            frame,
            text=label,
            font=('Arial', 11, 'bold'),
            bg=panel_bg,
            fg=fg_color,
            width=10,
            anchor='w'
        )
        label_widget.pack(side=tk.LEFT, padx=5)
        
        # 进度条容器
        progress_frame = tk.Frame(frame, bg=panel_bg)
        progress_frame.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        
        # 自定义进度条（使用 Canvas 绘制，更科技感）
        canvas = tk.Canvas(
            progress_frame,
            height=25,
            bg=panel_bg,
            highlightthickness=1,
            highlightbackground=accent_color
        )
        canvas.pack(fill=tk.X, padx=5)
        
        # 数值标签
        value_label = tk.Label(
            frame,
            text='0.00',
            font=('Arial', 10, 'bold'),
            bg=panel_bg,
            fg=accent_color,
            width=8
        )
        value_label.pack(side=tk.RIGHT, padx=5)
        
        self.control_widgets[key] = {
            'canvas': canvas,
            'value_label': value_label,
            'min': min_val,
            'max': max_val
        }
    
    def create_status_section(self, parent, fg_color, accent_color, danger_color, panel_bg):
        """创建状态信息区域"""
        status_frame = tk.LabelFrame(
            parent,
            text='系统状态',
            font=('Arial', 11, 'bold'),
            bg=panel_bg,
            fg=fg_color,
            relief=tk.RAISED,
            bd=2
        )
        status_frame.pack(fill=tk.X, padx=10, pady=10)
        
        self.status_labels = {}
        status_items = [
            ('leftTrack', '左履带', '0.00'),
            ('rightTrack', '右履带', '0.00'),
            ('gear', '档位', 'N'),
            ('throttle', '油门', '0.00'),
            ('brake', '刹车', '0.00'),
            ('speed_mode', '速度模式', 'turtle'),
            ('emergency_stop', '紧急停止', False),
            ('parking_brake', '驻车制动', False),
            ('power_enable', '电源使能', False)
        ]
        
        for i, (key, label, default) in enumerate(status_items):
            row_frame = tk.Frame(status_frame, bg=panel_bg)
            row_frame.pack(fill=tk.X, padx=10, pady=5)
            
            tk.Label(
                row_frame,
                text=f'{label}:',
                font=('Arial', 10),
                bg=panel_bg,
                fg=fg_color,
                width=12,
                anchor='w'
            ).pack(side=tk.LEFT)
            
            value_label = tk.Label(
                row_frame,
                text=str(default),
                font=('Arial', 10, 'bold'),
                bg=panel_bg,
                fg=accent_color
            )
            value_label.pack(side=tk.LEFT, padx=10)
            
            self.status_labels[key] = value_label
    
    def create_output_section(self, parent, fg_color, accent_color, danger_color, panel_bg):
        """创建输出信息区域"""
        output_frame = tk.LabelFrame(
            parent,
            text='输出信息（转换后）',
            font=('Arial', 11, 'bold'),
            bg=panel_bg,
            fg=accent_color,
            relief=tk.RAISED,
            bd=2
        )
        output_frame.pack(fill=tk.X, padx=10, pady=10)
        
        self.output_labels = {}
        output_items = [
            ('steering_target', '转向目标', '0.00', '[-800, 800]'),
            ('throttle', '油门', '0.00', '[0, 200]'),
            ('brake', '刹车', '0.00', '[350, 3900]'),
            ('gear_location', '档位', '0', '0=N, 1=D, 2=R'),
            ('speed', '目标速度', '0.00', 'm/s'),
            ('arm_angle', '大臂角度', '0.00', '度 [-800, 800]'),
            ('arm_enable', '大臂使能', False, ''),
            ('shovel_angle', '铲斗角度', '0.00', '度 [-800, 800]'),
            ('shovel_enable', '铲斗使能', False, ''),
            ('estop', '紧急停止', False, ''),
            ('parking_brake', '驻车制动', False, ''),
            ('engine_on_off', '发动机', False, '')
        ]
        
        for i, (key, label, default, unit) in enumerate(output_items):
            row_frame = tk.Frame(output_frame, bg=panel_bg)
            row_frame.pack(fill=tk.X, padx=10, pady=3)
            
            tk.Label(
                row_frame,
                text=f'{label}:',
                font=('Arial', 9),
                bg=panel_bg,
                fg=fg_color,
                width=14,
                anchor='w'
            ).pack(side=tk.LEFT)
            
            value_label = tk.Label(
                row_frame,
                text=str(default),
                font=('Arial', 9, 'bold'),
                bg=panel_bg,
                fg=accent_color,
                width=12,
                anchor='w'
            )
            value_label.pack(side=tk.LEFT, padx=5)
            
            if unit:
                unit_label = tk.Label(
                    row_frame,
                    text=unit,
                    font=('Arial', 8),
                    bg=panel_bg,
                    fg='#666666',
                    anchor='w'
                )
                unit_label.pack(side=tk.LEFT, padx=5)
            
            self.output_labels[key] = value_label
    
    def create_can_frame_section(self, parent, fg_color, accent_color, danger_color, panel_bg):
        """创建 CAN 帧信息显示区域"""
        can_frame_frame = tk.LabelFrame(
            parent,
            text='CAN 帧信息（下发给 VCU）',
            font=('Arial', 11, 'bold'),
            bg=panel_bg,
            fg=accent_color,
            relief=tk.RAISED,
            bd=2
        )
        can_frame_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # 创建文本显示区域（可滚动）
        text_frame = tk.Frame(can_frame_frame, bg=panel_bg)
        text_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # 文本显示区域
        self.can_frame_text = tk.Text(
            text_frame,
            height=8,
            width=40,
            bg='#0a0a0a',
            fg='#00ffff',
            font=('Courier', 9),
            wrap=tk.WORD,
            relief=tk.SUNKEN,
            bd=2,
            highlightthickness=1,
            highlightbackground=accent_color
        )
        self.can_frame_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        # 滚动条
        can_scrollbar = ttk.Scrollbar(text_frame, orient="vertical", command=self.can_frame_text.yview)
        can_scrollbar.pack(side=tk.RIGHT, fill="y")
        self.can_frame_text.configure(yscrollcommand=can_scrollbar.set)
        
        # 初始化显示
        self.can_frame_text.insert('1.0', '等待 CAN 帧数据...\n')
        self.can_frame_text.config(state=tk.DISABLED)  # 只读模式
    
    def update_can_frame_panel(self):
        """更新 CAN 帧信息显示"""
        with self.data_lock:
            can_frames = self.latest_can_frames.copy()
        
        if not can_frames:
            return
        
        # 更新文本显示（显示最近5条）
        self.can_frame_text.config(state=tk.NORMAL)
        self.can_frame_text.delete('1.0', tk.END)
        
        # 显示最近5条 CAN 帧（从新到旧）
        display_frames = can_frames[-5:] if len(can_frames) > 5 else can_frames
        display_frames.reverse()  # 最新的在顶部
        
        for frame in display_frames:
            can_id = frame.get('can_id', 'N/A')
            can_id_dec = frame.get('can_id_dec', 'N/A')
            dlc = frame.get('dlc', 'N/A')
            data_hex = frame.get('data_hex', 'N/A')
            
            # 格式化显示
            self.can_frame_text.insert(tk.END, f'ID: {can_id} ({can_id_dec})\n')
            self.can_frame_text.insert(tk.END, f'DLC: {dlc}\n')
            self.can_frame_text.insert(tk.END, f'Data: {data_hex}\n')
            self.can_frame_text.insert(tk.END, '-' * 35 + '\n')
        
        self.can_frame_text.see('1.0')  # 滚动到顶部
        self.can_frame_text.config(state=tk.DISABLED)
    
    def update_control_panel(self, data):
        """更新控制面板"""
        # 更新所有控制项（支持两种命名方式）
        for key in ['leftTrack', 'rightTrack', 'swing', 'boom', 'stick', 'bucket']:
            if key in ['leftTrack', 'rightTrack']:
                value = self.get_track_value(data, key)
            else:
                value = data.get(key, 0.0)
            self.update_control_item(key, value)
        
        # 更新状态信息（支持两种命名方式）
        leftTrack = self.get_track_value(data, 'leftTrack')
        rightTrack = self.get_track_value(data, 'rightTrack')
        self.status_labels['leftTrack'].config(text=f'{leftTrack:.2f}')
        self.status_labels['rightTrack'].config(text=f'{rightTrack:.2f}')
        
        # 根据履带值设置颜色
        left_color = '#00ff88' if leftTrack > 0.01 else '#ff8800' if leftTrack < -0.01 else '#666666'
        right_color = '#00ff88' if rightTrack > 0.01 else '#ff8800' if rightTrack < -0.01 else '#666666'
        self.status_labels['leftTrack'].config(fg=left_color)
        self.status_labels['rightTrack'].config(fg=right_color)
        
        self.status_labels['gear'].config(text=data.get('gear', 'N'))
        self.status_labels['throttle'].config(text=f'{data.get("throttle", 0.0):.2f}')
        self.status_labels['brake'].config(text=f'{data.get("brake", 0.0):.2f}')
        
        speed_mode = data.get('speed_mode', 'turtle')
        mode_text = '🐰 兔子' if speed_mode == 'rabbit' else '🐢 乌龟'
        self.status_labels['speed_mode'].config(text=mode_text)
        
        emergency_stop = data.get('emergency_stop', False)
        estop_text = '⚠️ 激活' if emergency_stop else '✓ 正常'
        estop_color = '#ff0044' if emergency_stop else '#00ff88'
        self.status_labels['emergency_stop'].config(text=estop_text, fg=estop_color)
        
        parking_brake = data.get('parking_brake', False)
        brake_text = '🔒 已启用' if parking_brake else '🔓 未启用'
        brake_color = '#ff8800' if parking_brake else '#00ff88'
        self.status_labels['parking_brake'].config(text=brake_text, fg=brake_color)
        
        power_enable = data.get('power_enable', False)
        power_text = '⚡ 开启' if power_enable else '⚫ 关闭'
        power_color = '#00ff88' if power_enable else '#666666'
        self.status_labels['power_enable'].config(text=power_text, fg=power_color)
    
    def update_output_panel(self, output_data):
        """更新输出信息面板"""
        if not hasattr(self, 'output_labels'):
            return
        
        # 更新输出数值
        gear_map = {0: 'N', 1: 'D', 2: 'R'}
        gear_location = output_data.get('gear_location', 0)
        
        self.output_labels['steering_target'].config(text=f'{output_data.get("steering_target", 0.0):.2f}')
        self.output_labels['throttle'].config(text=f'{output_data.get("throttle", 0.0):.2f}')
        self.output_labels['brake'].config(text=f'{output_data.get("brake", 0.0):.2f}')
        self.output_labels['gear_location'].config(text=f'{gear_map.get(gear_location, "?")} ({gear_location})')
        self.output_labels['speed'].config(text=f'{output_data.get("speed", 0.0):.2f}')
        self.output_labels['arm_angle'].config(text=f'{output_data.get("arm_angle", 0.0):.2f}')
        self.output_labels['arm_enable'].config(
            text='✓ 启用' if output_data.get('arm_enable', False) else '✗ 禁用',
            fg='#00ff88' if output_data.get('arm_enable', False) else '#666666'
        )
        self.output_labels['shovel_angle'].config(text=f'{output_data.get("shovel_angle", 0.0):.2f}')
        self.output_labels['shovel_enable'].config(
            text='✓ 启用' if output_data.get('shovel_enable', False) else '✗ 禁用',
            fg='#00ff88' if output_data.get('shovel_enable', False) else '#666666'
        )
        estop = output_data.get('estop', False)
        self.output_labels['estop'].config(
            text='⚠️ 是' if estop else '✓ 否',
            fg='#ff0044' if estop else '#00ff88'
        )
        parking_brake = output_data.get('parking_brake', False)
        self.output_labels['parking_brake'].config(
            text='🔒 是' if parking_brake else '🔓 否',
            fg='#ff8800' if parking_brake else '#00ff88'
        )
        engine = output_data.get('engine_on_off', False)
        self.output_labels['engine_on_off'].config(
            text='⚡ 开启' if engine else '⚫ 关闭',
            fg='#00ff88' if engine else '#666666'
        )
    
    def update_control_item(self, key, value):
        """更新单个控制项"""
        if key not in self.control_widgets:
            return
        
        widget = self.control_widgets[key]
        canvas = widget['canvas']
        value_label = widget['value_label']
        min_val = widget['min']
        max_val = widget['max']
        
        # 清除画布
        canvas.delete("all")
        
        # 计算进度（0 到 1）
        progress = (value - min_val) / (max_val - min_val)
        progress = max(0.0, min(1.0, progress))
        
        # 获取画布尺寸
        width = canvas.winfo_width()
        if width < 10:
            width = 200  # 默认宽度
        
        height = 25
        
        # 绘制背景
        canvas.create_rectangle(0, 0, width, height, fill='#1a1f3a', outline='#00ffff', width=1)
        
        # 绘制进度条
        progress_width = int(width * progress)
        
        # 根据值选择颜色
        if abs(value) < 0.01:
            color = '#333366'  # 静止
        elif value > 0:
            color = '#00ff88'  # 正向
        else:
            color = '#ff8800'  # 负向
        
        canvas.create_rectangle(0, 0, progress_width, height, fill=color, outline='')
        
        # 绘制中心线
        center_x = width // 2
        canvas.create_line(center_x, 0, center_x, height, fill='#00ffff', width=1, dash=(2, 2))
        
        # 更新数值标签
        value_label.config(text=f'{value:.2f}')
        
        # 根据值改变标签颜色
        if abs(value) < 0.01:
            value_label.config(fg='#666666')
        elif value > 0:
            value_label.config(fg='#00ff88')
        else:
            value_label.config(fg='#ff8800')


def main(args=None):
    rclpy.init(args=args)
    node = TeleopVisualizer()
    
    try:
        # 在单独的线程中运行 ROS2
        def spin_ros():
            try:
                while node.gui_running:
                    rclpy.spin_once(node, timeout_sec=0.1)
            except Exception as e:
                if node.gui_running:
                    node.get_logger().error(f'ROS2 spin 错误: {e}')
        
        ros_thread = threading.Thread(target=spin_ros, daemon=True)
        ros_thread.start()
        
        # 等待 GUI 线程结束
        node.gui_thread.join()
        
    except KeyboardInterrupt:
        node.gui_running = False
    except Exception as e:
        node.get_logger().error(f'程序错误: {e}')
        node.gui_running = False
    finally:
        node.gui_running = False
        try:
            node.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
