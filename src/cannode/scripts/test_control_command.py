#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试控制命令发布
"""

import sys
import os

# 设置 protobuf 路径
script_dir = os.path.dirname(os.path.abspath(__file__))
protobuf_path = os.path.join(script_dir, '../protobuf/out')
if os.path.exists(protobuf_path) and protobuf_path not in sys.path:
    sys.path.insert(0, protobuf_path)

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    import cyber_msgs.msg
    from control_msgs.control_cmd_pb2 import ControlCommand
    from common_msgs.basic_msgs.header_pb2 import Header
except ImportError as e:
    # print(f"错误: 无法导入模块: {e}")
    sys.exit(1)

class TestControlCommandPublisher(Node):
    def __init__(self):
        super().__init__('test_control_command_publisher')
        
        # 创建发布者
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
        
        # 创建定时器，每秒发布一次测试命令
        self.timer = self.create_timer(1.0, self.publish_test_command)
        self.counter = 0
    
    def publish_test_command(self):
        """发布测试控制命令"""
        self.counter += 1
        
        # 创建 ControlCommand 消息
        cmd = ControlCommand()
        
        # 设置 header
        cmd.header.timestamp_sec = self.get_clock().now().nanoseconds / 1e9
        cmd.header.frame_id = "base_link"
        cmd.header.sequence_num = self.counter
        
        # 设置 xiaosong 专用的电流控制字段
        cmd.arm_up_current = 300.0  # 300mA
        cmd.arm_down_current = 0.0
        cmd.stick_retract_current = 0.0
        cmd.stick_extend_current = 0.0
        cmd.bucket_close_current = 0.0
        cmd.bucket_dump_current = 0.0
        cmd.rotate_left_current = 0.0
        cmd.rotate_right_current = 0.0
        cmd.left_track_forward_current = 0.0
        cmd.left_track_backward_current = 0.0
        cmd.right_track_forward_current = 0.0
        cmd.right_track_backward_current = 0.0
        cmd.switch_bits = 0
        
        # 序列化 protobuf 消息
        serialized_data = cmd.SerializeToString()
        
        # 创建 ROS2 消息
        ros_msg = cyber_msgs.msg.ProtoAdapter()
        # pb 字段需要 bytes 对象的列表，每个元素是单个字节的 bytes 对象
        ros_msg.pb = [bytes([b]) for b in serialized_data]
        
        # 发布消息
        self.cmd_pub.publish(ros_msg)
        
        self.get_logger().info(f'已发布测试控制命令 #{self.counter}: arm_up_current={cmd.arm_up_current}mA, 序列化大小={len(serialized_data)} 字节')

def main(args=None):
    rclpy.init(args=args)
    
    node = TestControlCommandPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

