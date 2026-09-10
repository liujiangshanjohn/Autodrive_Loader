#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试脚本：模拟发布多摄像头目标检测结果
话题：/perception/object_detection
消息类型：cyber_msgs/msg/MultiCameraObjectDetection
发布频率：10Hz

支持两种测试场景：
1. 检测到行人（--with-person 或默认周期性切换）
2. 没有检测到行人（--no-person 或默认周期性切换）
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import time
import random
import argparse

try:
    from cyber_msgs.msg import MultiCameraObjectDetection, CameraObjectArray, Object2D
    from builtin_interfaces.msg import Time
    HAS_CYBER_MSGS = True
except ImportError as e:
    print(f"错误: 无法导入 cyber_msgs: {e}")
    print("请确保已编译 cyber_msgs 包: colcon build --packages-select cyber_msgs")
    HAS_CYBER_MSGS = False


class ObjectDetectionPublisher(Node):
    def __init__(self, mode='auto'):
        """
        初始化发布者
        Args:
            mode: 'auto' - 自动在两种场景间切换（每5秒）
                  'with_person' - 始终检测到行人
                  'no_person' - 始终不检测到行人
        """
        super().__init__('test_object_detection_publisher')
        
        if not HAS_CYBER_MSGS:
            self.get_logger().error("cyber_msgs 不可用，退出")
            return
        
        # 创建发布者，使用 Reliable QoS 匹配订阅者
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )
        
        self.pub = self.create_publisher(
            MultiCameraObjectDetection,
            '/perception/object_detection',
            qos_profile
        )
        
        self.mode = mode
        self.get_logger().info('已创建发布者 /perception/object_detection')
        self.get_logger().info('发布频率: 10Hz (每 0.1 秒)')
        
        if mode == 'auto':
            self.get_logger().info('模式: 自动切换（每5秒在"检测到行人"和"未检测到行人"之间切换）')
        elif mode == 'with_person':
            self.get_logger().info('模式: 始终检测到行人')
        elif mode == 'no_person':
            self.get_logger().info('模式: 始终不检测到行人')
        
        # 创建定时器，10Hz = 0.1 秒
        self.timer = self.create_timer(0.1, self.publish_detection)
        
        # 计数器，用于生成不同的测试数据
        self.counter = 0
        
        # 预定义的检测类别
        self.class_names = ["person", "car", "truck", "bicycle", "motorcycle", 
                           "bus", "traffic_cone", "dog", "cat", "unknown"]
        
        # 预定义的摄像头ID
        self.camera_ids = ["front", "left", "right", "rear"]
    
    def create_object2d(self, obj_id, class_name, confidence, center_x, center_y, width, height):
        """创建 Object2D 消息"""
        obj = Object2D()
        obj.id = obj_id
        obj.class_name = class_name
        obj.confidence = confidence
        obj.center_x = center_x
        obj.center_y = center_y
        obj.width = width
        obj.height = height
        return obj
    
    def create_camera_array(self, camera_id, objects):
        """创建 CameraObjectArray 消息"""
        camera = CameraObjectArray()
        camera.camera_id = camera_id
        camera.count = len(objects)
        camera.objects = objects
        return camera
    
    def should_detect_person(self):
        """根据模式决定是否检测到行人"""
        if self.mode == 'with_person':
            return True
        elif self.mode == 'no_person':
            return False
        else:  # auto mode
            # 每5秒切换一次（50次 * 0.1秒 = 5秒）
            return (self.counter // 50) % 2 == 0
    
    def generate_test_detection(self):
        """生成测试检测数据"""
        msg = MultiCameraObjectDetection()
        
        # 设置时间戳
        now = self.get_clock().now()
        msg.stamp.sec = now.nanoseconds // 1_000_000_000
        msg.stamp.nanosec = now.nanoseconds % 1_000_000_000
        
        # 决定是否检测到行人
        detect_person = self.should_detect_person()
        
        # 模拟不同场景的检测结果
        cameras = []
        
        if detect_person:
            # 场景：检测到行人
            # 前摄像头：检测到行人和车辆
            front_objects = []
            # 总是包含一个行人
            front_objects.append(self.create_object2d(
                obj_id=1,
                class_name="person",
                confidence=0.80 + random.random() * 0.15,
                center_x=0.4 + (random.random() - 0.5) * 0.3,
                center_y=0.65 + random.random() * 0.2,
                width=0.06 + random.random() * 0.04,
                height=0.12 + random.random() * 0.08
            ))
            # 可能还有车辆
            if random.random() > 0.3:
                front_objects.append(self.create_object2d(
                    obj_id=2,
                    class_name="car",
                    confidence=0.85 + random.random() * 0.1,
                    center_x=0.5 + (random.random() - 0.5) * 0.2,
                    center_y=0.6 + (random.random() - 0.5) * 0.1,
                    width=0.15 + random.random() * 0.1,
                    height=0.2 + random.random() * 0.1
                ))
            cameras.append(self.create_camera_array("front", front_objects))
            
            # 左摄像头：可能检测到行人
            left_objects = []
            if random.random() > 0.5:
                left_objects.append(self.create_object2d(
                    obj_id=3,
                    class_name="person",
                    confidence=0.75 + random.random() * 0.15,
                    center_x=0.2 + random.random() * 0.3,
                    center_y=0.5 + random.random() * 0.3,
                    width=0.05 + random.random() * 0.05,
                    height=0.1 + random.random() * 0.1
                ))
            # 可能还有车辆
            if random.random() > 0.4:
                left_objects.append(self.create_object2d(
                    obj_id=4,
                    class_name=random.choice(["car", "truck"]),
                    confidence=0.7 + random.random() * 0.2,
                    center_x=0.2 + random.random() * 0.3,
                    center_y=0.5 + random.random() * 0.3,
                    width=0.1 + random.random() * 0.15,
                    height=0.15 + random.random() * 0.2
                ))
            cameras.append(self.create_camera_array("left", left_objects))
            
            # 右摄像头：可能检测到行人
            right_objects = []
            if random.random() > 0.5:
                right_objects.append(self.create_object2d(
                    obj_id=5,
                    class_name="person",
                    confidence=0.75 + random.random() * 0.15,
                    center_x=0.5 + random.random() * 0.3,
                    center_y=0.5 + random.random() * 0.3,
                    width=0.05 + random.random() * 0.05,
                    height=0.1 + random.random() * 0.1
                ))
            # 可能还有车辆或自行车
            if random.random() > 0.4:
                right_objects.append(self.create_object2d(
                    obj_id=6,
                    class_name=random.choice(["car", "bicycle"]),
                    confidence=0.7 + random.random() * 0.2,
                    center_x=0.5 + random.random() * 0.3,
                    center_y=0.5 + random.random() * 0.3,
                    width=0.1 + random.random() * 0.15,
                    height=0.15 + random.random() * 0.2
                ))
            cameras.append(self.create_camera_array("right", right_objects))
            
            # 后摄像头：通常不检测行人，主要是车辆
            rear_objects = []
            if random.random() > 0.6:
                rear_objects.append(self.create_object2d(
                    obj_id=7,
                    class_name=random.choice(["car", "truck"]),
                    confidence=0.8 + random.random() * 0.15,
                    center_x=0.5 + (random.random() - 0.5) * 0.2,
                    center_y=0.3 + random.random() * 0.2,
                    width=0.12 + random.random() * 0.1,
                    height=0.18 + random.random() * 0.12
                ))
            cameras.append(self.create_camera_array("rear", rear_objects))
        
        else:
            # 场景：没有检测到行人（只有车辆、障碍物等）
            # 前摄像头：只有车辆，没有行人
            front_objects = []
            if random.random() > 0.2:
                front_objects.append(self.create_object2d(
                    obj_id=1,
                    class_name="car",
                    confidence=0.85 + random.random() * 0.1,
                    center_x=0.5 + (random.random() - 0.5) * 0.2,
                    center_y=0.6 + (random.random() - 0.5) * 0.1,
                    width=0.15 + random.random() * 0.1,
                    height=0.2 + random.random() * 0.1
                ))
            if random.random() > 0.5:
                front_objects.append(self.create_object2d(
                    obj_id=2,
                    class_name=random.choice(["truck", "bus"]),
                    confidence=0.80 + random.random() * 0.15,
                    center_x=0.3 + random.random() * 0.4,
                    center_y=0.5 + random.random() * 0.2,
                    width=0.18 + random.random() * 0.12,
                    height=0.25 + random.random() * 0.15
                ))
            cameras.append(self.create_camera_array("front", front_objects))
            
            # 左摄像头：只有车辆或障碍物，没有行人
            left_objects = []
            if random.random() > 0.4:
                left_objects.append(self.create_object2d(
                    obj_id=3,
                    class_name=random.choice(["car", "truck", "traffic_cone"]),
                    confidence=0.7 + random.random() * 0.2,
                    center_x=0.2 + random.random() * 0.3,
                    center_y=0.5 + random.random() * 0.3,
                    width=0.1 + random.random() * 0.15,
                    height=0.15 + random.random() * 0.2
                ))
            cameras.append(self.create_camera_array("left", left_objects))
            
            # 右摄像头：只有车辆或自行车，没有行人
            right_objects = []
            if random.random() > 0.4:
                right_objects.append(self.create_object2d(
                    obj_id=4,
                    class_name=random.choice(["car", "bicycle", "motorcycle"]),
                    confidence=0.7 + random.random() * 0.2,
                    center_x=0.5 + random.random() * 0.3,
                    center_y=0.5 + random.random() * 0.3,
                    width=0.1 + random.random() * 0.15,
                    height=0.15 + random.random() * 0.2
                ))
            cameras.append(self.create_camera_array("right", right_objects))
            
            # 后摄像头：只有车辆
            rear_objects = []
            if random.random() > 0.5:
                rear_objects.append(self.create_object2d(
                    obj_id=5,
                    class_name=random.choice(["car", "truck"]),
                    confidence=0.8 + random.random() * 0.15,
                    center_x=0.5 + (random.random() - 0.5) * 0.2,
                    center_y=0.3 + random.random() * 0.2,
                    width=0.12 + random.random() * 0.1,
                    height=0.18 + random.random() * 0.12
                ))
            cameras.append(self.create_camera_array("rear", rear_objects))
        
        
        msg.camera_count = len(cameras)
        msg.cameras = cameras
        
        return msg
    
    def publish_detection(self):
        """发布检测结果"""
        if not HAS_CYBER_MSGS:
            return
        
        msg = self.generate_test_detection()
        self.pub.publish(msg)
        
        # 打印统计信息（每10次打印一次，避免日志过多）
        if self.counter % 10 == 0:
            total_objects = sum(len(camera.objects) for camera in msg.cameras)
            detect_person = self.should_detect_person()
            person_count = sum(
                sum(1 for obj in camera.objects if obj.class_name == "person")
                for camera in msg.cameras
            )
            
            mode_str = "检测到行人" if detect_person else "未检测到行人"
            self.get_logger().info(
                f'发布检测结果 [计数: {self.counter}] [{mode_str}] '
                f'摄像头数: {msg.camera_count}, '
                f'总目标数: {total_objects}, '
                f'行人数: {person_count}'
            )
            
            # 打印每个摄像头的检测数量
            for camera in msg.cameras:
                if len(camera.objects) > 0:
                    obj_info = ", ".join([
                        f"{obj.class_name}({obj.confidence:.2f})" 
                        for obj in camera.objects
                    ])
                    self.get_logger().info(
                        f'  - {camera.camera_id}: {len(camera.objects)} 个目标 [{obj_info}]'
                    )
        
        self.counter += 1


def main(args=None):
    # 解析命令行参数
    parser = argparse.ArgumentParser(
        description='模拟发布多摄像头目标检测结果',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用示例:
  # 自动在两种场景间切换（每5秒）
  python3 test_object_detection_publisher.py
  
  # 始终检测到行人
  python3 test_object_detection_publisher.py --with-person
  
  # 始终不检测到行人
  python3 test_object_detection_publisher.py --no-person
        """
    )
    parser.add_argument(
        '--with-person',
        action='store_const',
        const='with_person',
        dest='mode',
        help='始终检测到行人'
    )
    parser.add_argument(
        '--no-person',
        action='store_const',
        const='no_person',
        dest='mode',
        help='始终不检测到行人'
    )
    parser.set_defaults(mode='auto')
    
    # 解析参数（需要先处理 args，因为 rclpy.init 也会处理 args）
    if args is None:
        import sys
        parsed_args = parser.parse_args()
    else:
        # 分离 ROS2 参数和脚本参数
        ros_args = []
        script_args = []
        skip_next = False
        for i, arg in enumerate(args):
            if skip_next:
                skip_next = False
                continue
            if arg.startswith('--'):
                if arg in ['--with-person', '--no-person']:
                    script_args.append(arg)
                else:
                    ros_args.append(arg)
            else:
                ros_args.append(arg)
        parsed_args = parser.parse_args(script_args)
        args = ros_args
    
    rclpy.init(args=args if 'args' in locals() else None)
    
    if not HAS_CYBER_MSGS:
        print("错误: 无法导入 cyber_msgs 消息类型")
        print("请先编译 cyber_msgs 包:")
        print("  colcon build --packages-select cyber_msgs")
        return
    
    node = ObjectDetectionPublisher(mode=parsed_args.mode)
    
    try:
        print("\n" + "="*60)
        print("多摄像头目标检测测试发布器")
        print("="*60)
        print("话题: /perception/object_detection")
        print("消息类型: cyber_msgs/msg/MultiCameraObjectDetection")
        print("发布频率: 10Hz")
        if parsed_args.mode == 'auto':
            print("模式: 自动切换（每5秒在'检测到行人'和'未检测到行人'之间切换）")
        elif parsed_args.mode == 'with_person':
            print("模式: 始终检测到行人")
        elif parsed_args.mode == 'no_person':
            print("模式: 始终不检测到行人")
        print("="*60)
        print("按 Ctrl+C 停止发布\n")
        
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\n停止发布...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

