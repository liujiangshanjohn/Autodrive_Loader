#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import re
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import paho.mqtt.client as mqtt


class ChassisToMqtt(Node):
    """订阅 cannode/chassis_feedback 话题，通过 MQTT WebSocket 上传遥测到远程服务器。"""

    def __init__(self):
        super().__init__('chassis_to_mqtt')

        self.declare_parameter('mqtt_ws_url', 'ws://10.106.241.152:8083/mqtt')
        self.declare_parameter('mqtt_telemetry_topic', 'device/excavator-001/telemetry')

        ws_url = self.get_parameter('mqtt_ws_url').value
        self.telemetry_topic = self.get_parameter('mqtt_telemetry_topic').value

        # 解析 ws://host:port/path
        from urllib.parse import urlparse
        parsed = urlparse(ws_url)
        self.mqtt_host = parsed.hostname or '10.106.241.152'
        self.mqtt_port = parsed.port or 8083
        self.ws_path = parsed.path or '/mqtt'

        self._mqtt_client = mqtt.Client(transport='websockets')
        self._mqtt_client.ws_set_options(path=self.ws_path)
        # 无用户名密码，不调用 username_pw_set
        try:
            self._mqtt_client.connect(self.mqtt_host, self.mqtt_port, 60)
            self._mqtt_client.loop_start()  # 后台线程维持连接、处理 keepalive 和 ACK
            self._mqtt_connected = True
            self.get_logger().info(
                'MQTT WebSocket 已连接: %s:%d%s，遥测话题: %s'
                % (self.mqtt_host, self.mqtt_port, self.ws_path, self.telemetry_topic)
            )
        except Exception as e:
            self._mqtt_connected = False
            self.get_logger().warn('MQTT 连接失败: %s，仅本地打印' % str(e))

        self.subscription = self.create_subscription(
            String,
            'cannode/chassis_feedback',
            self.callback,
            10,
        )
        self.get_logger().info('已订阅 cannode/chassis_feedback，等待消息并发布为MQTT')

    def callback(self, msg):
        payload = msg.data
        # self.get_logger().info('收到消息: %s' % payload)
        if self._mqtt_connected:
            try:
                self._mqtt_client.publish(
                    self.telemetry_topic, payload, qos=1
                )
                # self.get_logger().info('MQTT 发送成功')
            except Exception as e:
                self.get_logger().error('MQTT 发送失败: %s' % str(e))


def main(args=None):
    rclpy.init(args=args)
    node = ChassisToMqtt()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, '_mqtt_client') and node._mqtt_connected:
            node._mqtt_client.loop_stop()
            node._mqtt_client.disconnect()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass  # 忽略重复 shutdown


if __name__ == '__main__':
    main()
