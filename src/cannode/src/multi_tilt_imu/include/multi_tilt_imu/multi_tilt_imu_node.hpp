#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>  // 将用于发布相对角（大臂/斗杆/铲斗）

// 自定义姿态消息：使用 roll/pitch/yaw 字段
#include "cannode/msg/tilt_rpy.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// 所有代码内注释均使用中文

struct TiltSensorConfig
{
  // 节点 ID，用于组成 CAN ID：请求 0x600 + node_id，响应 0x580 + node_id
  uint8_t node_id;
  // 对应的 ROS2 话题名
  std::string topic_name;
};

class MultiTiltImuNode : public rclcpp::Node
{
public:
  explicit MultiTiltImuNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~MultiTiltImuNode() override;

private:
  // 初始化参数与 CAN 设备
  void init_parameters();
  bool init_can_socket();

  // 周期发送读取角度命令
  void send_request_timer_callback();

  // 接收线程函数
  void recv_loop();

  // 解析一帧倾角仪数据并发布
  void handle_tilt_frame(const struct can_frame & frame);

  // BCD 压缩格式解析函数：PITCH/ROLL 使用 3 字节 SX XX YY
  static bool decode_bcd_angle_3bytes(const uint8_t * data, double & angle_deg);

  // YAW 使用 2 字节 SX XX（无小数），根据手册描述做简单实现
  static bool decode_bcd_angle_2bytes(const uint8_t * data, double & angle_deg);

  // 挖掘机相对角计算（集成 changan 相对角思路）
  // 使用各 IMU 的 PITCH（俯仰角）：
  //   boom_rel_body_deg    = (boom_pitch - body_pitch)   + boom_rel_body_zero_deg
  //   stick_rel_boom_deg   = (stick_pitch - boom_pitch)  + stick_rel_boom_zero_deg
  //   bucket_rel_stick_deg = (bucket_pitch - stick_pitch)+ bucket_rel_stick_zero_deg
  void update_excavator_relative_angles();

  // ROS 相关
  std::vector<TiltSensorConfig> sensor_configs_;
  std::map<uint32_t, size_t> resp_id_to_index_;
  // 发布倾角姿态，消息中字段为 roll/pitch/yaw
  std::vector<rclcpp::Publisher<cannode::msg::TiltRPY>::SharedPtr> publishers_;
  // 发布频率限制：记录每个传感器上次发布时间
  std::vector<std::chrono::steady_clock::time_point> last_publish_time_;
  std::mutex publish_rate_mutex_;  // 保护发布频率限制

  // 挖掘机相对角发布器（Vector3: x=大臂对车身, y=斗杆对大臂, z=铲斗对斗杆）
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr excavator_angles_pub_;

  // 记录 /tilt_body /tilt_boom /tilt_stick /tilt_bucket 在 publishers_ 中的索引
  // 若某个索引为 -1，表示当前未配置该传感器。
  int body_index_{-1};
  int boom_index_{-1};
  int stick_index_{-1};
  int bucket_index_{-1};

  // 挖掘机各部位当前 PITCH 绝对角（单位：度）
  double body_pitch_deg_{0.0};
  double boom_pitch_deg_{0.0};
  double stick_pitch_deg_{0.0};
  double bucket_pitch_deg_{0.0};

  // 车身倾角仪 PITCH 零点偏置（单位：度），从参数加载（用于内部相对角计算）
  double body_pitch_offset_deg_{0.0};
  // 仅用于修正 /tilt_body 话题中 pitch 的零点（不会影响相对角计算）
  double body_topic_pitch_offset_deg_{0.0};

  // 相对角零点偏置（单位：度），从参数加载
  double boom_rel_body_zero_deg_{0.0};
  double stick_rel_boom_zero_deg_{0.0};
  double bucket_rel_stick_zero_deg_{0.0};

  // CAN 相关
  std::string can_interface_;
  // CAN 波特率，仅用于参数记录和日志输出，实际波特率建议在系统层面配置
  int can_bitrate_{0};
  // 是否启用周期请求帧（为 false 时完全被动监听）
  bool enable_request_{true};
  // 请求帧基础 CAN ID，例如 0x600，实际请求 ID = request_base_can_id_ + node_id
  uint32_t request_base_can_id_{0x600};
  // 响应帧基础 CAN ID，例如 0x580，实际响应 ID = response_base_can_id_ + node_id
  uint32_t response_base_can_id_{0x580};
  int can_fd_{-1};
  std::atomic<bool> running_{false};
  std::thread recv_thread_;
  std::mutex can_send_mutex_;  // 发送加锁，避免多线程竞争

  // 定时器
  rclcpp::TimerBase::SharedPtr request_timer_;
};

