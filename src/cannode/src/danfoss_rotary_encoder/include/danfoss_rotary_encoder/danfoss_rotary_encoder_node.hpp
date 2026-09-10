#pragma once

#include <rclcpp/rclcpp.hpp>
#include "cannode/msg/rotary_encoder_angle.hpp"

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

struct RotaryEncoderConfig
{
  // CAN ID（可配置，例如 0x18FF0B15）
  uint32_t can_id;
  // 对应的 ROS2 话题名
  std::string topic_name;
  // 零点偏置值（度），用于校准编码器零点
  double angle1_offset{0.0};  // Angle1 零点偏置
  double angle2_offset{0.0};  // Angle2 零点偏置
};

class DanfossRotaryEncoderNode : public rclcpp::Node
{
public:
  explicit DanfossRotaryEncoderNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~DanfossRotaryEncoderNode() override;

private:
  // 初始化参数与 CAN 设备
  void init_parameters();
  bool init_can_socket();

  // 接收线程函数
  void recv_loop();

  // 解析一帧丹佛斯回转编码器数据并发布
  void handle_encoder_frame(const struct can_frame & frame);

  // 解析 16-bit unsigned integer 角度值（分辨率 0.1 deg）
  static double decode_angle_16bit(const uint8_t * data);

  // ROS 相关
  std::vector<RotaryEncoderConfig> encoder_configs_;
  std::map<uint32_t, size_t> can_id_to_index_;
  // 发布回转编码器角度，消息中字段为 angle1/angle2/error_code
  std::vector<rclcpp::Publisher<cannode::msg::RotaryEncoderAngle>::SharedPtr> publishers_;
  // 发布频率限制：记录每个传感器上次发布时间
  std::vector<std::chrono::steady_clock::time_point> last_publish_time_;
  std::mutex publish_rate_mutex_;  // 保护发布频率限制

  // CAN 相关
  std::string can_interface_;
  // CAN 波特率，仅用于参数记录和日志输出，实际波特率建议在系统层面配置
  int can_bitrate_{0};
  int can_fd_{-1};
  std::atomic<bool> running_{false};
  std::thread recv_thread_;
};

