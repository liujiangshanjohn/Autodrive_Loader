#include "danfoss_rotary_encoder/danfoss_rotary_encoder_node.hpp"

#include <chrono>
#include <cstring>
#include <iostream>

using namespace std::chrono_literals;

// 构造函数：初始化参数、CAN 设备、接收线程
DanfossRotaryEncoderNode::DanfossRotaryEncoderNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("danfoss_rotary_encoder_node", options)
{
  // 声明参数（带默认值）
  this->declare_parameter<std::string>("can_interface", "can1");
  // CAN 波特率（仅用于记录和日志，不主动修改系统网络配置）
  this->declare_parameter<int>("can_bitrate", 250000);  // 默认 250K
  // CAN ID 列表（可配置，例如 [0x18FF0B15]）
  this->declare_parameter<std::vector<int64_t>>("can_ids", {0x18FF0B15});  // 默认 PGN 65291, 源地址 0x15
  // 对应的 ROS2 话题名
  this->declare_parameter<std::vector<std::string>>(
    "encoder_topics",
    {"/rotary_encoder_angle"});
  // 发布频率限制（毫秒），默认 20ms（50Hz）
  this->declare_parameter<int>("publish_period_ms", 20);
  // 零点偏置值（度），用于校准编码器零点，默认 0.0
  // 可以为每个编码器单独配置，数组长度应与 can_ids 一致
  this->declare_parameter<std::vector<double>>("angle1_offsets", std::vector<double>{});
  this->declare_parameter<std::vector<double>>("angle2_offsets", std::vector<double>{});

  init_parameters();

  if (!init_can_socket()) {
    // RCLCPP_FATAL(this->get_logger(), "初始化 CAN 设备失败");
    throw std::runtime_error("failed to init CAN");
  }

  // 启动接收线程
  running_.store(true);
  recv_thread_ = std::thread(&DanfossRotaryEncoderNode::recv_loop, this);

  // RCLCPP_INFO(this->get_logger(),
  //   "danfoss_rotary_encoder_node 启动，CAN 接口=%s，编码器数量=%zu",
  //   can_interface_.c_str(), encoder_configs_.size());
}

DanfossRotaryEncoderNode::~DanfossRotaryEncoderNode()
{
  running_.store(false);
  if (recv_thread_.joinable()) {
    recv_thread_.join();
  }
  if (can_fd_ >= 0) {
    ::close(can_fd_);
    can_fd_ = -1;
  }
}

void DanfossRotaryEncoderNode::init_parameters()
{
  // 读取 CAN 接口名
  can_interface_ = this->get_parameter("can_interface").as_string();

  // 读取 CAN 波特率（不直接在代码中配置 CAN 口，仅用于记录）
  if (this->has_parameter("can_bitrate")) {
    can_bitrate_ = this->get_parameter("can_bitrate").as_int();
  } else {
    can_bitrate_ = 0;
  }

  // 读取 CAN ID 与话题名列表
  auto can_ids = this->get_parameter("can_ids").as_integer_array();
  auto topics = this->get_parameter("encoder_topics").as_string_array();
  
  // 读取零点偏置值
  std::vector<double> angle1_offsets;
  std::vector<double> angle2_offsets;
  if (this->has_parameter("angle1_offsets")) {
    angle1_offsets = this->get_parameter("angle1_offsets").as_double_array();
  }
  if (this->has_parameter("angle2_offsets")) {
    angle2_offsets = this->get_parameter("angle2_offsets").as_double_array();
  }

  if (can_ids.size() != topics.size()) {
    // RCLCPP_WARN(this->get_logger(),
    //   "参数 can_ids 与 encoder_topics 长度不一致，将按照最小长度截断");
  }

  size_t count = std::min(can_ids.size(), topics.size());
  encoder_configs_.clear();
  publishers_.clear();
  can_id_to_index_.clear();
  last_publish_time_.clear();

  for (size_t i = 0; i < count; ++i) {
    if (can_ids[i] < 0 || can_ids[i] > 0x7FFFFFFF) {
      // RCLCPP_WARN(this->get_logger(),
      //   "CAN ID %ld 非法（必须 0~0x7FFFFFFF），跳过", static_cast<long>(can_ids[i]));
      continue;
    }
    RotaryEncoderConfig cfg;
    cfg.can_id = static_cast<uint32_t>(can_ids[i]);
    cfg.topic_name = topics[i];
    // 读取零点偏置值（如果配置了）
    if (i < angle1_offsets.size()) {
      cfg.angle1_offset = angle1_offsets[i];
    }
    if (i < angle2_offsets.size()) {
      cfg.angle2_offset = angle2_offsets[i];
    }
    encoder_configs_.push_back(cfg);

    // 创建 ROS2 发布器，使用自定义消息 RotaryEncoderAngle
    // 增加队列大小以减少消息丢失
    rclcpp::QoS qos_profile(50);
    qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
    qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    auto pub = this->create_publisher<cannode::msg::RotaryEncoderAngle>(
      cfg.topic_name, qos_profile);
    publishers_.push_back(pub);

    // 建立 CAN ID 到索引的映射
    can_id_to_index_[cfg.can_id] = publishers_.size() - 1;
    
    // 初始化该编码器的上次发布时间
    last_publish_time_.push_back(std::chrono::steady_clock::time_point{});
  }

  if (encoder_configs_.empty()) {
    // RCLCPP_WARN(this->get_logger(),
    //   "未配置任何有效的回转编码器，节点将不会解析任何数据");
  }

  // 输出零点偏置信息
  for (size_t i = 0; i < encoder_configs_.size(); ++i) {
    const auto & cfg = encoder_configs_[i];
    if (cfg.angle1_offset != 0.0 || cfg.angle2_offset != 0.0) {
      // RCLCPP_INFO(this->get_logger(),
      //   "编码器[%zu] CAN ID=0x%X: Angle1偏置=%.2f°, Angle2偏置=%.2f°",
      //   i, cfg.can_id, cfg.angle1_offset, cfg.angle2_offset);
    }
  }
  
  // RCLCPP_INFO(this->get_logger(),
  //   "参数初始化完成：CAN 接口=%s，CAN 波特率=%d，编码器数量=%zu",
  //   can_interface_.c_str(), can_bitrate_,
  //   encoder_configs_.size());
}

bool DanfossRotaryEncoderNode::init_can_socket()
{
  // 创建 RAW CAN 套接字
  can_fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (can_fd_ < 0) {
    // RCLCPP_ERROR(this->get_logger(), "创建 CAN 套接字失败");
    return false;
  }

  // 绑定到指定 CAN 接口（如 can1）
  struct ifreq ifr;
  std::memset(&ifr, 0, sizeof(ifr));
  std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
  if (ioctl(can_fd_, SIOCGIFINDEX, &ifr) < 0) {
    // RCLCPP_ERROR(this->get_logger(), "获取 CAN 接口索引失败: %s", can_interface_.c_str());
    ::close(can_fd_);
    can_fd_ = -1;
    return false;
  }

  struct sockaddr_can addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(can_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    // RCLCPP_ERROR(this->get_logger(), "绑定 CAN 设备失败: %s", can_interface_.c_str());
    ::close(can_fd_);
    can_fd_ = -1;
    return false;
  }

  // 这里假定用户已经通过 ip link / ifconfig 将 CAN 接口设置为指定波特率并 up
  // RCLCPP_INFO(this->get_logger(), "CAN 套接字初始化完成，接口=%s", can_interface_.c_str());
  return true;
}

void DanfossRotaryEncoderNode::recv_loop()
{
  // 接收线程：阻塞等待 CAN 帧，按 CAN ID 分发给对应编码器
  while (rclcpp::ok() && running_.load()) {
    if (can_fd_ < 0) {
      std::this_thread::sleep_for(100ms);
      continue;
    }

    struct can_frame frame;
    ssize_t nbytes = ::read(can_fd_, &frame, sizeof(frame));
    if (nbytes < 0) {
      // 读取错误时简单等待一段时间重试
      std::this_thread::sleep_for(10ms);
      continue;
    }
    if (nbytes < static_cast<ssize_t>(sizeof(struct can_frame))) {
      // 非完整帧，忽略
      continue;
    }

    // 处理标准帧和扩展帧
    handle_encoder_frame(frame);
  }
}

void DanfossRotaryEncoderNode::handle_encoder_frame(const struct can_frame & frame)
{
  // 检查是否为配置的编码器 CAN ID（PGN 65291: 0x18FF0BXX）
  // 根据帧类型提取 CAN ID：标准帧使用 CAN_SFF_MASK，扩展帧使用 CAN_EFF_MASK
  uint32_t can_id;
  if (frame.can_id & CAN_EFF_FLAG) {
    can_id = frame.can_id & CAN_EFF_MASK;
  } else {
    can_id = frame.can_id & CAN_SFF_MASK;
  }
  auto it = can_id_to_index_.find(can_id);
  if (it == can_id_to_index_.end()) {
    // 非本节点关心的帧
    return;
  }

  if (frame.can_dlc != 8) {
    // 丹佛斯回转编码器数据帧应为 8 字节
    // RCLCPP_WARN(this->get_logger(), "ID 0x%X DLC=%d 非 8 字节，忽略",
    //   can_id, frame.can_dlc);
    return;
  }

  // 按手册解析 PGN 65291 数据：
  // Byte 0-1: Angle 1 (MSB, LSB) - 16-bit unsigned integer，分辨率 0.1 deg
  // Byte 2-3: Angle 2 (MSB, LSB) - 16-bit unsigned integer，分辨率 0.1 deg
  // Byte 4-6: Reserved (24 bits)
  // Byte 7: Error Code

  // 检查错误标志：如果 Angle 1 和 Angle 2 的 MSB 和 LSB 都是 0xFF，表示错误
  if (frame.data[0] == 0xFF && frame.data[1] == 0xFF &&
      frame.data[2] == 0xFF && frame.data[3] == 0xFF) {
    // RCLCPP_WARN(this->get_logger(), "ID 0x%X 检测到错误标志（0xFF 0xFF 0xFF 0xFF）", can_id);
    // 仍然发布，但角度值无效
  }

  double angle1_deg = decode_angle_16bit(&frame.data[0]);
  double angle2_deg = decode_angle_16bit(&frame.data[2]);
  uint8_t error_code = frame.data[7];

  // 使用 angle1/angle2/error_code 三个字段发布
  size_t index = it->second;
  if (index >= publishers_.size() || index >= encoder_configs_.size()) {
    return;
  }
  
  // 应用零点偏置值
  const auto & cfg = encoder_configs_[index];
  angle1_deg += cfg.angle1_offset;
  angle2_deg += cfg.angle2_offset;

  // 发布频率限制
  int publish_period_ms = 20;  // 默认 20ms
  if (this->has_parameter("publish_period_ms")) {
    publish_period_ms = this->get_parameter("publish_period_ms").as_int();
    if (publish_period_ms <= 0) {
      publish_period_ms = 20;
    }
  }

  auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lk(publish_rate_mutex_);
  
  // 确保 last_publish_time_ 大小足够
  if (index >= last_publish_time_.size()) {
    last_publish_time_.resize(index + 1, std::chrono::steady_clock::time_point{});
  }
  
  auto time_since_last = std::chrono::duration_cast<std::chrono::milliseconds>(
    now - last_publish_time_[index]);
  
  // 如果距离上次发布时间不足指定周期，跳过本次发布
  if (time_since_last.count() < publish_period_ms) {
    return;
  }
  
  last_publish_time_[index] = now;

  cannode::msg::RotaryEncoderAngle msg;
  // angle1 用 360 度减去原值
  msg.angle1 = static_cast<float>(360.0 - angle1_deg);
  msg.angle2 = static_cast<float>(angle2_deg);
  msg.error_code = error_code;

  publishers_[index]->publish(msg);

  // 可选：输出调试信息
  if (error_code != 0) {
    // RCLCPP_WARN(this->get_logger(), 
    //   "ID 0x%X 错误代码: 0x%02X (Angle1=%.1f°, Angle2=%.1f°)",
    //   can_id, error_code, angle1_deg, angle2_deg);
  }
}

double DanfossRotaryEncoderNode::decode_angle_16bit(const uint8_t * data)
{
  // 解析 16-bit unsigned integer，分辨率 0.1 deg
  // 格式：MSB (data[0]), LSB (data[1])
  // 例如：008Ah = 138 = 13.8 deg
  
  uint16_t raw_value = (static_cast<uint16_t>(data[0]) << 8) | static_cast<uint16_t>(data[1]);
  
  // 检查是否为错误标志（0xFFFF）
  if (raw_value == 0xFFFF) {
    return 0.0;  // 返回 0 表示无效值
  }
  
  // 转换为角度（度）：raw_value * 0.1
  return static_cast<double>(raw_value) * 0.1;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DanfossRotaryEncoderNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

