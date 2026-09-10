#include "multi_tilt_imu/multi_tilt_imu_node.hpp"

#include <chrono>
#include <cstring>
#include <iostream>

using namespace std::chrono_literals;

// 构造函数：初始化参数、CAN 设备、定时器与接收线程
MultiTiltImuNode::MultiTiltImuNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("multi_tilt_imu_node", options)
{
  // 声明参数（带默认值）
  this->declare_parameter<std::string>("can_interface", "can1");
  // CAN 波特率（仅用于记录和日志，不主动修改系统网络配置）
  this->declare_parameter<int>("can_bitrate", 250000);
  // 请求 / 响应基础 CAN ID，默认 0x600 / 0x580
  this->declare_parameter<int>("request_base_can_id", 0x600);
  this->declare_parameter<int>("response_base_can_id", 0x580);
  this->declare_parameter<std::vector<int64_t>>("sensor_ids", {5});
  this->declare_parameter<std::vector<std::string>>(
    "sensor_topics",
    {"/tilt_sensor_5"});
  this->declare_parameter<int>("request_period_ms", 50);  // 请求周期，默认 20Hz
  this->declare_parameter<bool>("enable_request", true);  // 是否启用周期请求

  // 车身倾角仪零点偏置（单位：度）
  this->declare_parameter<double>("body_pitch_offset_deg", 0.0);
  // 仅用于修正 /tilt_body 话题中 pitch 的零点（不影响相对角计算）
  this->declare_parameter<double>("body_topic_pitch_offset_deg", 0.0);
  
  // 挖掘机相对角零点配置（单位：度）
  this->declare_parameter<double>("boom_rel_body_zero_deg", 0.0);
  this->declare_parameter<double>("stick_rel_boom_zero_deg", 0.0);
  this->declare_parameter<double>("bucket_rel_stick_zero_deg", 0.0);

  init_parameters();

  if (!init_can_socket()) {
    // RCLCPP_FATAL(this->get_logger(), "初始化 CAN 设备失败");
    throw std::runtime_error("failed to init CAN");
  }

  // 创建定时器，周期发送读取三轴角度命令（可配置关闭）
  int req_period_ms =
    this->get_parameter("request_period_ms").get_value<int>();
  if (req_period_ms <= 0) {
    req_period_ms = 50;
  }

  if (enable_request_) {
    request_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(req_period_ms),
      std::bind(&MultiTiltImuNode::send_request_timer_callback, this));
  } else {
    // RCLCPP_INFO(this->get_logger(), "已关闭周期请求功能，仅被动监听倾角仪数据");
  }

  // 启动接收线程
  running_.store(true);
  recv_thread_ = std::thread(&MultiTiltImuNode::recv_loop, this);

  // RCLCPP_INFO(this->get_logger(),
  //   "multi_tilt_imu_node 启动，CAN 接口=%s，传感器数量=%zu",
  //   can_interface_.c_str(), sensor_configs_.size());
}

MultiTiltImuNode::~MultiTiltImuNode()
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

void MultiTiltImuNode::init_parameters()
{
  // 读取 CAN 接口名
  can_interface_ = this->get_parameter("can_interface").as_string();

  // 读取 CAN 波特率（不直接在代码中配置 CAN 口，仅用于记录）
  if (this->has_parameter("can_bitrate")) {
    can_bitrate_ = this->get_parameter("can_bitrate").as_int();
  } else {
    can_bitrate_ = 0;
  }

  // 读取是否启用请求
  enable_request_ = this->get_parameter("enable_request").as_bool();

  // 读取请求 / 响应基础 CAN ID
  request_base_can_id_ =
    static_cast<uint32_t>(this->get_parameter("request_base_can_id").as_int());
  response_base_can_id_ =
    static_cast<uint32_t>(this->get_parameter("response_base_can_id").as_int());

  // 读取车身倾角仪 PITCH 零点偏置
  body_pitch_offset_deg_ =
    this->get_parameter("body_pitch_offset_deg").get_value<double>();
  // 读取仅作用于 /tilt_body 话题的 PITCH 偏置
  body_topic_pitch_offset_deg_ =
    this->get_parameter("body_topic_pitch_offset_deg").get_value<double>();
  
  // 读取挖掘机相对角零点配置
  boom_rel_body_zero_deg_ =
    this->get_parameter("boom_rel_body_zero_deg").get_value<double>();
  stick_rel_boom_zero_deg_ =
    this->get_parameter("stick_rel_boom_zero_deg").get_value<double>();
  bucket_rel_stick_zero_deg_ =
    this->get_parameter("bucket_rel_stick_zero_deg").get_value<double>();

  // 读取传感器节点 ID 与话题名列表
  auto ids = this->get_parameter("sensor_ids").as_integer_array();
  auto topics = this->get_parameter("sensor_topics").as_string_array();

  if (ids.size() != topics.size()) {
    // RCLCPP_WARN(this->get_logger(),
    //   "参数 sensor_ids 与 sensor_topics 长度不一致，将按照最小长度截断");
  }

  size_t count = std::min(ids.size(), topics.size());
  sensor_configs_.clear();
  publishers_.clear();
  resp_id_to_index_.clear();
  last_publish_time_.clear();

  // 重置索引
  body_index_ = -1;
  boom_index_ = -1;
  stick_index_ = -1;
  bucket_index_ = -1;

  for (size_t i = 0; i < count; ++i) {
    if (ids[i] < 0 || ids[i] > 0x7F) {
      // RCLCPP_WARN(this->get_logger(),
      //   "传感器 ID %ld 非法（必须 0~127），跳过", static_cast<long>(ids[i]));
      continue;
    }
    TiltSensorConfig cfg;
    cfg.node_id = static_cast<uint8_t>(ids[i]);
    cfg.topic_name = topics[i];
    sensor_configs_.push_back(cfg);

    // 创建 ROS2 发布器，使用自定义消息 TiltRPY，字段为 roll/pitch/yaw
    // 增加队列大小以减少消息丢失（20Hz 发布频率，队列大小设为 50 可缓存约 2.5 秒的数据）
    rclcpp::QoS qos_profile(50);
    qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
    qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    auto pub = this->create_publisher<cannode::msg::TiltRPY>(
      cfg.topic_name, qos_profile);
    publishers_.push_back(pub);

    // 建立 CAN 响应 ID 到索引的映射：response_base_can_id_ + node_id
    uint32_t resp_id = response_base_can_id_ + cfg.node_id;
    resp_id_to_index_[resp_id] = publishers_.size() - 1;
    
    // 初始化该传感器的上次发布时间
    last_publish_time_.push_back(std::chrono::steady_clock::time_point{});

    // 根据话题名记录 body/boom/stick/bucket 对应的索引（仅按默认命名匹配）
    size_t idx = publishers_.size() - 1;
    if (cfg.topic_name == "/tilt_body") {
      body_index_ = static_cast<int>(idx);
    } else if (cfg.topic_name == "/tilt_boom") {
      boom_index_ = static_cast<int>(idx);
    } else if (cfg.topic_name == "/tilt_stick") {
      stick_index_ = static_cast<int>(idx);
    } else if (cfg.topic_name == "/tilt_bucket") {
      bucket_index_ = static_cast<int>(idx);
    }
  }

  if (sensor_configs_.empty()) {
    // RCLCPP_WARN(this->get_logger(),
    //   "未配置任何有效的倾角仪传感器，节点将不会解析任何数据");
  }

  // RCLCPP_INFO(this->get_logger(),
  //   "参数初始化完成：CAN 接口=%s，CAN 波特率=%d，请求基 ID=0x%X，响应基 ID=0x%X，倾角仪数量=%zu",
  //   can_interface_.c_str(), can_bitrate_,
  //   request_base_can_id_, response_base_can_id_,
  //   sensor_configs_.size());

  // 创建挖掘机相对角发布器（始终创建，若后续未正确配置传感器索引则仅不发布数据）
  if (!excavator_angles_pub_) {
    excavator_angles_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(
      "/excavator_joint_angles", 10);
  }
}

bool MultiTiltImuNode::init_can_socket()
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

  // 这里假定用户已经通过 ip link / ifconfig 将 can1 设置为 250K 波特率并 up
  // RCLCPP_INFO(this->get_logger(), "CAN 套接字初始化完成，接口=%s", can_interface_.c_str());
  return true;
}

void MultiTiltImuNode::send_request_timer_callback()
{
  if (can_fd_ < 0) {
    return;
  }

  std::lock_guard<std::mutex> lk(can_send_mutex_);
  for (const auto & cfg : sensor_configs_) {
    struct can_frame frame;
    std::memset(&frame, 0, sizeof(frame));
    // 请求帧 ID：request_base_can_id_ + node_id
    frame.can_id = request_base_can_id_ + cfg.node_id;
    frame.can_dlc = 8;
    // 按手册发送"读取三轴角度 PITCH、ROLL、YAW"命令
    frame.data[0] = 0x40;
    frame.data[1] = 0x04;
    frame.data[2] = 0x10;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    ssize_t nbytes = ::write(can_fd_, &frame, sizeof(frame));
    if (nbytes != static_cast<ssize_t>(sizeof(frame))) {
      // RCLCPP_WARN(this->get_logger(), "发送读取命令到 ID 0x%X 失败", frame.can_id);
    }
  }
}

void MultiTiltImuNode::recv_loop()
{
  // 接收线程：阻塞等待 CAN 帧，按响应 ID 分发给对应传感器
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

    // 只处理标准帧（本项目使用标准帧）
    if (frame.can_id & CAN_EFF_FLAG) {
      continue;
    }

    handle_tilt_frame(frame);
  }
}

void MultiTiltImuNode::handle_tilt_frame(const struct can_frame & frame)
{
  // 检查是否为配置的倾角仪响应 ID
  uint32_t can_id = frame.can_id & CAN_SFF_MASK;
  auto it = resp_id_to_index_.find(can_id);
  if (it == resp_id_to_index_.end()) {
    // 非本节点关心的帧
    return;
  }

  if (frame.can_dlc != 8) {
    // 倾角仪数据帧应为 8 字节
    // RCLCPP_WARN(this->get_logger(), "ID 0x%X DLC=%d 非 8 字节，忽略",
    //   can_id, frame.can_dlc);
    return;
  }

  // 按手册解析：D1-D3 为 PITCH，D4-D6 为 ROLL，D7-D8 为 YAW
  double pitch_deg = 0.0;
  double roll_deg = 0.0;
  double yaw_deg = 0.0;

  if (!decode_bcd_angle_3bytes(&frame.data[0], pitch_deg)) {
    // RCLCPP_WARN(this->get_logger(), "ID 0x%X 解析 PITCH 失败", can_id);
    return;
  }
  if (!decode_bcd_angle_3bytes(&frame.data[3], roll_deg)) {
    // RCLCPP_WARN(this->get_logger(), "ID 0x%X 解析 ROLL 失败", can_id);
    return;
  }
  if (!decode_bcd_angle_2bytes(&frame.data[6], yaw_deg)) {
    // RCLCPP_WARN(this->get_logger(), "ID 0x%X 解析 YAW 失败", can_id);
    return;
  }

  // 使用 roll/pitch/yaw 三个字段发布
  size_t index = it->second;
  if (index >= publishers_.size()) {
    return;
  }

  // 发布频率限制：每20ms最多发布一次（50Hz）
  auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lk(publish_rate_mutex_);
  
  // 确保 last_publish_time_ 大小足够
  if (index >= last_publish_time_.size()) {
    last_publish_time_.resize(index + 1, std::chrono::steady_clock::time_point{});
  }
  
  auto time_since_last = std::chrono::duration_cast<std::chrono::milliseconds>(
    now - last_publish_time_[index]);
  
  // 如果距离上次发布时间不足20ms，跳过本次发布
  if (time_since_last.count() < 20) {
    return;
  }
  
  last_publish_time_[index] = now;

  cannode::msg::TiltRPY msg;
  msg.roll = static_cast<float>(roll_deg);
  // 对 /tilt_body 话题的 pitch 应用单独的零点偏置，不影响内部相对角计算
  if (static_cast<int>(index) == body_index_) {
    msg.pitch = static_cast<float>(pitch_deg + body_topic_pitch_offset_deg_);
  } else {
    msg.pitch = static_cast<float>(pitch_deg);
  }
  msg.yaw = static_cast<float>(yaw_deg);

  publishers_[index]->publish(msg);

  // 更新挖掘机各部位的绝对 PITCH（俯仰角），并计算相对角
  // 注意：使用 PITCH 作为结构角度
  if (static_cast<int>(index) == body_index_) {
    // 应用车身倾角仪的 PITCH 零点偏置（仅对车身倾角仪应用）
    body_pitch_deg_ = pitch_deg + body_pitch_offset_deg_;
  } else if (static_cast<int>(index) == boom_index_) {
    boom_pitch_deg_ = pitch_deg;
  } else if (static_cast<int>(index) == stick_index_) {
    stick_pitch_deg_ = pitch_deg;
  } else if (static_cast<int>(index) == bucket_index_) {
    bucket_pitch_deg_ = pitch_deg;
  }

  update_excavator_relative_angles();
}

void MultiTiltImuNode::update_excavator_relative_angles()
{
  // 确保至少 body、boom、stick 三个传感器索引有效（bucket 可选）
  if (body_index_ < 0 || boom_index_ < 0 || stick_index_ < 0) {
    return;
  }

  if (!excavator_angles_pub_) {
    return;
  }

  // 计算相对角（度），使用 PITCH 角度
  double boom_rel_body_deg =
    (boom_pitch_deg_ - body_pitch_deg_) + boom_rel_body_zero_deg_;
  double stick_rel_boom_deg =
    (stick_pitch_deg_ - boom_pitch_deg_) + stick_rel_boom_zero_deg_;
  
  // 如果配置了铲斗倾角仪，计算铲斗相对斗杆的角度；否则设为 0
  double bucket_rel_stick_deg = 0.0;
  if (bucket_index_ >= 0) {
    bucket_rel_stick_deg =
      (bucket_pitch_deg_ - stick_pitch_deg_) + bucket_rel_stick_zero_deg_;
  }

  geometry_msgs::msg::Vector3 joints;
  joints.x = static_cast<double>(boom_rel_body_deg);
  joints.y = static_cast<double>(stick_rel_boom_deg);
  joints.z = static_cast<double>(bucket_rel_stick_deg);

  excavator_angles_pub_->publish(joints);
}

bool MultiTiltImuNode::decode_bcd_angle_3bytes(const uint8_t * data, double & angle_deg)
{
  // 手册说明：3 字节压缩 BCD，格式为 SX XX YY
  // S 为符号位（0=正，1=负），XXX 为三位整数，YY 为两位小数
  // 每个 4bit 为 1 个 BCD 数字：byte0 高 4bit 为 S，低 4bit 为百位
  // byte1 高 4bit 为十位，低 4bit 为个位
  // byte2 高 4bit 为十分位，低 4bit 为百分位

  uint8_t b0 = data[0];
  uint8_t b1 = data[1];
  uint8_t b2 = data[2];

  uint8_t sign = (b0 & 0xF0) >> 4;
  uint8_t h = b0 & 0x0F;
  uint8_t t = (b1 & 0xF0) >> 4;
  uint8_t o = b1 & 0x0F;
  uint8_t d1 = (b2 & 0xF0) >> 4;
  uint8_t d2 = b2 & 0x0F;

  // 检查是否为合法 BCD 数字
  auto valid_digit = [](uint8_t d) { return d <= 9; };
  if (!valid_digit(h) || !valid_digit(t) || !valid_digit(o) ||
    !valid_digit(d1) || !valid_digit(d2))
  {
    return false;
  }

  double integer_part = static_cast<double>(h) * 100.0 +
    static_cast<double>(t) * 10.0 +
    static_cast<double>(o);
  double decimal_part = static_cast<double>(d1) * 0.1 +
    static_cast<double>(d2) * 0.01;
  angle_deg = integer_part + decimal_part;

  if (sign == 1) {
    angle_deg = -angle_deg;
  }

  return true;
}

bool MultiTiltImuNode::decode_bcd_angle_2bytes(const uint8_t * data, double & angle_deg)
{
  // 手册中 YAW 字段为 2 字节，说明"同 PITCH"，但未明确小数位。
  // 这里按常见实现假定格式为 SX XX，即符号 + 三位整数，无小数。

  uint8_t b0 = data[0];
  uint8_t b1 = data[1];

  uint8_t sign = (b0 & 0xF0) >> 4;
  uint8_t h = b0 & 0x0F;
  uint8_t t = (b1 & 0xF0) >> 4;
  uint8_t o = b1 & 0x0F;

  auto valid_digit = [](uint8_t d) { return d <= 9; };
  if (!valid_digit(h) || !valid_digit(t) || !valid_digit(o)) {
    return false;
  }

  double integer_part = static_cast<double>(h) * 100.0 +
    static_cast<double>(t) * 10.0 +
    static_cast<double>(o);
  angle_deg = integer_part;
  if (sign == 1) {
    angle_deg = -angle_deg;
  }
  return true;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MultiTiltImuNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

