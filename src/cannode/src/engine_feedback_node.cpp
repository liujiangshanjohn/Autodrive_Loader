#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <linux/can.h>
#include <sys/time.h>

#include <chrono>
#include <cmath>
#include <limits>
#include <mutex>
#include <sstream>
#include <string>

#include "socket_can_interface.h"

namespace cannode
{

class EngineFeedbackNode : public rclcpp::Node
{
public:
  EngineFeedbackNode()
  : Node("engine_feedback_node"),
    can_interface_(this->declare_parameter<std::string>("can_interface", "can1")),
    can_iface_(can_interface_)
  {
    engine_speed_stale_timeout_sec_ =
      this->declare_parameter<double>("engine_speed_stale_timeout_sec", 5.0);
    et1_stale_timeout_sec_ =
      this->declare_parameter<double>("et1_stale_timeout_sec", 20.0);

    if (!(engine_speed_stale_timeout_sec_ > 0.0) ||
      !std::isfinite(engine_speed_stale_timeout_sec_))
    {
      RCLCPP_WARN(
        this->get_logger(),
        "engine_speed_stale_timeout_sec=%.6g 非法，改用 5.0",
        engine_speed_stale_timeout_sec_);
      engine_speed_stale_timeout_sec_ = 5.0;
    }
    if (!(et1_stale_timeout_sec_ > 0.0) || !std::isfinite(et1_stale_timeout_sec_)) {
      RCLCPP_WARN(
        this->get_logger(),
        "et1_stale_timeout_sec=%.6g 非法，改用 20.0",
        et1_stale_timeout_sec_);
      et1_stale_timeout_sec_ = 20.0;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "EngineFeedbackNode started, interface=%s engine_speed_stale_timeout_sec=%.3f "
      "et1_stale_timeout_sec=%.3f（超时则 rpm 或水温/油温=-1）",
      can_interface_.c_str(),
      engine_speed_stale_timeout_sec_, et1_stale_timeout_sec_);

    // 发布到 /cannode/engine_feedback，使用可靠 QoS，方便前端订阅
    rclcpp::QoS qos(10);
    qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
    qos.durability(rclcpp::DurabilityPolicy::Volatile);
    qos.history(rclcpp::HistoryPolicy::KeepLast);

    feedback_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/cannode/engine_feedback", qos);

    running_ = true;
    recv_thread_ = std::thread(&EngineFeedbackNode::recvLoop, this);
  }

  ~EngineFeedbackNode() override
  {
    running_ = false;
    if (recv_thread_.joinable())
    {
      recv_thread_.join();
    }
  }

private:
  void recvLoop()
  {
    struct timeval tv{};
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    (void)can_iface_.SetRecvTimeout(tv);

    can_frame frame{};
    struct timeval ts{};
    std::int32_t read_bytes = 0;

    while (rclcpp::ok() && running_)
    {
      int ret = can_iface_.ReadCan(frame, ts, read_bytes);
      if (ret != 0 || read_bytes <= 0)
      {
        continue;
      }

      uint32_t can_id = frame.can_id & CAN_EFF_MASK;

      bool updated = false;

      switch (can_id)
      {
        // 0x0CF00400 Engine_Speed
        case 0x0CF00400:
          updated |= handleEngineSpeed(frame);
          break;

        // 0x18FEE500 Accumulated_running_time
        case 0x18FEE500:
          updated |= handleAccumulatedRunningTime(frame);
          break;

        // 0x18FEEE00 ET1 (coolant temp, oil temp)
        case 0x18FEEE00:
          updated |= handleTemperatures(frame);
          break;

        // 0x18FEEF00 EFL/P1 (oil pressure, coolant level)
        case 0x18FEEF00:
          updated |= handleOilPressureAndCoolantLevel(frame);
          break;

        default:
          break;
      }

      if (updated)
      {
        publishFeedback();
      }
    }
  }

  // Engine_Speed, PGN 0xF004, ID 0x0CF00400
  bool handleEngineSpeed(const can_frame &frame)
  {
    // J1939 标准：发动机转速在 byte 3-4（起始 bit 24），分辨率 0.125 rpm/bit
    if (frame.can_dlc < 5)
    {
      return false;
    }
    uint16_t raw = static_cast<uint16_t>(frame.data[3]) |
                   (static_cast<uint16_t>(frame.data[4]) << 8);
    if (raw == 0xFFFF)
    {
      return false;  // Not available
    }

    double rpm = raw * 0.125;  // factor

    std::lock_guard<std::mutex> lk(data_mutex_);
    engine_speed_rpm_ = rpm;
    last_engine_speed_frame_steady_ = std::chrono::steady_clock::now();
    return true;
  }

  // Accumulated_running_time, PGN 0xFEE5, ID 0x18FEE500
  bool handleAccumulatedRunningTime(const can_frame &frame)
  {
    if (frame.can_dlc < 8)
    {
      return false;
    }
    uint32_t raw = static_cast<uint32_t>(frame.data[0]) |
                   (static_cast<uint32_t>(frame.data[1]) << 8) |
                   (static_cast<uint32_t>(frame.data[2]) << 16) |
                   (static_cast<uint32_t>(frame.data[3]) << 24);
    if (raw == 0xFFFFFFFFu)
    {
      return false;
    }

    double hours = raw * 0.05;  // factor: 0.05 h/bit

    std::lock_guard<std::mutex> lk(data_mutex_);
    accumulated_running_time_h_ = hours;
    return true;
  }

  // ET1, ID 0x18FEEE00: coolant temp, engine oil temp
  bool handleTemperatures(const can_frame &frame)
  {
    if (frame.can_dlc < 6)
    {
      return false;
    }

    const auto now = std::chrono::steady_clock::now();
    const uint8_t raw_coolant = frame.data[0];
    const uint16_t raw_oil = static_cast<uint16_t>(frame.data[2]) |
      (static_cast<uint16_t>(frame.data[3]) << 8);

    bool updated = false;
    std::lock_guard<std::mutex> lk(data_mutex_);

    // Coolant temperature, 1 °C/bit, offset -40 °C, byte 0
    if (raw_coolant != 0xFF)
    {
      coolant_temp_degC_ = static_cast<double>(static_cast<int>(raw_coolant)) - 40.0;
      last_coolant_valid_steady_ = now;
      updated = true;
    }

    // Engine oil temperature, 0.03125 °C/bit, offset -273 °C, bytes 2-3
    if (raw_oil != 0xFFFF)
    {
      oil_temp_degC_ = static_cast<double>(raw_oil) * 0.03125 - 273.0;
      last_oil_valid_steady_ = now;
      updated = true;
    }

    return updated;
  }

  // EFL/P1, ID 0x18FEEF00: oil pressure, coolant level
  bool handleOilPressureAndCoolantLevel(const can_frame &frame)
  {
    if (frame.can_dlc < 8)
    {
      return false;
    }

    bool updated = false;

    // Oil pressure, 4 kPa/bit, byte 3 (bit 24-31)
    uint8_t raw_press = frame.data[3];
    if (raw_press != 0xFF)
    {
      double press_kpa = raw_press * 4.0;
      std::lock_guard<std::mutex> lk(data_mutex_);
      oil_press_kpa_ = press_kpa;
      updated = true;
    }

    // Coolant level, 0.4 %/bit, byte 7 (bit 56-63)
    uint8_t raw_level = frame.data[7];
    if (raw_level != 0xFF)
    {
      double level_pct = raw_level * 0.4;
      std::lock_guard<std::mutex> lk(data_mutex_);
      coolant_level_pct_ = level_pct;
      updated = true;
    }

    return updated;
  }

  void publishFeedback()
  {
    std::lock_guard<std::mutex> lk(data_mutex_);

    std::ostringstream oss;
    oss.setf(std::ios::fixed);
    oss.precision(2);

    oss << "{";

    // 使用 NaN 判断是否有有效值
    bool first = true;

    auto append_field = [&](const std::string &name, double value) {
      if (std::isnan(value))
      {
        return;
      }
      if (!first)
      {
        oss << ",";
      }
      first = false;
      oss << "\"" << name << "\":" << value;
    };

    const auto now_steady = std::chrono::steady_clock::now();

    // 从未收到有效转速，或超过 engine_speed_stale_timeout_sec 未收到新的 0x0CF00400：报 -1
    double rpm_out = -1.0;
    if (!std::isnan(engine_speed_rpm_)) {
      const auto dt =
        std::chrono::duration<double>(now_steady - last_engine_speed_frame_steady_);
      if (dt.count() <= engine_speed_stale_timeout_sec_) {
        rpm_out = engine_speed_rpm_;
      }
    }
    append_field("engine_speed_rpm", rpm_out);
    append_field("accumulated_running_time_h", accumulated_running_time_h_);

    // 水温/油温：超过 et1_stale_timeout_sec 未收到新的有效 ET1（对应字节）则报 -1
    double coolant_out = -1.0;
    if (!std::isnan(coolant_temp_degC_)) {
      const auto dt =
        std::chrono::duration<double>(now_steady - last_coolant_valid_steady_);
      if (dt.count() <= et1_stale_timeout_sec_) {
        coolant_out = coolant_temp_degC_;
      }
    }
    append_field("coolant_temp_degC", coolant_out);

    double oil_out = -1.0;
    if (!std::isnan(oil_temp_degC_)) {
      const auto dt = std::chrono::duration<double>(now_steady - last_oil_valid_steady_);
      if (dt.count() <= et1_stale_timeout_sec_) {
        oil_out = oil_temp_degC_;
      }
    }
    append_field("oil_temp_degC", oil_out);
    append_field("oil_press_kpa", oil_press_kpa_);
    append_field("coolant_level_pct", coolant_level_pct_);

    oss << "}";

    auto msg = std_msgs::msg::String();
    msg.data = oss.str();
    feedback_pub_->publish(msg);
  }

  // 成员变量
  std::string can_interface_;
  SocketCanInterface can_iface_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr feedback_pub_;

  std::thread recv_thread_;
  std::atomic<bool> running_{false};

  // 最新的 gear 数值（来自 ControlCommand.pto_cmd）
  std::mutex cmd_mutex_;
  uint8_t engine_gear_level_{0};

  // 解析后的数据
  std::mutex data_mutex_;
  double engine_speed_stale_timeout_sec_{5.0};
  double et1_stale_timeout_sec_{20.0};
  std::chrono::steady_clock::time_point last_engine_speed_frame_steady_{};
  std::chrono::steady_clock::time_point last_coolant_valid_steady_{};
  std::chrono::steady_clock::time_point last_oil_valid_steady_{};
  double engine_speed_rpm_ = std::numeric_limits<double>::quiet_NaN();
  double accumulated_running_time_h_ = std::numeric_limits<double>::quiet_NaN();
  double coolant_temp_degC_ = std::numeric_limits<double>::quiet_NaN();
  double oil_temp_degC_ = std::numeric_limits<double>::quiet_NaN();
  double oil_press_kpa_ = std::numeric_limits<double>::quiet_NaN();
  double coolant_level_pct_ = std::numeric_limits<double>::quiet_NaN();
};

}  // namespace cannode

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cannode::EngineFeedbackNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}



