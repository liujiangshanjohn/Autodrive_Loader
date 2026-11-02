/**
 * 装载机 teleop → ControlCommand 转换
 * 协议：装载机_上位机通讯协议_20260721
 *
 * JSON (/controls/teleop) → protobuf (/vehicle_command)
 * - gear: "N"/"D"/"R" → 前进/后退开关
 * - throttle/brake: [0,1] → [0,100]%
 * - steering: [-1,1] → steering_target [-100,100]%
 * - boom/bucket: [-1,1] → 大臂/铲斗升落设定值 0~1000
 * - horn/light/emergency_stop
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <map>
#include <mutex>
#include <sstream>
#include <string>

#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <cyber_msgs/msg/proto_adapter.hpp>

#include "simple_json_parser.hpp"

#include "common_msgs/basic_msgs/header.pb.h"
#include "common_msgs/basic_msgs/vehicle_signal.pb.h"
#include "common_msgs/chassis_msgs/chassis.pb.h"
#include "control_msgs/control_cmd.pb.h"

using namespace std::chrono_literals;

class Teleop2CanTransformerLoader : public rclcpp::Node {
 public:
  Teleop2CanTransformerLoader() : Node("teleop2can_transformer_loader") {
    this->declare_parameter<double>("steering_deadzone", 0.05);
    this->declare_parameter<double>("throttle_deadzone", 0.05);
    this->declare_parameter<double>("brake_deadzone", 0.05);
    this->declare_parameter<double>("boom_deadzone", 0.05);
    this->declare_parameter<double>("bucket_deadzone", 0.05);
    this->declare_parameter<double>("teleop_timeout_sec", 0.5);
    this->declare_parameter<bool>("verbose_log", true);

    steering_deadzone_ = this->get_parameter("steering_deadzone").as_double();
    throttle_deadzone_ = this->get_parameter("throttle_deadzone").as_double();
    brake_deadzone_ = this->get_parameter("brake_deadzone").as_double();
    boom_deadzone_ = this->get_parameter("boom_deadzone").as_double();
    bucket_deadzone_ = this->get_parameter("bucket_deadzone").as_double();
    teleop_timeout_sec_ = this->get_parameter("teleop_timeout_sec").as_double();
    verbose_log_ = this->get_parameter("verbose_log").as_bool();

    rclcpp::QoS teleop_qos(10);
    teleop_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    teleop_qos.durability(rclcpp::DurabilityPolicy::Volatile);
    teleop_qos.history(rclcpp::HistoryPolicy::KeepLast);

    teleop_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/controls/teleop", teleop_qos,
        std::bind(&Teleop2CanTransformerLoader::teleop_callback, this, std::placeholders::_1));

    rclcpp::QoS cmd_qos(10);
    cmd_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
    cmd_qos.durability(rclcpp::DurabilityPolicy::Volatile);
    cmd_qos.history(rclcpp::HistoryPolicy::KeepLast);

    vehicle_cmd_pub_ =
        this->create_publisher<cyber_msgs::msg::ProtoAdapter>("/vehicle_command", cmd_qos);
    debug_pub_ = this->create_publisher<std_msgs::msg::String>("/vehicle_command_debug", cmd_qos);

    last_teleop_time_ = this->now();
    timeout_timer_ = this->create_wall_timer(
        100ms, std::bind(&Teleop2CanTransformerLoader::check_timeout, this));

    RCLCPP_INFO(this->get_logger(),
                "teleop2can_transformer_loader 已启动（SJTU 装载机协议）");
  }

 private:
  static double clamp(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
  }

  static double apply_deadzone(double value, double deadzone) {
    if (std::abs(value) < deadzone) {
      return 0.0;
    }
    if (value > 0) {
      return (value - deadzone) / (1.0 - deadzone);
    }
    return (value + deadzone) / (1.0 - deadzone);
  }

  /** [-1,1] 有符号轴 → 正方向 setpoint A / 负方向 setpoint B，范围 0~1000 */
  static void axis_to_pair(double axis, double deadzone, double *pos_out, double *neg_out) {
    axis = clamp(axis, -1.0, 1.0);
    axis = apply_deadzone(axis, deadzone);
    if (axis > 0.0) {
      *pos_out = axis * 1000.0;
      *neg_out = 0.0;
    } else if (axis < 0.0) {
      *pos_out = 0.0;
      *neg_out = (-axis) * 1000.0;
    } else {
      *pos_out = 0.0;
      *neg_out = 0.0;
    }
  }

  control::canbus::Chassis::GearPosition parse_gear(const std::string &g) {
    if (g == "D" || g == "d" || g == "1") {
      return control::canbus::Chassis::GEAR_DRIVE;
    }
    if (g == "R" || g == "r" || g == "-1") {
      return control::canbus::Chassis::GEAR_REVERSE;
    }
    return control::canbus::Chassis::GEAR_NEUTRAL;
  }

  void publish_cmd(const control::ControlCommand &cmd) {
    std::string serialized;
    if (!cmd.SerializeToString(&serialized)) {
      RCLCPP_ERROR(this->get_logger(), "ControlCommand 序列化失败");
      return;
    }
    cyber_msgs::msg::ProtoAdapter out;
    out.pb.assign(serialized.begin(), serialized.end());
    vehicle_cmd_pub_->publish(out);

    std_msgs::msg::String dbg;
    std::ostringstream oss;
    oss << "{";
    oss << "\"gear\":" << static_cast<int>(cmd.gear_location()) << ",";
    oss << "\"throttle\":" << cmd.throttle() << ",";
    oss << "\"brake\":" << cmd.brake() << ",";
    oss << "\"steering_target\":" << cmd.steering_target() << ",";
    oss << "\"boom_up\":" << cmd.arm_up_current() << ",";
    oss << "\"boom_down\":" << cmd.arm_down_current() << ",";
    oss << "\"bucket_up\":" << cmd.bucket_close_current() << ",";
    oss << "\"bucket_down\":" << cmd.bucket_dump_current() << ",";
    oss << "\"horn\":" << (cmd.signal().horn() ? "true" : "false") << ",";
    oss << "\"light\":" << (cmd.signal().low_beam() ? "true" : "false") << ",";
    oss << "\"estop\":" << (cmd.estop() ? "true" : "false");
    oss << "}";
    dbg.data = oss.str();
    debug_pub_->publish(dbg);
  }

  void fill_safe_stop(control::ControlCommand *cmd) {
    auto *header = cmd->mutable_header();
    header->set_timestamp_sec(this->now().seconds());
    header->set_frame_id("base_link");
    cmd->set_gear_location(control::canbus::Chassis::GEAR_NEUTRAL);
    cmd->set_throttle(0.0);
    cmd->set_brake(100.0);
    cmd->set_steering_target(0.0);
    cmd->set_arm_up_current(0.0);
    cmd->set_arm_down_current(0.0);
    cmd->set_bucket_close_current(0.0);
    cmd->set_bucket_dump_current(0.0);
    cmd->set_estop(true);
    auto *sig = cmd->mutable_signal();
    sig->set_horn(false);
    sig->set_low_beam(false);
  }

  void check_timeout() {
    rclcpp::Time last;
    {
      std::lock_guard<std::mutex> lk(mutex_);
      last = last_teleop_time_;
    }
    double elapsed = (this->now() - last).seconds();
    if (elapsed > teleop_timeout_sec_) {
      if (!timeout_warned_) {
        RCLCPP_WARN(this->get_logger(),
                    "/controls/teleop 超时 %.2fs，发送安全停车指令", elapsed);
        timeout_warned_ = true;
      }
      control::ControlCommand cmd;
      fill_safe_stop(&cmd);
      publish_cmd(cmd);
    }
  }

  void teleop_callback(const std_msgs::msg::String::SharedPtr msg) {
    try {
      {
        std::lock_guard<std::mutex> lk(mutex_);
        last_teleop_time_ = this->now();
      }
      if (timeout_warned_) {
        RCLCPP_INFO(this->get_logger(), "已恢复 teleop 指令");
        timeout_warned_ = false;
      }

      auto data = SimpleJsonParser::parse(msg->data);
      control::ControlCommand cmd;
      auto *header = cmd.mutable_header();
      header->set_timestamp_sec(this->now().seconds());
      header->set_frame_id("base_link");

      bool estop = false;
      if (data.count("emergency_stop")) {
        estop = SimpleJsonParser::get_bool(data["emergency_stop"]);
      }
      cmd.set_estop(estop);

      if (estop) {
        fill_safe_stop(&cmd);
        publish_cmd(cmd);
        return;
      }

      // 档位
      if (data.count("gear")) {
        cmd.set_gear_location(parse_gear(data["gear"]));
      } else if (data.count("forward") || data.count("backward")) {
        bool fwd = data.count("forward") && SimpleJsonParser::get_bool(data["forward"]);
        bool bwd = data.count("backward") && SimpleJsonParser::get_bool(data["backward"]);
        if (fwd && !bwd) {
          cmd.set_gear_location(control::canbus::Chassis::GEAR_DRIVE);
        } else if (bwd && !fwd) {
          cmd.set_gear_location(control::canbus::Chassis::GEAR_REVERSE);
        } else {
          cmd.set_gear_location(control::canbus::Chassis::GEAR_NEUTRAL);
        }
      } else {
        cmd.set_gear_location(control::canbus::Chassis::GEAR_NEUTRAL);
      }

      // 油门/刹车 [0,1] → [0,100]
      double thr = data.count("throttle") ? SimpleJsonParser::get_double(data["throttle"]) : 0.0;
      thr = clamp(thr, 0.0, 1.0);
      thr = apply_deadzone(thr, throttle_deadzone_);
      if (thr < 0) thr = 0;
      cmd.set_throttle(thr * 100.0);

      double brk = data.count("brake") ? SimpleJsonParser::get_double(data["brake"]) : 0.0;
      brk = clamp(brk, 0.0, 1.0);
      brk = apply_deadzone(brk, brake_deadzone_);
      if (brk < 0) brk = 0;
      cmd.set_brake(brk * 100.0);

      // 转向 [-1,1] → [-100,100]
      double steer =
          data.count("steering") ? SimpleJsonParser::get_double(data["steering"]) : 0.0;
      steer = clamp(steer, -1.0, 1.0);
      steer = apply_deadzone(steer, steering_deadzone_);
      cmd.set_steering_target(steer * 100.0);

      // 大臂：+升 / -落 → 0~1000
      double boom = data.count("boom") ? SimpleJsonParser::get_double(data["boom"]) : 0.0;
      double boom_up = 0.0, boom_down = 0.0;
      axis_to_pair(boom, boom_deadzone_, &boom_up, &boom_down);
      cmd.set_arm_up_current(boom_up);
      cmd.set_arm_down_current(boom_down);

      // 铲斗：+升 / -落 → close=升, dump=落
      double bucket =
          data.count("bucket") ? SimpleJsonParser::get_double(data["bucket"]) : 0.0;
      double bucket_up = 0.0, bucket_down = 0.0;
      axis_to_pair(bucket, bucket_deadzone_, &bucket_up, &bucket_down);
      cmd.set_bucket_close_current(bucket_up);
      cmd.set_bucket_dump_current(bucket_down);

      auto *sig = cmd.mutable_signal();
      bool horn = data.count("horn") && SimpleJsonParser::get_bool(data["horn"]);
      bool light = false;
      if (data.count("light")) {
        light = SimpleJsonParser::get_bool(data["light"]);
      } else if (data.count("light_code")) {
        light = SimpleJsonParser::get_int(data["light_code"]) != 0;
      }
      sig->set_horn(horn);
      sig->set_low_beam(light);

      if (verbose_log_) {
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 500,
            "gear=%d thr=%.0f brk=%.0f steer=%.0f boom=%.0f/%.0f bucket=%.0f/%.0f horn=%d light=%d",
            static_cast<int>(cmd.gear_location()), cmd.throttle(), cmd.brake(),
            cmd.steering_target(), cmd.arm_up_current(), cmd.arm_down_current(),
            cmd.bucket_close_current(), cmd.bucket_dump_current(), horn, light);
      }

      publish_cmd(cmd);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "teleop 解析失败: %s", e.what());
    }
  }

  double steering_deadzone_{0.05};
  double throttle_deadzone_{0.05};
  double brake_deadzone_{0.05};
  double boom_deadzone_{0.05};
  double bucket_deadzone_{0.05};
  double teleop_timeout_sec_{0.5};
  bool verbose_log_{true};

  std::mutex mutex_;
  rclcpp::Time last_teleop_time_;
  bool timeout_warned_{false};

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr teleop_sub_;
  rclcpp::Publisher<cyber_msgs::msg::ProtoAdapter>::SharedPtr vehicle_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Teleop2CanTransformerLoader>());
  rclcpp::shutdown();
  return 0;
}
