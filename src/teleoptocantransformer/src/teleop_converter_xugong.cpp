#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <cyber_msgs/msg/proto_adapter.hpp>
#include <rclcpp/qos.hpp>
#include <chrono>
#include <cmath>
#include <string>
#include <sstream>
#include <map>
#include <functional>
#include "simple_json_parser.hpp"

// Protobuf 头文件
#include "control_msgs/control_cmd.pb.h"
#include "common_msgs/chassis_msgs/chassis.pb.h"
#include "common_msgs/basic_msgs/header.pb.h"
#include <cyber_msgs/msg/chassis_status.hpp>

using namespace std::chrono_literals;

class Teleop2CanTransformer : public rclcpp::Node
{
public:
    Teleop2CanTransformer() : Node("teleop2can_transformer")
    {
        // 声明参数
        this->declare_parameter<double>("steering_deadzone", 0.05);
        this->declare_parameter<double>("throttle_deadzone", 0.05);
        this->declare_parameter<double>("brake_deadzone", 0.05);
        this->declare_parameter<double>("boom_deadzone", 0.05);
        this->declare_parameter<double>("bucket_deadzone", 0.05);
        // 各发布开关（yaml 可配置）
        this->declare_parameter<bool>("publish_vehicle_command", true);         // 是否发布 /vehicle_command
        this->declare_parameter<bool>("publish_vehicle_command_debug", true);   // 是否发布 /vehicle_command_debug
        this->declare_parameter<bool>("publish_chassis_feedback", true);        // 是否发布 cannode/chassis_feedback
        this->declare_parameter<bool>("verbose_log", false);                    // 是否打印详细日志，默认关闭以降低CPU
        
        // 角度映射范围（度）
        // 大臂范围：-800~800
        // 铲斗范围：-800~800
        this->declare_parameter<double>("arm_angle_min", -800.0);
        this->declare_parameter<double>("arm_angle_max", 800.0);
        this->declare_parameter<double>("shovel_angle_min", -800.0);
        this->declare_parameter<double>("shovel_angle_max", 800.0);
        
        // 速度限制（m/s）
        this->declare_parameter<double>("max_speed", 3.0);
        
        // 获取参数
        steering_deadzone_ = this->get_parameter("steering_deadzone").as_double();
        throttle_deadzone_ = this->get_parameter("throttle_deadzone").as_double();
        brake_deadzone_ = this->get_parameter("brake_deadzone").as_double();
        boom_deadzone_ = this->get_parameter("boom_deadzone").as_double();
        bucket_deadzone_ = this->get_parameter("bucket_deadzone").as_double();
        arm_angle_min_ = this->get_parameter("arm_angle_min").as_double();
        arm_angle_max_ = this->get_parameter("arm_angle_max").as_double();
        shovel_angle_min_ = this->get_parameter("shovel_angle_min").as_double();
        shovel_angle_max_ = this->get_parameter("shovel_angle_max").as_double();
        max_speed_ = this->get_parameter("max_speed").as_double();
        publish_vehicle_command_ = this->get_parameter("publish_vehicle_command").as_bool();
        publish_vehicle_command_debug_ = this->get_parameter("publish_vehicle_command_debug").as_bool();
        publish_chassis_feedback_ = this->get_parameter("publish_chassis_feedback").as_bool();
        verbose_log_ = this->get_parameter("verbose_log").as_bool();
        
        // 创建订阅者（订阅远程端控制指令）
        // 使用 BEST_EFFORT QoS 以匹配发布者（参考 keyboard_piston_joint_publisher_2_updated.py）
        rclcpp::QoS teleop_qos(10);
        teleop_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        teleop_qos.durability(rclcpp::DurabilityPolicy::Volatile);
        teleop_qos.history(rclcpp::HistoryPolicy::KeepLast);
        
        teleop_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/controls/teleop", 
            teleop_qos,
            std::bind(&Teleop2CanTransformer::teleop_callback, this, std::placeholders::_1)
        );
        
        // 创建发布者（发布到 cannode）
        // 使用 RELIABLE QoS 以匹配 cannode 的订阅者
        rclcpp::QoS vehicle_cmd_qos(10);
        vehicle_cmd_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
        vehicle_cmd_qos.durability(rclcpp::DurabilityPolicy::Volatile);
        vehicle_cmd_qos.history(rclcpp::HistoryPolicy::KeepLast);
        
        vehicle_cmd_pub_ = this->create_publisher<cyber_msgs::msg::ProtoAdapter>(
            "/vehicle_command",
            vehicle_cmd_qos
        );
        
        // 创建非序列化消息发布者（用于调试和查看）
        vehicle_cmd_debug_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/vehicle_command_debug",
            vehicle_cmd_qos
        );

        // 订阅 cannode 输出的底盘状态并转成 JSON 后发布
        rclcpp::QoS chassis_qos(10);
        chassis_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
        chassis_qos.durability(rclcpp::DurabilityPolicy::Volatile);
        chassis_qos.history(rclcpp::HistoryPolicy::KeepLast);

        chassis_status_sub_ = this->create_subscription<cyber_msgs::msg::ChassisStatus>(
            "/chassis_status",
            chassis_qos,
            std::bind(&Teleop2CanTransformer::chassis_status_callback, this, std::placeholders::_1)
        );

        chassis_feedback_pub_ = this->create_publisher<std_msgs::msg::String>(
            "cannode/chassis_feedback",
            chassis_qos
        );
        
        // RCLCPP_INFO(this->get_logger(), "QoS 配置: /controls/teleop (BEST_EFFORT), /vehicle_command (RELIABLE)");
        // RCLCPP_INFO(this->get_logger(), "订阅话题: /controls/teleop");
        // RCLCPP_INFO(this->get_logger(), "发布话题: /vehicle_command (序列化)");
        // RCLCPP_INFO(this->get_logger(), "发布话题: /vehicle_command_debug (非序列化，可用 ros2 topic echo 查看)");
        
        // 使用定时器定期检查连接状态
        auto connection_check_timer = this->create_wall_timer(
            std::chrono::seconds(5),
            [this]() {
                size_t pub_count = teleop_sub_->get_publisher_count();
                size_t sub_count = vehicle_cmd_pub_->get_subscription_count();
                // RCLCPP_INFO(this->get_logger(), "连接状态检查: /controls/teleop 发布者数=%zu, /vehicle_command 订阅者数=%zu", 
                //            pub_count, sub_count);
            }
        );
        
        // 初始化控制命令
        last_gear_ = control::canbus::Chassis::GEAR_NEUTRAL;
        last_steering_ = 0.0;
        last_throttle_ = 0.0;
        last_brake_ = 0.0;
        last_arm_angle_ = 0.0;
        last_shovel_angle_ = 0.0;
        
        // RCLCPP_INFO(this->get_logger(), "Teleop2CanTransformer 节点已启动");
        // RCLCPP_INFO(this->get_logger(), "死区设置: steering=%.3f, throttle=%.3f, brake=%.3f, boom=%.3f, bucket=%.3f",
        //             steering_deadzone_, throttle_deadzone_, brake_deadzone_, boom_deadzone_, bucket_deadzone_);
        // RCLCPP_INFO(this->get_logger(), "发布开关: publish_vehicle_command=%s, publish_vehicle_command_debug=%s, publish_chassis_feedback=%s",
        //             publish_vehicle_command_ ? "true" : "false",
        //             publish_vehicle_command_debug_ ? "true" : "false",
        //             publish_chassis_feedback_ ? "true" : "false");
        // RCLCPP_INFO(this->get_logger(), "详细日志: verbose_log=%s (默认关闭以降低CPU)", verbose_log_ ? "true" : "false");
    }

private:
    // 死区处理函数
    double apply_deadzone(double value, double deadzone)
    {
        if (std::abs(value) < deadzone) {
            return 0.0;
        }
        // 线性缩放，使死区外的值映射到 [0, 1] 或 [-1, 1]
        if (value > 0) {
            return (value - deadzone) / (1.0 - deadzone);
        } else {
            return (value + deadzone) / (1.0 - deadzone);
        }
    }
    
    // 限制值在范围内
    double clamp(double value, double min_val, double max_val)
    {
        return std::max(min_val, std::min(max_val, value));
    }

    // 将 ChassisStatus 转成 JSON 字符串
    std::string chassis_status_to_json(const cyber_msgs::msg::ChassisStatus & msg)
    {
        auto bool_to_str = [](bool v) { return v ? "true" : "false"; };
        std::ostringstream oss;
        oss << "{";
        oss << "\"header\":{";
        oss << "\"stamp\":{\"sec\":" << msg.header.stamp.sec << ",\"nanosec\":" << msg.header.stamp.nanosec << "},";
        oss << "\"frame_id\":\"" << msg.header.frame_id << "\"";
        oss << "},";

        // 0x18FF0001
        oss << "\"high_voltage_status\":" << bool_to_str(msg.high_voltage_status) << ",";
        oss << "\"parking_brake\":" << bool_to_str(msg.parking_brake) << ",";
        oss << "\"horn_status\":" << bool_to_str(msg.horn_status) << ",";
        oss << "\"left_turn_signal\":" << bool_to_str(msg.left_turn_signal) << ",";
        oss << "\"right_turn_signal\":" << bool_to_str(msg.right_turn_signal) << ",";
        oss << "\"walk_motor_mode\":" << static_cast<int>(msg.walk_motor_mode) << ",";
        oss << "\"wet_brake_alarm\":" << bool_to_str(msg.wet_brake_alarm) << ",";
        oss << "\"emergency_stop\":" << bool_to_str(msg.emergency_stop) << ",";
        oss << "\"gear_signal\":" << static_cast<int>(msg.gear_signal) << ",";
        oss << "\"rotation_alarm\":" << static_cast<int>(msg.rotation_alarm) << ",";
        oss << "\"heartbeat_status\":" << bool_to_str(msg.heartbeat_status) << ",";
        oss << "\"brake_control\":" << msg.brake_control << ",";
        oss << "\"front_rear_angle\":" << msg.front_rear_angle << ",";
        oss << "\"battery_level\":" << static_cast<int>(msg.battery_level) << ",";
        oss << "\"charging_status\":" << bool_to_str(msg.charging_status) << ",";
        oss << "\"hydraulic_lock\":" << bool_to_str(msg.hydraulic_lock) << ",";
        oss << "\"fault_level\":" << static_cast<int>(msg.fault_level) << ",";
        oss << "\"turtle_rabbit_gear\":" << bool_to_str(msg.turtle_rabbit_gear) << ",";
        oss << "\"work_light\":" << bool_to_str(msg.work_light) << ",";
        oss << "\"vehicle_mode\":" << static_cast<int>(msg.vehicle_mode) << ",";

        // 0x18FF0002
        oss << "\"boom_lift_current\":" << msg.boom_lift_current << ",";
        oss << "\"boom_lower_current\":" << msg.boom_lower_current << ",";
        oss << "\"bucket_close_current\":" << msg.bucket_close_current << ",";
        oss << "\"bucket_open_current\":" << msg.bucket_open_current << ",";

        // 0x18FF0003
        oss << "\"boom_big_pressure\":" << msg.boom_big_pressure << ",";
        oss << "\"boom_small_pressure\":" << msg.boom_small_pressure << ",";
        oss << "\"bucket_big_pressure\":" << msg.bucket_big_pressure << ",";
        oss << "\"bucket_small_pressure\":" << msg.bucket_small_pressure << ",";

        // 0x18FF0004
        oss << "\"hydraulic_motor_speed\":" << msg.hydraulic_motor_speed << ",";
        oss << "\"hydraulic_motor_torque\":" << msg.hydraulic_motor_torque << ",";
        oss << "\"hydraulic_motor_current\":" << msg.hydraulic_motor_current << ",";
        oss << "\"hydraulic_motor_enable\":" << bool_to_str(msg.hydraulic_motor_enable) << ",";

        // 0x18FF0005
        oss << "\"walk_motor_current\":" << msg.walk_motor_current << ",";
        oss << "\"walk_motor_torque\":" << msg.walk_motor_torque << ",";
        // 车辆速度 = 行走电机反馈速度 * 3.6 （直接按输入值换算）
        oss << "\"walk_motor_speed\":" << msg.walk_motor_speed << ",";
        oss << "\"vehicle_speed\":" << msg.walk_motor_speed * 3.6 << ",";
        oss << "\"walk_motor_enable\":" << bool_to_str(msg.walk_motor_enable) << ",";

        // 0x18FF0006
        oss << "\"throttle_opening\":" << static_cast<int>(msg.throttle_opening) << ",";
        oss << "\"brake_opening\":" << static_cast<int>(msg.brake_opening) << ",";
        oss << "\"heartbeat_signal\":" << bool_to_str(msg.heartbeat_signal) << ",";
        oss << "\"can_loss_1\":" << bool_to_str(msg.can_loss_1) << ",";
        oss << "\"can_loss_2\":" << bool_to_str(msg.can_loss_2) << ",";
        oss << "\"can_loss_3\":" << bool_to_str(msg.can_loss_3) << ",";
        oss << "\"can_loss_4\":" << bool_to_str(msg.can_loss_4) << ",";
        oss << "\"walk_motor_fault\":" << bool_to_str(msg.walk_motor_fault) << ",";
        oss << "\"hydraulic_motor_fault\":" << bool_to_str(msg.hydraulic_motor_fault) << ",";
        oss << "\"boom_lift_valve_fault\":" << bool_to_str(msg.boom_lift_valve_fault) << ",";
        oss << "\"boom_lower_valve_fault\":" << bool_to_str(msg.boom_lower_valve_fault) << ",";
        oss << "\"bucket_close_valve_fault\":" << bool_to_str(msg.bucket_close_valve_fault) << ",";
        oss << "\"bucket_open_valve_fault\":" << bool_to_str(msg.bucket_open_valve_fault) << ",";
        oss << "\"foot_brake_valve_fault\":" << bool_to_str(msg.foot_brake_valve_fault) << ",";
        oss << "\"turn_valve_fault\":" << bool_to_str(msg.turn_valve_fault) << ",";
        oss << "\"low_beam\":" << bool_to_str(msg.low_beam) << ",";
        oss << "\"high_beam\":" << bool_to_str(msg.high_beam) << ",";
        oss << "\"hydraulic_motor_voltage\":" << static_cast<int>(msg.hydraulic_motor_voltage) << ",";
        oss << "\"turn_valve_current\":" << msg.turn_valve_current << ",";
        oss << "\"hydraulic_motor_mode\":" << static_cast<int>(msg.hydraulic_motor_mode) << ",";
        oss << "\"vehicle_mode_2\":" << static_cast<int>(msg.vehicle_mode_2) << ",";

        // 0x18FF0009
        oss << "\"rear_motor_current\":" << msg.rear_motor_current << ",";
        oss << "\"rear_motor_voltage\":" << static_cast<int>(msg.rear_motor_voltage) << ",";
        oss << "\"front_motor_current\":" << msg.front_motor_current << ",";
        oss << "\"front_motor_voltage\":" << static_cast<int>(msg.front_motor_voltage) << ",";
        oss << "\"hydraulic_motor_current_2\":" << msg.hydraulic_motor_current_2 << ",";

        // 0x18FF000A
        oss << "\"boom_angle\":" << msg.boom_angle << ",";
        oss << "\"bucket_angle\":" << msg.bucket_angle;

        oss << "}";
        return oss.str();
    }

    // 底盘状态订阅回调
    void chassis_status_callback(const cyber_msgs::msg::ChassisStatus::SharedPtr msg)
    {
        if (publish_chassis_feedback_) {
            std_msgs::msg::String json_msg;
            json_msg.data = chassis_status_to_json(*msg);
            chassis_feedback_pub_->publish(json_msg);
        }
    }
    
    // 将档位字段转换为 GearPosition（兼容老的字符档位和新的数值 1~7）
    control::canbus::Chassis::GearPosition parse_gear_field(const std::string& gear_raw)
    {
        // 优先尝试按整数解析（新格式 1~7 档）
        try {
            int gear_int = SimpleJsonParser::get_int(gear_raw);
            if (gear_int <= 0) {
                return control::canbus::Chassis::GEAR_NEUTRAL;
            } else {
                // 目前仅区分“有前进档/无前进档”，所有正数视为前进档
                return control::canbus::Chassis::GEAR_DRIVE;
            }
        } catch (const std::exception&) {
            // 回退到旧格式: "D"/"R"/"N"/"P" 等
        }

        if (gear_raw == "D" || gear_raw == "d") {
            return control::canbus::Chassis::GEAR_DRIVE;
        } else if (gear_raw == "R" || gear_raw == "r") {
            return control::canbus::Chassis::GEAR_REVERSE;
        } else if (gear_raw == "N" || gear_raw == "n") {
            return control::canbus::Chassis::GEAR_NEUTRAL;
        } else {
            return control::canbus::Chassis::GEAR_NEUTRAL;
        }
    }
    
    void teleop_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (verbose_log_) {
            // RCLCPP_INFO(this->get_logger(), "🔔 回调函数被调用！");
        }
        try {
            // 打印接收到的原始输入
            if (verbose_log_) {
                // RCLCPP_INFO(this->get_logger(), "============================================================");
                // RCLCPP_INFO(this->get_logger(), "📥 收到控制指令: %s", msg->data.c_str());
                // RCLCPP_INFO(this->get_logger(), "📥 消息长度: %zu 字节", msg->data.length());
            }
            
            // 解析 JSON
            std::map<std::string, std::string> data = SimpleJsonParser::parse(msg->data);
            
            // 创建 ControlCommand protobuf 消息
            control::ControlCommand cmd;
            
            // 设置 header
            auto* header = cmd.mutable_header();
            auto now = this->now();
            header->set_timestamp_sec(now.seconds());
            header->set_frame_id("base_link");
            
            // 处理转向控制 (steering: -1..1 -> steering_target: -800..800)
            // 支持新格式的 steering 字段
            if (data.find("steering") != data.end()) {
                double steering_input = clamp(SimpleJsonParser::get_double(data["steering"]), -1.0, 1.0);
                
                // 应用死区
                steering_input = apply_deadzone(steering_input, steering_deadzone_);
                
                // 映射到角度范围 [-800, 800]
                // 注意：根据 Python 脚本，转向是反向的
                double steering_target = -steering_input * 800.0;
                cmd.set_steering_target(steering_target);
                last_steering_ = steering_target;
            } else {
                // 如果没有输入，保持上次值或设为0
                cmd.set_steering_target(last_steering_);
            }
            
            // 处理左履带 (leftTrack: -1..1) - 保留格式但不映射到输出
            if (data.find("leftTrack") != data.end()) {
                double left_track = clamp(SimpleJsonParser::get_double(data["leftTrack"]), -1.0, 1.0);
                // RCLCPP_DEBUG(this->get_logger(), "收到 leftTrack: %.2f (保留格式，不映射)", left_track);
            }
            
            // 处理右履带 (rightTrack: -1..1) - 保留格式但不映射到输出
            if (data.find("rightTrack") != data.end()) {
                double right_track = clamp(SimpleJsonParser::get_double(data["rightTrack"]), -1.0, 1.0);
                // RCLCPP_DEBUG(this->get_logger(), "收到 rightTrack: %.2f (保留格式，不映射)", right_track);
            }
            
            // 处理驾驶室旋转 (swing: -1..1) - 保留格式但不映射到输出
            if (data.find("swing") != data.end()) {
                double swing = clamp(SimpleJsonParser::get_double(data["swing"]), -1.0, 1.0);
                // RCLCPP_DEBUG(this->get_logger(), "收到 swing: %.2f (保留格式，不映射)", swing);
            }
            
            // 处理小臂 (stick: -1..1) - 保留格式但不映射到输出（cannode没有小臂字段）
            if (data.find("stick") != data.end()) {
                double stick = clamp(SimpleJsonParser::get_double(data["stick"]), -1.0, 1.0);
                // RCLCPP_DEBUG(this->get_logger(), "收到 stick: %.2f (保留格式，不映射)", stick);
            }
            
            // 处理油门控制 (throttle: -1..1 -> throttle: 200..0，反向映射)
            if (data.find("throttle") != data.end()) {
                double throttle_input = clamp(SimpleJsonParser::get_double(data["throttle"]), -1.0, 1.0);
                throttle_input = apply_deadzone(throttle_input, throttle_deadzone_);
                // 映射：-1 -> 200, 1 -> 0
                double throttle_value = (1.0 - throttle_input) / 2.0 * 200.0;
                cmd.set_throttle(throttle_value);
                last_throttle_ = throttle_value;
            } else {
                cmd.set_throttle(last_throttle_);
            }
            
            // 处理刹车控制 (brake: -1..1 -> brake: 3900..350，反向映射)
            if (data.find("brake") != data.end()) {
                double brake_input = clamp(SimpleJsonParser::get_double(data["brake"]), -1.0, 1.0);
                brake_input = apply_deadzone(brake_input, brake_deadzone_);
                // 映射：-1 -> 3900, 1 -> 350
                const double brake_min = 350.0;
                const double brake_max = 3900.0;
                double brake_value = (1.0 - brake_input) / 2.0 * (brake_max - brake_min) + brake_min;
                cmd.set_brake(brake_value);
                last_brake_ = brake_value;
            } else {
                cmd.set_brake(last_brake_);
            }
            
            // 处理档位控制
            // /controls/teleop 的 gear 字段已从字符串('P','R','N','D'等)改为数值 1~7
            // 这里做两件事：
            //  1) 仍然为 ControlCommand 设置一个简化的 gear_location（仅区分前进/倒车/空档）
            //  2) 将数值 gear（若存在）写入 pto_cmd 字段，供发动机油门控制器使用
            if (data.find("gear") != data.end()) {
                const std::string& gear_raw = data["gear"];

                // 1) 设置底盘 gear_location（仅用于速度方向等逻辑）
                control::canbus::Chassis::GearPosition gear = parse_gear_field(gear_raw);
                cmd.set_gear_location(gear);
                last_gear_ = gear;

                // 2) 如果可以解析成 1~7 的整数，则写入 pto_cmd，后续在 cannode 中映射到 0x18FD0821 报文
                try {
                    int gear_int = SimpleJsonParser::get_int(gear_raw);
                    if (gear_int < 0) gear_int = 0;
                    if (gear_int > 7) gear_int = 7;
                    cmd.set_pto_cmd(static_cast<uint32_t>(gear_int));
                } catch (const std::exception&) {
                    // 旧格式下不设置 pto_cmd
                }
            } else {
                cmd.set_gear_location(last_gear_);
            }
            
            // 处理驻车制动
            if (data.find("parking_brake") != data.end()) {
                cmd.set_parking_brake(SimpleJsonParser::get_bool(data["parking_brake"]));
            }
            
            // 处理大臂控制 (boom: -1..1 -> arm_angle: -800~800度)
            if (data.find("boom") != data.end()) {
                double boom_input = clamp(SimpleJsonParser::get_double(data["boom"]), -1.0, 1.0);
                boom_input = apply_deadzone(boom_input, boom_deadzone_);
                
                // 映射到角度范围（度）：-1 -> -800，1 -> 800
                double arm_angle = boom_input * (arm_angle_max_ - arm_angle_min_) / 2.0 + 
                                  (arm_angle_max_ + arm_angle_min_) / 2.0;
                cmd.set_arm_angle(arm_angle);
                cmd.set_arm_enable(true);
                last_arm_angle_ = arm_angle;
            } else {
                cmd.set_arm_angle(last_arm_angle_);
                cmd.set_arm_enable(false);
            }
            
            // 处理铲斗控制 (bucket: -1..1 -> shovel_angle: -800~800度)
            if (data.find("bucket") != data.end()) {
                double bucket_input = clamp(SimpleJsonParser::get_double(data["bucket"]), -1.0, 1.0);
                bucket_input = apply_deadzone(bucket_input, bucket_deadzone_);
                
                // 映射到角度范围（度）：-1 -> 800（因为反向），1 -> -800
                // 注意：根据 Python 脚本，bucket 是反向的
                double shovel_angle = -bucket_input * (shovel_angle_max_ - shovel_angle_min_) / 2.0 + 
                                     (shovel_angle_max_ + shovel_angle_min_) / 2.0;
                cmd.set_shovel_angle(shovel_angle);
                cmd.set_shovel_enable(true);
                last_shovel_angle_ = shovel_angle;
            } else {
                cmd.set_shovel_angle(last_shovel_angle_);
                cmd.set_shovel_enable(false);
            }
            
            // 处理紧急停止
            if (data.find("emergency_stop") != data.end()) {
                bool estop = SimpleJsonParser::get_bool(data["emergency_stop"]);
                cmd.set_estop(estop);
            }
            
            // 处理发动机开关 (power_enable: boolean -> engine_on_off)
            if (data.find("power_enable") != data.end()) {
                cmd.set_engine_on_off(SimpleJsonParser::get_bool(data["power_enable"]));
            }
            
            // 处理喇叭 (horn: boolean) - 使用deprecated字段但保留格式
            if (data.find("horn") != data.end()) {
                bool horn = SimpleJsonParser::get_bool(data["horn"]);
                // ControlCommand 的 horn 字段是 deprecated，但可以设置
                // 注意：protobuf 可能不支持直接设置 deprecated 字段，这里仅记录日志
                // RCLCPP_DEBUG(this->get_logger(), "收到 horn: %s (保留格式)", horn ? "true" : "false");
            }
            
            // 处理速度模式 (speed_mode: 'turtle'|'rabbit') - 影响max_speed但不直接映射
            double effective_max_speed = max_speed_;
            if (data.find("speed_mode") != data.end()) {
                std::string speed_mode = data["speed_mode"];
                if (speed_mode == "rabbit") {
                    effective_max_speed = max_speed_;  // 兔子模式：最大速度
                } else if (speed_mode == "turtle") {
                    effective_max_speed = max_speed_ * 0.5;  // 乌龟模式：一半速度
                }
                // RCLCPP_DEBUG(this->get_logger(), "速度模式: %s, 有效最大速度: %.2f m/s", 
                //            speed_mode.c_str(), effective_max_speed);
            }
            
            // 处理灯光代码 (light_code: number) - 保留格式但不映射到输出
            if (data.find("light_code") != data.end()) {
                int light_code = SimpleJsonParser::get_int(data["light_code"]);
                // RCLCPP_DEBUG(this->get_logger(), "收到 light_code: 0x%02X (保留格式，不映射)", light_code);
            }
            
            // 处理液压锁 (hydraulic_lock: boolean) - 保留格式但不映射到输出
            if (data.find("hydraulic_lock") != data.end()) {
                bool hydraulic_lock = SimpleJsonParser::get_bool(data["hydraulic_lock"]);
                // RCLCPP_DEBUG(this->get_logger(), "收到 hydraulic_lock: %s (保留格式，不映射)", 
                //            hydraulic_lock ? "true" : "false");
            }
            
            // 根据油门和档位计算目标速度
            // 速度模式会影响最大速度
            if (cmd.has_throttle() && cmd.has_gear_location()) {
                double speed = 0.0;
                if (cmd.gear_location() == control::canbus::Chassis::GEAR_DRIVE) {
                    speed = (cmd.throttle() / 200.0) * effective_max_speed;
                } else if (cmd.gear_location() == control::canbus::Chassis::GEAR_REVERSE) {
                    speed = -(cmd.throttle() / 200.0) * effective_max_speed;
                }
                cmd.set_speed(speed);
            }
            
            // 打印转换后的输出信息（仅在 verbose 模式下打印，减少CPU）
            if (verbose_log_) {
                // RCLCPP_INFO(this->get_logger(), "📤 转换后的控制命令:");
                
                // 转向 (steering_target: [-800, 800])
                double steering_target = cmd.steering_target();
                // RCLCPP_INFO(this->get_logger(), "   转向 (steering_target): %.2f", steering_target);
                if (steering_target < -800.0 || steering_target > 800.0) {
                    // RCLCPP_WARN(this->get_logger(), "     ⚠ 警告: steering_target 超出范围 [-800, 800]");
                }
                
                // 油门 (throttle: [0, 200])
                double throttle = cmd.throttle();
                // RCLCPP_INFO(this->get_logger(), "   油门 (throttle): %.2f", throttle);
                if (throttle < 0.0 || throttle > 200.0) {
                    // RCLCPP_WARN(this->get_logger(), "     ⚠ 警告: throttle 超出范围 [0, 200]");
                }
                
                // 刹车 (brake: [350, 3900])
                double brake = cmd.brake();
                // RCLCPP_INFO(this->get_logger(), "   刹车 (brake): %.2f", brake);
                if (brake < 350.0 || brake > 3900.0) {
                    // RCLCPP_WARN(this->get_logger(), "     ⚠ 警告: brake 超出范围 [350, 3900]");
                }
                
                // 档位字符串
                std::string gear_str = "UNKNOWN";
                switch(cmd.gear_location()) {
                    case control::canbus::Chassis::GEAR_DRIVE: gear_str = "D(前进)"; break;
                    case control::canbus::Chassis::GEAR_REVERSE: gear_str = "R(后退)"; break;
                    case control::canbus::Chassis::GEAR_NEUTRAL: gear_str = "N(空档)"; break;
                    default: gear_str = "N(空档)"; break;
                }
                // RCLCPP_INFO(this->get_logger(), "   档位 (gear_location): %s (%d)", gear_str.c_str(), static_cast<int>(cmd.gear_location()));
                
                if (cmd.has_speed()) {
                    // RCLCPP_INFO(this->get_logger(), "   目标速度 (speed): %.2f m/s", cmd.speed());
                }
                
                // 大臂角度 (arm_angle: [-800, 800]°)
                if (cmd.has_arm_angle()) {
                    double arm_angle = cmd.arm_angle();
                    if (cmd.arm_enable()) {
                        // RCLCPP_INFO(this->get_logger(), "   大臂角度 (arm_angle): %.2f° [启用]", arm_angle);
                    } else {
                        // RCLCPP_INFO(this->get_logger(), "   大臂角度 (arm_angle): %.2f° [禁用]", arm_angle);
                    }
                    if (arm_angle < -800.0 || arm_angle > 800.0) {
                        // RCLCPP_WARN(this->get_logger(), "     ⚠ 警告: arm_angle 超出范围 [-800, 800]°");
                    }
                }
                
                // 铲斗角度 (shovel_angle: [-800, 800]°)
                if (cmd.has_shovel_angle()) {
                    double shovel_angle = cmd.shovel_angle();
                    if (cmd.shovel_enable()) {
                        // RCLCPP_INFO(this->get_logger(), "   铲斗角度 (shovel_angle): %.2f° [启用]", shovel_angle);
                    } else {
                        // RCLCPP_INFO(this->get_logger(), "   铲斗角度 (shovel_angle): %.2f° [禁用]", shovel_angle);
                    }
                    if (shovel_angle < -800.0 || shovel_angle > 800.0) {
                        // RCLCPP_WARN(this->get_logger(), "     ⚠ 警告: shovel_angle 超出范围 [-800, 800]°");
                    }
                }
                
                if (cmd.has_estop()) {
                    // RCLCPP_INFO(this->get_logger(), "   紧急停止 (estop): %s", cmd.estop() ? "是" : "否");
                }
                
                if (cmd.has_parking_brake()) {
                    // RCLCPP_INFO(this->get_logger(), "   驻车制动 (parking_brake): %s", cmd.parking_brake() ? "是" : "否");
                }
                
                if (cmd.has_engine_on_off()) {
                    // RCLCPP_INFO(this->get_logger(), "   发动机 (engine_on_off): %s", cmd.engine_on_off() ? "开启" : "关闭");
                }
                
                // 打印新格式字段（保留格式但不映射到输出）
                if (data.find("leftTrack") != data.end() || data.find("rightTrack") != data.end() ||
                    data.find("swing") != data.end() || data.find("stick") != data.end() ||
                    data.find("speed_mode") != data.end() || data.find("light_code") != data.end() ||
                    data.find("hydraulic_lock") != data.end() || data.find("horn") != data.end()) {
                    // RCLCPP_INFO(this->get_logger(), "   新格式字段（保留格式，不映射到输出）:");
                    if (data.find("leftTrack") != data.end()) {
                        // RCLCPP_INFO(this->get_logger(), "     leftTrack: %.2f", SimpleJsonParser::get_double(data["leftTrack"]));
                    }
                    if (data.find("rightTrack") != data.end()) {
                        // RCLCPP_INFO(this->get_logger(), "     rightTrack: %.2f", SimpleJsonParser::get_double(data["rightTrack"]));
                    }
                    if (data.find("swing") != data.end()) {
                        // RCLCPP_INFO(this->get_logger(), "     swing: %.2f", SimpleJsonParser::get_double(data["swing"]));
                    }
                    if (data.find("stick") != data.end()) {
                        // RCLCPP_INFO(this->get_logger(), "     stick: %.2f", SimpleJsonParser::get_double(data["stick"]));
                    }
                    if (data.find("speed_mode") != data.end()) {
                        // RCLCPP_INFO(this->get_logger(), "     speed_mode: %s", data["speed_mode"].c_str());
                    }
                    if (data.find("light_code") != data.end()) {
                        // RCLCPP_INFO(this->get_logger(), "     light_code: 0x%02X", SimpleJsonParser::get_int(data["light_code"]));
                    }
                    if (data.find("hydraulic_lock") != data.end()) {
                        // RCLCPP_INFO(this->get_logger(), "     hydraulic_lock: %s", SimpleJsonParser::get_bool(data["hydraulic_lock"]) ? "true" : "false");
                    }
                    if (data.find("horn") != data.end()) {
                        // RCLCPP_INFO(this->get_logger(), "     horn: %s", SimpleJsonParser::get_bool(data["horn"]) ? "true" : "false");
                    }
                }
            }
            
            // 序列化 protobuf 消息
            std::string serialized_data;
            cmd.SerializeToString(&serialized_data);
            
            // 创建 ROS2 消息（序列化版本），可配置是否发布，关闭时直接跳过构造与发布
            if (publish_vehicle_command_) {  // 根据参数决定是否发布
                auto ros_msg = cyber_msgs::msg::ProtoAdapter();
                ros_msg.pb.assign(serialized_data.begin(), serialized_data.end());
                vehicle_cmd_pub_->publish(ros_msg);
            }
            
            // 创建并发布非序列化消息（用于调试和查看）
        if (publish_vehicle_command_debug_) {
            const auto now_ns = this->now().nanoseconds();
            std::ostringstream oss;
            oss << "{";
            oss << "\"stamp_ns\":" << now_ns << ",";
            oss << "\"stamp_sec\":" << (static_cast<double>(now_ns) / 1e9) << ",";
            oss << "\"frame_id\":\"base_link\",";
            oss << "\"steering_target\":" << cmd.steering_target() << ",";
            oss << "\"throttle\":" << cmd.throttle() << ",";
            oss << "\"brake\":" << cmd.brake() << ",";
            oss << "\"gear_location\":" << static_cast<int32_t>(cmd.gear_location()) << ",";
            oss << "\"speed\":" << (cmd.has_speed() ? cmd.speed() : 0.0) << ",";
            oss << "\"arm_angle\":" << (cmd.has_arm_angle() ? cmd.arm_angle() : 0.0) << ",";
            oss << "\"arm_enable\":" << (cmd.has_arm_angle() && cmd.arm_enable() ? "true" : "false") << ",";
            oss << "\"shovel_angle\":" << (cmd.has_shovel_angle() ? cmd.shovel_angle() : 0.0) << ",";
            oss << "\"shovel_enable\":" << (cmd.has_shovel_angle() && cmd.shovel_enable() ? "true" : "false") << ",";
            oss << "\"estop\":" << (cmd.has_estop() && cmd.estop() ? "true" : "false") << ",";
            oss << "\"parking_brake\":" << (cmd.has_parking_brake() && cmd.parking_brake() ? "true" : "false") << ",";
            oss << "\"engine_on_off\":" << (cmd.has_engine_on_off() && cmd.engine_on_off() ? "true" : "false");
            oss << "}";

            std_msgs::msg::String debug_msg;
            debug_msg.data = oss.str();
            vehicle_cmd_debug_pub_->publish(debug_msg);
        }
            
            // RCLCPP_INFO(this->get_logger(), "✅ 已发布到 /vehicle_command (protobuf 大小: %zu 字节)", serialized_data.size());
            // RCLCPP_INFO(this->get_logger(), "✅ 已发布到 /vehicle_command_debug (非序列化消息)");
            // RCLCPP_INFO(this->get_logger(), "============================================================");
            
        } catch (const std::exception& e) {
            // RCLCPP_ERROR(this->get_logger(), "处理控制命令时出错: %s", e.what());
        }
    }
    
    // 订阅者和发布者
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr teleop_sub_;
    rclcpp::Publisher<cyber_msgs::msg::ProtoAdapter>::SharedPtr vehicle_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr vehicle_cmd_debug_pub_;
    rclcpp::Subscription<cyber_msgs::msg::ChassisStatus>::SharedPtr chassis_status_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr chassis_feedback_pub_;
    
    // 死区参数
    double steering_deadzone_;
    double throttle_deadzone_;
    double brake_deadzone_;
    double boom_deadzone_;
    double bucket_deadzone_;
    
    // 角度映射范围
    double arm_angle_min_;
    double arm_angle_max_;
    double shovel_angle_min_;
    double shovel_angle_max_;
    
    // 速度限制
    double max_speed_;

    // 是否发布/vehicle_command
    bool publish_vehicle_command_;
    bool publish_vehicle_command_debug_;
    bool publish_chassis_feedback_;
    // 是否打印详细日志（降低CPU时可关闭）
    bool verbose_log_;
    
    // 上次的值（用于保持状态）
    control::canbus::Chassis::GearPosition last_gear_;
    double last_steering_;
    double last_throttle_;
    double last_brake_;
    double last_arm_angle_;
    double last_shovel_angle_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Teleop2CanTransformer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
