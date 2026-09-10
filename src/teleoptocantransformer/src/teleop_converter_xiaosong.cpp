#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <std_msgs/msg/string.hpp>
#include <cyber_msgs/msg/proto_adapter.hpp>
#include <geometry_msgs/msg/vector3.hpp>  // 挖掘机相对关节角 /excavator_joint_angles
#include "cannode/msg/rotary_encoder_angle.hpp"  // Danfoss 编码器角度
#include "cannode/msg/tilt_rpy.hpp"  // 车身倾角仪 roll/pitch/yaw（/tilt_body）
#include "msg_interfaces/msg/hcinspvatzcb.hpp"  // CGI-430 惯导 /chcnav/devpvt（包含经纬高 + 姿态）
#include <rclcpp/qos.hpp>
#include <chrono>
#include <cmath>
#include <cctype>
#include <string>
#include <sstream>
#include <map>
#include <functional>
#include <vector>
#include <algorithm>
#include "simple_json_parser.hpp"

namespace {

/** 从 TiltRPY 取用于限幅的角度（度），axis 不区分大小写：pitch / roll / yaw，默认 pitch */
double tilt_axis_value(const cannode::msg::TiltRPY & m, const std::string & axis_in)
{
    std::string a = axis_in;
    std::transform(a.begin(), a.end(), a.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (a == "roll") {
        return static_cast<double>(m.roll);
    }
    if (a == "yaw") {
        return static_cast<double>(m.yaw);
    }
    return static_cast<double>(m.pitch);
}

/** /cannode/engine_feedback JSON 中的 engine_speed_rpm；-1 表示尚无有效转速帧；解析失败视为 0 */
double parse_engine_speed_rpm_from_engine_feedback_json(const std::string& json)
{
    constexpr const char kKey[] = "\"engine_speed_rpm\":";
    const size_t pos = json.find(kKey);
    if (pos == std::string::npos) {
        return 0.0;
    }
    size_t i = pos + sizeof(kKey) - 1u;
    while (i < json.size() && (json[i] == ' ' || json[i] == '\t')) {
        ++i;
    }
    if (i >= json.size()) {
        return 0.0;
    }
    size_t end = i;
    while (end < json.size()) {
        const unsigned char c = static_cast<unsigned char>(json[end]);
        if (std::isdigit(c) || c == '.' || c == '-' || c == '+' || c == 'e' || c == 'E') {
            ++end;
        } else {
            break;
        }
    }
    try {
        return std::stod(json.substr(i, end - i));
    } catch (...) {
        return 0.0;
    }
}

}  // namespace

// Protobuf 头文件
#include "control_msgs/control_cmd.pb.h"
#include "common_msgs/chassis_msgs/chassis.pb.h"
#include "common_msgs/basic_msgs/header.pb.h"
#include <cyber_msgs/msg/chassis_status.hpp>
#include "cyber_msgs/msg/multi_camera_object_detection.hpp"

using namespace std::chrono_literals;

class Teleop2CanTransformerXiaosong : public rclcpp::Node
{
public:
    Teleop2CanTransformerXiaosong() : Node("teleop2can_transformer_xiaosong")
    {
        // 声明参数
        this->declare_parameter<double>("arm_deadzone", 0.05);      // 大臂死区
        this->declare_parameter<double>("stick_deadzone", 0.05);    // 斗杆死区
        this->declare_parameter<double>("bucket_deadzone", 0.05);    // 铲斗死区
        this->declare_parameter<double>("swing_deadzone", 0.05);    // 回转死区
        this->declare_parameter<double>("track_deadzone", 0.05);    // 履带死区
        // 各发布开关（yaml 可配置）
        this->declare_parameter<bool>("publish_vehicle_command", true);         // 是否发布 /vehicle_command
        this->declare_parameter<bool>("publish_vehicle_command_debug", true); // 是否发布 /vehicle_command_debug
        this->declare_parameter<bool>("publish_chassis_feedback", true);      // 是否发布 cannode/chassis_feedback
        this->declare_parameter<bool>("verbose_log", false);                  // 是否打印详细日志，默认关闭以降低CPU
        
        // 电流映射范围（mA）
        // xiaosong 协议：所有控制都是直接电流控制，范围 200~700mA
        this->declare_parameter<double>("max_current", 700.0);      // 最大电流 (mA)
        // 各动作方向独立：死区后幅度 |x| 与电流 (mA) 线性段；区间外钳位，再钳到 [0,max_current]
        declare_current_axis_curve_params_("arm_up");
        declare_current_axis_curve_params_("arm_down");
        declare_current_axis_curve_params_("stick_extend");
        declare_current_axis_curve_params_("stick_retract");
        declare_current_axis_curve_params_("bucket_close");
        declare_current_axis_curve_params_("bucket_dump");
        declare_current_axis_curve_params_("swing_right");
        declare_current_axis_curve_params_("swing_left");
        declare_current_axis_curve_params_("left_track_forward");
        declare_current_axis_curve_params_("left_track_backward");
        declare_current_axis_curve_params_("right_track_forward");
        declare_current_axis_curve_params_("right_track_backward");

        // 网络 QoS 阈值与滤波参数（yaml 可配置）
        this->declare_parameter<double>("network_qos_rtt_level2_ms", 150.0);
        this->declare_parameter<double>("network_qos_rtt_level3_ms", 300.0);
        this->declare_parameter<double>("network_qos_loss_level2_pct", 2.0);
        this->declare_parameter<double>("network_qos_loss_level3_pct", 10.0);
        this->declare_parameter<double>("network_qos_jitter_level2_ms", 30.0);
        this->declare_parameter<double>("network_qos_rtt_level3_hold_sec", 1.0);
        this->declare_parameter<double>("network_level2_input_limit", 0.4);
        this->declare_parameter<double>("network_level2_filter_alpha", 0.3);
        this->declare_parameter<int>("network_upgrade_required_count", 5);
        
        // 安全超时参数（可配置）
        this->declare_parameter<double>("teleop_timeout_sec", 1.0);  // /controls/teleop 超时时间（秒），默认1.0s

        // 倾角仪动作限幅（与 multi_tilt_imu 话题对应，默认关闭）
        this->declare_parameter<bool>("angle_limit_enable", false);
        this->declare_parameter<double>("boom_angle_min_deg", -360.0);
        this->declare_parameter<double>("boom_angle_max_deg", 360.0);
        this->declare_parameter<double>("stick_angle_min_deg", -360.0);
        this->declare_parameter<double>("stick_angle_max_deg", 360.0);
        this->declare_parameter<double>("bucket_angle_min_deg", -360.0);
        this->declare_parameter<double>("bucket_angle_max_deg", 360.0);
        this->declare_parameter<double>("body_angle_min_deg", -360.0);
        this->declare_parameter<double>("body_angle_max_deg", 360.0);
        this->declare_parameter<std::string>("tilt_boom_topic", "/tilt_boom");
        this->declare_parameter<std::string>("tilt_stick_topic", "/tilt_stick");
        this->declare_parameter<std::string>("tilt_bucket_topic", "/tilt_bucket");
        this->declare_parameter<std::string>("tilt_body_topic", "/tilt_body");
        this->declare_parameter<std::string>("angle_limit_boom_axis", "pitch");
        this->declare_parameter<std::string>("angle_limit_stick_axis", "pitch");
        this->declare_parameter<std::string>("angle_limit_bucket_axis", "pitch");
        this->declare_parameter<std::string>("angle_limit_body_axis", "yaw");
        // 倾角增大方向若与“抬升/伸出/收斗/右转”相反，置 true：达几何高端仍应禁止抬升而非误禁下降
        this->declare_parameter<bool>("boom_angle_limit_swap_sides", false);
        this->declare_parameter<bool>("stick_angle_limit_swap_sides", false);
        this->declare_parameter<bool>("bucket_angle_limit_swap_sides", false);
        this->declare_parameter<bool>("body_angle_limit_swap_sides", false);

        // /chcnav/devpvt 偏置补偿（可配置）
        // 单位：
        //  - latitude/longitude: deg
        //  - altitude: m
        //  - roll/pitch/yaw: deg
        this->declare_parameter<double>("devpvt_lat_bias_deg", 0.0);
        this->declare_parameter<double>("devpvt_lon_bias_deg", 0.0);
        this->declare_parameter<double>("devpvt_alt_bias_m", 0.0);
        this->declare_parameter<double>("devpvt_roll_bias_deg", 0.0);
        this->declare_parameter<double>("devpvt_pitch_bias_deg", 0.0);
        this->declare_parameter<double>("devpvt_yaw_bias_deg", 0.0);
        
        // 获取参数
        arm_deadzone_ = this->get_parameter("arm_deadzone").as_double();
        stick_deadzone_ = this->get_parameter("stick_deadzone").as_double();
        bucket_deadzone_ = this->get_parameter("bucket_deadzone").as_double();
        swing_deadzone_ = this->get_parameter("swing_deadzone").as_double();
        track_deadzone_ = this->get_parameter("track_deadzone").as_double();
        max_current_ = this->get_parameter("max_current").as_double();
        arm_up_current_curve_ = load_current_axis_curve_("arm_up");
        arm_down_current_curve_ = load_current_axis_curve_("arm_down");
        stick_extend_current_curve_ = load_current_axis_curve_("stick_extend");
        stick_retract_current_curve_ = load_current_axis_curve_("stick_retract");
        bucket_close_current_curve_ = load_current_axis_curve_("bucket_close");
        bucket_dump_current_curve_ = load_current_axis_curve_("bucket_dump");
        swing_right_current_curve_ = load_current_axis_curve_("swing_right");
        swing_left_current_curve_ = load_current_axis_curve_("swing_left");
        left_track_forward_current_curve_ = load_current_axis_curve_("left_track_forward");
        left_track_backward_current_curve_ = load_current_axis_curve_("left_track_backward");
        right_track_forward_current_curve_ = load_current_axis_curve_("right_track_forward");
        right_track_backward_current_curve_ = load_current_axis_curve_("right_track_backward");
        publish_vehicle_command_ = this->get_parameter("publish_vehicle_command").as_bool();
        publish_vehicle_command_debug_ = this->get_parameter("publish_vehicle_command_debug").as_bool();
        publish_chassis_feedback_ = this->get_parameter("publish_chassis_feedback").as_bool();
        verbose_log_ = this->get_parameter("verbose_log").as_bool();

        // 网络 QoS 阈值与滤波参数
        qos_rtt_level2_ms_ = this->get_parameter("network_qos_rtt_level2_ms").as_double();
        qos_rtt_level3_ms_ = this->get_parameter("network_qos_rtt_level3_ms").as_double();
        qos_loss_level2_pct_ = this->get_parameter("network_qos_loss_level2_pct").as_double();
        qos_loss_level3_pct_ = this->get_parameter("network_qos_loss_level3_pct").as_double();
        qos_jitter_level2_ms_ = this->get_parameter("network_qos_jitter_level2_ms").as_double();
        qos_rtt_level3_hold_sec_ = this->get_parameter("network_qos_rtt_level3_hold_sec").as_double();
        level2_input_limit_ = this->get_parameter("network_level2_input_limit").as_double();
        level2_filter_alpha_ = this->get_parameter("network_level2_filter_alpha").as_double();
        upgrade_required_count_ = this->get_parameter("network_upgrade_required_count").as_int();
        teleop_timeout_sec_ = this->get_parameter("teleop_timeout_sec").as_double();

        // /chcnav/devpvt 偏置补偿
        devpvt_lat_bias_deg_ = this->get_parameter("devpvt_lat_bias_deg").as_double();
        devpvt_lon_bias_deg_ = this->get_parameter("devpvt_lon_bias_deg").as_double();
        devpvt_alt_bias_m_ = this->get_parameter("devpvt_alt_bias_m").as_double();
        devpvt_roll_bias_deg_ = this->get_parameter("devpvt_roll_bias_deg").as_double();
        devpvt_pitch_bias_deg_ = this->get_parameter("devpvt_pitch_bias_deg").as_double();
        devpvt_yaw_bias_deg_ = this->get_parameter("devpvt_yaw_bias_deg").as_double();

        angle_limit_enable_ = this->get_parameter("angle_limit_enable").as_bool();
        boom_angle_min_deg_ = this->get_parameter("boom_angle_min_deg").as_double();
        boom_angle_max_deg_ = this->get_parameter("boom_angle_max_deg").as_double();
        stick_angle_min_deg_ = this->get_parameter("stick_angle_min_deg").as_double();
        stick_angle_max_deg_ = this->get_parameter("stick_angle_max_deg").as_double();
        bucket_angle_min_deg_ = this->get_parameter("bucket_angle_min_deg").as_double();
        bucket_angle_max_deg_ = this->get_parameter("bucket_angle_max_deg").as_double();
        body_angle_min_deg_ = this->get_parameter("body_angle_min_deg").as_double();
        body_angle_max_deg_ = this->get_parameter("body_angle_max_deg").as_double();
        tilt_boom_topic_ = this->get_parameter("tilt_boom_topic").as_string();
        tilt_stick_topic_ = this->get_parameter("tilt_stick_topic").as_string();
        tilt_bucket_topic_ = this->get_parameter("tilt_bucket_topic").as_string();
        tilt_body_topic_ = this->get_parameter("tilt_body_topic").as_string();
        angle_limit_boom_axis_ = this->get_parameter("angle_limit_boom_axis").as_string();
        angle_limit_stick_axis_ = this->get_parameter("angle_limit_stick_axis").as_string();
        angle_limit_bucket_axis_ = this->get_parameter("angle_limit_bucket_axis").as_string();
        angle_limit_body_axis_ = this->get_parameter("angle_limit_body_axis").as_string();
        boom_angle_limit_swap_sides_ = this->get_parameter("boom_angle_limit_swap_sides").as_bool();
        stick_angle_limit_swap_sides_ = this->get_parameter("stick_angle_limit_swap_sides").as_bool();
        bucket_angle_limit_swap_sides_ = this->get_parameter("bucket_angle_limit_swap_sides").as_bool();
        body_angle_limit_swap_sides_ = this->get_parameter("body_angle_limit_swap_sides").as_bool();

        // 允许运行时通过 ros2 param set 更新 devpvt 偏置（与 JSON 输出线程安全）
        devpvt_param_cb_handle_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> & parameters) -> rcl_interfaces::msg::SetParametersResult {
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                result.reason = "ok";
                std::lock_guard<std::mutex> lk(devpvt_bias_mutex_);
                for (const auto & p : parameters) {
                    const std::string & n = p.get_name();
                    if (n == "devpvt_lat_bias_deg") {
                        devpvt_lat_bias_deg_ = p.as_double();
                    } else if (n == "devpvt_lon_bias_deg") {
                        devpvt_lon_bias_deg_ = p.as_double();
                    } else if (n == "devpvt_alt_bias_m") {
                        devpvt_alt_bias_m_ = p.as_double();
                    } else if (n == "devpvt_roll_bias_deg") {
                        devpvt_roll_bias_deg_ = p.as_double();
                    } else if (n == "devpvt_pitch_bias_deg") {
                        devpvt_pitch_bias_deg_ = p.as_double();
                    } else if (n == "devpvt_yaw_bias_deg") {
                        devpvt_yaw_bias_deg_ = p.as_double();
                    }
                }
                return result;
            });

        RCLCPP_INFO(
            this->get_logger(),
            "chcnav_devpvt: 经纬高+yaw 来自 /chcnav/devpvt（偏置 lat=%.3f lon=%.3f alt_m=%.3f yaw=%.3f）；"
            "roll/pitch 取自 %s（与 roll_angle/pitch_angle 一致）。若改 yaml 不变请 colcon build 或 xiaosong_params_file:=<yaml>",
            devpvt_lat_bias_deg_, devpvt_lon_bias_deg_, devpvt_alt_bias_m_, devpvt_yaw_bias_deg_,
            tilt_body_topic_.c_str());
        
        // 创建订阅者（订阅远程端控制指令）
        // 使用 BEST_EFFORT QoS 以匹配发布者
        rclcpp::QoS teleop_qos(10);
        teleop_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        teleop_qos.durability(rclcpp::DurabilityPolicy::Volatile);
        teleop_qos.history(rclcpp::HistoryPolicy::KeepLast);
        
        teleop_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/controls/teleop", 
            teleop_qos,
            std::bind(&Teleop2CanTransformerXiaosong::teleop_callback, this, std::placeholders::_1)
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
            std::bind(&Teleop2CanTransformerXiaosong::chassis_status_callback, this, std::placeholders::_1)
        );

        chassis_feedback_pub_ = this->create_publisher<std_msgs::msg::String>(
            "cannode/chassis_feedback",
            chassis_qos
        );
        
        // 订阅 VCU 反馈数据（0x18FF1002）
        vcu_feedback_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/cannode/vcu_feedback",
            chassis_qos,
            std::bind(&Teleop2CanTransformerXiaosong::vcu_feedback_callback, this, std::placeholders::_1)
        );

        // 订阅发动机/VCU 扩展反馈数据（来自 cannode/engine_feedback_node）
        // 话题：/cannode/engine_feedback，消息：std_msgs/String，内容为 JSON
        engine_feedback_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/cannode/engine_feedback",
            chassis_qos,
            std::bind(&Teleop2CanTransformerXiaosong::engine_feedback_callback, this, std::placeholders::_1)
        );

        // 订阅多摄像头目标检测结果（来自感知模块）
        // 话题：/perception/object_detection，消息：cyber_msgs/msg/MultiCameraObjectDetection
        object_detection_sub_ = this->create_subscription<cyber_msgs::msg::MultiCameraObjectDetection>(
            "/perception/object_detection",
            chassis_qos,
            std::bind(&Teleop2CanTransformerXiaosong::object_detection_callback, this, std::placeholders::_1)
        );

        // 订阅挖掘机相对关节角（来自 multi_tilt_imu_node）
        // 话题：/excavator_joint_angles，消息：geometry_msgs/Vector3
        // x = 大臂相对车身 (deg), y = 斗杆相对大臂 (deg), z = 铲斗相对斗杆 (deg)
        excavator_angles_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/excavator_joint_angles",
            rclcpp::QoS(10).reliable(),
            std::bind(
                &Teleop2CanTransformerXiaosong::excavator_angles_callback,
                this,
                std::placeholders::_1));

        // 订阅 Danfoss 编码器角度（来自 danfoss_rotary_encoder_node）
        // 话题：/rotary_encoder_angle，消息：cannode/msg/RotaryEncoderAngle
        // angle1 用于回转角度（swing_angle）
        // 注意：使用 BestEffort QoS 以匹配发布者的 QoS 设置
        rclcpp::QoS rotary_encoder_qos(10);
        rotary_encoder_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        rotary_encoder_qos.durability(rclcpp::DurabilityPolicy::Volatile);
        rotary_encoder_sub_ = this->create_subscription<cannode::msg::RotaryEncoderAngle>(
            "/rotary_encoder_angle",
            rotary_encoder_qos,
            std::bind(
                &Teleop2CanTransformerXiaosong::rotary_encoder_callback,
                this,
                std::placeholders::_1));

        // 订阅各倾角仪（来自 multi_tilt_imu_node，话题名与 multi_tilt_imu.yaml 中 sensor_topics 一致，可在 YAML 覆盖）
        rclcpp::QoS tilt_imu_qos(50);
        tilt_imu_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        tilt_imu_qos.durability(rclcpp::DurabilityPolicy::Volatile);

        tilt_boom_sub_ = this->create_subscription<cannode::msg::TiltRPY>(
            tilt_boom_topic_,
            tilt_imu_qos,
            [this](const cannode::msg::TiltRPY::SharedPtr m) {
                std::lock_guard<std::mutex> lk(tilt_data_mutex_);
                tilt_boom_angle_deg_ = tilt_axis_value(*m, angle_limit_boom_axis_);
                tilt_boom_valid_ = true;
            });

        tilt_stick_sub_ = this->create_subscription<cannode::msg::TiltRPY>(
            tilt_stick_topic_,
            tilt_imu_qos,
            [this](const cannode::msg::TiltRPY::SharedPtr m) {
                std::lock_guard<std::mutex> lk(tilt_data_mutex_);
                tilt_stick_angle_deg_ = tilt_axis_value(*m, angle_limit_stick_axis_);
                tilt_stick_valid_ = true;
            });

        tilt_bucket_sub_ = this->create_subscription<cannode::msg::TiltRPY>(
            tilt_bucket_topic_,
            tilt_imu_qos,
            [this](const cannode::msg::TiltRPY::SharedPtr m) {
                std::lock_guard<std::mutex> lk(tilt_data_mutex_);
                tilt_bucket_angle_deg_ = tilt_axis_value(*m, angle_limit_bucket_axis_);
                tilt_bucket_valid_ = true;
            });

        tilt_body_sub_ = this->create_subscription<cannode::msg::TiltRPY>(
            tilt_body_topic_,
            tilt_imu_qos,
            std::bind(
                &Teleop2CanTransformerXiaosong::tilt_body_callback,
                this,
                std::placeholders::_1));

        // 订阅 CGI 惯导 /chcnav/devpvt（包含经纬高 + roll/pitch/yaw）
        devpvt_sub_ = this->create_subscription<msg_interfaces::msg::Hcinspvatzcb>(
            "/chcnav/devpvt",
            rclcpp::QoS(10).reliable(),
            std::bind(
                &Teleop2CanTransformerXiaosong::devpvt_callback,
                this,
                std::placeholders::_1));

        // 订阅网络 QoS 信息（来自前端 /excavator/network_qos）
        // 示例: {"cmd":"network_qos","rtt_ms":18.9,"packet_loss_pct":0.6,"jitter_ms":4.8}
        network_qos_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/excavator/network_qos",
            rclcpp::QoS(10).best_effort(),
            std::bind(
                &Teleop2CanTransformerXiaosong::network_qos_callback,
                this,
                std::placeholders::_1));

        // 订阅 cannode 下发的 CAN 帧（包括 0x18210005），用于计算 bucket_open_degree
        can_frame_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/can_frame_sent",
            rclcpp::QoS(50).best_effort(),
            std::bind(
                &Teleop2CanTransformerXiaosong::can_frame_sent_callback,
                this,
                std::placeholders::_1));
        
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
        
        // 创建超时检查定时器（每100ms检查一次）
        auto timeout_check_timer = this->create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() {
                this->check_teleop_timeout();
            }
        );
        
        // 初始化控制命令
        last_arm_up_current_ = 0.0;
        last_arm_down_current_ = 0.0;
        last_stick_retract_current_ = 0.0;
        last_stick_extend_current_ = 0.0;
        last_bucket_close_current_ = 0.0;
        last_bucket_dump_current_ = 0.0;
        last_rotate_left_current_ = 0.0;
        last_rotate_right_current_ = 0.0;
        last_left_track_forward_current_ = 0.0;
        last_left_track_backward_current_ = 0.0;
        last_right_track_forward_current_ = 0.0;
        last_right_track_backward_current_ = 0.0;
        last_request_remote_control_ = false;
        last_estop_ = false;
        last_engine_gear_ = 1;
        
        // 初始化 last_teleop_time_ 为当前时间（避免启动时立即触发超时）
        {
            std::lock_guard<std::mutex> lk(network_state_mutex_);
            last_teleop_time_ = this->now();
        }
        
        // RCLCPP_INFO(this->get_logger(), "Teleop2CanTransformerXiaosong 节点已启动（适配小松协议）");
        // RCLCPP_INFO(this->get_logger(), "死区设置: arm=%.3f, stick=%.3f, bucket=%.3f, swing=%.3f, track=%.3f",
        //             arm_deadzone_, stick_deadzone_, bucket_deadzone_, swing_deadzone_, track_deadzone_);
        // RCLCPP_INFO(this->get_logger(), "电流范围: 最大电流=%.1f mA",
        //             max_current_);
        // RCLCPP_INFO(this->get_logger(), "发布开关: publish_vehicle_command=%s, publish_vehicle_command_debug=%s, publish_chassis_feedback=%s",
        //             publish_vehicle_command_ ? "true" : "false",
        //             publish_vehicle_command_debug_ ? "true" : "false",
        //             publish_chassis_feedback_ ? "true" : "false");
        // RCLCPP_INFO(this->get_logger(), "详细日志: verbose_log=%s (默认关闭以降低CPU)", verbose_log_ ? "true" : "false");
    }

private:
    // 单轴电流曲线：死区后幅度 |x| 与输出电流 (mA)
    struct CurrentAxisCurve {
        double input_min{0.0};
        double input_max{1.0};
        double output_min{0.0};
        double output_max{700.0};
    };

    // 网络降级级别
    enum class NetworkDegradeLevel {
        GREEN = 1,   // 正常
        YELLOW = 2,  // 警告 / 降级：速度限制 + 平滑
        RED = 3      // 危险：禁止所有运动指令，但允许安全指令
    };

    // 网络 QoS 阈值与滤波参数（可通过 YAML 配置）
    double qos_rtt_level2_ms_;        // 级别 2 起始 RTT
    double qos_rtt_level3_ms_;        // 级别 3 起始 RTT
    double qos_loss_level2_pct_;      // 级别 2 起始丢包率
    double qos_loss_level3_pct_;      // 级别 3 严重丢包
    double qos_jitter_level2_ms_;     // 级别 2 抖动阈值
    double qos_rtt_level3_hold_sec_;  // RTT>300ms 持续时间

    // 级别 2：最大输入幅度与平滑系数（可配置）
    double level2_input_limit_;       // 将最大速度限制在约 40%
    double level2_filter_alpha_;      // 一阶低通滤波系数

    // 升级恢复所需的连续样本数量（可配置）
    int upgrade_required_count_{5};
    
    // 安全超时参数
    double teleop_timeout_sec_{1.0};  // /controls/teleop 超时时间（秒）
    int yellow_recovery_count_{0};    // 从 YELLOW 恢复到 GREEN 的计数
    int red_recovery_count_{0};       // 从 RED 恢复到 YELLOW/GREEN 的计数

    // 铲斗开度状态（根据 0x18210005 的开斗/关斗 bit 记忆）
    std::mutex bucket_open_mutex_;
    double bucket_open_degree_state_{0.0};  // 0=关斗（默认），45=开斗

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
    
    // 死区后幅度 |x| → 电流：在 [input_min,input_max] 上线性映射到 [output_min,output_max]；区间外钳位
    double input_to_current(double input, double deadzone, const CurrentAxisCurve& curve)
    {
        input = apply_deadzone(input, deadzone);
        const double a = std::abs(input);
        if (a < 1e-12) {
            return 0.0;
        }

        const double in_min = curve.input_min;
        const double in_max = curve.input_max;
        const double out_min = curve.output_min;
        const double out_max = curve.output_max;
        const double out_floor = std::min(out_min, out_max);
        const double out_ceil = std::max(out_min, out_max);

        double raw;
        if (in_max <= in_min + 1e-9) {
            raw = a * out_max;
        } else if (a <= in_min) {
            raw = out_min;
        } else if (a >= in_max) {
            raw = out_max;
        } else {
            const double t = (a - in_min) / (in_max - in_min);
            raw = out_min + t * (out_max - out_min);
        }

        raw = clamp(raw, out_floor, out_ceil);
        return clamp(raw, 0.0, max_current_);
    }

    void declare_current_axis_curve_params_(const std::string& axis_prefix)
    {
        const std::string pre = axis_prefix + "_current_";
        this->declare_parameter<double>(pre + "input_min", 0.0);
        this->declare_parameter<double>(pre + "input_max", 1.0);
        this->declare_parameter<double>(pre + "output_min", 0.0);
        this->declare_parameter<double>(pre + "output_max", 700.0);
    }

    CurrentAxisCurve load_current_axis_curve_(const std::string& axis_prefix) const
    {
        const std::string pre = axis_prefix + "_current_";
        CurrentAxisCurve c;
        c.input_min = this->get_parameter(pre + "input_min").as_double();
        c.input_max = this->get_parameter(pre + "input_max").as_double();
        c.output_min = this->get_parameter(pre + "output_min").as_double();
        c.output_max = this->get_parameter(pre + "output_max").as_double();
        return c;
    }

    // 将 ChassisStatus 转成 JSON 字符串
    std::string chassis_status_to_json(const cyber_msgs::msg::ChassisStatus & msg)
    {
        auto bool_to_str = [](bool v) { return v ? "true" : "false"; };
        // 安全输出浮点数，NaN/Inf 转换为 0
        auto safe_float = [](float v) -> float {
            if (std::isnan(v) || std::isinf(v)) {
                return 0.0f;
            }
            return v;
        };
        std::ostringstream oss;
        oss << "{";
        oss << "\"header\":{";
        oss << "\"stamp\":{\"sec\":" << msg.header.stamp.sec << ",\"nanosec\":" << msg.header.stamp.nanosec << "},";
        oss << "\"frame_id\":\"" << msg.header.frame_id << "\"";
        oss << "},";

        // 基本状态
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
        oss << "\"front_rear_angle\":" << safe_float(msg.front_rear_angle) << ",";
        oss << "\"battery_level\":" << static_cast<int>(msg.battery_level) << ",";
        oss << "\"charging_status\":" << bool_to_str(msg.charging_status) << ",";
        oss << "\"hydraulic_lock\":" << bool_to_str(msg.hydraulic_lock) << ",";
        oss << "\"fault_level\":" << static_cast<int>(msg.fault_level) << ",";
        oss << "\"turtle_rabbit_gear\":" << bool_to_str(msg.turtle_rabbit_gear) << ",";
        oss << "\"work_light\":" << bool_to_str(msg.work_light) << ",";
        oss << "\"vehicle_mode\":" << static_cast<int>(msg.vehicle_mode) << ",";

        // 电流和压力
        oss << "\"boom_lift_current\":" << msg.boom_lift_current << ",";
        oss << "\"boom_lower_current\":" << msg.boom_lower_current << ",";
        oss << "\"bucket_close_current\":" << msg.bucket_close_current << ",";
        oss << "\"bucket_open_current\":" << msg.bucket_open_current << ",";
        oss << "\"boom_big_pressure\":" << msg.boom_big_pressure << ",";
        oss << "\"boom_small_pressure\":" << msg.boom_small_pressure << ",";
        oss << "\"bucket_big_pressure\":" << msg.bucket_big_pressure << ",";
        oss << "\"bucket_small_pressure\":" << msg.bucket_small_pressure << ",";

        // 电机状态
        oss << "\"hydraulic_motor_speed\":" << msg.hydraulic_motor_speed << ",";
        oss << "\"hydraulic_motor_torque\":" << msg.hydraulic_motor_torque << ",";
        oss << "\"hydraulic_motor_current\":" << msg.hydraulic_motor_current << ",";
        oss << "\"hydraulic_motor_enable\":" << bool_to_str(msg.hydraulic_motor_enable) << ",";
        oss << "\"walk_motor_current\":" << msg.walk_motor_current << ",";
        oss << "\"walk_motor_torque\":" << msg.walk_motor_torque << ",";
        // 安全处理 walk_motor_speed，避免 NaN 值
        float safe_walk_speed = safe_float(msg.walk_motor_speed);
        oss << "\"walk_motor_speed\":" << safe_walk_speed << ",";
        oss << "\"vehicle_speed\":" << safe_float(safe_walk_speed * 3.6f) << ",";
        oss << "\"walk_motor_enable\":" << bool_to_str(msg.walk_motor_enable) << ",";

        // 当前网络降级级别（1=GREEN,2=YELLOW,3=RED,4=BLACK）
        int network_level_int = 1;
        {
            std::lock_guard<std::mutex> lk(network_state_mutex_);
            network_level_int = static_cast<int>(network_level_);
        }
        oss << "\"network_degrade_level\":" << network_level_int << ",";
        oss << "\"action_limit_exceed\":" << compute_action_limit_exceed() << ",";

        // CGI 惯导 /chcnav/devpvt：经纬高 + yaw 来自 /chcnav/devpvt；roll/pitch 与 roll_angle/pitch_angle 同源（/tilt_body）
        {
            bool devpvt_ok = false;
            double ins_lat{0.0};
            double ins_lon{0.0};
            double ins_alt{0.0};
            double ins_yaw{0.0};
            {
                std::lock_guard<std::mutex> lk(devpvt_mutex_);
                if (devpvt_valid_) {
                    devpvt_ok = true;
                    ins_lat = static_cast<double>(latest_devpvt_.latitude);
                    ins_lon = static_cast<double>(latest_devpvt_.longitude);
                    ins_alt = static_cast<double>(latest_devpvt_.altitude);
                    ins_yaw = static_cast<double>(latest_devpvt_.yaw);
                }
            }
            double tilt_roll{0.0};
            double tilt_pitch{0.0};
            bool tilt_ok = false;
            {
                std::lock_guard<std::mutex> lk(tilt_data_mutex_);
                if (tilt_body_valid_) {
                    tilt_ok = true;
                    tilt_roll = tilt_body_roll_deg_;
                    tilt_pitch = tilt_body_pitch_deg_;
                }
            }
            if (devpvt_ok) {
                // latitude / longitude 保留 6 位小数，altitude/roll/pitch/yaw 保留 1 位小数
                auto fmt6 = [](double v) {
                    std::ostringstream s;
                    s.setf(std::ios::fixed);
                    s.precision(6);
                    s << v;
                    return s.str();
                };
                auto fmt1 = [](double v) {
                    std::ostringstream s;
                    s.setf(std::ios::fixed);
                    s.precision(1);
                    s << v;
                    return s.str();
                };

                oss << "\"chcnav_devpvt\":{";
                double lat_bias{0.0};
                double lon_bias{0.0};
                double alt_bias{0.0};
                double yaw_bias{0.0};
                {
                    std::lock_guard<std::mutex> lk_bias(devpvt_bias_mutex_);
                    lat_bias = devpvt_lat_bias_deg_;
                    lon_bias = devpvt_lon_bias_deg_;
                    alt_bias = devpvt_alt_bias_m_;
                    yaw_bias = devpvt_yaw_bias_deg_;
                }
                const double lat_deg = ins_lat + lat_bias;
                const double lon_deg = ins_lon + lon_bias;
                const double alt_m = ins_alt + alt_bias;
                const double roll_deg = tilt_ok ? tilt_roll : 0.0;
                const double pitch_deg = tilt_ok ? tilt_pitch : 0.0;
                const double yaw_deg = ins_yaw + yaw_bias;

                oss << "\"latitude\":"  << fmt6(lat_deg)  << ",";
                oss << "\"longitude\":" << fmt6(lon_deg) << ",";
                oss << "\"altitude\":"  << fmt1(alt_m)   << ",";
                // 与下方 roll_angle / pitch_angle 使用相同数值与 safe_float 格式
                oss << "\"roll\":"  << safe_float(static_cast<float>(roll_deg)) << ",";
                oss << "\"pitch\":" << safe_float(static_cast<float>(pitch_deg)) << ",";
                // yaw 仍用 GNSS + devpvt_yaw_bias_deg（与改 tilt 前一致，fmt1）
                oss << "\"yaw\":" << fmt1(yaw_deg);
                oss << "},";
            }
        }

        // 角度信息（优先使用挖掘机 IMU 计算得到的相对角）
        double boom_angle = msg.boom_angle;
        double bucket_angle = msg.bucket_angle;
        double stick_angle = 0.0;
        {
            std::lock_guard<std::mutex> lk(excavator_angle_mutex_);
            if (excavator_angles_valid_) {
                // x: 大臂对车身, y: 斗杆对大臂, z: 铲斗对斗杆
                boom_angle = excavator_boom_angle_deg_;
                stick_angle = excavator_stick_angle_deg_;
                bucket_angle = excavator_bucket_angle_deg_;
            }
        }
        // 安全处理角度值，避免 NaN
        oss << "\"boom_angle\":" << safe_float(static_cast<float>(boom_angle)) << ",";
        oss << "\"stick_angle\":" << safe_float(static_cast<float>(stick_angle)) << ",";
        oss << "\"bucket_angle\":" << safe_float(static_cast<float>(-bucket_angle)) << ",";  // 方向取反

        // 铲斗开度（根据下行 CAN 控制 0x18210005 的开斗/关斗位进行“记忆式”控制）
        double bucket_open_degree = 0.0;
        {
            std::lock_guard<std::mutex> lk(bucket_open_mutex_);
            bucket_open_degree = bucket_open_degree_state_;
        }
        oss << "\"bucket_open_degree\":" << safe_float(static_cast<float>(bucket_open_degree)) << ",";

        // 车身姿态角（来自 /tilt_body，单位度）
        double roll_angle = 0.0;
        double pitch_angle = 0.0;
        {
            std::lock_guard<std::mutex> lk(tilt_data_mutex_);
            if (tilt_body_valid_) {
                roll_angle = tilt_body_roll_deg_;
                pitch_angle = tilt_body_pitch_deg_;
            }
        }
        oss << "\"roll_angle\":" << safe_float(static_cast<float>(roll_angle)) << ",";
        oss << "\"pitch_angle\":" << safe_float(static_cast<float>(pitch_angle)) << ",";
        
        // 回转角度（来自 Danfoss 编码器）
        double swing_angle = 0.0;
        {
            std::lock_guard<std::mutex> lk(rotary_encoder_mutex_);
            if (rotary_encoder_valid_) {
                swing_angle = rotary_encoder_angle1_deg_;
            }
        }
        oss << "\"swing_angle\":" << safe_float(static_cast<float>(swing_angle));
        
        // VCU 反馈数据 (0x18FF1002) - 从订阅的话题获取
        // 根据 excavator-can-vcu2host_utf8.csv: 周期50ms, VCU →上位机
        // bit 0: 车梯收起到位信号, IN_43_DI(115#)
        bool vcu_communication_timeout = false;  // VCU通讯超时标志
        {
            std::lock_guard<std::mutex> lk(vcu_feedback_mutex_);
            if (!vcu_feedback_json_.empty()) {
                // 解析车梯收起到位信号（从 vcu_feedback JSON 中提取 in_43_di）
                // in_43_di 在 digital_inputs 对象中，格式为 "in_43_di":true 或 "in_43_di":false
                // bit 0: 车梯收起到位信号, IN_43_DI(115#)
                bool ladder_retract_ok = false;
                size_t in_43_pos = vcu_feedback_json_.find("\"in_43_di\":");
                if (in_43_pos != std::string::npos) {
                    // 查找值开始位置（跳过冒号和可能的空格）
                    size_t value_start = vcu_feedback_json_.find_first_not_of(" :", in_43_pos + 11);
                    if (value_start != std::string::npos) {
                        // 检查是 true 还是 false
                        if (value_start + 4 <= vcu_feedback_json_.length() && 
                            vcu_feedback_json_.substr(value_start, 4) == "true") {
                            ladder_retract_ok = true;
                        } else if (value_start + 5 <= vcu_feedback_json_.length() && 
                                   vcu_feedback_json_.substr(value_start, 5) == "false") {
                            ladder_retract_ok = false;
                        }
                    }
                }
                
                // 解析 vcu_communication_timeout 字段（从 vcu_feedback JSON 中提取）
                size_t timeout_pos = vcu_feedback_json_.find("\"vcu_communication_timeout\":");
                if (timeout_pos != std::string::npos) {
                    // 查找值开始位置（跳过冒号和可能的空格）
                    size_t value_start = vcu_feedback_json_.find_first_not_of(" :", timeout_pos + 28);
                    if (value_start != std::string::npos) {
                        // 检查是 true 还是 false
                        if (value_start + 4 <= vcu_feedback_json_.length() && 
                            vcu_feedback_json_.substr(value_start, 4) == "true") {
                            vcu_communication_timeout = true;
                        } else if (value_start + 5 <= vcu_feedback_json_.length() && 
                                   vcu_feedback_json_.substr(value_start, 5) == "false") {
                            vcu_communication_timeout = false;
                        }
                    }
                }
                
                // 移除 JSON 字符串的外层大括号，移除 vcu_communication_timeout 字段，添加 ladder_retract_ok 字段
                std::string vcu_data = vcu_feedback_json_;
                if (vcu_data.front() == '{' && vcu_data.back() == '}') {
                    vcu_data = vcu_data.substr(1, vcu_data.length() - 2);
                }
                // 移除 vcu_communication_timeout 字段（如果存在）
                size_t timeout_field_pos = vcu_data.find("\"vcu_communication_timeout\":");
                if (timeout_field_pos != std::string::npos) {
                    // 找到字段的结束位置（下一个逗号或结束），保留该逗号以分隔后续的 digital_inputs
                    size_t field_end = vcu_data.find_first_of(",}", timeout_field_pos);
                    if (field_end != std::string::npos) {
                        // 移除该字段（包括前面的逗号），但不移除 field_end 处的逗号
                        size_t field_start = timeout_field_pos;
                        if (field_start > 0 && vcu_data[field_start - 1] == ',') {
                            field_start--;  // 包括前面的逗号
                        }
                        vcu_data.erase(field_start, field_end - field_start);
                    }
                }
                // 在 vcu_feedback 对象中添加 ladder_retract_ok 字段
                oss << ",\"vcu_feedback\":{" << vcu_data << ",\"ladder_retract_ok\":" << bool_to_str(ladder_retract_ok) << "}";
            } else {
                // 如果没有收到 VCU 反馈数据，使用默认值（包括超时标志）
                vcu_communication_timeout = true;  // 没有收到数据，认为超时
                oss << ",\"vcu_feedback\":{";
                oss << "\"digital_inputs\":{";
                oss << "\"in_43_di\":false,\"in_44_di\":false,\"in_45_di\":false,\"in_46_di\":false,";
                oss << "\"in_47_di\":false,\"in_48_di\":false,\"in_49_di\":false,\"in_50_di\":false";
                oss << "},";
                oss << "\"is_remote\":false,\"is_emergency\":false,";
                oss << "\"hydraulic_lock_status\":false,";
                oss << "\"ladder_retract_ok\":false";
                oss << "}";
            }
        }
        
        // 添加 VCU 通讯超时字段到主 JSON 的顶层
        oss << ",\"vcu_communication_timeout\":" << bool_to_str(vcu_communication_timeout);

        // 发动机/VCU 扩展反馈数据（来自 /cannode/engine_feedback）
        {
            std::lock_guard<std::mutex> lk(engine_feedback_mutex_);
            if (!engine_feedback_json_.empty()) {
                std::string eng_data = engine_feedback_json_;
                if (eng_data.front() == '{' && eng_data.back() == '}') {
                    eng_data = eng_data.substr(1, eng_data.length() - 2);
                }
                oss << ",\"engine_feedback\":{" << eng_data << "}";
            }
        }

        // 多摄像头目标检测结果（来自 /perception/object_detection）
        {
            std::lock_guard<std::mutex> lk(object_detection_mutex_);
            if (object_detection_valid_) {
                oss << ",\"object_detection\":{";
                oss << "\"stamp\":{\"sec\":" << latest_object_detection_.stamp.sec 
                    << ",\"nanosec\":" << latest_object_detection_.stamp.nanosec << "},";
                oss << "\"camera_count\":" << static_cast<int>(latest_object_detection_.camera_count) << ",";
                oss << "\"cameras\":[";
                for (size_t i = 0; i < latest_object_detection_.cameras.size(); ++i) {
                    if (i > 0) oss << ",";
                    const auto& camera = latest_object_detection_.cameras[i];
                    oss << "{";
                    oss << "\"camera_id\":\"" << camera.camera_id << "\",";
                    oss << "\"count\":" << camera.count << ",";
                    oss << "\"objects\":[";
                    for (size_t j = 0; j < camera.objects.size(); ++j) {
                        if (j > 0) oss << ",";
                        const auto& obj = camera.objects[j];
                        oss << "{";
                        oss << "\"id\":" << obj.id << ",";
                        oss << "\"class_name\":\"" << obj.class_name << "\",";
                        oss << "\"confidence\":" << safe_float(obj.confidence) << ",";
                        oss << "\"center_x\":" << safe_float(obj.center_x) << ",";
                        oss << "\"center_y\":" << safe_float(obj.center_y) << ",";
                        oss << "\"width\":" << safe_float(obj.width) << ",";
                        oss << "\"height\":" << safe_float(obj.height);
                        oss << "}";
                    }
                    oss << "]";
                    oss << "}";
                }
                oss << "]";
                oss << "}";
            }
        }

        oss << "}";
        return oss.str();
    }

    // VCU 反馈数据订阅回调
    void vcu_feedback_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lk(vcu_feedback_mutex_);
        vcu_feedback_json_ = msg->data;
    }

    // 发动机/VCU 扩展反馈订阅回调
    void engine_feedback_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lk(engine_feedback_mutex_);
        engine_feedback_json_ = msg->data;
    }

    // 多摄像头目标检测订阅回调
    void object_detection_callback(const cyber_msgs::msg::MultiCameraObjectDetection::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lk(object_detection_mutex_);
        latest_object_detection_ = *msg;
        object_detection_valid_ = true;
    }

    // CGI 惯导 /chcnav/devpvt 订阅回调（经纬高 + 姿态）
    void devpvt_callback(const msg_interfaces::msg::Hcinspvatzcb::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lk(devpvt_mutex_);
        latest_devpvt_ = *msg;
        devpvt_valid_ = true;
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
            
            // 记录最新 teleop 心跳，并基于 QoS/心跳更新当前降级级别
            auto now = this->now();
            NetworkDegradeLevel level_snapshot;
            bool was_timeout = false;
            {
                std::lock_guard<std::mutex> lk(network_state_mutex_);
                // 检查之前是否处于超时状态
                double elapsed = (now - last_teleop_time_).seconds();
                was_timeout = (elapsed > teleop_timeout_sec_);
                last_teleop_time_ = now;
                update_network_level_locked(now);
                level_snapshot = network_level_;
            }
            
            // 如果之前处于超时状态，现在收到新指令，记录恢复日志
            if (was_timeout) {
                RCLCPP_INFO(this->get_logger(), 
                    "✅ 已恢复：收到新的 /controls/teleop 指令，恢复正常发送");
            }
            
            // 解析 JSON
            std::map<std::string, std::string> data = SimpleJsonParser::parse(msg->data);
            
            // 创建 ControlCommand protobuf 消息
            control::ControlCommand cmd;
            
            // 设置 header
            auto* header = cmd.mutable_header();
            header->set_timestamp_sec(now.seconds());
            header->set_frame_id("base_link");
            
            // 初始化所有电流字段为上次值（确保即使值不变，字段也会被设置）
            // 这样可以避免 protobuf 反序列化后 has_xxx() 返回 false 导致使用默认值 0
            cmd.set_arm_up_current(last_arm_up_current_);
            cmd.set_arm_down_current(last_arm_down_current_);
            cmd.set_stick_retract_current(last_stick_retract_current_);
            cmd.set_stick_extend_current(last_stick_extend_current_);
            cmd.set_bucket_close_current(last_bucket_close_current_);
            cmd.set_bucket_dump_current(last_bucket_dump_current_);
            cmd.set_rotate_left_current(last_rotate_left_current_);
            cmd.set_rotate_right_current(last_rotate_right_current_);
            cmd.set_left_track_forward_current(last_left_track_forward_current_);
            cmd.set_left_track_backward_current(last_left_track_backward_current_);
            cmd.set_right_track_forward_current(last_right_track_forward_current_);
            cmd.set_right_track_backward_current(last_right_track_backward_current_);
            
            // 如果处于危险状态，直接锁死所有运动指令，仅保留后续的安全 / 辅助指令
            if (level_snapshot == NetworkDegradeLevel::RED) {
                // 清零所有运动相关电流
                cmd.set_arm_up_current(0.0);
                cmd.set_arm_down_current(0.0);
                cmd.set_stick_retract_current(0.0);
                cmd.set_stick_extend_current(0.0);
                cmd.set_bucket_close_current(0.0);
                cmd.set_bucket_dump_current(0.0);
                cmd.set_rotate_left_current(0.0);
                cmd.set_rotate_right_current(0.0);
                cmd.set_left_track_forward_current(0.0);
                cmd.set_left_track_backward_current(0.0);
                cmd.set_right_track_forward_current(0.0);
                cmd.set_right_track_backward_current(0.0);

                // 覆盖缓存，避免“保持上次值”恢复运动
                last_arm_up_current_ = 0.0;
                last_arm_down_current_ = 0.0;
                last_stick_retract_current_ = 0.0;
                last_stick_extend_current_ = 0.0;
                last_bucket_close_current_ = 0.0;
                last_bucket_dump_current_ = 0.0;
                last_rotate_left_current_ = 0.0;
                last_rotate_right_current_ = 0.0;
                last_left_track_forward_current_ = 0.0;
                last_left_track_backward_current_ = 0.0;
                last_right_track_forward_current_ = 0.0;
                last_right_track_backward_current_ = 0.0;
            } else {
                // 级别 1/2：根据网络状态进行速度限制和平滑滤波
            
            // 处理大臂控制 (boom: -1..1 -> arm_up_current / arm_down_current)
            // 修正：0 到 1：抬升方向，映射到 arm_up_current 0~700
            // -1 到 0：下降方向，映射到 arm_down_current 700~0
            if (data.find("boom") != data.end()) {
                double boom_input = clamp(SimpleJsonParser::get_double(data["boom"]), -1.0, 1.0);
                    // 根据网络降级状态做速度限制和平滑
                    boom_input = apply_network_filter_axis(boom_input, filtered_boom_input_);
                
                if (boom_input > 0) {
                    // 抬升：0~1 映射到 0~700
                    double current = input_to_current(boom_input, arm_deadzone_, arm_up_current_curve_);
                    cmd.set_arm_up_current(current);
                    cmd.set_arm_down_current(0.0);
                    last_arm_up_current_ = current;
                    last_arm_down_current_ = 0.0;
                } else if (boom_input < 0) {
                    // 下降：-1~0 映射到 700~0
                    double current = input_to_current(boom_input, arm_deadzone_, arm_down_current_curve_);
                    cmd.set_arm_up_current(0.0);
                    cmd.set_arm_down_current(current);
                    last_arm_up_current_ = 0.0;
                    last_arm_down_current_ = current;
                } else {
                    // 无输入
                    cmd.set_arm_up_current(0.0);
                    cmd.set_arm_down_current(0.0);
                    last_arm_up_current_ = 0.0;
                    last_arm_down_current_ = 0.0;
                }
            } else {
                // 保持上次值
                cmd.set_arm_up_current(last_arm_up_current_);
                cmd.set_arm_down_current(last_arm_down_current_);
            }
            
            // 处理斗杆控制 (stick: -1..1 -> stick_retract_current / stick_extend_current)
            // 注意：根据你反馈的实际动作方向，这里约定：
            //   stick > 0 表示斗杆伸出；stick < 0 表示斗杆收回
            //   伸出：0~1 映射到 stick_extend_current 0~700
            //   收回：-1~0 映射到 stick_retract_current 700~0
            if (data.find("stick") != data.end()) {
                double stick_input = clamp(SimpleJsonParser::get_double(data["stick"]), -1.0, 1.0);
                    stick_input = apply_network_filter_axis(stick_input, filtered_stick_input_);
                
                if (stick_input > 0) {
                    // 伸出：0~1 映射到 0~700（extend）
                    double current = input_to_current(stick_input, stick_deadzone_, stick_extend_current_curve_);
                    cmd.set_stick_retract_current(0.0);
                    cmd.set_stick_extend_current(current);
                    last_stick_retract_current_ = 0.0;
                    last_stick_extend_current_ = current;
                } else if (stick_input < 0) {
                    // 收回：-1~0 映射到 700~0（retract）
                    double current = input_to_current(stick_input, stick_deadzone_, stick_retract_current_curve_);
                    cmd.set_stick_retract_current(current);
                    cmd.set_stick_extend_current(0.0);
                    last_stick_retract_current_ = current;
                    last_stick_extend_current_ = 0.0;
                } else {
                    // 无输入
                    cmd.set_stick_retract_current(0.0);
                    cmd.set_stick_extend_current(0.0);
                    last_stick_retract_current_ = 0.0;
                    last_stick_extend_current_ = 0.0;
                }
            } else {
                // 保持上次值
                cmd.set_stick_retract_current(last_stick_retract_current_);
                cmd.set_stick_extend_current(last_stick_extend_current_);
            }
            
            // 处理铲斗控制 (bucket: -1..1 -> bucket_close_current / bucket_dump_current)
            // 0 到 1：收斗方向，映射到 bucket_close_current 0~700
            // -1 到 0：翻斗方向，映射到 bucket_dump_current 700~0
            if (data.find("bucket") != data.end()) {
                double bucket_input = clamp(SimpleJsonParser::get_double(data["bucket"]), -1.0, 1.0);
                    bucket_input = apply_network_filter_axis(bucket_input, filtered_bucket_input_);
                
                if (bucket_input > 0) {
                    // 收斗：0~1 映射到 0~700
                    double current = input_to_current(bucket_input, bucket_deadzone_, bucket_close_current_curve_);
                    cmd.set_bucket_close_current(current);
                    cmd.set_bucket_dump_current(0.0);
                    last_bucket_close_current_ = current;
                    last_bucket_dump_current_ = 0.0;
                } else if (bucket_input < 0) {
                    // 翻斗：-1~0 映射到 700~0
                    double current = input_to_current(bucket_input, bucket_deadzone_, bucket_dump_current_curve_);
                    cmd.set_bucket_close_current(0.0);
                    cmd.set_bucket_dump_current(current);
                    last_bucket_close_current_ = 0.0;
                    last_bucket_dump_current_ = current;
                } else {
                    // 无输入
                    cmd.set_bucket_close_current(0.0);
                    cmd.set_bucket_dump_current(0.0);
                    last_bucket_close_current_ = 0.0;
                    last_bucket_dump_current_ = 0.0;
                }
            } else {
                // 保持上次值
                cmd.set_bucket_close_current(last_bucket_close_current_);
                cmd.set_bucket_dump_current(last_bucket_dump_current_);
            }
            
            // 处理回转控制 (swing: -1..1 -> rotate_left_current / rotate_right_current)
            // -1 到 0：左转方向，映射到 rotate_left_current 700~0
            // 0 到 1：右转方向，映射到 rotate_right_current 0~700
            if (data.find("swing") != data.end()) {
                double swing_input = clamp(SimpleJsonParser::get_double(data["swing"]), -1.0, 1.0);
                    swing_input = apply_network_filter_axis(swing_input, filtered_swing_input_);
                
                if (swing_input > 0) {
                    // 右转：0~1 映射到 0~700
                    double current = input_to_current(swing_input, swing_deadzone_, swing_right_current_curve_);
                    cmd.set_rotate_left_current(0.0);
                    cmd.set_rotate_right_current(current);
                    last_rotate_left_current_ = 0.0;
                    last_rotate_right_current_ = current;
                } else if (swing_input < 0) {
                    // 左转：-1~0 映射到 700~0
                    double current = input_to_current(swing_input, swing_deadzone_, swing_left_current_curve_);
                    cmd.set_rotate_left_current(current);
                    cmd.set_rotate_right_current(0.0);
                    last_rotate_left_current_ = current;
                    last_rotate_right_current_ = 0.0;
                } else {
                    // 无输入
                    cmd.set_rotate_left_current(0.0);
                    cmd.set_rotate_right_current(0.0);
                    last_rotate_left_current_ = 0.0;
                    last_rotate_right_current_ = 0.0;
                }
            } else {
                // 保持上次值
                cmd.set_rotate_left_current(last_rotate_left_current_);
                cmd.set_rotate_right_current(last_rotate_right_current_);
            }
            
            // 处理左履带控制 (leftTrack/left_track: -1..1 -> left_track_forward_current / left_track_backward_current)
            // 0 到 1：前进方向，映射到 left_track_forward_current 0~700
            // -1 到 0：后退方向，映射到 left_track_backward_current 700~0
            {
                bool has_left_track = false;
                double left_track_input = 0.0;
                auto it_left_camel = data.find("leftTrack");
                auto it_left_snake = data.find("left_track");
                if (it_left_camel != data.end()) {
                    left_track_input = clamp(SimpleJsonParser::get_double(it_left_camel->second), -1.0, 1.0);
                    has_left_track = true;
                } else if (it_left_snake != data.end()) {
                    left_track_input = clamp(SimpleJsonParser::get_double(it_left_snake->second), -1.0, 1.0);
                    has_left_track = true;
                }

                if (has_left_track) {
                        left_track_input = apply_network_filter_axis(left_track_input, filtered_left_track_input_);

                    if (left_track_input > 0) {
                        // 前进：0~1 映射到 0~700
                        double current = input_to_current(left_track_input, track_deadzone_, left_track_forward_current_curve_);
                        cmd.set_left_track_forward_current(current);
                        cmd.set_left_track_backward_current(0.0);
                        last_left_track_forward_current_ = current;
                        last_left_track_backward_current_ = 0.0;
                    } else if (left_track_input < 0) {
                        // 后退：-1~0 映射到 700~0
                        double current = input_to_current(left_track_input, track_deadzone_, left_track_backward_current_curve_);
                        cmd.set_left_track_forward_current(0.0);
                        cmd.set_left_track_backward_current(current);
                        last_left_track_forward_current_ = 0.0;
                        last_left_track_backward_current_ = current;
                    } else {
                        // 无输入
                        cmd.set_left_track_forward_current(0.0);
                        cmd.set_left_track_backward_current(0.0);
                        last_left_track_forward_current_ = 0.0;
                        last_left_track_backward_current_ = 0.0;
                    }
                } else {
                    // 保持上次值
                    cmd.set_left_track_forward_current(last_left_track_forward_current_);
                    cmd.set_left_track_backward_current(last_left_track_backward_current_);
                }
            }
            
            // 处理右履带控制 (rightTrack/right_track: -1..1 -> right_track_forward_current / right_track_backward_current)
            // 0 到 1：前进方向，映射到 right_track_forward_current 0~700
            // -1 到 0：后退方向，映射到 right_track_backward_current 700~0
            {
                bool has_right_track = false;
                double right_track_input = 0.0;
                auto it_right_camel = data.find("rightTrack");
                auto it_right_snake = data.find("right_track");
                if (it_right_camel != data.end()) {
                    right_track_input = clamp(SimpleJsonParser::get_double(it_right_camel->second), -1.0, 1.0);
                    has_right_track = true;
                } else if (it_right_snake != data.end()) {
                    right_track_input = clamp(SimpleJsonParser::get_double(it_right_snake->second), -1.0, 1.0);
                    has_right_track = true;
                }

                if (has_right_track) {
                        right_track_input = apply_network_filter_axis(right_track_input, filtered_right_track_input_);

                    if (right_track_input > 0) {
                        // 前进：0~1 映射到 0~700
                        double current = input_to_current(right_track_input, track_deadzone_, right_track_forward_current_curve_);
                        cmd.set_right_track_forward_current(current);
                        cmd.set_right_track_backward_current(0.0);
                        last_right_track_forward_current_ = current;
                        last_right_track_backward_current_ = 0.0;
                    } else if (right_track_input < 0) {
                        // 后退：-1~0 映射到 700~0
                        double current = input_to_current(right_track_input, track_deadzone_, right_track_backward_current_curve_);
                        cmd.set_right_track_forward_current(0.0);
                        cmd.set_right_track_backward_current(current);
                        last_right_track_forward_current_ = 0.0;
                        last_right_track_backward_current_ = current;
                    } else {
                        // 无输入
                        cmd.set_right_track_forward_current(0.0);
                        cmd.set_right_track_backward_current(0.0);
                        last_right_track_forward_current_ = 0.0;
                        last_right_track_backward_current_ = 0.0;
                    }
                } else {
                    // 保持上次值
                    cmd.set_right_track_forward_current(last_right_track_forward_current_);
                    cmd.set_right_track_backward_current(last_right_track_backward_current_);
                    }
                }
            }
            
            // 处理远程申请进入遥控
            if (data.find("request_remote_control") != data.end()) {
                bool request_remote = SimpleJsonParser::get_bool(data["request_remote_control"]);
                cmd.set_request_remote_control(request_remote);
                last_request_remote_control_ = request_remote;
            } else {
                // 保持上次值
                cmd.set_request_remote_control(last_request_remote_control_);
            }
            
            // 处理紧急停止
            if (data.find("emergency_stop") != data.end()) {
                bool estop = SimpleJsonParser::get_bool(data["emergency_stop"]);
                cmd.set_estop(estop);
                last_estop_ = estop;
            } else {
                // 保持上次值
                cmd.set_estop(last_estop_);
            }
            
            // 处理驻车制动
            if (data.find("parking_brake") != data.end()) {
                cmd.set_parking_brake(SimpleJsonParser::get_bool(data["parking_brake"]));
            }
            
            // ====== 数字输出 / 开关量控制（0x18210005）======
            //
            // 使用 ControlCommand::switch_bits 打包 26 个 DOH/DOL 开关量：
            // bit0  -> 车梯收起电磁阀         (OUT_21_DOH)
            // bit1  -> 车梯放下电磁阀         (OUT_22_DOH)
            // bit3  -> 先导安全锁定电磁阀     (OUT_24_DOH)
            // bit9  -> 开斗油缸电磁阀         (OUT_30_DOH)
            // bit10 -> 关斗油缸电磁阀         (OUT_31_DOH)
            // bit11 -> 喇叭                   (OUT_19_DOH(128#线))
            // bit12 -> 灯光1                  (OUT_17_DOH)
            // bit13 -> 灯光2                  (OUT_18_DOH)
            //
            // /controls/teleop 提供的三个高级字段：
            //   bucket_switch: 0/1/2 = 无动作 / 开斗电磁阀 / 关斗电磁阀
            //   ladder_switch: 0/1/2 = 无动作 / 车梯收起电磁阀 / 车梯放下电磁阀
            //   horn: false/true      = 无动作 / 按喇叭
            //   hydraulic_lock: 0/1   = 液压锁不工作 / 液压锁工作（驱动先导安全锁定电磁阀）
            //   light_code: 0/1/2/3   = 关灯 / 开灯1 / 开灯2 / 开灯1+2
            //
            // 可以同时配合原始 switch_bits 一起使用：先读取原始位图，再覆盖相关位。
            uint32_t switch_bits = 0;
            bool has_any_switch_bits = false;
            
            // 先读取原始 switch_bits（如果存在）
            auto it_switch_bits = data.find("switch_bits");
            if (it_switch_bits != data.end()) {
                switch_bits = static_cast<uint32_t>(SimpleJsonParser::get_int(it_switch_bits->second));
                has_any_switch_bits = true;
            }
            
            // 处理 ladder_switch（车梯收起/放下）
            auto it_ladder = data.find("ladder_switch");
            if (it_ladder != data.end()) {
                int ladder = SimpleJsonParser::get_int(it_ladder->second);
                // 清除 bit0 / bit1，避免残留
                switch_bits &= ~((1u << 0) | (1u << 1));
                if (ladder == 1) {
                    // 车梯收起
                    switch_bits |= (1u << 0);
                } else if (ladder == 2) {
                    // 车梯放下
                    switch_bits |= (1u << 1);
                }
                has_any_switch_bits = true;
            }
            
            // 处理 bucket_switch（开斗 / 关斗）
            auto it_bucket = data.find("bucket_switch");
            if (it_bucket != data.end()) {
                int bucket = SimpleJsonParser::get_int(it_bucket->second);
                // 清除 bit9 / bit10，避免残留
                switch_bits &= ~((1u << 9) | (1u << 10));
                if (bucket == 1) {
                    // 开斗油缸电磁阀
                    switch_bits |= (1u << 9);
                } else if (bucket == 2) {
                    // 关斗油缸电磁阀
                    switch_bits |= (1u << 10);
                }
                has_any_switch_bits = true;
            }

            // 处理 hydraulic_lock（先导安全锁定电磁阀）
            // 根据 excavator-can-host2vcu_utf8.csv: bit 3, OUT_24_DOH(103#线)
            // 支持两种格式：
            //   - 数值: 0/1 (0是液压锁不工作, 1是液压锁工作)
            //   - 布尔: true/false
            auto it_hyd = data.find("hydraulic_lock");
            if (it_hyd != data.end()) {
                // 优先按 bool 解析，其次按 int 解析
                bool hyd_bool = SimpleJsonParser::get_bool(it_hyd->second);
                int hyd_int = SimpleJsonParser::get_int(it_hyd->second);
                bool hyd_on = hyd_bool || (hyd_int == 1);
                // 清除 bit3，再按需置位
                switch_bits &= ~(1u << 3);
                if (hyd_on) {
                    switch_bits |= (1u << 3);
                }
                has_any_switch_bits = true;
            }

            // 处理 light_code（灯光1 / 灯光2）
            // 根据 excavator-can-host2vcu_utf8.csv:
            //   bit 12: 灯光1, OUT_19_DOH(128#线)
            //   bit 13: 灯光2, OUT_29_DOH(242#线)
            // 0: 关闭所有灯光, 1: 打开灯光1, 2: 打开灯光2, 3: 打开灯光1和灯光2
            auto it_light = data.find("light_code");
            if (it_light != data.end()) {
                int code = SimpleJsonParser::get_int(it_light->second);
                // 清除 bit12 / bit13，再按需置位
                switch_bits &= ~((1u << 12) | (1u << 13));
                if (code & 0x1) {
                    // 打开灯光1
                    switch_bits |= (1u << 12);
                }
                if (code & 0x2) {
                    // 打开灯光2
                    switch_bits |= (1u << 13);
                }
                has_any_switch_bits = true;
            }

            // 处理 horn（喇叭）
            // 根据 excavator-can-host2vcu_utf8.csv: bit 11, OUT_28_DOH(241#线)
            auto it_horn = data.find("horn");
            if (it_horn != data.end()) {
                bool horn = SimpleJsonParser::get_bool(it_horn->second);
                // 清除 bit11，再按需置位
                switch_bits &= ~(1u << 11);
                if (horn) {
                    switch_bits |= (1u << 11);
                }
                has_any_switch_bits = true;
            }

            // 处理 power_enable（上电）
            // 根据 excavator-can-host2vcu_utf8.csv: bit 34, 上电 (1是有效)
            // 使用 bit 26 作为临时存储，后续在 cannode 中映射到 switch_ext_bits bit 34
            auto it_power = data.find("power_enable");
            if (it_power != data.end()) {
                bool power_enable = SimpleJsonParser::get_bool(it_power->second);
                // 清除 bit26，再按需置位（临时存储，将在 cannode 中映射到 switch_ext_bits bit 34）
                switch_bits &= ~(1u << 26);
                if (power_enable) {
                    switch_bits |= (1u << 26);
                }
                has_any_switch_bits = true;
            }

            // 处理 start_up（发动机启动）
            // 根据 excavator-can-host2vcu_utf8.csv: bit 35, 发动机启动 (1是有效)
            // 使用 bit 27 作为临时存储，后续在 cannode 中映射到 switch_ext_bits bit 35
            auto it_startup = data.find("start_up");
            if (it_startup != data.end()) {
                bool startup = SimpleJsonParser::get_bool(it_startup->second);
                // 清除 bit27，再按需置位（临时存储，将在 cannode 中映射到 switch_ext_bits bit 35）
                switch_bits &= ~(1u << 27);
                if (startup) {
                    switch_bits |= (1u << 27);
                }
                has_any_switch_bits = true;
            }

            // 处理 acc（ACC 点火指令）
            // 根据 excavator-can-host2vcu_utf8.csv: bit 36, acc指令(DOH_OUT25_194), 1是有效
            // 使用 bit 28 作为临时存储，后续在 cannode 中映射到 switch_ext_bits bit 36
            auto it_acc = data.find("acc");
            if (it_acc != data.end()) {
                bool acc_on = SimpleJsonParser::get_bool(it_acc->second);
                // 清除 bit28，再按需置位（临时存储，将在 cannode 中映射到 switch_ext_bits bit 36）
                switch_bits &= ~(1u << 28);
                if (acc_on) {
                    switch_bits |= (1u << 28);
                }
                has_any_switch_bits = true;
            }

            // 发动机已运转（J1939 转速反馈 > 0）时，不响应上层「发动机启动」= 1/true（0x18210005 bit35，经 switch_bits bit27 传递）
            {
                std::string eng_json;
                {
                    std::lock_guard<std::mutex> lk(engine_feedback_mutex_);
                    eng_json = engine_feedback_json_;
                }
                const double eng_rpm = parse_engine_speed_rpm_from_engine_feedback_json(eng_json);
                if (eng_rpm > 0.0) {
                    if (switch_bits & (1u << 27)) {
                        switch_bits &= ~(1u << 27);
                        RCLCPP_DEBUG_THROTTLE(
                            this->get_logger(), *this->get_clock(), 5000,
                            "发动机转速反馈 %.1f rpm > 0，忽略发动机启动指令（switch_bits bit27）",
                            eng_rpm);
                    }
                }
            }
            
            if (has_any_switch_bits) {
                cmd.set_switch_bits(switch_bits);
            }
            
            // 打印转换后的输出信息（仅在 verbose 模式下打印）
            if (verbose_log_) {
                // RCLCPP_INFO(this->get_logger(), "📤 转换后的控制命令 (xiaosong 协议):");
                // RCLCPP_INFO(this->get_logger(), "   大臂: 抬升=%.1f mA, 下降=%.1f mA", 
                //            cmd.arm_up_current(), cmd.arm_down_current());
                // RCLCPP_INFO(this->get_logger(), "   斗杆: 收回=%.1f mA, 伸出=%.1f mA", 
                //            cmd.stick_retract_current(), cmd.stick_extend_current());
                // RCLCPP_INFO(this->get_logger(), "   铲斗: 收斗=%.1f mA, 翻斗=%.1f mA", 
                //            cmd.bucket_close_current(), cmd.bucket_dump_current());
                // RCLCPP_INFO(this->get_logger(), "   回转: 左转=%.1f mA, 右转=%.1f mA", 
                //            cmd.rotate_left_current(), cmd.rotate_right_current());
                // RCLCPP_INFO(this->get_logger(), "   左履带: 前进=%.1f mA, 后退=%.1f mA", 
                //            cmd.left_track_forward_current(), cmd.left_track_backward_current());
                // RCLCPP_INFO(this->get_logger(), "   右履带: 前进=%.1f mA, 后退=%.1f mA", 
                //            cmd.right_track_forward_current(), cmd.right_track_backward_current());
                if (cmd.has_switch_bits()) {
                    // RCLCPP_INFO(this->get_logger(), "   开关量位图: 0x%08X", cmd.switch_bits());
                }
                if (cmd.has_estop()) {
                    // RCLCPP_INFO(this->get_logger(), "   紧急停止: %s", cmd.estop() ? "是" : "否");
                }
                if (cmd.has_parking_brake()) {
                    // RCLCPP_INFO(this->get_logger(), "   驻车制动: %s", cmd.parking_brake() ? "是" : "否");
                }
            }

            // 处理发动机油门档位 gear（新格式: 1~7 的数值）
            // 该字段通过 ControlCommand.pto_cmd 传递，在 cannode 的 XiaosongCANParser 中映射到 0x18FD0821 报文
            auto it_gear = data.find("gear");
            if (it_gear != data.end()) {
                try {
                    int gear_int = SimpleJsonParser::get_int(it_gear->second);
                    if (gear_int < 1) gear_int = 1;
                    if (gear_int > 7) gear_int = 7;
                    last_engine_gear_ = static_cast<uint32_t>(gear_int);
                    cmd.set_pto_cmd(last_engine_gear_);
                } catch (const std::exception&) {
                    // 如果解析失败（例如仍旧是旧格式的字符档位），保持上一次档位
                    cmd.set_pto_cmd(last_engine_gear_);
                }
            } else {
                // 未提供 gear 时保持上一次档位
                cmd.set_pto_cmd(last_engine_gear_);
            }

            // 倾角仪动作限幅：超限方向比例阀输出置 0，角度回到范围内后自动恢复
            apply_angle_limits(cmd);
            sync_last_currents_from_cmd(cmd);

            // 序列化 protobuf 消息
            std::string serialized_data;
            cmd.SerializeToString(&serialized_data);
            
            // 创建 ROS2 消息（序列化版本）
            if (publish_vehicle_command_) {
                auto ros_msg = cyber_msgs::msg::ProtoAdapter();
                ros_msg.pb.assign(serialized_data.begin(), serialized_data.end());
                vehicle_cmd_pub_->publish(ros_msg);
            }
            
            // 创建并发布非序列化消息（用于调试和查看）
            if (publish_vehicle_command_debug_) {
                const auto now_ns = this->now().nanoseconds();
                const bool estop = cmd.has_estop() ? cmd.estop() : false;
                const bool parking_brake = cmd.has_parking_brake() ? cmd.parking_brake() : false;
                const bool request_remote_control = cmd.has_request_remote_control() ? cmd.request_remote_control() : false;
                const uint32_t switch_bits = cmd.has_switch_bits() ? static_cast<uint32_t>(cmd.switch_bits()) : 0U;
                const uint32_t pto_cmd = cmd.has_pto_cmd() ? static_cast<uint32_t>(cmd.pto_cmd()) : last_engine_gear_;

                std::ostringstream oss;
                oss << "{";
                oss << "\"stamp_ns\":" << now_ns << ",";
                oss << "\"stamp_sec\":" << (static_cast<double>(now_ns) / 1e9) << ",";
                oss << "\"frame_id\":\"base_link\",";
                oss << "\"arm_up_current\":" << cmd.arm_up_current() << ",";
                oss << "\"arm_down_current\":" << cmd.arm_down_current() << ",";
                oss << "\"stick_retract_current\":" << cmd.stick_retract_current() << ",";
                oss << "\"stick_extend_current\":" << cmd.stick_extend_current() << ",";
                oss << "\"bucket_close_current\":" << cmd.bucket_close_current() << ",";
                oss << "\"bucket_dump_current\":" << cmd.bucket_dump_current() << ",";
                oss << "\"rotate_left_current\":" << cmd.rotate_left_current() << ",";
                oss << "\"rotate_right_current\":" << cmd.rotate_right_current() << ",";
                oss << "\"left_track_forward_current\":" << cmd.left_track_forward_current() << ",";
                oss << "\"left_track_backward_current\":" << cmd.left_track_backward_current() << ",";
                oss << "\"right_track_forward_current\":" << cmd.right_track_forward_current() << ",";
                oss << "\"right_track_backward_current\":" << cmd.right_track_backward_current() << ",";
                oss << "\"switch_bits\":" << switch_bits << ",";
                oss << "\"estop\":" << (estop ? "true" : "false") << ",";
                oss << "\"parking_brake\":" << (parking_brake ? "true" : "false") << ",";
                oss << "\"request_remote_control\":" << (request_remote_control ? "true" : "false") << ",";
                oss << "\"pto_cmd\":" << pto_cmd;
                oss << "}";

                std_msgs::msg::String debug_msg;
                debug_msg.data = oss.str();
                vehicle_cmd_debug_pub_->publish(debug_msg);
            }
            
            if (verbose_log_) {
                // RCLCPP_INFO(this->get_logger(), "✅ 已发布到 /vehicle_command (protobuf 大小: %zu 字节)", serialized_data.size());
                // RCLCPP_INFO(this->get_logger(), "✅ 已发布到 /vehicle_command_debug (非序列化消息)");
                // RCLCPP_INFO(this->get_logger(), "============================================================");
            }
            
        } catch (const std::exception& e) {
            // RCLCPP_ERROR(this->get_logger(), "处理控制命令时出错: %s", e.what());
        }
    }
    
    // 检查 /controls/teleop 超时，如果超时则发送默认值（安全机制）
    void check_teleop_timeout() {
        auto now = this->now();
        rclcpp::Time last_teleop;
        {
            std::lock_guard<std::mutex> lk(network_state_mutex_);
            last_teleop = last_teleop_time_;
        }
        
        double elapsed = (now - last_teleop).seconds();
        if (elapsed > teleop_timeout_sec_) {
            // 超时，发送默认值（所有电流为0，开关量为0）
            static bool timeout_warned = false;
            if (!timeout_warned) {
                RCLCPP_WARN(this->get_logger(), 
                    "⚠️  /controls/teleop 超时 (%.3f秒 > %.3f秒)，发送默认值（安全机制）", 
                    elapsed, teleop_timeout_sec_);
                timeout_warned = true;
            }
            
            // 发送默认控制命令（仅在超时时发送，一旦收到新指令，last_teleop_time_ 会更新，下次检查就不会再发送）
            send_default_command();
        } else {
            // 未超时，恢复正常状态（清除警告标志）
            static bool timeout_warned = false;
            if (timeout_warned) {
                timeout_warned = false;
            }
        }
    }
    
    // 发送默认控制命令（所有电流为0，开关量为0）
    void send_default_command() {
        try {
            control::ControlCommand cmd;
            
            // 设置 header
            auto* header = cmd.mutable_header();
            header->set_timestamp_sec(this->now().nanoseconds() / 1e9);
            header->set_frame_id("base_link");
            
            // 所有电流设为0
            cmd.set_arm_up_current(0.0);
            cmd.set_arm_down_current(0.0);
            cmd.set_stick_retract_current(0.0);
            cmd.set_stick_extend_current(0.0);
            cmd.set_bucket_close_current(0.0);
            cmd.set_bucket_dump_current(0.0);
            cmd.set_rotate_left_current(0.0);
            cmd.set_rotate_right_current(0.0);
            cmd.set_left_track_forward_current(0.0);
            cmd.set_left_track_backward_current(0.0);
            cmd.set_right_track_forward_current(0.0);
            cmd.set_right_track_backward_current(0.0);
            
            // 开关量设为0
            cmd.set_switch_bits(0);
            
            // 保持安全相关状态（不改变急停、驻车制动等）
            cmd.set_estop(last_estop_);
            cmd.set_parking_brake(false);
            cmd.set_request_remote_control(false);
            
            // 序列化并发布
            std::string serialized_data;
            cmd.SerializeToString(&serialized_data);
            
            if (publish_vehicle_command_) {
                auto ros_msg = cyber_msgs::msg::ProtoAdapter();
                ros_msg.pb.assign(serialized_data.begin(), serialized_data.end());
                vehicle_cmd_pub_->publish(ros_msg);
            }

            // 同步发布调试信息（JSON 字符串），便于直接 `ros2 topic echo /vehicle_command_debug`
            if (publish_vehicle_command_debug_) {
                const auto now_ns = this->now().nanoseconds();
                const bool estop = cmd.has_estop() ? cmd.estop() : last_estop_;
                const bool parking_brake = cmd.has_parking_brake() ? cmd.parking_brake() : false;
                const bool request_remote_control = cmd.has_request_remote_control() ? cmd.request_remote_control() : false;
                const uint32_t switch_bits = cmd.has_switch_bits() ? static_cast<uint32_t>(cmd.switch_bits()) : 0U;
                const uint32_t pto_cmd = cmd.has_pto_cmd() ? static_cast<uint32_t>(cmd.pto_cmd()) : last_engine_gear_;

                std::ostringstream oss;
                oss << "{";
                oss << "\"stamp_ns\":" << now_ns << ",";
                oss << "\"stamp_sec\":" << (static_cast<double>(now_ns) / 1e9) << ",";
                oss << "\"frame_id\":\"base_link\",";
                oss << "\"arm_up_current\":" << cmd.arm_up_current() << ",";
                oss << "\"arm_down_current\":" << cmd.arm_down_current() << ",";
                oss << "\"stick_retract_current\":" << cmd.stick_retract_current() << ",";
                oss << "\"stick_extend_current\":" << cmd.stick_extend_current() << ",";
                oss << "\"bucket_close_current\":" << cmd.bucket_close_current() << ",";
                oss << "\"bucket_dump_current\":" << cmd.bucket_dump_current() << ",";
                oss << "\"rotate_left_current\":" << cmd.rotate_left_current() << ",";
                oss << "\"rotate_right_current\":" << cmd.rotate_right_current() << ",";
                oss << "\"left_track_forward_current\":" << cmd.left_track_forward_current() << ",";
                oss << "\"left_track_backward_current\":" << cmd.left_track_backward_current() << ",";
                oss << "\"right_track_forward_current\":" << cmd.right_track_forward_current() << ",";
                oss << "\"right_track_backward_current\":" << cmd.right_track_backward_current() << ",";
                oss << "\"switch_bits\":" << switch_bits << ",";
                oss << "\"estop\":" << (estop ? "true" : "false") << ",";
                oss << "\"parking_brake\":" << (parking_brake ? "true" : "false") << ",";
                oss << "\"request_remote_control\":" << (request_remote_control ? "true" : "false") << ",";
                oss << "\"pto_cmd\":" << pto_cmd;
                oss << "}";

                std_msgs::msg::String debug_msg;
                debug_msg.data = oss.str();
                vehicle_cmd_debug_pub_->publish(debug_msg);
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "发送默认控制命令时出错: %s", e.what());
        }
    }
    
    // 网络 QoS 订阅回调：根据 RTT / 丢包 / 抖动更新原始网络状态
    void network_qos_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        try {
            std::map<std::string, std::string> data = SimpleJsonParser::parse(msg->data);

            double rtt_ms = 0.0;
            double loss_pct = 0.0;
            double jitter_ms = 0.0;

            if (data.find("rtt_ms") != data.end()) {
                rtt_ms = SimpleJsonParser::get_double(data["rtt_ms"]);
            }
            if (data.find("packet_loss_pct") != data.end()) {
                loss_pct = SimpleJsonParser::get_double(data["packet_loss_pct"]);
            }
            if (data.find("jitter_ms") != data.end()) {
                jitter_ms = SimpleJsonParser::get_double(data["jitter_ms"]);
            }

            auto now = this->now();
            {
                std::lock_guard<std::mutex> lk(network_state_mutex_);
                last_qos_time_ = now;
                last_rtt_ms_ = rtt_ms;
                last_packet_loss_pct_ = loss_pct;
                last_jitter_ms_ = jitter_ms;

                if (rtt_ms > qos_rtt_level3_ms_) {
                    if (!rtt_high_active_) {
                        rtt_high_active_ = true;
                        rtt_high_start_time_ = now;
                    }
                } else {
                    rtt_high_active_ = false;
                }

                update_network_level_locked(now);
            }
        } catch (const std::exception& e) {
            // RCLCPP_ERROR(this->get_logger(), "解析 /excavator/network_qos 失败: %s", e.what());
        }
    }

    // 订阅 /can_frame_sent，监控 0x18210005 中的开斗/关斗 bit，用于更新 bucket_open_degree_state_
    void can_frame_sent_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        const std::string & s = msg->data;
        try {
            // 提取 can_id_dec
            std::size_t pos_id = s.find("\"can_id_dec\"");
            if (pos_id == std::string::npos) {
                return;
            }
            pos_id = s.find(':', pos_id);
            if (pos_id == std::string::npos) {
                return;
            }
            ++pos_id;
            // 跳过空白
            while (pos_id < s.size() && std::isspace(static_cast<unsigned char>(s[pos_id]))) {
                ++pos_id;
            }
            char *endptr = nullptr;
            long can_id_dec = std::strtol(s.c_str() + pos_id, &endptr, 10);
            if (endptr == s.c_str() + pos_id) {
                return;
            }
            // 只关心 0x18210005
            if (can_id_dec != 0x18210005) {
                return;
            }

            // 提取 data 数组
            std::size_t pos_data = s.find("\"data\"");
            if (pos_data == std::string::npos) {
                return;
            }
            pos_data = s.find('[', pos_data);
            if (pos_data == std::string::npos) {
                return;
            }
            ++pos_data;
            std::vector<int> bytes;
            bytes.reserve(8);
            while (pos_data < s.size() && bytes.size() < 8) {
                // 跳过空白
                while (pos_data < s.size() && std::isspace(static_cast<unsigned char>(s[pos_data]))) {
                    ++pos_data;
                }
                if (pos_data >= s.size() || s[pos_data] == ']') {
                    break;
                }
                char *num_end = nullptr;
                long v = std::strtol(s.c_str() + pos_data, &num_end, 10);
                if (num_end == s.c_str() + pos_data) {
                    break;
                }
                bytes.push_back(static_cast<int>(v));
                pos_data = static_cast<std::size_t>(num_end - s.c_str());
                // 跳过逗号
                while (pos_data < s.size() && (std::isspace(static_cast<unsigned char>(s[pos_data])) || s[pos_data] == ',')) {
                    ++pos_data;
                }
            }
            if (bytes.size() < 2) {
                return;
            }

            // 根据 excavator-can-host2vcu_utf8.csv:
            //  开斗油缸电磁阀 -> bit9  (byte1 bit1)
            //  关斗油缸电磁阀 -> bit10 (byte1 bit2)
            uint8_t b1 = static_cast<uint8_t>(bytes[1]);
            bool open_cmd  = (b1 & (1u << 1)) != 0;  // bit9
            bool close_cmd = (b1 & (1u << 2)) != 0;  // bit10

            std::lock_guard<std::mutex> lk(bucket_open_mutex_);
            if (open_cmd) {
                // 当检测到开斗 bit==1 时：设置为 45 并保持
                bucket_open_degree_state_ = 45.0;
            } else if (close_cmd) {
                // 当检测到关斗 bit==1 时：设置为 0 并保持
                bucket_open_degree_state_ = 0.0;
            }
        } catch (...) {
            // 解析失败直接忽略
            return;
        }
    }

    // 网络状态机：根据 QoS 和心跳更新时间，计算 3 级降级级别
    void update_network_level_locked(const rclcpp::Time & now)
    {
        // 当前 QoS 对应的“生的”级别（不考虑恢复计数）
        bool rtt_high_1s =
            rtt_high_active_ && ((now - rtt_high_start_time_).seconds() >= qos_rtt_level3_hold_sec_);
        bool red_cond =
            (rtt_high_1s || last_packet_loss_pct_ >= qos_loss_level3_pct_);
        bool yellow_cond =
            (last_rtt_ms_ >= qos_rtt_level2_ms_ ||
             last_packet_loss_pct_ >= qos_loss_level2_pct_ ||
             last_jitter_ms_ >= qos_jitter_level2_ms_);

        switch (network_level_) {
        case NetworkDegradeLevel::GREEN:
            // 从 GREEN 降级：立即响应
            yellow_recovery_count_ = 0;
            red_recovery_count_ = 0;
            if (red_cond) {
                network_level_ = NetworkDegradeLevel::RED;
            } else if (yellow_cond) {
                network_level_ = NetworkDegradeLevel::YELLOW;
            }
            break;

        case NetworkDegradeLevel::YELLOW:
            // 从 YELLOW 降级到 RED：立即
            if (red_cond) {
                network_level_ = NetworkDegradeLevel::RED;
                yellow_recovery_count_ = 0;
                red_recovery_count_ = 0;
                break;
            }

            // 从 YELLOW 升级到 GREEN：需要连续若干次满足“非 yellow 条件”
            if (!yellow_cond && !red_cond) {
                ++yellow_recovery_count_;
                if (yellow_recovery_count_ >= upgrade_required_count_) {
                    network_level_ = NetworkDegradeLevel::GREEN;
                    yellow_recovery_count_ = 0;
                }
            } else {
                // 一旦再次满足 yellow 条件，清零计数
                yellow_recovery_count_ = 0;
            }
            break;

        case NetworkDegradeLevel::RED:
        default:
            // 在 RED 下：任何再次满足 red 条件都重置恢复计数
            if (red_cond) {
                red_recovery_count_ = 0;
                break;
            }

            // QoS 已不再满足 red_cond，开始计数恢复
            ++red_recovery_count_;

            // 根据当前 QoS，优先恢复到 YELLOW，若已达 GREEN 条件则可直接恢复到 GREEN
            if (!yellow_cond) {
                // QoS 已经足够好（GREEN），连续 N 次后直接恢复到 GREEN
                if (red_recovery_count_ >= upgrade_required_count_) {
                    network_level_ = NetworkDegradeLevel::GREEN;
                    red_recovery_count_ = 0;
                }
            } else {
                // 仍处于 YELLOW 区间，连续 N 次后先从 RED 恢复到 YELLOW
                if (red_recovery_count_ >= upgrade_required_count_) {
                    network_level_ = NetworkDegradeLevel::YELLOW;
                    red_recovery_count_ = 0;
                }
            }
            break;
        }
    }

    // 速度限制 + 平滑滤波（针对单个轴的 -1~1 输入）
    double apply_network_filter_axis(double raw_input, double & filtered_state)
    {
        std::lock_guard<std::mutex> lk(network_state_mutex_);
        switch (network_level_) {
        case NetworkDegradeLevel::GREEN:
            filtered_state = raw_input;
            return raw_input;
        case NetworkDegradeLevel::YELLOW: {
            // 级别 2：限制最大输入幅度 + 一阶低通滤波
            double limited = clamp(raw_input, -level2_input_limit_, level2_input_limit_);
            filtered_state =
                level2_filter_alpha_ * limited + (1.0 - level2_filter_alpha_) * filtered_state;
            return filtered_state;
        }
        case NetworkDegradeLevel::RED:
        default:
            // 级别 3：禁止运动
            filtered_state = 0.0;
            return 0.0;
        }
    }

    
    // 订阅者和发布者
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr teleop_sub_;
    rclcpp::Publisher<cyber_msgs::msg::ProtoAdapter>::SharedPtr vehicle_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr vehicle_cmd_debug_pub_;
    rclcpp::Subscription<cyber_msgs::msg::ChassisStatus>::SharedPtr chassis_status_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr vcu_feedback_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr engine_feedback_sub_;
    rclcpp::Subscription<cyber_msgs::msg::MultiCameraObjectDetection>::SharedPtr object_detection_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr can_frame_sub_;
    rclcpp::Subscription<msg_interfaces::msg::Hcinspvatzcb>::SharedPtr devpvt_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr chassis_feedback_pub_;
    
    // VCU 反馈数据缓存
    std::mutex vcu_feedback_mutex_;
    std::string vcu_feedback_json_;  // 缓存的 VCU 反馈 JSON 数据

    // 发动机/VCU 扩展反馈缓存
    std::mutex engine_feedback_mutex_;
    std::string engine_feedback_json_;

    // 多摄像头目标检测结果缓存
    std::mutex object_detection_mutex_;
    cyber_msgs::msg::MultiCameraObjectDetection latest_object_detection_;
    bool object_detection_valid_{false};

    // CGI 惯导 /chcnav/devpvt 数据缓存
    std::mutex devpvt_mutex_;
    msg_interfaces::msg::Hcinspvatzcb latest_devpvt_;
    bool devpvt_valid_{false};

    // /chcnav/devpvt 偏置补偿（通过 teleop_xiaosong.yaml 或 ros2 param set 配置）
    std::mutex devpvt_bias_mutex_;
    double devpvt_lat_bias_deg_{0.0};
    double devpvt_lon_bias_deg_{0.0};
    double devpvt_alt_bias_m_{0.0};
    double devpvt_roll_bias_deg_{0.0};
    double devpvt_pitch_bias_deg_{0.0};
    double devpvt_yaw_bias_deg_{0.0};
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr devpvt_param_cb_handle_;
    
    // 死区参数
    double arm_deadzone_;
    double stick_deadzone_;
    double bucket_deadzone_;
    double swing_deadzone_;
    double track_deadzone_;
    
    // 电流参数
    double max_current_;           // 最大电流 (mA)
    CurrentAxisCurve arm_up_current_curve_;
    CurrentAxisCurve arm_down_current_curve_;
    CurrentAxisCurve stick_extend_current_curve_;
    CurrentAxisCurve stick_retract_current_curve_;
    CurrentAxisCurve bucket_close_current_curve_;
    CurrentAxisCurve bucket_dump_current_curve_;
    CurrentAxisCurve swing_right_current_curve_;
    CurrentAxisCurve swing_left_current_curve_;
    CurrentAxisCurve left_track_forward_current_curve_;
    CurrentAxisCurve left_track_backward_current_curve_;
    CurrentAxisCurve right_track_forward_current_curve_;
    CurrentAxisCurve right_track_backward_current_curve_;
    
    // 是否发布/vehicle_command
    bool publish_vehicle_command_;
    bool publish_vehicle_command_debug_;
    bool publish_chassis_feedback_;
    // 是否打印详细日志（降低CPU时可关闭）
    bool verbose_log_;

    // 挖掘机关节角反馈（来自 /excavator_joint_angles）
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr excavator_angles_sub_;
    mutable std::mutex excavator_angle_mutex_;
    bool excavator_angles_valid_{false};
    double excavator_boom_angle_deg_{0.0};    // 大臂相对车身
    double excavator_stick_angle_deg_{0.0};   // 斗杆相对大臂
    double excavator_bucket_angle_deg_{0.0};  // 铲斗相对斗杆

    void excavator_angles_callback(const geometry_msgs::msg::Vector3::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lk(excavator_angle_mutex_);
        excavator_boom_angle_deg_ = msg->x;
        excavator_stick_angle_deg_ = msg->y;
        excavator_bucket_angle_deg_ = msg->z;
        excavator_angles_valid_ = true;
    }

    // 倾角仪（multi_tilt_imu）：车身姿态 JSON + 动作限幅用角度
    rclcpp::Subscription<cannode::msg::TiltRPY>::SharedPtr tilt_boom_sub_;
    rclcpp::Subscription<cannode::msg::TiltRPY>::SharedPtr tilt_stick_sub_;
    rclcpp::Subscription<cannode::msg::TiltRPY>::SharedPtr tilt_bucket_sub_;
    rclcpp::Subscription<cannode::msg::TiltRPY>::SharedPtr tilt_body_sub_;
    mutable std::mutex tilt_data_mutex_;
    bool tilt_body_valid_{false};
    double tilt_body_roll_deg_{0.0};
    double tilt_body_pitch_deg_{0.0};
    double tilt_body_yaw_deg_{0.0};
    bool tilt_boom_valid_{false};
    double tilt_boom_angle_deg_{0.0};
    bool tilt_stick_valid_{false};
    double tilt_stick_angle_deg_{0.0};
    bool tilt_bucket_valid_{false};
    double tilt_bucket_angle_deg_{0.0};
    /** 车身限幅用角度（由 angle_limit_body_axis 从 TiltRPY 选取） */
    double tilt_body_limit_angle_deg_{0.0};

    bool angle_limit_enable_{false};
    double boom_angle_min_deg_{-360.0};
    double boom_angle_max_deg_{360.0};
    double stick_angle_min_deg_{-360.0};
    double stick_angle_max_deg_{360.0};
    double bucket_angle_min_deg_{-360.0};
    double bucket_angle_max_deg_{360.0};
    double body_angle_min_deg_{-360.0};
    double body_angle_max_deg_{360.0};
    std::string tilt_boom_topic_;
    std::string tilt_stick_topic_;
    std::string tilt_bucket_topic_;
    std::string tilt_body_topic_;
    std::string angle_limit_boom_axis_;
    std::string angle_limit_stick_axis_;
    std::string angle_limit_bucket_axis_;
    std::string angle_limit_body_axis_;
    bool boom_angle_limit_swap_sides_{false};
    bool stick_angle_limit_swap_sides_{false};
    bool bucket_angle_limit_swap_sides_{false};
    bool body_angle_limit_swap_sides_{false};

    void tilt_body_callback(const cannode::msg::TiltRPY::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lk(tilt_data_mutex_);
        tilt_body_roll_deg_ = static_cast<double>(msg->roll);
        tilt_body_pitch_deg_ = static_cast<double>(msg->pitch);
        tilt_body_yaw_deg_ = static_cast<double>(msg->yaw);
        tilt_body_limit_angle_deg_ = tilt_axis_value(*msg, angle_limit_body_axis_);
        tilt_body_valid_ = true;
    }

    void apply_angle_limits(control::ControlCommand & cmd)
    {
        if (!angle_limit_enable_) {
            return;
        }
        const double boom_lo = std::min(boom_angle_min_deg_, boom_angle_max_deg_);
        const double boom_hi = std::max(boom_angle_min_deg_, boom_angle_max_deg_);
        const double stick_lo = std::min(stick_angle_min_deg_, stick_angle_max_deg_);
        const double stick_hi = std::max(stick_angle_min_deg_, stick_angle_max_deg_);
        const double bucket_lo = std::min(bucket_angle_min_deg_, bucket_angle_max_deg_);
        const double bucket_hi = std::max(bucket_angle_min_deg_, bucket_angle_max_deg_);
        const double body_lo = std::min(body_angle_min_deg_, body_angle_max_deg_);
        const double body_hi = std::max(body_angle_min_deg_, body_angle_max_deg_);

        std::lock_guard<std::mutex> lk(tilt_data_mutex_);
        if (tilt_boom_valid_) {
            const bool at_hi = tilt_boom_angle_deg_ >= boom_hi;
            const bool at_lo = tilt_boom_angle_deg_ <= boom_lo;
            if (boom_angle_limit_swap_sides_) {
                if (at_hi) {
                    cmd.set_arm_down_current(0.0);
                }
                if (at_lo) {
                    cmd.set_arm_up_current(0.0);
                }
            } else {
                if (at_hi) {
                    cmd.set_arm_up_current(0.0);
                }
                if (at_lo) {
                    cmd.set_arm_down_current(0.0);
                }
            }
        }
        if (tilt_stick_valid_) {
            const bool at_hi = tilt_stick_angle_deg_ >= stick_hi;
            const bool at_lo = tilt_stick_angle_deg_ <= stick_lo;
            if (stick_angle_limit_swap_sides_) {
                if (at_hi) {
                    cmd.set_stick_retract_current(0.0);
                }
                if (at_lo) {
                    cmd.set_stick_extend_current(0.0);
                }
            } else {
                if (at_hi) {
                    cmd.set_stick_extend_current(0.0);
                }
                if (at_lo) {
                    cmd.set_stick_retract_current(0.0);
                }
            }
        }
        if (tilt_bucket_valid_) {
            const bool at_hi = tilt_bucket_angle_deg_ >= bucket_hi;
            const bool at_lo = tilt_bucket_angle_deg_ <= bucket_lo;
            if (bucket_angle_limit_swap_sides_) {
                if (at_hi) {
                    cmd.set_bucket_dump_current(0.0);
                }
                if (at_lo) {
                    cmd.set_bucket_close_current(0.0);
                }
            } else {
                if (at_hi) {
                    cmd.set_bucket_close_current(0.0);
                }
                if (at_lo) {
                    cmd.set_bucket_dump_current(0.0);
                }
            }
        }
        if (tilt_body_valid_) {
            const bool at_hi = tilt_body_limit_angle_deg_ >= body_hi;
            const bool at_lo = tilt_body_limit_angle_deg_ <= body_lo;
            if (body_angle_limit_swap_sides_) {
                if (at_hi) {
                    cmd.set_rotate_left_current(0.0);
                }
                if (at_lo) {
                    cmd.set_rotate_right_current(0.0);
                }
            } else {
                if (at_hi) {
                    cmd.set_rotate_right_current(0.0);
                }
                if (at_lo) {
                    cmd.set_rotate_left_current(0.0);
                }
            }
        }
    }

    /** 任一动关节角超出 [min,max]（与 apply_angle_limits 一致）则为 1，否则 0；未启用 angle_limit 时为 0 */
    int compute_action_limit_exceed() const
    {
        if (!angle_limit_enable_) {
            return 0;
        }
        std::lock_guard<std::mutex> lk(tilt_data_mutex_);
        auto at_limit = [](bool valid, double a, double lo, double hi) {
            if (!valid) {
                return false;
            }
            return (a >= hi) || (a <= lo);
        };
        const double boom_lo = std::min(boom_angle_min_deg_, boom_angle_max_deg_);
        const double boom_hi = std::max(boom_angle_min_deg_, boom_angle_max_deg_);
        const double stick_lo = std::min(stick_angle_min_deg_, stick_angle_max_deg_);
        const double stick_hi = std::max(stick_angle_min_deg_, stick_angle_max_deg_);
        const double bucket_lo = std::min(bucket_angle_min_deg_, bucket_angle_max_deg_);
        const double bucket_hi = std::max(bucket_angle_min_deg_, bucket_angle_max_deg_);
        const double body_lo = std::min(body_angle_min_deg_, body_angle_max_deg_);
        const double body_hi = std::max(body_angle_min_deg_, body_angle_max_deg_);
        if (at_limit(tilt_boom_valid_, tilt_boom_angle_deg_, boom_lo, boom_hi)) {
            return 1;
        }
        if (at_limit(tilt_stick_valid_, tilt_stick_angle_deg_, stick_lo, stick_hi)) {
            return 1;
        }
        if (at_limit(tilt_bucket_valid_, tilt_bucket_angle_deg_, bucket_lo, bucket_hi)) {
            return 1;
        }
        if (at_limit(tilt_body_valid_, tilt_body_limit_angle_deg_, body_lo, body_hi)) {
            return 1;
        }
        return 0;
    }

    void sync_last_currents_from_cmd(const control::ControlCommand & cmd)
    {
        last_arm_up_current_ = cmd.arm_up_current();
        last_arm_down_current_ = cmd.arm_down_current();
        last_stick_retract_current_ = cmd.stick_retract_current();
        last_stick_extend_current_ = cmd.stick_extend_current();
        last_bucket_close_current_ = cmd.bucket_close_current();
        last_bucket_dump_current_ = cmd.bucket_dump_current();
        last_rotate_left_current_ = cmd.rotate_left_current();
        last_rotate_right_current_ = cmd.rotate_right_current();
        last_left_track_forward_current_ = cmd.left_track_forward_current();
        last_left_track_backward_current_ = cmd.left_track_backward_current();
        last_right_track_forward_current_ = cmd.right_track_forward_current();
        last_right_track_backward_current_ = cmd.right_track_backward_current();
    }

    // Danfoss 编码器角度反馈（来自 /rotary_encoder_angle）
    rclcpp::Subscription<cannode::msg::RotaryEncoderAngle>::SharedPtr rotary_encoder_sub_;
    mutable std::mutex rotary_encoder_mutex_;
    bool rotary_encoder_valid_{false};
    double rotary_encoder_angle1_deg_{0.0};  // 编码器 angle1，用于回转角度

    void rotary_encoder_callback(const cannode::msg::RotaryEncoderAngle::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lk(rotary_encoder_mutex_);
        rotary_encoder_angle1_deg_ = static_cast<double>(msg->angle1);
        rotary_encoder_valid_ = true;
        if (verbose_log_) {
            // RCLCPP_INFO(this->get_logger(), "收到编码器数据: angle1=%.2f°", rotary_encoder_angle1_deg_);
        }
    }
    
    // 网络 QoS / 降级状态
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr network_qos_sub_;
    mutable std::mutex network_state_mutex_;
    NetworkDegradeLevel network_level_{NetworkDegradeLevel::GREEN};
    rclcpp::Time last_qos_time_;
    rclcpp::Time last_teleop_time_;
    double last_rtt_ms_{0.0};
    double last_packet_loss_pct_{0.0};
    double last_jitter_ms_{0.0};
    bool rtt_high_active_{false};
    rclcpp::Time rtt_high_start_time_;

    // 各通道的网络滤波后的输入（-1~1）
    double filtered_boom_input_{0.0};
    double filtered_stick_input_{0.0};
    double filtered_bucket_input_{0.0};
    double filtered_swing_input_{0.0};
    double filtered_left_track_input_{0.0};
    double filtered_right_track_input_{0.0};
    
    // 上次的值（用于保持状态）
    double last_arm_up_current_;
    double last_arm_down_current_;
    double last_stick_retract_current_;
    double last_stick_extend_current_;
    double last_bucket_close_current_;
    double last_bucket_dump_current_;
    double last_rotate_left_current_;
    double last_rotate_right_current_;
    double last_left_track_forward_current_;
    double last_left_track_backward_current_;
    double last_right_track_forward_current_;
    double last_right_track_backward_current_;
    bool last_request_remote_control_;
    bool last_estop_;
    uint32_t last_engine_gear_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Teleop2CanTransformerXiaosong>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

