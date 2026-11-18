
#include <chrono>
#include <thread>
#include <cstdlib>
#include <cctype>
#include <cmath>
#include <pthread.h>
#include <sstream>
#include <iomanip>
//#include "control/control_command.pb.h"
#include "control_msgs/control_cmd.pb.h"
//#include "control/canbus/chassis.pb.h"
#include "common_msgs/chassis_msgs/chassis.pb.h"
#include "msg_manager.h"
#include "common/file.hpp"

namespace {

/** JSON 中 engine_speed_rpm=-1 表示无有效反馈；向 VCU（小松 0x18210005 转速字段）发 8000 rpm */
uint16_t ParseEngineSpeedRpmFromEngineFeedbackJson(const std::string& json) {
  constexpr const char kKey[] = "\"engine_speed_rpm\":";
  size_t pos = json.find(kKey);
  if (pos == std::string::npos) {
    return 0;
  }
  pos += sizeof(kKey) - 1u;
  while (pos < json.size() && (json[pos] == ' ' || json[pos] == '\t')) {
    ++pos;
  }
  if (pos >= json.size()) {
    return 0;
  }
  try {
    size_t end = pos;
    while (end < json.size()) {
      const unsigned char c = static_cast<unsigned char>(json[end]);
      if (std::isdigit(c) || c == '.' || c == '-' || c == '+' || c == 'e' || c == 'E') {
        ++end;
      } else {
        break;
      }
    }
    double v = std::stod(json.substr(pos, end - pos));
    const long vr = std::lround(v);
    if (vr == -1) {
      return 8000u;
    }
    if (v < 0.0) {
      v = 0.0;
    }
    if (v > 8000.0) {
      v = 8000.0;
    }
    return static_cast<uint16_t>(std::lround(v));
  } catch (...) {
    return 0;
  }
}

}  // namespace

namespace cannode {

MsgManager::MsgManager(): Node("chassis_node") {
  // 声明超时参数（可配置）
  this->declare_parameter<double>("vehicle_cmd_timeout_sec", 1.0);  // /vehicle_command 超时时间（秒），默认1.0s
  vehicle_cmd_timeout_sec_ = this->get_parameter("vehicle_cmd_timeout_sec").as_double();
  
  // 初始化最后收到 vehicle_command 的时间为当前时间（避免启动时立即触发超时）
  last_vehicle_cmd_time_ = this->now();
  
  // 创建 CAN node
  cannodeLoadPara(std::string(CAN_CFG_FILE));
  cannodeCfg();
  cannodeCreateInstance();

  // 创建底盘状态发布者
  chassis_status_pub = this->create_publisher<cyber_msgs::msg::ProtoAdapter>("/SaVehicleReportV2", 10);
  chassis_cmd_sub = this->create_subscription<cyber_msgs::msg::ProtoAdapter>(
      "/vehicle_command", 10, std::bind(&MsgManager::procControlImplement, this, std::placeholders::_1));

  // 创建底盘状态发布者ros2消息
  chassis_status_pub_ros2 = this->create_publisher<cyber_msgs::msg::ChassisStatus>("/chassis_status", 10);
  
  // 创建 VCU 反馈数据发布者（用于传递 0x18FF1002 数据）
  vcu_feedback_pub = this->create_publisher<std_msgs::msg::String>("/cannode/vcu_feedback", 10);
  
  // 创建 CAN 帧信息发布者
  can_frame_pub = this->create_publisher<std_msgs::msg::String>("/can_frame_sent", 10);
  
  // 创建超时检查定时器（每100ms检查一次）
  vehicle_cmd_timeout_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() {
          this->check_vehicle_cmd_timeout();
      }
  );
  
  // 注意：CAN 帧发送回调函数将在 cannodeCreateInstance() 之后设置
  // 因为 chassisCanHandler 是在 cannodeCreateInstance() 中创建的
}

MsgManager::~MsgManager() {
  if (chassisCanHandler) {
    chassisCanHandler.reset();
  }
}


// void VehicleConfigHelper::Init(const std::string &config_file) {
//   VehicleConfig params;
//   ACHECK(planning::common::GetProtoFromFile(config_file, &params))
//       << "Unable to parse vehicle config file " << config_file;
//   Init(params);
//   AINFO << "vehicle param   " << params.DebugString();
// }

bool MsgManager::init(void) {
    // 设置 CAN 帧发送回调函数（在 chassisCanHandler 创建并初始化之后）
    if (chassisCanHandler) {
      chassisCanHandler->setCanFrameSendCallback(
        [this](uint32_t can_id, uint8_t dlc, const uint8_t* data) {
          this->publishCanFrame(can_id, dlc, data);
        }
      );
    }

    engine_feedback_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/cannode/engine_feedback",
        rclcpp::QoS(10),
        [this](const std_msgs::msg::String::SharedPtr msg) {
          if (!chassisCanHandler || !msg) {
            return;
          }
          const uint16_t rpm = ParseEngineSpeedRpmFromEngineFeedbackJson(msg->data);
          chassisCanHandler->setHostEngineSpeedRpm(rpm);
        });

    std::thread cannode_thread([&](){
    //ProcessScheduler::setThreadPriority(SCHED_FIFO, 95);
    uint64_t beginTime = 0;
    uint64_t endTime = 0;
    while (true)
    {
        beginTime = std::chrono::duration_cast<std::chrono::milliseconds>
                    (std::chrono::system_clock::now().time_since_epoch()).count();
        msgProc();
        endTime = std::chrono::duration_cast<std::chrono::milliseconds>
                    (std::chrono::system_clock::now().time_since_epoch()).count();
        auto taskDuration = endTime - beginTime;
        // LOG_I("cannode, taskDuration: {}",  (int32_t)taskDuration);
        if (taskDuration < PERIODTIME) {
            std::chrono::milliseconds sleepDuration(PERIODTIME - taskDuration);
            std::this_thread::sleep_for(sleepDuration);
        } else {
            // std::cout << "cannode overtime:" << (int32_t)taskDuration << std::endl;
        }
        // std::cout << "cannode, taskDuration," << (int32_t)taskDuration << std::;
        
    }
    });
    // 在detach之前设置线程名称
    pthread_setname_np(cannode_thread.native_handle(), "cannode_thread");
    cannode_thread.detach();
  return true;
}

void MsgManager::msgProc() {
    PublishChassisStatus();
}

// 接收处理控制命令信息
void MsgManager::procControlImplement(const cyber_msgs::msg::ProtoAdapter::SharedPtr msg) const {
    // 检查之前是否处于超时状态
    auto now = this->now();
    bool was_timeout = false;
    {
        std::lock_guard<std::mutex> lk(vehicle_cmd_timeout_mutex_);
        double elapsed = (now - last_vehicle_cmd_time_).seconds();
        was_timeout = (elapsed > vehicle_cmd_timeout_sec_);
        last_vehicle_cmd_time_ = now;
    }
    
    // 如果之前处于超时状态，现在收到新指令，记录恢复日志
    if (was_timeout) {
        RCLCPP_INFO(this->get_logger(), 
            "✅ 已恢复：收到新的 /vehicle_command 指令，恢复正常发送");
    }
    
    // 从 ROS2 消息中提取数据
    std::string serialized_data(msg->pb.begin(), msg->pb.end());
    // std::cout << "Received len: " << msg->pb.size() << std::endl;
    // std::cout << "Received: len = " << serialized_data.size() << std::endl;
    // std::cout << "rec data (hex): ";
    // for (char c : serialized_data) {
    //   std::cout << std::hex << (static_cast<int>(c) & 0xFF) << " ";
    // }
    // std::cout << std::endl;
    // 使用 ParseFromString 反序列化
    control::ControlCommand protobuf_cmd;

    // both usage is ok
    //if(protobuf_msg.ParseFromArray(msg->data.data(), msg->data.size())){
    if (protobuf_cmd.ParseFromString(serialized_data)) {
    //   RCLCPP_INFO(this->get_logger(), "Received: name='%s', id=%d, value=%.2f", 
    //             protobuf_cmd.name().c_str(), protobuf_cmd.id(), protobuf_cmd.value());
    } else {
    //  RCLCPP_ERROR(this->get_logger(), "Failed to parse protobuf message");
    }
    //std::cout << std::setprecision(10) << "proto_steering_rate=" << protobuf_cmd.steering_rate() << std::endl;
    
    // LOG_I("cmd_throttle={},brake={},gear={},parking={}", 
    //     protobuf_cmd.throttle(),protobuf_cmd.brake(),protobuf_cmd.gear_location(),protobuf_cmd.parking_brake());
    // LOG_I("cmd_steering_rate={},arm={},a_en={},shovel={},s_en={}", protobuf_cmd.steering_rate(),
    //     protobuf_cmd.arm_angle(), protobuf_cmd.arm_enable(), protobuf_cmd.shovel_angle(), protobuf_cmd.shovel_enable());

    // 打印 xiaosong 专用的电流控制字段（如果存在）
    if (protobuf_cmd.has_arm_up_current() || protobuf_cmd.has_arm_down_current() || 
        protobuf_cmd.has_stick_retract_current() || protobuf_cmd.has_stick_extend_current() ||
        protobuf_cmd.has_bucket_close_current() || protobuf_cmd.has_bucket_dump_current() ||
        protobuf_cmd.has_rotate_left_current() || protobuf_cmd.has_rotate_right_current() ||
        protobuf_cmd.has_left_track_forward_current() || protobuf_cmd.has_left_track_backward_current() ||
        protobuf_cmd.has_right_track_forward_current() || protobuf_cmd.has_right_track_backward_current() ||
        protobuf_cmd.has_switch_bits()) {
        // LOG_I("xiaosong电流控制: arm_up={},arm_dn={},stick_r={},stick_e={},bucket_c={},bucket_d={},rot_l={},rot_r={},L_fwd={},L_bwd={},R_fwd={},R_bwd={},sw_bits={}", 
            // protobuf_cmd.has_arm_up_current() ? protobuf_cmd.arm_up_current() : 0.0,
            // protobuf_cmd.has_arm_down_current() ? protobuf_cmd.arm_down_current() : 0.0,
            // protobuf_cmd.has_stick_retract_current() ? protobuf_cmd.stick_retract_current() : 0.0,
            // protobuf_cmd.has_stick_extend_current() ? protobuf_cmd.stick_extend_current() : 0.0,
            // protobuf_cmd.has_bucket_close_current() ? protobuf_cmd.bucket_close_current() : 0.0,
            // protobuf_cmd.has_bucket_dump_current() ? protobuf_cmd.bucket_dump_current() : 0.0,
            // protobuf_cmd.has_rotate_left_current() ? protobuf_cmd.rotate_left_current() : 0.0,
            // protobuf_cmd.has_rotate_right_current() ? protobuf_cmd.rotate_right_current() : 0.0,
            // protobuf_cmd.has_left_track_forward_current() ? protobuf_cmd.left_track_forward_current() : 0.0,
            // protobuf_cmd.has_left_track_backward_current() ? protobuf_cmd.left_track_backward_current() : 0.0,
            // protobuf_cmd.has_right_track_forward_current() ? protobuf_cmd.right_track_forward_current() : 0.0,
            // protobuf_cmd.has_right_track_backward_current() ? protobuf_cmd.right_track_backward_current() : 0.0,
            // protobuf_cmd.has_switch_bits() ? protobuf_cmd.switch_bits() : 0);
    }

    chassisCanHandler->setCtlCmd(protobuf_cmd);
  }
/*
flatbuffers::DetachedBuffer MsgManager::genWireControlVehicleFaultOutput(void) {
  // Generate a wire control information buffer
  if (chassisCanHandler) {
    return chassisCanHandler->GenWireControlVehicleFaultOutput();
  }
  return flatbuffers::DetachedBuffer();
}

flatbuffers::DetachedBuffer MsgManager::genWireControlVehicleStatusOutput(void) {
  // Generate a wire control information buffer
  if (chassisCanHandler) {
    return chassisCanHandler->GenWireControlVehicleStatusOutput();
  }
  return flatbuffers::DetachedBuffer();
}
*/
void MsgManager::PublishChassisStatus()
{
    auto message = cyber_msgs::msg::ProtoAdapter(); //ros数据msg
    control::canbus::Chassis chassisptmsg;
    std::string str;

    chassisptmsg = chassisCanHandler->get_chassis_status_msg(); // 获取最新的底盘状态消息
    
    /*debug*/
    //chassisptmsg.set_gear_location(control::canbus::Chassis::GEAR_DRIVE);
    //chassisptmsg.set_driving_mode(control::canbus::Chassis::COMPLETE_AUTO_DRIVE);

    //double timestamp_sec =
    //ConvertStampToTimestampPb(vehicle_report_msg->header.stamp.sec, vehicle_report_msg->header.stamp.nanosec);
    // 设置 header 信息
    //protomsg.mutable_header()->set_timestamp_sec(controlcommand.header().timestamp_sec());
    //protomsg.mutable_header()->set_frame_id(controlcommand.header().frame_id());

    // 设置高压状态
    // switch (vehicle_info_.vehicle_mode) {
    // case 0:
    //   chassisptmsg.set_driving_mode(control::canbus::Chassis_DrivingMode::Chassis_DrivingMode_COMPLETE_MANUAL);
    //   break;
    // case 1:
    //   chassisptmsg.set_driving_mode(control::canbus::Chassis_DrivingMode::Chassis_DrivingMode_COMPLETE_AUTO_DRIVE);
    //   break;
    // default:
    //   chassisptmsg.set_driving_mode(control::canbus::Chassis_DrivingMode::Chassis_DrivingMode_COMPLETE_MANUAL);
    //   break;
    // }
    
     // 设置转向模式
    //Chassisptmsg.set_steer_mode(controlcommand.steering_model());
     chassisptmsg.SerializeToString(&str);
     // std::cout << "len: " << str.size() << std::endl;
     message.pb.assign(str.begin(), str.end());

     chassis_status_pub->publish(message);
    PublishChassisStatus_ros(chassisptmsg);
    //  RCLCPP_INFO(this->get_logger(), "Publishing: mode=%d", chassisptmsg.driving_mode());
    //  LOG_I("st_mode={}, gear={},speed={}", chassisptmsg.driving_mode(), 
    //       chassisptmsg.gear_location(),chassisptmsg.speed_mps());
    //  LOG_I("st_steering={}, arm={}, shovel={}", 
    //       chassisptmsg.steering_percentage(), chassisptmsg.arm_angle(), chassisptmsg.shovel_angle());
     //std::cout << "publish data." << " mode: "<< chassisptmsg.driving_mode() <<std::endl;
}

void MsgManager::PublishChassisStatus_ros(const control::canbus::Chassis chassisptmsg)
{
    auto message = cyber_msgs::msg::ChassisStatus();
    double time = chassisptmsg.header().timestamp_sec();

    // 说明：
    // - cannode 主线程（MsgManager::msgProc）循环周期由 PERIODTIME 控制，为 50ms（20Hz）
    // - 这里通过「每 2 次循环发布 1 次」来将 ROS2 的 /chassis_status 频率设置为约 10Hz，
    //   从而 /cannode/chassis_feedback 也约为 10Hz
    static uint8_t ros2pub_cnt = 0;
    if (++ros2pub_cnt < 2) {
      return;
    }
    ros2pub_cnt = 0;

    message.header.stamp.sec = time;
    message.header.stamp.nanosec = (time - message.header.stamp.sec)*1e9;

    message.header.frame_id = "cannode";

    // 上高压状态（从 engine_started 字段读取，changan 协议中 engine_started 表示上高压状态）
    message.high_voltage_status = chassisptmsg.has_engine_started() ? chassisptmsg.engine_started() : false;
    
    // 驻车制动状态
    message.parking_brake = chassisptmsg.has_parking_brake() ? chassisptmsg.parking_brake() : false;
    
    // 转向角度
    message.front_rear_angle = 0.0f; //转角 deg
    message.vehicle_mode = chassisptmsg.driving_mode();            //模式 0人工，1自动
    message.walk_motor_speed = chassisptmsg.speed_mps();           //车速 m/s
    message.boom_angle = static_cast<float>(chassisptmsg.arm_angle());                 //臂角度 deg
    message.bucket_angle = static_cast<float>(chassisptmsg.shovel_angle());            //铲斗角度 deg
    message.gear_signal = chassisptmsg.gear_location();         //档位 1 D，2 N，3 R
    message.throttle_opening = uint8_t(chassisptmsg.throttle_percentage());
    message.brake_opening = uint8_t(chassisptmsg.brake_percentage());
    
    // 电池电量
    message.battery_level = chassisptmsg.has_battery_soc_percentage() ? 
                            static_cast<uint8_t>(chassisptmsg.battery_soc_percentage()) : 0;
    
    // 压力数据暂时设置为0，因为protobuf中没有对应字段
    // 如果需要压力数据，需要从CAN消息中解析并添加到protobuf定义中
    message.boom_big_pressure = 0;
    message.boom_small_pressure = 0;
    message.bucket_big_pressure = 0;
    message.bucket_small_pressure = 0;
    
    // 获取 VCU 反馈数据（0x18FF1002）- 仅对 xiaosong 协议有效
    // 根据 excavator-can-vcu2host_utf8.csv: 周期50ms, VCU →上位机
    // 使用 dynamic_cast 检查是否为 XiaosongCANParser 实例
    auto* xiaosong_parser = dynamic_cast<XiaosongCANParser*>(chassisCanHandler.get());
    if (xiaosong_parser) {
        // 发布 VCU 反馈数据到独立话题
        std_msgs::msg::String vcu_feedback_msg;
        std::ostringstream oss;
        oss << "{";
        // bit 16: BIT0: is_remote (1是遥控驾驶有效)
        oss << "\"is_remote\":" << (xiaosong_parser->get_vcu_is_remote() ? "true" : "false") << ",";
        // bit 17: BIT1: is_emergency (1是急停有效)
        oss << "\"is_emergency\":" << (xiaosong_parser->get_vcu_is_emergency() ? "true" : "false") << ",";
        // 检查VCU通讯超时（1秒内没有收到反馈数据）
        bool vcu_timeout = xiaosong_parser->get_vcu_communication_timeout(1.0);
        oss << "\"vcu_communication_timeout\":" << (vcu_timeout ? "true" : "false") << ",";
        oss << "\"digital_inputs\":{";
        const uint8_t* inputs = xiaosong_parser->get_vcu_digital_inputs();
        // bit 0: 车梯收起到位信号, IN_43_DI(115#)
        oss << "\"in_43_di\":" << (inputs[0] ? "true" : "false") << ",";
        // bit 1-7: 开关量输入, IN_44_DI(140#) 到 IN_50_DI(196#)
        oss << "\"in_44_di\":" << (inputs[1] ? "true" : "false") << ",";  // bit 1: IN_44_DI(140#)
        oss << "\"in_45_di\":" << (inputs[2] ? "true" : "false") << ",";  // bit 2: IN_45_DI(155#)
        oss << "\"in_46_di\":" << (inputs[3] ? "true" : "false") << ",";  // bit 3: IN_46_DI(156#)
        oss << "\"in_47_di\":" << (inputs[4] ? "true" : "false") << ",";  // bit 4: IN_47_DI(172#)
        oss << "\"in_48_di\":" << (inputs[5] ? "true" : "false") << ",";  // bit 5: IN_48_DI(191#)
        oss << "\"in_49_di\":" << (inputs[6] ? "true" : "false") << ",";  // bit 6: IN_49_DI(192#)
        oss << "\"in_50_di\":" << (inputs[7] ? "true" : "false");         // bit 7: IN_50_DI(196#)
        oss << "}";
        // bit 19 BIT3: 液压锁状态 (1为可以动作; 0为不可以动作)，远控 vcu_feedback.hydraulic_lock_status
        oss << ",\"hydraulic_lock_status\":" << (xiaosong_parser->get_vcu_hydraulic_lock_status() ? "true" : "false");
        oss << "}";
        vcu_feedback_msg.data = oss.str();
        vcu_feedback_pub->publish(vcu_feedback_msg);
    }

    // 装载机协议反馈（前进/后退/转向电流等）
    auto* loader_parser = dynamic_cast<LoaderCANParser*>(chassisCanHandler.get());
    if (loader_parser) {
        std_msgs::msg::String vcu_feedback_msg;
        std::ostringstream oss;
        oss << std::boolalpha;
        oss << "{";
        oss << "\"protocol\":\"sjtu_loader\",";
        oss << "\"fb_forward\":" << loader_parser->get_fb_forward() << ",";
        oss << "\"fb_backward\":" << loader_parser->get_fb_backward() << ",";
        oss << "\"out_forward\":" << loader_parser->get_fb_out_forward() << ",";
        oss << "\"out_backward\":" << loader_parser->get_fb_out_backward() << ",";
        oss << "\"auto_mode\":" << loader_parser->get_auto_mode() << ",";
        oss << "\"throttle_out\":" << loader_parser->get_throttle_out() << ",";
        oss << "\"brake_out\":" << loader_parser->get_brake_out() << ",";
        oss << "\"left_steer_current_mA\":" << loader_parser->get_left_steer_current() << ",";
        oss << "\"right_steer_current_mA\":" << loader_parser->get_right_steer_current() << ",";
        oss << "\"boom_angle_x_deg\":" << loader_parser->get_boom_angle_x() << ",";
        oss << "\"bucket_angle_x_deg\":" << loader_parser->get_bucket_angle_x() << ",";
        oss << "\"wireless_estop\":" << loader_parser->get_wireless_estop() << ",";
        oss << "\"steer_angle_deg\":" << loader_parser->get_steer_angle_deg() << ",";
        oss << "\"speed_kmh\":" << loader_parser->get_speed_kmh() << ",";
        oss << "\"system_voltage\":" << loader_parser->get_system_voltage() << ",";
        oss << "\"controller_fault_code\":" << static_cast<int>(loader_parser->get_controller_fault_code()) << ",";
        oss << "\"vcu_communication_timeout\":"
            << loader_parser->get_vcu_communication_timeout(1.0);
        oss << "}";
        vcu_feedback_msg.data = oss.str();
        vcu_feedback_pub->publish(vcu_feedback_msg);
    }
    
    // LOG_I("front_angle={}, steering={}, vehicle_mode={}, driving_mode={}, walk_motor_speed={}, speed_mps={}", 
    //       message.front_rear_angle, chassisptmsg.steering_percentage(), message.vehicle_mode,
    //       chassisptmsg.driving_mode(), message.walk_motor_speed, chassisptmsg.speed_mps());
    // LOG_I("boom_angle={} , arm_angle={}, bucket_angle={}, shovel_angle={}",
    //       message.boom_angle, chassisptmsg.arm_angle()
    //       , message.bucket_angle, chassisptmsg.shovel_angle());

    // 发布消息
    chassis_status_pub_ros2->publish(message);
}

bool MsgManager::cannodeLoadPara(const std::string &path) {
    if(!GetProtoFromASCIIFile(path, &params)){
      LOG_E("vehicle param file parse error, use default param");
      return false;
    }
 
    vcu_type = params.brand();
    chassis_can_port = params.can_port();

    LOG_I("vehicle param brand = {}, can_port = {}", params.brand(), params.can_port());
  /*try {
    std::ifstream ifs(path, std::ios_base::binary);
    const auto cfg_file = toml::parse(ifs);

    // find a value with the specified type from a table
    std::string title = toml::find<std::string>(cfg_file, "title");
    LOG_Channel_Info(LOG_CHANNEL_NAME_CAN_ADAPTOR) << "title: " << title;

    // find a value from a table
    vehicle_number =
        toml::find<std::int64_t>(cfg_file, "vehicle", "self", "vehicle_number");
    LOG_Channel_Info(LOG_CHANNEL_NAME_CAN_ADAPTOR)
        << "vehicle_number: " << vehicle_number;

    // find parameters of PORT table
    const auto &port = toml::find(cfg_file, "CAN");
    chassis_can_port = toml::find<std::string>(port, "vcu_port", 0);
    chassis_can_baud = toml::find<int64_t>(port, "vcu_port", 1);
    LOG_Channel_Info(LOG_CHANNEL_NAME_CAN_ADAPTOR)
        << "vcu_port:" << chassis_can_port << "|" << chassis_can_baud;

    // find a value from a table
    vcu_type = toml::find<int>(cfg_file, "vehicle", "self", "vcu_type");
    //    vcu_type_ = toml::find<int>(vcuType, "type");
    // std::cout<<"vcu_type : "<<vcu_type_<<std::endl;
    LOG_Channel_Info(LOG_CHANNEL_NAME_CAN_ADAPTOR)
        << "vcu_type: " << vcu_type;

    return true;
  } catch (...) {
    LOG_Channel_Error(LOG_CHANNEL_NAME_CAN_ADAPTOR)
        << "can adaptor read cfg file fail, pls check.";
    return false;
  }*/
  return true;
}

void MsgManager::cannodeCfg() {
  std::string cmd;

  cmd = "ifconfig " + chassis_can_port + " down";
  // std::cout << cmd << std::endl;
  std::system(cmd.c_str());

  cmd = "ip link set " + chassis_can_port + " up type can bitrate " + std::to_string(chassis_can_baud) + " restart-ms 50";
  // std::cout << cmd << std::endl;
  std::system(cmd.c_str());

  cmd = "ifconfig " + chassis_can_port + " up";
  // std::cout << cmd << std::endl;
  std::system(cmd.c_str());
}

void MsgManager::cannodeCreateInstance() {
  switch (vcu_type) {
    case XU_GONG:
      // XugongCANParser 暂时未实现（xugong.h 被 #if 0 禁用），使用 ShantuiCANParser 作为替代
      // TODO: 如果需要 xugong 支持，需要从 eiloader 复制 xugong.h 和 xugong.cc 文件
      chassisCanHandler = std::make_shared<ShantuiCANParser>(chassis_can_port);
      break;
    case CHANG_AN:
      chassisCanHandler = std::make_shared<ChanganCANParser>(chassis_can_port);
      break;
    case SHAN_TUI:
      chassisCanHandler = std::make_shared<ShantuiCANParser>(chassis_can_port);
      break;
    case 13:  // XIAO_SONG
      chassisCanHandler = std::make_shared<XiaosongCANParser>(chassis_can_port);
      break;
    case 14:  // SJTU_LOADER
      chassisCanHandler = std::make_shared<LoaderCANParser>(chassis_can_port);
      break;
    default:
      chassisCanHandler = std::make_shared<ShantuiCANParser>(chassis_can_port);
      break;
  }
  // 在对象创建后显式调用初始化
  chassisCanHandler->Initialize();
}

void MsgManager::publishCanFrame(uint32_t can_id, uint8_t dlc, const uint8_t* data) {
  // 创建 JSON 格式的 CAN 帧信息
  std::ostringstream json_stream;
  json_stream << "{";
  json_stream << "\"can_id\":\"0x" << std::hex << std::setfill('0') << std::setw(8) << can_id << "\",";
  json_stream << "\"can_id_dec\":" << std::dec << can_id << ",";
  json_stream << "\"dlc\":" << static_cast<int>(dlc) << ",";
  json_stream << "\"data\":[";
  for (int i = 0; i < dlc; i++) {
    if (i > 0) json_stream << ",";
    // JSON 中数字必须是十进制，不能使用 0x 前缀
    json_stream << static_cast<int>(data[i]);
  }
  json_stream << "],";
  json_stream << "\"data_hex\":\"";
  for (int i = 0; i < dlc; i++) {
    json_stream << std::hex << std::setfill('0') << std::setw(2) 
                << static_cast<int>(data[i]);
    if (i < dlc - 1) json_stream << " ";
  }
  json_stream << "\"";
  json_stream << "}";
  
  // 发布消息
  auto msg = std_msgs::msg::String();
  msg.data = json_stream.str();
  can_frame_pub->publish(msg);
}

// 检查 /vehicle_command 超时，如果超时则发送默认值（安全机制）
void MsgManager::check_vehicle_cmd_timeout() {
    auto now = this->now();
    rclcpp::Time last_cmd_time;
    {
        std::lock_guard<std::mutex> lk(vehicle_cmd_timeout_mutex_);
        last_cmd_time = last_vehicle_cmd_time_;
    }
    
    double elapsed = (now - last_cmd_time).seconds();
    if (elapsed > vehicle_cmd_timeout_sec_) {
        // 超时，发送默认值（所有电流为0，开关量为0）
        static bool timeout_warned = false;
        if (!timeout_warned) {
            RCLCPP_WARN(this->get_logger(), 
                "⚠️  /vehicle_command 超时 (%.3f秒 > %.3f秒)，发送默认值（安全机制）", 
                elapsed, vehicle_cmd_timeout_sec_);
            timeout_warned = true;
        }
        
        // 发送默认控制命令（仅在超时时发送，一旦收到新指令，last_vehicle_cmd_time_ 会更新，下次检查就不会再发送）
        send_default_vehicle_command();
    } else {
        // 未超时，恢复正常状态（清除警告标志）
        static bool timeout_warned = false;
        if (timeout_warned) {
            timeout_warned = false;
        }
    }
}

// 发送默认控制命令（所有电流为0，开关量为0）
void MsgManager::send_default_vehicle_command() {
    if (!chassisCanHandler) {
        return;
    }
    
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
        
        // 安全相关状态设为默认值
        cmd.set_estop(false);
        cmd.set_parking_brake(false);
        cmd.set_request_remote_control(false);
        
        // 发送到 chassisCanHandler
        chassisCanHandler->setCtlCmd(cmd);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "发送默认控制命令时出错: %s", e.what());
    }
}

}  // namespace cannode
