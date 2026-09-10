
/**
 * *****************************************************************************
 * @file        msg_manager.h
 * @brief
 * @author      andy
 * @date        2025-09-15
 * @copyright Copyright (c) 2025
 * *****************************************************************************
 */
#ifndef MSG_MANAGER_H
#define MSG_MANAGER_H

#include <cstdint>
#include <array>
#include <iostream>
#include <fstream>
#include <string>
#include <thread>
#include <mutex>
#include "common/log.h"
#include "rclcpp/rclcpp.hpp"
#include "cyber_msgs/msg/proto_adapter.hpp"
#include "cyber_msgs/msg/chassis_status.hpp"
#include "std_msgs/msg/string.hpp"
#include "chassis_driver.h"
#include "vehicle_protocol/shantui.h"
#include "vehicle_protocol/changan.h"
#include "vehicle_protocol/xiaosong.h"
#include <cstring>
// #include "vehicle_protocol/xugong.h"  // 暂时注释，因为 xugong.h 被 #if 0 禁用
#include "common_msgs/config_msgs/vehicle_config.pb.h"

using namespace control::common;

namespace cannode {

#define CAN_CFG_FILE "vehicle_param.pb.txt"
#define PERIODTIME 50  // ms

class MsgManager : public rclcpp::Node {
 public:
  /*
   * @brief constructor function
   */
  MsgManager();

  /*
   * @brief destructor function
   */
  ~MsgManager();

  /**
   * @brief Initialize msg manager
   * @param
   * @return
   */
  bool init(void);
  /**
   * @brief Main processing function for publishing ROS messages
   * @param
   * @return
   */
  void msgProc();
  /**
   * @brief Process control command information #从控制获取protobuf消息
   * @param
   * @return
   */
  void procControlImplement(const cyber_msgs::msg::ProtoAdapter::SharedPtr msg) const;
  /**
   * @brief Generate a wire control information buffer # 生成线控信息反馈给控制
   * @param
   * @return
   */
  void PublishChassisStatus(); // 发布底盘状态消息
  void PublishChassisStatus_ros(const control::canbus::Chassis chassisptmsg);
  void publishCanFrame(uint32_t can_id, uint8_t dlc, const uint8_t* data); // 发布 CAN 帧信息
  void check_vehicle_cmd_timeout();  // 检查 /vehicle_command 超时，如果超时则发送默认值
  void send_default_vehicle_command();  // 发送默认控制命令（安全机制）

 private:
  // ROS2 发布订阅句柄指针
  rclcpp::Publisher<cyber_msgs::msg::ProtoAdapter>::SharedPtr chassis_status_pub;
  rclcpp::Subscription<cyber_msgs::msg::ProtoAdapter>::SharedPtr chassis_cmd_sub;
  rclcpp::Publisher<cyber_msgs::msg::ChassisStatus>::SharedPtr chassis_status_pub_ros2;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr vcu_feedback_pub;  // VCU 反馈数据发布者
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr can_frame_pub;  // CAN 帧信息发布者
  /// 订阅 engine_feedback_node 发布的 J1939 发动机转速，转发给小松 0x18210005
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr engine_feedback_sub_;
  //构建socket can节点实例所需参数，及cannode内部使用参数
  int32_t vcu_type{SHAN_TUI};
  std::string chassis_can_port = "can0";
  int64_t chassis_can_baud{250000};
  std::shared_ptr<ChassisCan> chassisCanHandler;
  VehicleParam params;
  
  // 安全超时机制
  mutable std::mutex vehicle_cmd_timeout_mutex_;
  mutable rclcpp::Time last_vehicle_cmd_time_;  // 最后收到 /vehicle_command 的时间（mutable 以便在 const 函数中修改）
  double vehicle_cmd_timeout_sec_{1.0};  // /vehicle_command 超时时间（秒），默认1.0s
  rclcpp::TimerBase::SharedPtr vehicle_cmd_timeout_timer_;  // 超时检查定时器

 private:
  bool cannodeLoadPara(const std::string &path);
  void cannodeCfg();
  void cannodeCreateInstance();
};

}  // namespace can

#endif // MSG_MANAGER_H
