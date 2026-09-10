#pragma once

#include <string>
#include <cstdint>
#include <optional>
#include <unordered_map>
#include <mutex> 
#include <linux/can.h> 
#include <sys/time.h> 
#include "common/log.h"
#include "socket_can_interface.h"
#include "cyber_msgs/msg/proto_adapter.hpp"
#include "../protobuf/out/common_msgs/chassis_msgs/chassis.pb.h"
#include "../protobuf/out/control_msgs/control_cmd.pb.h"
#include <functional>
namespace cannode {

#define PERIODTIME_SEND_THREAD 10  // ms
#define CAN_DLEN 8
struct Canframe {
  struct can_frame frame;
  struct timeval timestamp;
};

class ChassisCan : public SocketCanInterface {
public:
  ChassisCan(std::string canPort);
  virtual ~ChassisCan();
  control::canbus::Chassis& get_chassis_status_msg();
  void setCtlCmd(const control::ControlCommand& cmd);
  void SendCmdProc();
  void Initialize();
  void StartRecv();
  void HandleRecvData(struct Canframe *recvCanFrame);
  void callHandleWireCanCmdFunc(struct Canframe *recvCanFrame);

public:
  //映射命令处理函数
  typedef void (ChassisCan::*handleChassisCanFuncPtr)(struct Canframe *);
  std::unordered_map<uint32_t, handleChassisCanFuncPtr>
      handleChassisCanFuncMap;
  virtual void initFuncMap() = 0;
  virtual void SendCmdFunc() = 0;
  control::canbus::Chassis chassisProtoMsg; // 底盘状态 protobuf消息(各个车型管理赋值)
  control::ControlCommand ctl_cmd; // 控制命令 protobuf消息（回调赋值）
  std::mutex vehicleStatusMutex;
  std::mutex vehicleCtlCmdMutex;
  uint16_t VCU_heartbeat_cnt = 0;
  bool VCU_Life1 = true;
  
  // CAN 帧发送回调函数（用于通知外部发送的 CAN 帧信息）
  std::function<void(uint32_t can_id, uint8_t dlc, const uint8_t* data)> can_frame_send_callback_;
  
  // 设置 CAN 帧发送回调函数
  void setCanFrameSendCallback(std::function<void(uint32_t, uint8_t, const uint8_t*)> callback) {
    can_frame_send_callback_ = callback;
  }

  /// 原车发动机转速反馈（rpm），由 MsgManager 订阅 /cannode/engine_feedback 等注入；默认忽略。
  virtual void setHostEngineSpeedRpm(uint16_t rpm) { (void)rpm; }
};  // class ChassisCan
}  // namespace cannode
