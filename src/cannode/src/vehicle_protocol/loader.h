#pragma once

/**
 * 上海交大无人装载机线控协议（装载机_上位机通讯协议_20261102）
 * - CAN 标准帧，波特率 250kbps，周期 50ms
 * - 上位机→装载机: 0x100 / 0x101 / 0x102（0x103~0x105 预留）
 * - 装载机→上位机: 0x200 ~ 0x209
 */

#include <atomic>
#include <chrono>
#include <cstdint>
#include <linux/can.h>
#include <mutex>
#include <string>

#include "../chassis_driver.h"

namespace cannode {

enum LoaderCanProtoId {
  // 上位机 → 装载机
  LD_HOST_SWITCH   = 0x100,  // 前进/后退/喇叭/照明/心跳
  LD_HOST_PEDAL    = 0x101,  // 油门/刹车/左转/右转
  LD_HOST_ACTUATOR = 0x102,  // 大臂升落/铲斗升落

  // 装载机 → 上位机
  LD_FB_STATUS     = 0x200,  // 开关量/输出/故障/心跳反馈
  LD_FB_ANGLE      = 0x201,  // 倾角
  LD_FB_PRESSURE1  = 0x202,  // 压力1
  LD_FB_PRESSURE2  = 0x203,  // 压力2 + 油门/刹车输出
  LD_FB_CURRENT1   = 0x204,  // 阀电流1
  LD_FB_CURRENT2   = 0x205,  // 阀电流2 + 系统电压 + 转向角
  LD_FB_SPEED      = 0x206,  // 行驶速度 + 故障码
  LD_FB_ANGLE_RAW  = 0x207,  // 倾角原始值
  LD_FB_PRESS_RAW1 = 0x208,  // 压力传感器原始值1
  LD_FB_PRESS_RAW2 = 0x209,  // 压力/速度/电子尺原始值
};

#pragma pack(push, 1)

// 0x100 开关量 + 心跳
typedef struct LoaderHostSwitch {
  uint8_t forward : 1;      // byte0 bit0 前进
  uint8_t backward : 1;     // byte0 bit1 后退（与前进互斥）
  uint8_t horn : 1;         // byte0 bit2 喇叭
  uint8_t light : 1;        // byte0 bit3 照明
  uint8_t reserved0 : 4;
  uint8_t reserved1[6];
  uint8_t reserved7 : 7;
  uint8_t heartbeat : 1;    // byte7 bit7，500ms 翻转；连续 2s 收不到则停机
} LD_HostSwitch;

// 0x101 踏板与转向比例阀（小端 uint16，0~1000 → 0~100.0%）
typedef struct LoaderHostPedal {
  uint16_t throttle;        // byte0-1 油门
  uint16_t brake;           // byte2-3 刹车
  uint16_t steer_left;      // byte4-5 左转（与右转互斥）
  uint16_t steer_right;     // byte6-7 右转
} LD_HostPedal;

// 0x102 大臂/铲斗比例阀（小端 uint16，0~1000；同轴互斥）
typedef struct LoaderHostActuator {
  uint16_t boom_up;         // 大臂升
  uint16_t boom_down;       // 大臂落
  uint16_t bucket_up;       // 铲斗升
  uint16_t bucket_down;     // 铲斗落
} LD_HostActuator;

// 0x200 状态反馈（位域摘要，便于解析关键字段）
typedef struct LoaderFbStatus {
  // byte0: 设定值反馈
  uint8_t fb_forward : 1;
  uint8_t fb_backward : 1;
  uint8_t fb_horn : 1;
  uint8_t fb_light : 1;
  uint8_t reserved0 : 4;
  // byte1: 开关输入反馈
  uint8_t auto_mode : 1;       // 0手动 1自动
  uint8_t wireless_estop : 1;  // 无线急停
  uint8_t brake_sw1 : 1;
  uint8_t brake_sw2 : 1;
  uint8_t throttle_sw : 1;
  uint8_t reserved1 : 3;
  // byte2: 输出反馈
  uint8_t out_oil_switch : 1;
  uint8_t out_forward : 1;
  uint8_t out_backward : 1;
  uint8_t out_auto_lamp : 1;
  uint8_t out_reserve1 : 1;
  uint8_t out_reserve2 : 1;
  uint8_t out_boom_motor : 1;
  uint8_t out_bucket_motor : 1;
  // byte3
  uint8_t out_horn : 1;
  uint8_t out_light : 1;
  uint8_t reserved3 : 6;
  uint8_t reserved4;
  // byte5: 传感器故障
  uint8_t fault_sensors;
  // byte6: 阀/控制器故障
  uint8_t fault_valves;
  // byte7
  uint8_t reserved7 : 7;
  uint8_t heartbeat : 1;
} LD_FbStatus;

typedef struct LoaderFbAngle {
  uint16_t boom_x;          // 大臂前后倾角 X
  uint16_t boom_y;          // 大臂左右倾角 Y
  uint16_t bucket_x;        // 铲斗前后倾角 X
  uint16_t bucket_y;        // 铲斗左右倾角 Y
} LD_FbAngle;

typedef struct LoaderFbPressure1 {
  uint16_t left_steer_pressure;   // 0~400 → 0~40.0 MPa
  uint16_t right_steer_pressure;
  uint16_t boom_up_pressure;
  uint16_t boom_down_pressure;
} LD_FbPressure1;

typedef struct LoaderFbPressure2 {
  uint16_t bucket_up_pressure;
  uint16_t bucket_down_pressure;
  uint16_t throttle_out;          // 0~1000
  uint16_t brake_out;             // 0~1000
} LD_FbPressure2;

typedef struct LoaderFbCurrent1 {
  uint16_t left_steer_current;    // mA
  uint16_t right_steer_current;
  uint16_t boom_up_current;
  uint16_t boom_down_current;
} LD_FbCurrent1;

typedef struct LoaderFbCurrent2 {
  uint16_t bucket_up_current;
  uint16_t bucket_down_current;
  uint16_t system_voltage;        // 0~32000 → 0~32.000 V
  uint16_t steer_angle;           // 0~36000 → -180.00~+180.00°（偏移 180°；正=左摆）
} LD_FbCurrent2;

typedef struct LoaderFbSpeed {
  uint8_t speed;                  // 0~255 → 0~25.5 km/h
  uint8_t fault_code;
  uint8_t reserved[6];
} LD_FbSpeed;

typedef struct LoaderFbAngleRaw {
  uint16_t boom_x;
  uint16_t boom_y;
  uint16_t bucket_x;
  uint16_t bucket_y;
} LD_FbAngleRaw;

typedef struct LoaderFbPressRaw1 {
  uint16_t left_steer_ua;
  uint16_t right_steer_ua;
  uint16_t boom_up_ua;
  uint16_t boom_down_ua;
} LD_FbPressRaw1;

typedef struct LoaderFbPressRaw2 {
  uint16_t bucket_up_ua;
  uint16_t bucket_down_ua;
  uint16_t speed_raw;
  uint16_t steer_ruler_mv;        // 0~10000 mV
} LD_FbPressRaw2;

#pragma pack(pop)

/**
 * @brief 装载机 CAN 协议解析/打包
 *
 * ControlCommand 映射：
 * - gear D/R/N → 前进/后退/空档
 * - throttle/brake [0,100]% → 0~1000
 * - steering_target [-100,100]% → 左/右转比例阀（互斥）
 * - arm_up/down_current、bucket_close/dump_current：直接作 0~1000 设定值
 *   （约定 close=铲斗升，dump=铲斗落）
 * - VehicleSignal.horn / light_code bit0 → 喇叭/照明
 */
class LoaderCANParser : public ChassisCan {
 public:
  explicit LoaderCANParser(std::string canPort);
  ~LoaderCANParser() override;

  void initFuncMap() override;
  void SendCmdFunc() override;

  // 反馈访问（供 msg_manager 发布 JSON）
  bool get_fb_forward() const { return fb_forward_.load(); }
  bool get_fb_backward() const { return fb_backward_.load(); }
  bool get_fb_out_forward() const { return fb_out_forward_.load(); }
  bool get_fb_out_backward() const { return fb_out_backward_.load(); }
  bool get_auto_mode() const { return fb_auto_mode_.load(); }
  uint16_t get_throttle_out() const { return fb_throttle_out_.load(); }
  uint16_t get_brake_out() const { return fb_brake_out_.load(); }
  uint16_t get_left_steer_current() const { return fb_left_steer_current_.load(); }
  uint16_t get_right_steer_current() const { return fb_right_steer_current_.load(); }
  double get_boom_angle_x() const { return fb_boom_x_deg_.load(); }
  double get_bucket_angle_x() const { return fb_bucket_x_deg_.load(); }
  bool get_wireless_estop() const { return fb_wireless_estop_.load(); }
  double get_steer_angle_deg() const { return fb_steer_angle_deg_.load(); }
  double get_speed_kmh() const { return fb_speed_kmh_.load(); }
  uint8_t get_controller_fault_code() const { return fb_fault_code_.load(); }
  double get_system_voltage() const { return fb_system_voltage_.load(); }
  bool get_vcu_communication_timeout(double timeout_sec) const;

 private:
  void sendSwitchFrame();
  void sendPedalFrame();
  void sendActuatorFrame();

  void handle0x200(struct Canframe *recvCanFrame);
  void handle0x201(struct Canframe *recvCanFrame);
  void handle0x202(struct Canframe *recvCanFrame);
  void handle0x203(struct Canframe *recvCanFrame);
  void handle0x204(struct Canframe *recvCanFrame);
  void handle0x205(struct Canframe *recvCanFrame);
  void handle0x206(struct Canframe *recvCanFrame);
  void handle0x207(struct Canframe *recvCanFrame);
  void handle0x208(struct Canframe *recvCanFrame);
  void handle0x209(struct Canframe *recvCanFrame);

  static uint16_t clamp_u16(double v, double max_v);
  static double decode_signed_angle_deg(uint16_t raw);  // int16 * 0.01°
  static double decode_steer_angle_deg(uint16_t raw);   // 0~36000 → ±180°

  void publishSentFrame(uint32_t can_id, const uint8_t *data);

  // 心跳：500ms 翻转
  std::chrono::steady_clock::time_point last_heartbeat_toggle_;
  bool heartbeat_bit_{false};
  bool heartbeat_inited_{false};

  // 发送节流：协议 50ms；发送线程 10ms，用计数对齐
  uint32_t send_div_cnt_{0};

  // 反馈缓存
  std::atomic<bool> fb_forward_{false};
  std::atomic<bool> fb_backward_{false};
  std::atomic<bool> fb_out_forward_{false};
  std::atomic<bool> fb_out_backward_{false};
  std::atomic<bool> fb_auto_mode_{false};
  std::atomic<uint16_t> fb_throttle_out_{0};
  std::atomic<uint16_t> fb_brake_out_{0};
  std::atomic<uint16_t> fb_left_steer_current_{0};
  std::atomic<uint16_t> fb_right_steer_current_{0};
  std::atomic<double> fb_boom_x_deg_{0.0};
  std::atomic<double> fb_bucket_x_deg_{0.0};
  std::atomic<bool> fb_wireless_estop_{false};
  std::atomic<double> fb_steer_angle_deg_{0.0};
  std::atomic<double> fb_speed_kmh_{0.0};
  std::atomic<uint8_t> fb_fault_code_{0};
  std::atomic<double> fb_system_voltage_{0.0};

  mutable std::mutex fb_time_mutex_;
  std::chrono::steady_clock::time_point last_fb_time_;
  bool last_fb_time_inited_{false};
};

}  // namespace cannode
