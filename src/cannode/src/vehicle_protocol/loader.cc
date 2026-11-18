#include "loader.h"

#include <algorithm>
#include <cstring>

namespace cannode {

namespace {
constexpr double kSetpointMax = 1000.0;  // 协议 0~1000 → 0~100.0%
constexpr int kSendPeriodDiv = 5;        // 10ms 线程 × 5 = 50ms
}  // namespace

LoaderCANParser::LoaderCANParser(std::string canPort) : ChassisCan(canPort) {
  initFuncMap();
  LOG_I("=================LoaderCANParser (SJTU 装载机协议 20261102)=================");
}

LoaderCANParser::~LoaderCANParser() = default;

void LoaderCANParser::initFuncMap() {
  handleChassisCanFuncMap[LD_FB_STATUS] =
      static_cast<void (ChassisCan::*)(Canframe *)>(&LoaderCANParser::handle0x200);
  handleChassisCanFuncMap[LD_FB_ANGLE] =
      static_cast<void (ChassisCan::*)(Canframe *)>(&LoaderCANParser::handle0x201);
  handleChassisCanFuncMap[LD_FB_PRESSURE1] =
      static_cast<void (ChassisCan::*)(Canframe *)>(&LoaderCANParser::handle0x202);
  handleChassisCanFuncMap[LD_FB_PRESSURE2] =
      static_cast<void (ChassisCan::*)(Canframe *)>(&LoaderCANParser::handle0x203);
  handleChassisCanFuncMap[LD_FB_CURRENT1] =
      static_cast<void (ChassisCan::*)(Canframe *)>(&LoaderCANParser::handle0x204);
  handleChassisCanFuncMap[LD_FB_CURRENT2] =
      static_cast<void (ChassisCan::*)(Canframe *)>(&LoaderCANParser::handle0x205);
  handleChassisCanFuncMap[LD_FB_SPEED] =
      static_cast<void (ChassisCan::*)(Canframe *)>(&LoaderCANParser::handle0x206);
  handleChassisCanFuncMap[LD_FB_ANGLE_RAW] =
      static_cast<void (ChassisCan::*)(Canframe *)>(&LoaderCANParser::handle0x207);
  handleChassisCanFuncMap[LD_FB_PRESS_RAW1] =
      static_cast<void (ChassisCan::*)(Canframe *)>(&LoaderCANParser::handle0x208);
  handleChassisCanFuncMap[LD_FB_PRESS_RAW2] =
      static_cast<void (ChassisCan::*)(Canframe *)>(&LoaderCANParser::handle0x209);
}

uint16_t LoaderCANParser::clamp_u16(double v, double max_v) {
  if (v < 0.0) {
    v = 0.0;
  }
  if (v > max_v) {
    v = max_v;
  }
  return static_cast<uint16_t>(v + 0.5);
}

double LoaderCANParser::decode_signed_angle_deg(uint16_t raw) {
  // 协议：0~65535 表示 -32768~32767，精度 0.01°
  const int16_t signed_raw = static_cast<int16_t>(raw);
  return static_cast<double>(signed_raw) * 0.01;
}

double LoaderCANParser::decode_steer_angle_deg(uint16_t raw) {
  // 0x205：0~36000 → -180.00~+180.00°（偏移 180°）。正=左摆，负=右摆，0=正前方
  if (raw > 36000) {
    raw = 36000;
  }
  return static_cast<double>(raw) * 0.01 - 180.0;
}

void LoaderCANParser::publishSentFrame(uint32_t can_id, const uint8_t *data) {
  if (can_frame_send_callback_) {
    can_frame_send_callback_(can_id, 8, data);
  }
}

void LoaderCANParser::SendCmdFunc() {
  // 对齐 50ms 发送周期
  if ((++send_div_cnt_ % kSendPeriodDiv) != 0) {
    return;
  }

  // 心跳 500ms 翻转
  auto now = std::chrono::steady_clock::now();
  if (!heartbeat_inited_) {
    last_heartbeat_toggle_ = now;
    heartbeat_inited_ = true;
  } else {
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                  now - last_heartbeat_toggle_)
                  .count();
    if (ms >= 500) {
      heartbeat_bit_ = !heartbeat_bit_;
      last_heartbeat_toggle_ = now;
    }
  }

  sendSwitchFrame();
  sendPedalFrame();
  sendActuatorFrame();
}

void LoaderCANParser::sendSwitchFrame() {
  LD_HostSwitch sw{};
  std::memset(&sw, 0, sizeof(sw));

  {
    std::lock_guard<std::mutex> lk(vehicleCtlCmdMutex);

    // 急停：强制空档、关喇叭照明
    if (ctl_cmd.has_estop() && ctl_cmd.estop()) {
      sw.forward = 0;
      sw.backward = 0;
      sw.horn = 0;
      sw.light = 0;
    } else {
      // 档位 → 前进/后退（互斥；同时为 1 无动作）
      using Gear = control::canbus::Chassis::GearPosition;
      Gear gear = ctl_cmd.has_gear_location()
                      ? ctl_cmd.gear_location()
                      : control::canbus::Chassis::GEAR_NEUTRAL;
      if (gear == control::canbus::Chassis::GEAR_DRIVE) {
        sw.forward = 1;
        sw.backward = 0;
      } else if (gear == control::canbus::Chassis::GEAR_REVERSE) {
        sw.forward = 0;
        sw.backward = 1;
      } else {
        sw.forward = 0;
        sw.backward = 0;
      }

      // 喇叭
      if (ctl_cmd.has_signal() && ctl_cmd.signal().has_horn()) {
        sw.horn = ctl_cmd.signal().horn() ? 1 : 0;
      }

      // 照明：优先用 light_code 低位，或 signal.low_beam
      if (ctl_cmd.has_signal()) {
        if (ctl_cmd.signal().has_low_beam() && ctl_cmd.signal().low_beam()) {
          sw.light = 1;
        } else if (ctl_cmd.signal().has_high_beam() && ctl_cmd.signal().high_beam()) {
          sw.light = 1;
        }
      }
    }

    sw.heartbeat = heartbeat_bit_ ? 1 : 0;
  }

  can_frame frame{};
  frame.can_id = LD_HOST_SWITCH;
  frame.can_dlc = 8;
  std::memcpy(frame.data, &sw, sizeof(sw));
  publishSentFrame(frame.can_id, frame.data);
  SendCanFrame(frame);
}

void LoaderCANParser::sendPedalFrame() {
  LD_HostPedal pedal{};
  std::memset(&pedal, 0, sizeof(pedal));

  {
    std::lock_guard<std::mutex> lk(vehicleCtlCmdMutex);

    if (ctl_cmd.has_estop() && ctl_cmd.estop()) {
      // 急停：油门清零，刹车拉满
      pedal.throttle = 0;
      pedal.brake = 1000;
      pedal.steer_left = 0;
      pedal.steer_right = 0;
    } else {
      // throttle/brake: [0,100]% → 0~1000
      double thr = ctl_cmd.has_throttle() ? ctl_cmd.throttle() : 0.0;
      double brk = ctl_cmd.has_brake() ? ctl_cmd.brake() : 0.0;
      pedal.throttle = clamp_u16(thr * 10.0, kSetpointMax);
      pedal.brake = clamp_u16(brk * 10.0, kSetpointMax);

      // steering_target: [-100,100]% → 左/右比例阀（互斥）
      double steer = ctl_cmd.has_steering_target() ? ctl_cmd.steering_target() : 0.0;
      if (steer > 0.0) {
        pedal.steer_right = clamp_u16(steer * 10.0, kSetpointMax);
        pedal.steer_left = 0;
      } else if (steer < 0.0) {
        pedal.steer_left = clamp_u16((-steer) * 10.0, kSetpointMax);
        pedal.steer_right = 0;
      } else {
        // 也可直接用 rotate_left/right_current 作为 0~1000 设定
        double left = ctl_cmd.has_rotate_left_current() ? ctl_cmd.rotate_left_current() : 0.0;
        double right = ctl_cmd.has_rotate_right_current() ? ctl_cmd.rotate_right_current() : 0.0;
        if (left > 0.0 && right > 0.0) {
          // 互斥：两者同时 >0 时比例阀无动作 → 强制清零
          pedal.steer_left = 0;
          pedal.steer_right = 0;
        } else {
          pedal.steer_left = clamp_u16(left, kSetpointMax);
          pedal.steer_right = clamp_u16(right, kSetpointMax);
        }
      }
    }
  }

  can_frame frame{};
  frame.can_id = LD_HOST_PEDAL;
  frame.can_dlc = 8;
  std::memcpy(frame.data, &pedal, sizeof(pedal));
  publishSentFrame(frame.can_id, frame.data);
  SendCanFrame(frame);
}

void LoaderCANParser::sendActuatorFrame() {
  LD_HostActuator act{};
  std::memset(&act, 0, sizeof(act));

  {
    std::lock_guard<std::mutex> lk(vehicleCtlCmdMutex);

    if (ctl_cmd.has_estop() && ctl_cmd.estop()) {
      // 急停清零机构
    } else {
      double boom_up = ctl_cmd.has_arm_up_current() ? ctl_cmd.arm_up_current() : 0.0;
      double boom_down = ctl_cmd.has_arm_down_current() ? ctl_cmd.arm_down_current() : 0.0;
      double bucket_up = ctl_cmd.has_bucket_close_current() ? ctl_cmd.bucket_close_current() : 0.0;
      double bucket_down = ctl_cmd.has_bucket_dump_current() ? ctl_cmd.bucket_dump_current() : 0.0;

      // 同轴互斥：同时 >0 则无动作
      if (boom_up > 0.0 && boom_down > 0.0) {
        boom_up = 0.0;
        boom_down = 0.0;
      }
      if (bucket_up > 0.0 && bucket_down > 0.0) {
        bucket_up = 0.0;
        bucket_down = 0.0;
      }

      act.boom_up = clamp_u16(boom_up, kSetpointMax);
      act.boom_down = clamp_u16(boom_down, kSetpointMax);
      act.bucket_up = clamp_u16(bucket_up, kSetpointMax);
      act.bucket_down = clamp_u16(bucket_down, kSetpointMax);
    }
  }

  can_frame frame{};
  frame.can_id = LD_HOST_ACTUATOR;
  frame.can_dlc = 8;
  std::memcpy(frame.data, &act, sizeof(act));
  publishSentFrame(frame.can_id, frame.data);
  SendCanFrame(frame);
}

void LoaderCANParser::handle0x200(struct Canframe *recvCanFrame) {
  LD_FbStatus st{};
  std::memcpy(&st, recvCanFrame->frame.data, sizeof(st));

  fb_forward_.store(st.fb_forward != 0);
  fb_backward_.store(st.fb_backward != 0);
  fb_out_forward_.store(st.out_forward != 0);
  fb_out_backward_.store(st.out_backward != 0);
  fb_auto_mode_.store(st.auto_mode != 0);
  fb_wireless_estop_.store(st.wireless_estop != 0);

  {
    std::lock_guard<std::mutex> lk(fb_time_mutex_);
    last_fb_time_ = std::chrono::steady_clock::now();
    last_fb_time_inited_ = true;
  }

  std::lock_guard<std::mutex> lk(vehicleStatusMutex);
  chassisProtoMsg.set_driving_mode(st.auto_mode
                                       ? control::canbus::Chassis::COMPLETE_AUTO_DRIVE
                                       : control::canbus::Chassis::COMPLETE_MANUAL);
  if (st.out_forward) {
    chassisProtoMsg.set_gear_location(control::canbus::Chassis::GEAR_DRIVE);
  } else if (st.out_backward) {
    chassisProtoMsg.set_gear_location(control::canbus::Chassis::GEAR_REVERSE);
  } else {
    chassisProtoMsg.set_gear_location(control::canbus::Chassis::GEAR_NEUTRAL);
  }
}

void LoaderCANParser::handle0x201(struct Canframe *recvCanFrame) {
  LD_FbAngle ang{};
  std::memcpy(&ang, recvCanFrame->frame.data, sizeof(ang));

  const double boom_x = decode_signed_angle_deg(ang.boom_x);
  const double bucket_x = decode_signed_angle_deg(ang.bucket_x);
  fb_boom_x_deg_.store(boom_x);
  fb_bucket_x_deg_.store(bucket_x);

  std::lock_guard<std::mutex> lk(vehicleStatusMutex);
  chassisProtoMsg.set_arm_angle(boom_x);
  chassisProtoMsg.set_shovel_angle(bucket_x);
}

void LoaderCANParser::handle0x202(struct Canframe * /*recvCanFrame*/) {
  // 压力反馈暂存于需要时可扩展；当前仅刷新通信时间戳
  std::lock_guard<std::mutex> lk(fb_time_mutex_);
  last_fb_time_ = std::chrono::steady_clock::now();
  last_fb_time_inited_ = true;
}

void LoaderCANParser::handle0x203(struct Canframe *recvCanFrame) {
  LD_FbPressure2 p{};
  std::memcpy(&p, recvCanFrame->frame.data, sizeof(p));
  fb_throttle_out_.store(p.throttle_out);
  fb_brake_out_.store(p.brake_out);

  std::lock_guard<std::mutex> lk(vehicleStatusMutex);
  chassisProtoMsg.set_throttle_percentage(static_cast<double>(p.throttle_out) / 10.0);
  chassisProtoMsg.set_brake_percentage(static_cast<double>(p.brake_out) / 10.0);
}

void LoaderCANParser::handle0x204(struct Canframe *recvCanFrame) {
  LD_FbCurrent1 c{};
  std::memcpy(&c, recvCanFrame->frame.data, sizeof(c));
  fb_left_steer_current_.store(c.left_steer_current);
  fb_right_steer_current_.store(c.right_steer_current);
}

void LoaderCANParser::handle0x205(struct Canframe *recvCanFrame) {
  LD_FbCurrent2 c{};
  std::memcpy(&c, recvCanFrame->frame.data, sizeof(c));
  const double voltage = static_cast<double>(c.system_voltage) * 0.001;
  const double steer_deg = decode_steer_angle_deg(c.steer_angle);
  fb_system_voltage_.store(voltage);
  fb_steer_angle_deg_.store(steer_deg);

  {
    std::lock_guard<std::mutex> lk(fb_time_mutex_);
    last_fb_time_ = std::chrono::steady_clock::now();
    last_fb_time_inited_ = true;
  }

  std::lock_guard<std::mutex> lk(vehicleStatusMutex);
  // ±180° → ±100%；协议正值=左摆
  chassisProtoMsg.set_steering_percentage(std::max(-100.0, std::min(100.0, steer_deg / 1.8)));
}

void LoaderCANParser::handle0x206(struct Canframe *recvCanFrame) {
  LD_FbSpeed sp{};
  std::memcpy(&sp, recvCanFrame->frame.data, sizeof(sp));
  const double kmh = static_cast<double>(sp.speed) * 0.1;
  fb_speed_kmh_.store(kmh);
  fb_fault_code_.store(sp.fault_code);

  std::lock_guard<std::mutex> lk(vehicleStatusMutex);
  chassisProtoMsg.set_speed_mps(kmh / 3.6);
}

void LoaderCANParser::handle0x207(struct Canframe * /*recvCanFrame*/) {
  std::lock_guard<std::mutex> lk(fb_time_mutex_);
  last_fb_time_ = std::chrono::steady_clock::now();
  last_fb_time_inited_ = true;
}

void LoaderCANParser::handle0x208(struct Canframe * /*recvCanFrame*/) {
  std::lock_guard<std::mutex> lk(fb_time_mutex_);
  last_fb_time_ = std::chrono::steady_clock::now();
  last_fb_time_inited_ = true;
}

void LoaderCANParser::handle0x209(struct Canframe * /*recvCanFrame*/) {
  std::lock_guard<std::mutex> lk(fb_time_mutex_);
  last_fb_time_ = std::chrono::steady_clock::now();
  last_fb_time_inited_ = true;
}

bool LoaderCANParser::get_vcu_communication_timeout(double timeout_sec) const {
  std::lock_guard<std::mutex> lk(fb_time_mutex_);
  if (!last_fb_time_inited_) {
    return true;
  }
  auto now = std::chrono::steady_clock::now();
  double elapsed =
      std::chrono::duration_cast<std::chrono::duration<double>>(now - last_fb_time_).count();
  return elapsed > timeout_sec;
}

}  // namespace cannode
