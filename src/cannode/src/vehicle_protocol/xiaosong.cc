#include "xiaosong.h"
#include <iostream>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <linux/can.h>
#include <cmath>
#include <sstream>
#include <iomanip>
#include "../common/log.h"

namespace cannode {

XiaosongCANParser::XiaosongCANParser(std::string canPort, const std::string& config_file_path)
    : ChassisCan(canPort),
      current_speed_(0.0),
      current_arm_angle_(0.0),
      current_bucket_angle_(0.0),
      current_stick_angle_(0.0),
      current_rotate_angle_(0.0),
      current_gear_(0),
      feedback_arm_up_current_(0),
      feedback_arm_down_current_(0),
      feedback_stick_retract_current_(0),
      feedback_stick_extend_current_(0),
      feedback_bucket_close_current_(0),
      feedback_bucket_dump_current_(0),
      feedback_rotate_left_current_(0),
      feedback_rotate_right_current_(0),
      feedback_left_forward_current_(0),
      feedback_left_backward_current_(0),
      feedback_right_backward_current_(0),
      vcu_is_remote_(false),
      vcu_is_emergency_(false),
      vcu_hydraulic_lock_status_(false),
      vcu_feedback_timeout_initialized_(false)
      // heartbeat_counter_(0) // Commented out
       {
        // 初始化 VCU 数字输入状态
        std::memset(vcu_digital_inputs_, 0, sizeof(vcu_digital_inputs_));
        // 初始化最后收到VCU反馈的时间为当前时间（避免启动时立即触发超时）
        last_vcu_feedback_time_ = std::chrono::steady_clock::now();
    
    checkDataStructure();
    
    // 所有控制改为直接电流控制，不再需要PID配置管理器
    // std::string actual_config_path = config_file_path.empty() ? 
    //     "pid_config.yaml" : config_file_path;
    // pid_config_manager_ = std::make_unique<PIDConfigManager>(actual_config_path);
    
    // initPIDControllers();  // 不再需要初始化PID控制器
    initFuncMap();
    LOG_I("======================================================");
    LOG_I("=================XiaosongCANParser====================");
    LOG_I("======================================================");
}

XiaosongCANParser::~XiaosongCANParser() {
}

// 注册各个 CAN ID 对应的解析函数
void XiaosongCANParser::initFuncMap() {
    // 注册CAN消息处理函数
    handleChassisCanFuncMap[XS_STATUS_FB_01] = 
        static_cast<void (ChassisCan::*)(Canframe*)>(&XiaosongCANParser::handle0x18200001);
    handleChassisCanFuncMap[XS_MOTOR_STATUS_02] = 
        static_cast<void (ChassisCan::*)(Canframe*)>(&XiaosongCANParser::handle0x18200002);
    handleChassisCanFuncMap[XS_WALKING_STATUS_L] = 
        static_cast<void (ChassisCan::*)(Canframe*)>(&XiaosongCANParser::handle0x18200003);
    handleChassisCanFuncMap[XS_WALKING_STATUS_R] = 
        static_cast<void (ChassisCan::*)(Canframe*)>(&XiaosongCANParser::handle0x18200004);
    handleChassisCanFuncMap[XS_PRESSURE_01] = 
        static_cast<void (ChassisCan::*)(Canframe*)>(&XiaosongCANParser::handle0x18200005);
    handleChassisCanFuncMap[XS_PRESSURE_02] = 
        static_cast<void (ChassisCan::*)(Canframe*)>(&XiaosongCANParser::handle0x18200006);
    handleChassisCanFuncMap[XS_ANGLE_INFO] = 
        static_cast<void (ChassisCan::*)(Canframe*)>(&XiaosongCANParser::handle0x18200007);
    handleChassisCanFuncMap[XS_VALVE_CURRENT_1] = 
        static_cast<void (ChassisCan::*)(Canframe*)>(&XiaosongCANParser::handle0x18200008);
    handleChassisCanFuncMap[XS_VALVE_CURRENT_2] = 
        static_cast<void (ChassisCan::*)(Canframe*)>(&XiaosongCANParser::handle0x18200009);
    handleChassisCanFuncMap[XS_INPUT_STATUS] = 
        static_cast<void (ChassisCan::*)(Canframe*)>(&XiaosongCANParser::handle0x1820000A);
    handleChassisCanFuncMap[XS_VCU_FEEDBACK] = 
        static_cast<void (ChassisCan::*)(Canframe*)>(&XiaosongCANParser::handle0x18FF1002);
}

// 初始化 PID 控制器（所有控制改为直接电流控制，不再需要PID控制器）
void XiaosongCANParser::initPIDControllers() {
    // 所有控制改为直接电流控制，不再需要PID控制器
    // 保留函数以保持接口兼容性，但不再初始化任何控制器
}

// ================= PID外部配置接口实现 =================
// 所有控制改为直接电流控制，不再需要PID控制器配置接口
// void XiaosongCANParser::setSpeedPid(double kp, double ki, double kd,
//                                    double max_output, double min_output, double max_integral) {
//     speed_controller_.init(kp, ki, kd, max_output, min_output, max_integral);
// }

// void XiaosongCANParser::setArmPid(double kp, double ki, double kd,
//                                  double max_output, double min_output, double max_integral, double deadzone) {
//     arm_controller_.init(kp, ki, kd, max_output, min_output, max_integral, deadzone);
// }

// void XiaosongCANParser::setBucketPid(double kp, double ki, double kd,
//                                     double max_output, double min_output, double max_integral, double deadzone) {
//     bucket_controller_.init(kp, ki, kd, max_output, min_output, max_integral, deadzone);
// }

// void XiaosongCANParser::setZeroOffsets(double boom_zero_deg, double bucket_zero_deg) {
//     // 如果需要零点偏置，可以在这里实现
// }

// bool XiaosongCANParser::reloadPIDConfig() {
//     if (!pid_config_manager_) {
//         std::cerr << "[XiaosongCANParser] PID配置管理器未初始化" << std::endl;
//         return false;
//     }
//     
//     bool success = pid_config_manager_->reloadConfig();
//     if (!success) {
//         std::cerr << "[XiaosongCANParser] 配置文件重新加载失败" << std::endl;
//         return false;
//     }
//     
//     initPIDControllers();
//     return true;
// }

// 周期发送入口：计算真实 dt，并分发发送函数
void XiaosongCANParser::SendCmdFunc() {
    // 计算真实 dt（秒）
    double dt = 0.01; // fallback (10ms)
    auto now = std::chrono::steady_clock::now();
    if (last_ctrl_time_inited_) {
        std::chrono::duration<double> d = now - last_ctrl_time_;
        dt = d.count();
        if (dt <= 0.0 || dt > 0.2) { dt = 0.01; } // 异常保护
    }
    last_ctrl_time_ = now;
    last_ctrl_time_inited_ = true;

    // 10ms发送一次控制命令
    sendActuatorControl1(dt);        // 动臂/斗杆
    sendActuatorControl2(dt);        // 铲斗/回转
    sendWalkingControl(dt);          // 行走控制
    sendReserveControl();            // 备用比例阀
    sendSwitchControl();             // 开关量映射
    sendEngineThrottleControl();     // 发动机油门电压控制 (0x18FD0821)

    if(chassisProtoMsg.driving_mode() == control::canbus::Chassis::COMPLETE_AUTO_DRIVE) {
        sequence_cnt++;
    }
}

// 发送 0x18210001 机构电磁阀控制1（动臂下降、动臂抬升、斗杆伸出、斗杆收回）
void XiaosongCANParser::sendActuatorControl1(double dt) {
    XS_ActuatorCtrl1 actuatorCtrl = {0};
    can_frame canFrame;
    
    canFrame.can_id = XS_ACTUATOR_CTRL_1;
    canFrame.can_dlc = 8;
    
    {
        std::lock_guard<std::mutex> lk(vehicleCtlCmdMutex);
        
        // 动臂控制 - 直接电流控制 (200~700mA，无死区)
        double arm_up = ctl_cmd.has_arm_up_current() ? ctl_cmd.arm_up_current() : 0.0;
        double arm_down = ctl_cmd.has_arm_down_current() ? ctl_cmd.arm_down_current() : 0.0;
        // 只对非零输入做200~700mA限制，允许0
        if (arm_up != 0.0) arm_up = std::max(200.0, std::min(700.0, arm_up));
        if (arm_down != 0.0) arm_down = std::max(200.0, std::min(700.0, arm_down));
        
        actuatorCtrl.arm_down_current = static_cast<uint16_t>(arm_down);
        actuatorCtrl.arm_up_current = static_cast<uint16_t>(arm_up);
        
        // 斗杆控制 - 直接电流控制 (0~700mA，无死区)
        double stick_extend = ctl_cmd.has_stick_extend_current() ? ctl_cmd.stick_extend_current() : 0.0;
        double stick_retract = ctl_cmd.has_stick_retract_current() ? ctl_cmd.stick_retract_current() : 0.0;
        if (stick_extend != 0.0) stick_extend = std::max(200.0, std::min(700.0, stick_extend));
        if (stick_retract != 0.0) stick_retract = std::max(200.0, std::min(700.0, stick_retract));
        
        actuatorCtrl.stick_extend_current = static_cast<uint16_t>(stick_extend);
        actuatorCtrl.stick_retract_current = static_cast<uint16_t>(stick_retract);
        
        if(sequence_cnt % log_output_interval_ == 0) {
            LOG_I("动臂: 下降={}mA, 抬升={}mA", static_cast<int>(actuatorCtrl.arm_down_current), static_cast<int>(actuatorCtrl.arm_up_current));
            LOG_I("斗杆: 伸出={}mA, 收回={}mA", static_cast<int>(actuatorCtrl.stick_extend_current), static_cast<int>(actuatorCtrl.stick_retract_current));
        }
    }
    
    memcpy(canFrame.data, &actuatorCtrl, sizeof(XS_ActuatorCtrl1));
    
    if (can_frame_send_callback_) {
      can_frame_send_callback_(canFrame.can_id & CAN_EFF_MASK, canFrame.can_dlc, canFrame.data);
    }
    
    SendCanFrame(canFrame);
}

// 发送 0x18210002 机构电磁阀控制2（铲斗翻斗、铲斗收斗、回转左、回转右）
void XiaosongCANParser::sendActuatorControl2(double dt) {
    XS_ActuatorCtrl2 actuatorCtrl = {0};
    can_frame canFrame;
    
    canFrame.can_id = XS_ACTUATOR_CTRL_2;
    canFrame.can_dlc = 8;
    
    {
        std::lock_guard<std::mutex> lk(vehicleCtlCmdMutex);
        
        // 铲斗控制 - 直接电流控制 (0~700mA，无死区)
        double bucket_dump = ctl_cmd.has_bucket_dump_current() ? ctl_cmd.bucket_dump_current() : 0.0;
        double bucket_close = ctl_cmd.has_bucket_close_current() ? ctl_cmd.bucket_close_current() : 0.0;
        if (bucket_dump != 0.0) bucket_dump = std::max(200.0, std::min(700.0, bucket_dump));
        if (bucket_close != 0.0) bucket_close = std::max(200.0, std::min(700.0, bucket_close));
        
        actuatorCtrl.bucket_dump_current = static_cast<uint16_t>(bucket_dump);
        actuatorCtrl.bucket_close_current = static_cast<uint16_t>(bucket_close);
        
        // 回转控制 - 直接电流控制 (0~700mA，无死区)
        double rotate_left = ctl_cmd.has_rotate_left_current() ? ctl_cmd.rotate_left_current() : 0.0;
        double rotate_right = ctl_cmd.has_rotate_right_current() ? ctl_cmd.rotate_right_current() : 0.0;
        if (rotate_left != 0.0) rotate_left = std::max(200.0, std::min(700.0, rotate_left));
        if (rotate_right != 0.0) rotate_right = std::max(200.0, std::min(700.0, rotate_right));
        
        actuatorCtrl.rotate_left_current = static_cast<uint16_t>(rotate_left);
        actuatorCtrl.rotate_right_current = static_cast<uint16_t>(rotate_right);
        
        if(sequence_cnt % log_output_interval_ == 0) {
            LOG_I("铲斗: 翻斗={}mA, 收斗={}mA", static_cast<int>(actuatorCtrl.bucket_dump_current), static_cast<int>(actuatorCtrl.bucket_close_current));
            LOG_I("回转: 左转={}mA, 右转={}mA", static_cast<int>(actuatorCtrl.rotate_left_current), static_cast<int>(actuatorCtrl.rotate_right_current));
        }
    }
    
    memcpy(canFrame.data, &actuatorCtrl, sizeof(XS_ActuatorCtrl2));
    
    if (can_frame_send_callback_) {
      can_frame_send_callback_(canFrame.can_id & CAN_EFF_MASK, canFrame.can_dlc, canFrame.data);
    }
    
    SendCanFrame(canFrame);
}

// 发送 0x18210003 行走控制（左前进、左后退、右前进、右后退）
void XiaosongCANParser::sendWalkingControl(double dt) {
    XS_WalkingCtrl walkingCtrl = {0};
    can_frame canFrame;
    
    canFrame.can_id = XS_WALKING_CTRL;
    canFrame.can_dlc = 8;
    
    {
        std::lock_guard<std::mutex> lk(vehicleCtlCmdMutex);
        
        // 从控制命令获取左右履带电流值（直接电流控制，0~700mA）
        double left_fwd_current = ctl_cmd.has_left_track_forward_current() ? ctl_cmd.left_track_forward_current() : 0.0;
        double left_bwd_current = ctl_cmd.has_left_track_backward_current() ? ctl_cmd.left_track_backward_current() : 0.0;
        double right_fwd_current = ctl_cmd.has_right_track_forward_current() ? ctl_cmd.right_track_forward_current() : 0.0;
        double right_bwd_current = ctl_cmd.has_right_track_backward_current() ? ctl_cmd.right_track_backward_current() : 0.0;
        if (left_fwd_current != 0.0) left_fwd_current = std::max(200.0, std::min(700.0, left_fwd_current));
        if (left_bwd_current != 0.0) left_bwd_current = std::max(200.0, std::min(700.0, left_bwd_current));
        if (right_fwd_current != 0.0) right_fwd_current = std::max(200.0, std::min(700.0, right_fwd_current));
        if (right_bwd_current != 0.0) right_bwd_current = std::max(200.0, std::min(700.0, right_bwd_current));
        
        // 直接使用电流值
        walkingCtrl.left_forward_current = static_cast<uint16_t>(left_fwd_current);
        walkingCtrl.left_backward_current = static_cast<uint16_t>(left_bwd_current);
        walkingCtrl.right_forward_current = static_cast<uint16_t>(right_fwd_current);
        walkingCtrl.right_backward_current = static_cast<uint16_t>(right_bwd_current);
        
        if(sequence_cnt % log_output_interval_ == 0) {
            LOG_I("左履带: 前进={}mA, 后退={}mA", 
                  static_cast<int>(walkingCtrl.left_forward_current),
                  static_cast<int>(walkingCtrl.left_backward_current));
            LOG_I("右履带: 前进={}mA, 后退={}mA", 
                  static_cast<int>(walkingCtrl.right_forward_current),
                  static_cast<int>(walkingCtrl.right_backward_current));
        }
    }
    
    memcpy(canFrame.data, &walkingCtrl, sizeof(XS_WalkingCtrl));
    
    if (can_frame_send_callback_) {
      can_frame_send_callback_(canFrame.can_id & CAN_EFF_MASK, canFrame.can_dlc, canFrame.data);
    }
    
    SendCanFrame(canFrame);
}

// 发送 0x18210004 备用比例阀控制
void XiaosongCANParser::sendReserveControl() {
    XS_ReserveCtrl reserveCtrl = {0};
    can_frame canFrame;
    
    canFrame.can_id = XS_RESERVE_CTRL;
    canFrame.can_dlc = 8;
    
    {
        std::lock_guard<std::mutex> lk(vehicleCtlCmdMutex);
        // 备用比例阀暂时都设置为0
        reserveCtrl.reserve_valve1 = 0;
        reserveCtrl.reserve_valve2 = 0;
        reserveCtrl.reserve_valve3 = 0;
        reserveCtrl.reserve_valve4 = 0;
    }
    
    memcpy(canFrame.data, &reserveCtrl, sizeof(XS_ReserveCtrl));
    
    if (can_frame_send_callback_) {
      can_frame_send_callback_(canFrame.can_id & CAN_EFF_MASK, canFrame.can_dlc, canFrame.data);
    }
    
    SendCanFrame(canFrame);
}

// 发送 0x18FD0821 发动机油门控制器 gear 报文
// 起始位: 48, 长度: 8bit, 分辨率: 0.05V/bit；档位 1~7 线性映射电压 4V~1V（1 档 4V，7 档 1V）
void XiaosongCANParser::sendEngineThrottleControl() {
    can_frame canFrame{};
    canFrame.can_id = XS_ENGINE_THROTTLE_CTRL;
    canFrame.can_dlc = 8;

    // 默认所有字节为 0x00（除映射字段外其余位保持默认 0）
    for (int i = 0; i < 8; ++i) {
        canFrame.data[i] = 0x00;
    }

    {
        std::lock_guard<std::mutex> lk(vehicleCtlCmdMutex);

        // 使用 ControlCommand.pto_cmd 作为发动机油门档位（1~7），默认 1 档
        uint8_t gear_level = 1;
        if (ctl_cmd.has_pto_cmd()) {
            uint32_t raw = ctl_cmd.pto_cmd();
            if (raw < 1u) {
                raw = 1u;
            }
            if (raw > 7u) {
                raw = 7u;
            }
            gear_level = static_cast<uint8_t>(raw);
        }

        // 1~7 档线性映射到 4~1V（1 档->4V，7 档->1V）
        const double ratio = static_cast<double>(gear_level - 1u) / 6.0;
        double voltage = 4.0 - ratio * 3.0;

        // 转换为原始计数，分辨率 0.05V/bit
        double raw_val = voltage / 0.05;
        if (raw_val < 0.0) raw_val = 0.0;
        if (raw_val > 255.0) raw_val = 255.0;
        uint8_t raw_byte = static_cast<uint8_t>(std::lround(raw_val));

        // 起始位 48 对应 byte 6，同时镜像到 byte 7
        canFrame.data[6] = raw_byte;
        canFrame.data[7] = raw_byte;

        if (sequence_cnt % log_output_interval_ == 0) {
            LOG_I("Engine throttle ctrl: gear_level={} -> voltage={:.2f}V (raw=0x{:02X})",
                  static_cast<int>(gear_level), voltage, static_cast<int>(raw_byte));
        }
    }

    if (can_frame_send_callback_) {
        can_frame_send_callback_(canFrame.can_id & CAN_EFF_MASK, canFrame.can_dlc, canFrame.data);
    }

    SendCanFrame(canFrame);
}

// 发送 0x18210005 开关量映射控制（26个开关量）
void XiaosongCANParser::sendSwitchControl() {
    XS_SwitchCtrl switchCtrl = {0};
    can_frame canFrame;
    
    canFrame.can_id = XS_SWITCH_CTRL;
    canFrame.can_dlc = 8;
    
    {
        std::lock_guard<std::mutex> lk(vehicleCtlCmdMutex);
        
        // 从protobuf读取开关量位图 (bit 0-25对应switch_21到switch_41)
        uint32_t switch_bits = ctl_cmd.has_switch_bits() ? ctl_cmd.switch_bits() : 0;
        // 扩展位（根据 excavator-can-host2vcu_utf8.csv）：
        // bit32: 远程申请进入遥控操作 (1是遥控驾驶有效)
        // bit33: 远程申请进入急停状态 (1是急停有效)
        // bit34: 上电 (1是有效)
        // bit35: 发动机启动 (1是有效)
        // bit36: ACC 指令(DOH_OUT25_194) (1是有效)
        uint32_t switch_ext_bits = 0;
        if (ctl_cmd.has_request_remote_control() && ctl_cmd.request_remote_control()) {
            switch_ext_bits |= 0x1;           // bit32: 远程申请进入遥控操作 (1是遥控驾驶有效)
        }
        if (ctl_cmd.has_estop() && ctl_cmd.estop()) {
            switch_ext_bits |= 0x2;           // bit33: 远程申请进入急停状态 (1是急停有效)
        }
        // 从 switch_bits 的 bit 26 读取 powerEnable，映射到 switch_ext_bits 的 bit 34
        if (switch_bits & (1u << 26)) {
            switch_ext_bits |= 0x4;           // bit34: 上电 (1是有效)
        }
        // 从 switch_bits 的 bit 27 读取 startUp，映射到 switch_ext_bits 的 bit 35
        if (switch_bits & (1u << 27)) {
            switch_ext_bits |= 0x8;           // bit35: 发动机启动 (1是有效)
        }
        // 从 switch_bits 的 bit 28 读取 ACC 指令，映射到 switch_ext_bits 的 bit 36
        if (switch_bits & (1u << 28)) {
            switch_ext_bits |= 0x10;          // bit36: ACC 指令 (1是有效)
        }
        // 清除 switch_bits 中的临时存储位（bit 26-28），避免影响其他开关量
        switch_bits &= ~((1u << 26) | (1u << 27) | (1u << 28));
        
        // 按位映射到结构体字段（按照xiaosong.h中的定义顺序，根据 excavator-can-host2vcu_utf8.csv）
        switchCtrl.switch_21 = (switch_bits >> 0) & 0x01;   // bit 0: 车梯收起电磁阀, DOH_OUT17_125(125#线)
        switchCtrl.switch_22 = (switch_bits >> 1) & 0x01;   // bit 1: 车梯放下电磁阀, DOH_OUT18_129(129#线)
        switchCtrl.switch_23 = (switch_bits >> 2) & 0x01;   // bit 2: 车梯防落电磁阀
        switchCtrl.switch_24 = (switch_bits >> 3) & 0x01;   // bit 3: 先导安全锁定电磁阀, OUT_24_DOH(103#线)
        switchCtrl.switch_25 = (switch_bits >> 4) & 0x01;   // bit 4: 机器上推电磁阀
        switchCtrl.switch_26 = (switch_bits >> 5) & 0x01;   // bit 5: 直线行走电磁阀
        switchCtrl.switch_27 = (switch_bits >> 6) & 0x01;   // bit 6: 回转优先电磁阀
        switchCtrl.switch_28 = (switch_bits >> 7) & 0x01;   // bit 7: 回转停车制动电磁阀
        switchCtrl.switch_29 = (switch_bits >> 8) & 0x01;   // bit 8: 动臂卸载电磁阀
        switchCtrl.switch_30 = (switch_bits >> 9) & 0x01;   // bit 9: 开斗油缸电磁阀, OUT_26_DOH(193#线)
        switchCtrl.switch_31 = (switch_bits >> 10) & 0x01;  // bit 10: 关斗油缸电磁阀, OUT_37_DOL(183#线)
        switchCtrl.switch_19 = (switch_bits >> 11) & 0x01;  // bit 11: 喇叭, OUT_28_DOH(241#线)
        switchCtrl.switch_17 = (switch_bits >> 12) & 0x01;  // bit 12: 灯光1, OUT_19_DOH(128#线)
        switchCtrl.switch_18 = (switch_bits >> 13) & 0x01;  // bit 13: 灯光2, OUT_29_DOH(242#线)
        switchCtrl.switch_32 = (switch_bits >> 14) & 0x01;  // bit 14: 备用1
        switchCtrl.switch_20 = (switch_bits >> 15) & 0x01;  // bit 15: 备用2
        switchCtrl.switch_38 = (switch_bits >> 16) & 0x01;  // bit 16
        switchCtrl.switch_39 = (switch_bits >> 17) & 0x01;  // bit 17
        switchCtrl.switch_40 = (switch_bits >> 18) & 0x01;  // bit 18
        switchCtrl.switch_42 = (switch_bits >> 19) & 0x01;  // bit 19
        switchCtrl.switch_43 = (switch_bits >> 20) & 0x01;  // bit 20
        switchCtrl.switch_44 = (switch_bits >> 21) & 0x01;  // bit 21
        switchCtrl.switch_45 = (switch_bits >> 22) & 0x01;  // bit 22
        switchCtrl.switch_46 = (switch_bits >> 23) & 0x01;  // bit 23
        switchCtrl.switch_37 = (switch_bits >> 24) & 0x01;  // bit 24
        switchCtrl.switch_41 = (switch_bits >> 25) & 0x01;  // bit 25
        // 扩展位直接写入扩展字段（占用 byte4-7）
        switchCtrl.switch_ext_bits = switch_ext_bits;
        
        if(sequence_cnt % log_output_interval_ == 0) {
            LOG_I("开关量位图: 0x{:08X} ({}个激活)", switch_bits, __builtin_popcount(switch_bits));
            if (switch_ext_bits != 0) {
                LOG_I("扩展开关位: 0x{:08X} (bit32:远程申请进入遥控操作=%d, bit33:远程申请进入急停状态=%d, bit34:上电=%d, bit35:发动机启动=%d, bit36:ACC=%d)",
                      switch_ext_bits,
                      static_cast<int>((switch_ext_bits & 0x1) != 0),
                      static_cast<int>((switch_ext_bits & 0x2) != 0),
                      static_cast<int>((switch_ext_bits & 0x4) != 0),
                      static_cast<int>((switch_ext_bits & 0x8) != 0),
                      static_cast<int>((switch_ext_bits & 0x10) != 0));
            }
        }
    }
    
    memcpy(canFrame.data, &switchCtrl, sizeof(XS_SwitchCtrl));

    // excavator-can-host2vcu_utf8.csv：起始位 40、长度 16，发动机转速低 8 位 byte5、高 8 位 byte6，范围 0~8000 rpm
    {
        uint16_t eng_rpm = host_engine_speed_rpm_.load(std::memory_order_relaxed);
        if (eng_rpm > 8000u) {
            eng_rpm = 8000u;
        }
        canFrame.data[5] = static_cast<uint8_t>(eng_rpm & 0xFFu);
        canFrame.data[6] = static_cast<uint8_t>((eng_rpm >> 8) & 0xFFu);
    }
    
    if (can_frame_send_callback_) {
      can_frame_send_callback_(canFrame.can_id & CAN_EFF_MASK, canFrame.can_dlc, canFrame.data);
    }
    
    SendCanFrame(canFrame);
}

void XiaosongCANParser::setHostEngineSpeedRpm(uint16_t rpm) {
    if (rpm > 8000u) {
        rpm = 8000u;
    }
    host_engine_speed_rpm_.store(rpm, std::memory_order_relaxed);
}

// ================================
// 接收消息处理函数
// ================================

// 解析 0x18200001 状态反馈
void XiaosongCANParser::handle0x18200001(struct Canframe *recvCanFrame) {
    if (recvCanFrame->frame.can_dlc != 8) {
        return;
    }
    
    VCU_heartbeat_cnt = 0;  // 重置VCU心跳计数
    
    union XiaosongCanFrameMsg canFrameUnion = {{0}};
    std::memmove(canFrameUnion.canFrameData, recvCanFrame->frame.data, CAN_DLEN);
    
    {
        std::lock_guard<std::mutex> lk(vehicleStatusMutex);
        
        // 更新上高压状态
        chassisProtoMsg.set_engine_started(canFrameUnion.status_fb_01.high_voltage_sts != 0);
        
        // 更新驾驶模式
        if (canFrameUnion.status_fb_01.vehicle_mode) {
            chassisProtoMsg.set_driving_mode(control::canbus::Chassis::COMPLETE_AUTO_DRIVE);
        } else {
            chassisProtoMsg.set_driving_mode(control::canbus::Chassis::COMPLETE_MANUAL);
        }
        
        // 更新电池电量 (分辨率0.4%)
        double battery_soc = canFrameUnion.status_fb_01.battery_soc * 0.4;
        chassisProtoMsg.set_battery_soc_percentage(static_cast<int32_t>(battery_soc));
        
        // 更新驻车制动状态
        chassisProtoMsg.set_parking_brake(canFrameUnion.status_fb_01.parking_brake != 0);
    }
}

// 解析 0x18200002 电机状态
void XiaosongCANParser::handle0x18200002(struct Canframe *recvCanFrame) {
    if (recvCanFrame->frame.can_dlc != 8) {
        return;
    }
    
    union XiaosongCanFrameMsg canFrameUnion = {{0}};
    std::memmove(canFrameUnion.canFrameData, recvCanFrame->frame.data, CAN_DLEN);
    
    {
        std::lock_guard<std::mutex> lk(vehicleStatusMutex);
        // 可以记录液压电机的实际转速、扭矩等信息
    }
}

// 解析 0x18200003 左行走状态
void XiaosongCANParser::handle0x18200003(struct Canframe *recvCanFrame) {
    if (recvCanFrame->frame.can_dlc != 8) {
        return;
    }
    
    union XiaosongCanFrameMsg canFrameUnion = {{0}};
    std::memmove(canFrameUnion.canFrameData, recvCanFrame->frame.data, CAN_DLEN);
    
    {
        std::lock_guard<std::mutex> lk(vehicleStatusMutex);
        
        // 更新左履带线速度 (分辨率0.01m/s)
        double left_speed = canFrameUnion.walking_status_l.left_line_speed * 0.01;
        // 根据前进/后退电流确定速度方向
        if (feedback_left_backward_current_ > 0) {  // 后退
            left_speed = -left_speed;
        }
        current_speed_ = left_speed;
        chassisProtoMsg.set_speed_mps(current_speed_);
        
        // 更新左履带电流反馈值
        feedback_left_forward_current_ = canFrameUnion.walking_status_l.left_current;  // 注意：这里应该是实际发送的电流值，不是反馈值
        // 实际应该从发送的CAN帧中获取，这里暂时使用接收到的值
    }
}

// 解析 0x18200004 右行走状态
void XiaosongCANParser::handle0x18200004(struct Canframe *recvCanFrame) {
    if (recvCanFrame->frame.can_dlc != 8) {
        return;
    }
    
    union XiaosongCanFrameMsg canFrameUnion = {{0}};
    std::memmove(canFrameUnion.canFrameData, recvCanFrame->frame.data, CAN_DLEN);
    
    {
        std::lock_guard<std::mutex> lk(vehicleStatusMutex);
        
        // 更新右履带线速度 (分辨率0.01m/s)
        double right_speed = canFrameUnion.walking_status_r.right_line_speed * 0.01;
        // 根据前进/后退电流确定速度方向
        if (feedback_right_backward_current_ > 0) {  // 后退
            right_speed = -right_speed;
        }
        // 取左右速度的平均值
        current_speed_ = (current_speed_ + right_speed) / 2.0;
        chassisProtoMsg.set_speed_mps(current_speed_);
        
        // 更新右履带电流反馈值
        feedback_right_forward_current_ = canFrameUnion.walking_status_r.right_current;  // 注意：这里应该是实际发送的电流值
    }
}

// 解析 0x18200005 压力信息1
void XiaosongCANParser::handle0x18200005(struct Canframe *recvCanFrame) {
    if (recvCanFrame->frame.can_dlc != 8) {
        return;
    }
    
    union XiaosongCanFrameMsg canFrameUnion = {{0}};
    std::memmove(canFrameUnion.canFrameData, recvCanFrame->frame.data, CAN_DLEN);
    
    {
        std::lock_guard<std::mutex> lk(vehicleStatusMutex);
        // 可以记录压力信息
    }
}

// 解析 0x18200006 压力信息2
void XiaosongCANParser::handle0x18200006(struct Canframe *recvCanFrame) {
    if (recvCanFrame->frame.can_dlc != 8) {
        return;
    }
    
    union XiaosongCanFrameMsg canFrameUnion = {{0}};
    std::memmove(canFrameUnion.canFrameData, recvCanFrame->frame.data, CAN_DLEN);
    
    {
        std::lock_guard<std::mutex> lk(vehicleStatusMutex);
        // 可以记录压力信息
    }
}

// 解析 0x18200007 角度信息
// 注意：倾角仪反馈的角度部分暂时注释掉，等待后续实现
void XiaosongCANParser::handle0x18200007(struct Canframe *recvCanFrame) {
    if (recvCanFrame->frame.can_dlc != 8) {
        return;
    }
    
    union XiaosongCanFrameMsg canFrameUnion = {{0}};
    std::memmove(canFrameUnion.canFrameData, recvCanFrame->frame.data, CAN_DLEN);
    
    {
        std::lock_guard<std::mutex> lk(vehicleStatusMutex);
        
        // 倾角仪反馈的角度部分暂时注释掉
        // 更新大臂角度 (0~1800对应0~180.0°，分辨率0.1°)
        // current_arm_angle_ = canFrameUnion.angle_info.arm_angle * 0.1;
        // chassisProtoMsg.set_arm_angle(static_cast<float>(current_arm_angle_));
        
        // 更新铲斗角度 (0~1800对应0~180.0°，分辨率0.1°)
        // current_bucket_angle_ = canFrameUnion.angle_info.bucket_angle * 0.1;
        // chassisProtoMsg.set_shovel_angle(static_cast<float>(current_bucket_angle_));
        
        // 注意：车体横滚角和俯仰角在协议中超出8字节范围，可能需要单独处理
    }
}

// 解析 0x18200008 阀电流反馈1（大臂/斗杆）
void XiaosongCANParser::handle0x18200008(struct Canframe *recvCanFrame) {
    if (recvCanFrame->frame.can_dlc != 8) {
        return;
    }
    
    union XiaosongCanFrameMsg canFrameUnion = {{0}};
    std::memmove(canFrameUnion.canFrameData, recvCanFrame->frame.data, CAN_DLEN);
    
    {
        std::lock_guard<std::mutex> lk(vehicleStatusMutex);
        
        // 更新电流反馈值（注意：反馈消息中的顺序可能与控制消息不同，需要根据实际协议确认）
        feedback_arm_down_current_ = canFrameUnion.valve_current_1.arm_down_current;
        feedback_arm_up_current_ = canFrameUnion.valve_current_1.arm_up_current;
        feedback_stick_extend_current_ = canFrameUnion.valve_current_1.stick_extend_current;
        feedback_stick_retract_current_ = canFrameUnion.valve_current_1.stick_retract_current;
    }
}

// 解析 0x18200009 阀电流反馈2（铲斗/回转）
void XiaosongCANParser::handle0x18200009(struct Canframe *recvCanFrame) {
    if (recvCanFrame->frame.can_dlc != 8) {
        return;
    }
    
    union XiaosongCanFrameMsg canFrameUnion = {{0}};
    std::memmove(canFrameUnion.canFrameData, recvCanFrame->frame.data, CAN_DLEN);
    
    {
        std::lock_guard<std::mutex> lk(vehicleStatusMutex);
        
        // 更新电流反馈值（注意：反馈消息中的顺序可能与控制消息不同，需要根据实际协议确认）
        feedback_bucket_dump_current_ = canFrameUnion.valve_current_2.bucket_dump_current;
        feedback_bucket_close_current_ = canFrameUnion.valve_current_2.bucket_close_current;
        feedback_rotate_left_current_ = canFrameUnion.valve_current_2.rotate_left_current;
        feedback_rotate_right_current_ = canFrameUnion.valve_current_2.rotate_right_current;
    }
}

// 解析 0x1820000A 输入状态
void XiaosongCANParser::handle0x1820000A(struct Canframe *recvCanFrame) {
    if (recvCanFrame->frame.can_dlc != 8) {
        return;
    }
    
    union XiaosongCanFrameMsg canFrameUnion = {{0}};
    std::memmove(canFrameUnion.canFrameData, recvCanFrame->frame.data, CAN_DLEN);
    
    {
        std::lock_guard<std::mutex> lk(vehicleStatusMutex);
        // 可以记录输入状态
    }
}

// 解析 0x18FF1002 VCU反馈消息 - 根据 excavator-can-vcu2host_utf8.csv
// 周期50ms, VCU →上位机
// bit 0: 车梯收起到位信号, IN_43_DI(115#)
// bit 1-7: 开关量输入, IN_44_DI(140#) 到 IN_50_DI(196#)
// bit 16: BIT0: is_remote (1是遥控驾驶有效)
// bit 17: BIT1: is_emergency (1是急停有效)
// bit 18: BIT2: VCU和域控通讯超时 (1是超时)
// bit 19: BIT3: 液压锁状态 (1为可以动作; 0为不可以动作)
void XiaosongCANParser::handle0x18FF1002(struct Canframe *recvCanFrame) {
    if (recvCanFrame->frame.can_dlc != 8) {
        return;
    }
    
    union XiaosongCanFrameMsg canFrameUnion = {{0}};
    std::memmove(canFrameUnion.canFrameData, recvCanFrame->frame.data, CAN_DLEN);
    
    {
        std::lock_guard<std::mutex> lk(vehicleStatusMutex);
        
        // 更新数字输入状态
        vcu_digital_inputs_[0] = canFrameUnion.vcu_feedback.in_43_di;
        vcu_digital_inputs_[1] = canFrameUnion.vcu_feedback.in_44_di;
        vcu_digital_inputs_[2] = canFrameUnion.vcu_feedback.in_45_di;
        vcu_digital_inputs_[3] = canFrameUnion.vcu_feedback.in_46_di;
        vcu_digital_inputs_[4] = canFrameUnion.vcu_feedback.in_47_di;
        vcu_digital_inputs_[5] = canFrameUnion.vcu_feedback.in_48_di;
        vcu_digital_inputs_[6] = canFrameUnion.vcu_feedback.in_49_di;
        vcu_digital_inputs_[7] = canFrameUnion.vcu_feedback.in_50_di;
        
        // 更新状态标志
        vcu_is_remote_ = (canFrameUnion.vcu_feedback.is_remote != 0);
        vcu_is_emergency_ = (canFrameUnion.vcu_feedback.is_emergency != 0);
        vcu_hydraulic_lock_status_ = (canFrameUnion.vcu_feedback.hydraulic_lock_status != 0);
        
        // 更新最后收到VCU反馈的时间
        last_vcu_feedback_time_ = std::chrono::steady_clock::now();
        vcu_feedback_timeout_initialized_ = true;
        
        if(sequence_cnt % log_output_interval_ == 0) {
            LOG_I("VCU反馈: is_remote={}, is_emergency={}", 
                  static_cast<int>(vcu_is_remote_),
                  static_cast<int>(vcu_is_emergency_));
        }
    }
}

// 静态断言校验各结构体大小为 8 字节
void XiaosongCANParser::checkDataStructure() {
    static_assert(sizeof(XS_ActuatorCtrl1) == 8, "Invalid XS_ActuatorCtrl1 size");
    static_assert(sizeof(XS_ActuatorCtrl2) == 8, "Invalid XS_ActuatorCtrl2 size");
    static_assert(sizeof(XS_WalkingCtrl) == 8, "Invalid XS_WalkingCtrl size");
    static_assert(sizeof(XS_ReserveCtrl) == 8, "Invalid XS_ReserveCtrl size");
    static_assert(sizeof(XS_SwitchCtrl) == 8, "Invalid XS_SwitchCtrl size");
    static_assert(sizeof(XS_StatusFB01) == 8, "Invalid XS_StatusFB01 size");
    static_assert(sizeof(XS_MotorStatus02) == 8, "Invalid XS_MotorStatus02 size");
    static_assert(sizeof(XS_WalkingStatusL) == 8, "Invalid XS_WalkingStatusL size");
    static_assert(sizeof(XS_WalkingStatusR) == 8, "Invalid XS_WalkingStatusR size");
    static_assert(sizeof(XS_Pressure01) == 8, "Invalid XS_Pressure01 size");
    static_assert(sizeof(XS_Pressure02) == 8, "Invalid XS_Pressure02 size");
    static_assert(sizeof(XS_AngleInfo) == 8, "Invalid XS_AngleInfo size");
    static_assert(sizeof(XS_ValveCurrent1) == 8, "Invalid XS_ValveCurrent1 size");
    static_assert(sizeof(XS_ValveCurrent2) == 8, "Invalid XS_ValveCurrent2 size");
    static_assert(sizeof(XS_InputStatus) == 8, "Invalid XS_InputStatus size");
    static_assert(sizeof(XS_VCUFeedback) == 8, "Invalid XS_VCUFeedback size");
}

// 检查VCU通讯是否超时（1秒内没有收到反馈数据）
bool XiaosongCANParser::get_vcu_communication_timeout(double timeout_sec) const {
    if (!vcu_feedback_timeout_initialized_) {
        // 如果还没有收到过VCU反馈，不认为超时（避免启动时立即触发）
        return false;
    }
    
    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = now - last_vcu_feedback_time_;
    
    return elapsed.count() > timeout_sec;
}

} // namespace cannode

