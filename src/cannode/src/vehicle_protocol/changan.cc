#include "changan.h"
#include <iostream>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <linux/can.h>
#include <cmath>
#include <sstream>
#include <iomanip>
#include "changan.h"
#include "../common/log.h"
namespace cannode {

ChanganCANParser::ChanganCANParser(std::string canPort, const std::string& config_file_path)
    : ChassisCan(canPort),
      current_speed_(0.0),
      current_arm_angle_(0.0),
      current_bucket_angle_(0.0),
      current_steer_angle_(0.0),
      current_gear_(0),
      danfoss_status_(1),  // 默认状态为初始化
      heartbeat_counter_(0)
       {  // 初始化为0
    
    checkDataStructure();
    
    // 初始化PID配置管理器
    std::string actual_config_path = config_file_path.empty() ? 
        "pid_config.yaml" : config_file_path;
    pid_config_manager_ = std::make_unique<PIDConfigManager>(actual_config_path);
    
    initPIDControllers();
    initFuncMap();
    LOG_I("======================================================");
    LOG_I("=================New-ChanganCANParser=================");
    LOG_I("======================================================");
    // 默认启用CSV记录（使用时间戳文件名，包含默认torque值）
    // enableCSVLogging("", current_torque_value_);  // 注释掉，等第一次sendWalkingControl时再创建
    // CSV记录功能已完全关闭

}

ChanganCANParser::~ChanganCANParser() {
    // closeCSVLogging();  // CSV记录功能已关闭
}

// 注册各个 CAN ID 对应的解析函数，用于将收到的帧分发至对应 handler
// 调用处：构造函数内
void ChanganCANParser::initFuncMap() {
    // 注册CAN消息处理函数
    handleChassisCanFuncMap[CA_STATUS_FB_01] = 
        static_cast<void (ChassisCan::*)(Canframe*)>(&ChanganCANParser::handle0x181F0001);
    handleChassisCanFuncMap[CA_MOTOR_STATUS_02] = 
        static_cast<void (ChassisCan::*)(Canframe*)>(&ChanganCANParser::handle0x181F0002);
    handleChassisCanFuncMap[CA_WALKING_STATUS] = 
        static_cast<void (ChassisCan::*)(Canframe*)>(&ChanganCANParser::handle0x181F0003);
    handleChassisCanFuncMap[CA_DANFOSS_STEERING] = 
        static_cast<void (ChassisCan::*)(Canframe*)>(&ChanganCANParser::handle0x181F0007);

    // 倾角仪与转向角编码器（新增）
    handleChassisCanFuncMap[CA_IMU_BODY_PITCH] =
        static_cast<void (ChassisCan::*)(Canframe*)>(&ChanganCANParser::handle0x00000581);
    handleChassisCanFuncMap[CA_IMU_BOOM_PITCH] =
        static_cast<void (ChassisCan::*)(Canframe*)>(&ChanganCANParser::handle0x00000582);
    handleChassisCanFuncMap[CA_IMU_BUCKET_PITCH] =
        static_cast<void (ChassisCan::*)(Canframe*)>(&ChanganCANParser::handle0x00000583);
    handleChassisCanFuncMap[CA_STEER_ENCODER] =
        static_cast<void (ChassisCan::*)(Canframe*)>(&ChanganCANParser::handle0x18FF0015);
}

// 初始化 4 个 PID 控制器（从配置文件加载参数）
// 调用处：构造函数内
void ChanganCANParser::initPIDControllers() {
    // 加载配置文件
    bool config_loaded = pid_config_manager_->loadConfig();
    if (!config_loaded) {
        // std::cout << "[ChanganCANParser] 使用默认PID参数" << std::endl;
    }
    
    // 获取配置
    const auto& config = pid_config_manager_->getConfig();
    
    // 初始化速度控制器 (加速度到速度)
    speed_controller_.init(
        config.speed_controller.kp,
        config.speed_controller.ki,
        config.speed_controller.kd,
        config.speed_controller.max_output,
        config.speed_controller.min_output,
        config.speed_controller.max_integral
    );
    
    // 初始化大臂角度控制器 (阀开度到角度)
    arm_controller_.init(
        config.arm_controller.kp,
        config.arm_controller.ki,
        config.arm_controller.kd,
        config.arm_controller.max_output,
        config.arm_controller.min_output,
        config.arm_controller.max_integral,
        config.arm_controller.deadzone
    );
    
    // 初始化铲斗角度控制器 (阀开度到角度)
    bucket_controller_.init(
        config.bucket_controller.kp,
        config.bucket_controller.ki,
        config.bucket_controller.kd,
        config.bucket_controller.max_output,
        config.bucket_controller.min_output,
        config.bucket_controller.max_integral,
        config.bucket_controller.deadzone
    );
    
    // 初始化转向角度控制器 (阀开度到角度)
    steer_controller_.init(
        config.steer_controller.kp,
        config.steer_controller.ki,
        config.steer_controller.kd,
        config.steer_controller.max_output,
        config.steer_controller.min_output,
        config.steer_controller.max_integral,
        config.steer_controller.deadzone
    );
    
    // 设置零点偏置
    setZeroOffsets(
        config.zero_offsets.boom_zero_deg,
        config.zero_offsets.bucket_zero_deg,
        config.zero_offsets.steer_zero_deg
    );
    
    // std::cout << "[ChanganCANParser] PID控制器初始化完成" << std::endl;
}

// ================= PID/零点外部配置接口实现 =================
void ChanganCANParser::setSpeedPid(double kp, double ki, double kd,
                                   double max_output, double min_output, double max_integral) {
    speed_controller_.init(kp, ki, kd, max_output, min_output, max_integral);
}

void ChanganCANParser::setArmPid(double kp, double ki, double kd,
                                 double max_output, double min_output, double max_integral, double deadzone) {
    arm_controller_.init(kp, ki, kd, max_output, min_output, max_integral, deadzone);
}

void ChanganCANParser::setBucketPid(double kp, double ki, double kd,
                                    double max_output, double min_output, double max_integral, double deadzone) {
    bucket_controller_.init(kp, ki, kd, max_output, min_output, max_integral, deadzone);
}

void ChanganCANParser::setSteerPid(double kp, double ki, double kd,
                                   double max_output, double min_output, double max_integral, double deadzone) {
    steer_controller_.init(kp, ki, kd, max_output, min_output, max_integral, deadzone);
}

void ChanganCANParser::setZeroOffsets(double boom_zero_deg, double bucket_zero_deg, double steer_zero_deg) {
    angle_sensors_.boom_zero_offset_deg = boom_zero_deg;
    angle_sensors_.bucket_zero_offset_deg = bucket_zero_deg;
    angle_sensors_.steer_zero_offset_deg = steer_zero_deg;
}

bool ChanganCANParser::reloadPIDConfig() {
    if (!pid_config_manager_) {
        // std::cerr << "[ChanganCANParser] PID配置管理器未初始化" << std::endl;
        return false;
    }
    
    // 重新加载配置文件
    bool success = pid_config_manager_->reloadConfig();
    if (!success) {
        // std::cerr << "[ChanganCANParser] 配置文件重新加载失败" << std::endl;
        return false;
    }
    
    // 重新初始化PID控制器
    initPIDControllers();
    
    // std::cout << "[ChanganCANParser] PID配置重新加载成功" << std::endl;
    return true;
}

// 周期发送入口：计算真实 dt，并分发 4 类发送函数
// 调用处：`ChassisCan::SendCmdProc()` 周期线程
void ChanganCANParser::SendCmdFunc() {

    // static int cnt_50ms = 0;  // 未使用，暂时注释
    
    // 计算真实 dt（秒）
    double dt = 0.02; // fallback
    auto now = std::chrono::steady_clock::now();
    if (last_ctrl_time_inited_) {
        std::chrono::duration<double> d = now - last_ctrl_time_;
        dt = d.count();
        if (dt <= 0.0 || dt > 0.2) { dt = 0.02; } // 异常保护
    }
    last_ctrl_time_ = now;
    last_ctrl_time_inited_ = true;

    // 20ms发送一次控制命令

    sendChassisControl();
    sendHydraulicControl(dt);
    sendWalkingControl(dt);
    sendActuatorControl(dt);
    

    // if (ctl_cmd.header().sequence_num() != sequence_num_) {
        // sequence_num_ = ctl_cmd.header().sequence_num();
    if(chassisProtoMsg.driving_mode() == control::canbus::Chassis::COMPLETE_AUTO_DRIVE)
    {
        sequence_cnt++;
    }
    // }

}

// 发送 0x18000001 底盘控制（上高压/心跳/模式等）
// 调用处：SendCmdFunc()
void ChanganCANParser::sendChassisControl() {
    CA_ChassisCtrl chassisCtrl = {0};
    can_frame canFrame;
    
    canFrame.can_id = CA_CHASSIS_CTRL;
    canFrame.can_dlc = 8;
 
    
    {
        std::lock_guard<std::mutex> lk(vehicleCtlCmdMutex);
        // 默认上高压
        chassisCtrl.high_voltage_enable = 1;
        
        // 软急停（可从control_cmd获取）
        chassisCtrl.soft_estop = ctl_cmd.estop() ? 1 : 0;
        
        // 心跳信号（至少200ms翻转一次，即每10次调用翻转一次，20ms*10=200ms实际，这里简化为计数，每次都翻转。）
        if (++heartbeat_counter_ >= 10) {  // 20ms * 10 = 200ms
            heartbeat_counter_ = 0;
            chassisCtrl.heartbeat = !chassisCtrl.heartbeat;
        } else {
            chassisCtrl.heartbeat = (heartbeat_counter_ < 5) ? 0 : 1;
        }
        
        // 整车模式（边缘触发：只有当值发生变化时才发送一次新值，否则翻转）
        // if (sequence_cnt==10){
        // // if (chassisProtoMsg.driving_mode() != control::canbus::Chassis::COMPLETE_AUTO_DRIVE) {
        //     chassisCtrl.driving_mode = 1;
        // }
        // else {
        chassisCtrl.driving_mode = 0;
        // }

        // 发送档位
        chassisCtrl.speed_gear = 0;
        // chassisCtrl.speed_gear = ctl_cmd.speed_gear() ? 1 : 0;

        // 清除转向阀错误
        // danfoss_status为0x00(正常)、0x11(初始化)、0xFF(故障)时，清除错误；其他值(未知状态)时不清除
        if(danfoss_status_ == 0xFF) {
            chassisCtrl.clear_steer_error = 1;  // 清除转向阀错误   bit 12: 清除转向阀错误
        }
        else{
            chassisCtrl.clear_steer_error = 0;  // 清除转向阀错误   bit 12: 清除转向阀错误
        }
        if(sequence_cnt % log_output_interval_ == 0) {
            LOG_I("--------sequence_cnt: {} (20ms周期计数器)---------", sequence_cnt);
            LOG_I("chassisCtrl.soft_estop: {} (0=正常,1=急停)", static_cast<int>(chassisCtrl.soft_estop));
            LOG_I("chassisCtrl.speed_gear: {} (0=低速档,1=高速档)", static_cast<int>(chassisCtrl.speed_gear));
            LOG_I("chassisCtrl.work_light: {} (0=关闭,1=打开)", static_cast<int>(chassisCtrl.work_light));
            LOG_I("chassisCtrl.clear_steer_error: {} (0=正常,1=清除错误)", static_cast<int>(chassisCtrl.clear_steer_error));
            LOG_I("chassisProtoMsg.driving_mode(): {} (0=手动,1=自动)", static_cast<int>(chassisProtoMsg.driving_mode()));
            LOG_I("danfoss_status: 0x{:02X} (0x00=正常,0x11=初始化,0xFF=故障,其他=未知)", danfoss_status_);
        }
        // 工作灯等其他控制可以根据需要添加
    }
    
    memcpy(canFrame.data, &chassisCtrl, sizeof(CA_ChassisCtrl));
    
    // 发布 CAN 帧信息（如果回调已设置）
    if (can_frame_send_callback_) {
      can_frame_send_callback_(canFrame.can_id & CAN_EFF_MASK, canFrame.can_dlc, canFrame.data);
    }
    
    SendCanFrame(canFrame);
}


// 发送 0x18000002 液压/转向控制（转向阀 PID 输出电流与方向）
// 调用处：SendCmdFunc()
void ChanganCANParser::sendHydraulicControl(double dt) {
    CA_HydraulicCtrl hydraulicCtrl = {0};
    can_frame canFrame;
    
    canFrame.can_id = CA_HYDRAULIC_CTRL;
    canFrame.can_dlc = 8;
    
    {
        std::lock_guard<std::mutex> lk(vehicleCtlCmdMutex);
        // 液压电机使能
        hydraulicCtrl.hyd_motor_enable = 1;
        
        // 液压电机工作模式: 1=转速模式, 2=扭矩模式
        hydraulicCtrl.hyd_work_mode = 1;  // 转速模式
        
        // 液压电机经济模式: 0=标准, 1=经济, 2=动力
        hydraulicCtrl.hyd_econ_mode = 0;  // 标准模式
        
        // 液压电机转速 (范围-15000~15000, 零值为15000，平时默认发送稳定转速，物理值2000rpm)
        // 注意：如果需要设置实际转速，可以从control_cmd中读取
        hydraulicCtrl.hyd_req_speed = 17000;  // 默认发送稳定转速，物理值2000rpm
        
        // 液压电机扭矩 (范围-3000~3000, 有3000的offset, 默认物理值为0Nm, CAN发送3000)
        hydraulicCtrl.hyd_req_torque = 3000;  // 零值，物理值0Nm
        

        // 转向控制 - 使用转向角度控制器
        // 向左转是正、向右转是负。
        double target_steer_angle = ctl_cmd.steering_target() * 180 / M_PI;  // 单位是弧度。
        if(target_steer_angle < -40.0) target_steer_angle = -40.0;
        if(target_steer_angle > 40.0) target_steer_angle = 40.0;

        double steer_valve_current = 0.0;
        double steer_valve_current_abs =0.0;
        if(steer_encoder_available_) {

            if(danfoss_status_ == 0x00 || danfoss_status_ == 0x11 || danfoss_status_ == 0xFF)
            {
                hydraulicCtrl.steer_valve_dir = 0;
                hydraulicCtrl.steer_valve_current = 0;
            }
            else
            {
                // 计算转向阀电流 (通过PID控制器)
                steer_valve_current = steer_controller_.compute(target_steer_angle, current_steer_angle_, dt);
                double steer_error = target_steer_angle - current_steer_angle_;

                // 取绝对值并限幅到 [0, 500] (上限是1500 mA）
                steer_valve_current_abs = std::fabs(steer_valve_current);
                if (steer_valve_current_abs > 800.0) steer_valve_current_abs = 800.0;
                
                // danfoss_status为0x00(正常)、0x11(初始化)、0xFF(故障)时，允许正常转向；其他值(未知状态)时不转向
                
                // 确定转向方向
                if (steer_error > 3.0) {
                    hydraulicCtrl.steer_valve_dir = 1;  // 左转
                    hydraulicCtrl.steer_valve_current = static_cast<uint16_t>(steer_valve_current_abs);
                } else if (steer_error < -3.0) {
                    hydraulicCtrl.steer_valve_dir = 2;  // 右转
                    hydraulicCtrl.steer_valve_current = static_cast<uint16_t>(steer_valve_current_abs);
                } else {
                    hydraulicCtrl.steer_valve_dir = 0;  // 不转向
                    hydraulicCtrl.steer_valve_current = 0;
                }
            }

            
        }
        else{
            hydraulicCtrl.steer_valve_dir = 0;
            hydraulicCtrl.steer_valve_current = 0;
        }
        if(sequence_cnt % log_output_interval_ == 0) {
            LOG_I("current_steer_angle_: {:.2f} 度 (范围: -40~40)", current_steer_angle_);
            LOG_I("target_steer_angle: {:.2f} 度 (范围: -40~40)", target_steer_angle);
            LOG_I("angle_error: {:.2f} 度", target_steer_angle - current_steer_angle_);
            LOG_I("steer_valve_dir: {} (0=不转向，1=左转，2=右转)", static_cast<uint16_t>(hydraulicCtrl.steer_valve_dir));
            LOG_I("steer_valve_current: {} mA (范围: 0~800)", static_cast<uint16_t>(hydraulicCtrl.steer_valve_current));
        }

    }
    
    memcpy(canFrame.data, &hydraulicCtrl, sizeof(CA_HydraulicCtrl));
    
    // 发布 CAN 帧信息（如果回调已设置）
    if (can_frame_send_callback_) {
      can_frame_send_callback_(canFrame.can_id & CAN_EFF_MASK, canFrame.can_dlc, canFrame.data);
    }
    
    SendCanFrame(canFrame);
}

// 发送 0x18000003 行走控制（速度闭环→扭矩/速度请求与方向）
// 调用处：SendCmdFunc()
void ChanganCANParser::sendWalkingControl(double dt) {
    CA_WalkingCtrl walkingCtrl = {0};
    can_frame canFrame;
    
    canFrame.can_id = CA_WALKING_CTRL;
    canFrame.can_dlc = 8;
    
    {
        std::lock_guard<std::mutex> lk(vehicleCtlCmdMutex);
        // 行走电机使能
        walkingCtrl.drive_motor_enable = 1;
        
        // 行走电机工作模式:  1=转速模式, 2=扭矩模式（按头文件注释统一）
        walkingCtrl.drive_work_mode = 2;  // 扭矩模式，后续按需求切换
        
        // 行走电机经济模式
        walkingCtrl.drive_econ_mode = 0;  // 标准模式
        
        // 档位一致性检查：只有当发送的档位与接收的档位一致时，才下发有效的速度、扭矩等控制参数
        bool gear_match = false;

        uint8_t gear_cmd = 0;  // 默认空档
        // 从控制命令获取目标档位
        if (ctl_cmd.has_gear_location()) {
            switch (ctl_cmd.gear_location()) {
                case control::canbus::Chassis::GEAR_INVALID:
                    gear_cmd = 0;
                    break;
                case control::canbus::Chassis::GEAR_DRIVE:
                    gear_cmd = 1;
                    break;
                case control::canbus::Chassis::GEAR_NEUTRAL:
                    gear_cmd = 0;
                    break;
                case control::canbus::Chassis::GEAR_REVERSE:
                    gear_cmd = 2;
                    break;
                default:
                    gear_cmd = 0;
                    break;
            }
        }
        // 根据档位设置行走方向 (0=空档,1=前进,2=空档,3=后退)

        gear_match = (gear_cmd == current_gear_);
        static int gear_match_cnt = 0;
        // 检查档位是否匹配
        
        // 从control_cmd获取目标速度
        // 与上层控制器统一，上层控制器通过Throttle消息类型发出扭矩请求，范围是0-3000，单位Nm。因最大值过大，目前限定在500Nm以内。
        int16_t torque_cmd = 0; 

        // 档位匹配计数器，只有档位匹配时才计数，不匹配时清零。
        if (gear_match)
        {
            gear_match_cnt++;
        }
        else
        {
            gear_match_cnt = 0;
        }


        // 档位匹配计数器大于200时，才设置扭矩。用于等待档位齿合。
        if (gear_match_cnt >= 200) {
            // 只有在档位匹配时才设置扭矩
             torque_cmd = static_cast<int16_t>(ctl_cmd.throttle() * 12.0);  // Nm
            //torque_cmd = static_cast<int16_t>(ctl_cmd.throttle());  // Nm
            // torque_cmd = 50 + static_cast<int16_t>(ctl_cmd.throttle()*10.0);  // Nm
            if (torque_cmd > 200) torque_cmd = 200;
            if (torque_cmd < -100) torque_cmd = -100;
            gear_match_cnt = 200;
        }
        
        // 设置控制参数（只赋值一次）
        walkingCtrl.drive_req_torque = static_cast<uint16_t>(torque_cmd + 3000);  // 数据下发增加一个3000的offset
        walkingCtrl.drive_direction = gear_cmd;

        // walkingCtrl.drive_direction = 1;
        // walkingCtrl.drive_req_torque = 3150;
        // 设置行走电机转速（范围-15000~15000）,有15000的offset，无作用，加速度由扭矩控制。
        walkingCtrl.drive_req_speed = static_cast<uint16_t>(15000);
        
        // 刹车控制（如果有驻车制动请求）
        if (ctl_cmd.brake()) {
            walkingCtrl.brake_valve_current = static_cast<uint16_t>(ctl_cmd.brake() * 8.0);  // 刹车电流 (400-1600mA)
            if (walkingCtrl.brake_valve_current > 400) walkingCtrl.brake_valve_current = 400;
            // if (walkingCtrl.brake_valve_current < 400) walkingCtrl.brake_valve_current = 400;
        } else {
            walkingCtrl.brake_valve_current = 0;   // 最小刹车电流
        }
        
        
        if(sequence_cnt % log_output_interval_ == 0) {
            LOG_I("gear_cmd: {} (0=空档,1=前进,2=后退)", static_cast<int>(gear_cmd));
            LOG_I("current_gear_: {} (0=空档,1=前进,2=后退)", static_cast<int>(current_gear_));
            LOG_I("gear_match: {} (0=不匹配,1=匹配)", static_cast<int>(gear_match));
            LOG_I("ctl_cmd.throttle(): {:.1f} (原始值)", static_cast<double>(ctl_cmd.throttle()));
            LOG_I("torque_cmd: {} Nm (范围: -100~200)", static_cast<int>(torque_cmd));
            LOG_I("walkingCtrl.drive_req_torque: {:.2f} (转矩请求，带3000偏移)", static_cast<double>(walkingCtrl.drive_req_torque));
            LOG_I("walkingCtrl.drive_direction: {} (0=空档,1=前进,2=后退)", static_cast<int>(walkingCtrl.drive_direction));
            LOG_I("current_speed_: {:.2f} m/s", static_cast<double>(current_speed_));
            LOG_I("ctl_cmd.brake(): {:.1f} % (范围: 0~100)", static_cast<double>(ctl_cmd.brake()));
            LOG_I("walkingCtrl.brake_valve_current: {} mA (范围: 0~300)", static_cast<int>(walkingCtrl.brake_valve_current));
        }
    }
    
    memcpy(canFrame.data, &walkingCtrl, sizeof(CA_WalkingCtrl));
    
    // 发布 CAN 帧信息（如果回调已设置）
    if (can_frame_send_callback_) {
      can_frame_send_callback_(canFrame.can_id & CAN_EFF_MASK, canFrame.can_dlc, canFrame.data);
    }
    
    SendCanFrame(canFrame);
}

// 发送 0x18000004 执行机构控制（大臂/铲斗角度闭环输出电流择向）
// 调用处：SendCmdFunc()
void ChanganCANParser::sendActuatorControl(double dt) {
    CA_ActuatorCtrl actuatorCtrl = {0};
    can_frame canFrame;
    
    canFrame.can_id = CA_ACTUATOR_CTRL;
    canFrame.can_dlc = 8;
    
    {
        std::lock_guard<std::mutex> lk(vehicleCtlCmdMutex);
        // dt 由 SendCmdFunc 实时计算传入
        
        // 大臂控制 - 使用角度控制器
        double target_arm_angle = ctl_cmd.arm_angle();  // 从control_cmd获取目标大臂角度
        if(target_arm_angle < 0.0) target_arm_angle = 0.0;
        if(target_arm_angle > 60.0) target_arm_angle = 60.0;
        double arm_valve_current = 0.0;
 
        // 检查大臂使能标志
        if (ctl_cmd.arm_enable()) {
            // target_arm_angle = 1.0;
            arm_valve_current = arm_controller_.compute(target_arm_angle, current_arm_angle_, dt);
            double arm_valve_current_abs = std::fabs(arm_valve_current);
            if (arm_valve_current_abs > 800.0) arm_valve_current_abs = 800.0;


            // 根据误差方向确定大臂动作
            if (arm_valve_current > 5) {  // 需要抬起
                actuatorCtrl.arm_up_current = static_cast<uint16_t>(arm_valve_current_abs) + 200;
                actuatorCtrl.arm_down_current = 0;
            } else if (arm_valve_current < -5) {  // 需要下降
                actuatorCtrl.arm_up_current = 0;
                actuatorCtrl.arm_down_current = static_cast<uint16_t>(arm_valve_current_abs) + 50;
            } else {  // 在死区内，不动作
                actuatorCtrl.arm_up_current = 0;
                actuatorCtrl.arm_down_current = 0;
            }
        } else {
            // 大臂未使能，停止所有动作
            actuatorCtrl.arm_up_current = 0;
            actuatorCtrl.arm_down_current = 0;
        }
        
        if(sequence_cnt % log_output_interval_ == 0) {
            LOG_I("current_arm_angle_: {:.2f} 度 (范围: 0~60)", current_arm_angle_);
            LOG_I("target_arm_angle: {:.2f} 度 (范围: 0~60)", target_arm_angle);
            LOG_I("arm_angle_error: {:.2f} 度", target_arm_angle - current_arm_angle_);
            LOG_I("arm_valve_current: {:.2f} mA (范围: 0~800)", arm_valve_current);
            LOG_I("actuatorCtrl.arm_up_current: {} mA (大臂举升，增加200mA偏移)", static_cast<int>(actuatorCtrl.arm_up_current));
            LOG_I("actuatorCtrl.arm_down_current: {} mA (大臂下降，增加50mA偏移)", static_cast<int>(actuatorCtrl.arm_down_current));
        }


        // 铲斗控制 - 使用角度控制器
        double target_bucket_angle = ctl_cmd.shovel_angle();  // 从control_cmd获取目标铲斗角度
        if(target_bucket_angle < -90.0) target_bucket_angle = -90.0;
        if(target_bucket_angle > 40.0) target_bucket_angle = 40.0;
        double bucket_valve_current = 0.0;

        // 检查铲斗使能标志
        if (ctl_cmd.shovel_enable()) {
            // target_bucket_angle = 1.0;
            bucket_valve_current = bucket_controller_.compute(target_bucket_angle, current_bucket_angle_, dt);

            double bucket_valve_current_abs = std::fabs(bucket_valve_current);
            if (bucket_valve_current_abs > 800.0) bucket_valve_current_abs = 800.0;
            
            // 根据误差方向确定铲斗动作
            if (bucket_valve_current < -5) {  // 需要外翻
                actuatorCtrl.bucket_out_current = static_cast<uint16_t>(bucket_valve_current_abs) + 50;
                actuatorCtrl.bucket_in_current = 0;
            } else if (bucket_valve_current > 5) {  // 需要内收
                actuatorCtrl.bucket_out_current = 0;
                actuatorCtrl.bucket_in_current = static_cast<uint16_t>(bucket_valve_current_abs) + 200;
            } else {  // 在死区内，不动作
                actuatorCtrl.bucket_out_current = 0;
                actuatorCtrl.bucket_in_current = 0;
            }
        } else {
            // 铲斗未使能，停止所有动作
            actuatorCtrl.bucket_out_current = 0;
            actuatorCtrl.bucket_in_current = 0;
        }

        if(sequence_cnt % log_output_interval_ == 0) {
            LOG_I("current_bucket_angle_: {:.2f} 度 (范围: -90~40)", current_bucket_angle_);
            LOG_I("target_bucket_angle: {:.2f} 度 (范围: -90~40)", target_bucket_angle);
            LOG_I("bucket_angle_error: {:.2f} 度", target_bucket_angle - current_bucket_angle_);
            LOG_I("bucket_valve_current: {:.2f} mA (范围: 0~800)", bucket_valve_current);
            LOG_I("actuatorCtrl.bucket_in_current: {} mA (铲斗内收，增加200mA偏移)", static_cast<int>(actuatorCtrl.bucket_in_current));
            LOG_I("actuatorCtrl.bucket_out_current: {} mA (铲斗外翻，增加50mA偏移)", static_cast<int>(actuatorCtrl.bucket_out_current));

        }


        // [YS][LJS] 直接设定bucket_out_current和bucket_in_current进行测试。
        // actuatorCtrl.bucket_out_current = 350;
        // actuatorCtrl.bucket_in_current = 0;
        // actuatorCtrl.bucket_out_current = 0;
        // actuatorCtrl.bucket_in_current = 400;

        // [YS][LJS] 直接设定arm_up_current和arm_down_current进行测试。
        // actuatorCtrl.arm_up_current = 0;
        // actuatorCtrl.arm_down_current = 0;
        // actuatorCtrl.arm_up_current = 0;
        // actuatorCtrl.arm_down_current = 350;
    }
    
    memcpy(canFrame.data, &actuatorCtrl, sizeof(CA_ActuatorCtrl));
    
    // 发布 CAN 帧信息（如果回调已设置）
    if (can_frame_send_callback_) {
      can_frame_send_callback_(canFrame.can_id & CAN_EFF_MASK, canFrame.can_dlc, canFrame.data);
    }
    
    SendCanFrame(canFrame);
}

// ================================
// 接收消息处理函数
// ================================

// 解析 0x181F0001 状态反馈，更新驾驶模式/电量/故障位
// 调用处：CAN 接收线程分发
void ChanganCANParser::handle0x181F0001(struct Canframe *recvCanFrame) {
    // 状态反馈消息
    if (recvCanFrame->frame.can_dlc != 8) {
        // std::cerr << "Unexpected CAN frame DLC for 0x181F0001: " 
        //           << static_cast<int>(recvCanFrame->frame.can_dlc) << std::endl;
        return;
    }
    
    VCU_heartbeat_cnt = 0;  // 重置VCU心跳计数
    
    union ChanganCanFrameMsg canFrameUnion = {{0}};
    std::memmove(canFrameUnion.canFrameData, recvCanFrame->frame.data, CAN_DLEN);
    
    {
        std::lock_guard<std::mutex> lk(vehicleStatusMutex);
        
        // 更新上高压状态（使用 engine_started 字段表示上高压状态）
        chassisProtoMsg.set_engine_started(canFrameUnion.status_fb_01.high_voltage_sts != 0);
        
        // 更新驾驶模式
        if (canFrameUnion.status_fb_01.vehicle_mode_sts) {
            chassisProtoMsg.set_driving_mode(control::canbus::Chassis::COMPLETE_AUTO_DRIVE);
        } else {
            chassisProtoMsg.set_driving_mode(control::canbus::Chassis::COMPLETE_MANUAL);
        }
        
        // 更新电池电量 (分辨率0.4)
        double battery_soc = canFrameUnion.status_fb_01.battery_soc * 0.4;
        chassisProtoMsg.set_battery_soc_percentage(static_cast<int32_t>(battery_soc));
        
        // 更新驻车制动状态
        chassisProtoMsg.set_parking_brake(canFrameUnion.status_fb_01.parking_brake != 0);
        
        // 更新故障信息
        // 可以根据需要处理各种故障位
    }
}

// 解析 0x181F0002 电机状态，更新档位/液压电机状态
// 调用处：CAN 接收线程分发
void ChanganCANParser::handle0x181F0002(struct Canframe *recvCanFrame) {
    // 电机状态消息
    // std::cout << "[DEBUG] handle0x181F0002 called, CAN ID: 0x" << std::hex << recvCanFrame->frame.can_id << std::dec 
    //           << ", DLC: " << static_cast<int>(recvCanFrame->frame.can_dlc) << std::endl;
    
    if (recvCanFrame->frame.can_dlc != 8) {
        // std::cout << "[ERROR] Unexpected DLC: " << static_cast<int>(recvCanFrame->frame.can_dlc) << std::endl;
        return;
    }
    
    union ChanganCanFrameMsg canFrameUnion = {{0}};
    std::memmove(canFrameUnion.canFrameData, recvCanFrame->frame.data, CAN_DLEN);
    
    
    {
        std::lock_guard<std::mutex> lk(vehicleStatusMutex);
        
        // 液压电机状态
        // 可以记录液压电机的实际转速、扭矩等信息
        
        // 更新档位信息
        uint8_t gear = canFrameUnion.motor_status_02.drive_gear_info;
        
        current_gear_ = gear;  // 保存当前档位状态，用于速度正负号判断.底层反馈为0空档,1前进,2后退
        control::canbus::Chassis::GearPosition gear_pos;
        switch(gear) {
            case 0:
                gear_pos = control::canbus::Chassis::GEAR_NEUTRAL;
                break;
            case 1:
                gear_pos = control::canbus::Chassis::GEAR_DRIVE;
                break;
            case 2:
                gear_pos = control::canbus::Chassis::GEAR_REVERSE;
                break;
            default:
                gear_pos = control::canbus::Chassis::GEAR_INVALID;
                break;
        }
        chassisProtoMsg.set_gear_location(gear_pos);
    }
}

// 解析 0x181F0003 行走状态，更新速度反馈（km/h→m/s）
// 调用处：CAN 接收线程分发
void ChanganCANParser::handle0x181F0003(struct Canframe *recvCanFrame) {
    // 行走电机状态消息
    if (recvCanFrame->frame.can_dlc != 8) {
        return;
    }
    
    union ChanganCanFrameMsg canFrameUnion = {{0}};
    std::memmove(canFrameUnion.canFrameData, recvCanFrame->frame.data, CAN_DLEN);
    
    {
        std::lock_guard<std::mutex> lk(vehicleStatusMutex);
        
        // 更新车速 (从轮速计算，分辨率0.16, 单位可能是km/h，需要转换为m/s)
        double wheel_speed_kmh = canFrameUnion.walking_status.wheel_speed * 0.16;
        // std::cout << "wheel_speed_kmh: " << std::fixed << std::setprecision(2) << wheel_speed_kmh << std::endl;
        current_speed_ = wheel_speed_kmh / 3.6;  // 转换为m/s
        
        // 根据当前档位确定速度正负号
        // 注意：这里需要从档位状态来判断方向，因为轮速传感器通常只提供绝对值
        // 如果车辆有方向传感器，应该优先使用方向传感器的信息
        // if (current_gear_ == 2) {  // 后退档
        //     current_speed_ = -current_speed_;  // 后退时速度为负
        // }
        // 前进档(1)和空档(0)时速度保持正值
        
        chassisProtoMsg.set_speed_mps(current_speed_);
        
        // 更新行走电机扭矩等信息（如果需要）
    }
}

// 解析 0x181F0007 丹佛斯转向；编码器缺失时可作为兜底角度
// 调用处：CAN 接收线程分发
void ChanganCANParser::handle0x181F0007(struct Canframe *recvCanFrame) {
    if (recvCanFrame->frame.can_dlc != 8) {
        return;
    }

    // 丹佛斯转向消息
    union ChanganCanFrameMsg canFrameUnion = {{0}};
    std::memmove(canFrameUnion.canFrameData, recvCanFrame->frame.data, CAN_DLEN);
    
    {
        // 保存danfoss_status到全局变量（直接赋值）
        std::lock_guard<std::mutex> lk(vehicleStatusMutex);
        danfoss_status_ = canFrameUnion.danfoss_steering.danfoss_status;
    }

}

// ================================
// 倾角仪与角度编码器解析
// ================================
// 尝试从8字节数据解析X轴横滚角（度）
// 设备为小端解析 data[1]+data[0]
// 4) 兜底返回0.0
static double parseXRollDegFromSenseFrame(const uint8_t data[8]) {
    // 尝试 int16 小端缩放0.01 (bytes 0..1)
    int16_t v_be = static_cast<int16_t>(static_cast<uint16_t>(data[1]) << 8 |
                                        static_cast<uint16_t>(data[0]));
    double deg_be = static_cast<double>(v_be) * 0.01;
    if (std::fabs(deg_be) <= 360.0) {
        return deg_be;
    }
    return 0.0;
}

// 依据原始倾角/编码器值，计算相对角并写入底盘状态
// [YS][LJS] 等北航装完新的线束之后，测试角度解析是否正确，以及设定零点offset。
void ChanganCANParser::updateRelativeAnglesAndChassis() {
    // 计算大臂到车身的相对角，加入零点标定
    // 0位置-铲斗贴地水平，0x581铲斗为-88度，0x582大臂为-144度，0x583车身为-3度。
    // 大臂对车身为-141度。
    angle_sensors_.boom_rel_body_deg =
        (angle_sensors_.boom_roll_x_deg - angle_sensors_.body_roll_x_deg) +
        angle_sensors_.boom_zero_offset_deg;
    // 铲斗的零位置
    angle_sensors_.bucket_rel_boom_deg =
        (angle_sensors_.bucket_roll_x_deg - angle_sensors_.boom_roll_x_deg) +
        angle_sensors_.bucket_zero_offset_deg;

    // 更新内部当前角度
    current_arm_angle_ = - angle_sensors_.boom_rel_body_deg;
    current_bucket_angle_ = - angle_sensors_.bucket_rel_boom_deg;

    // 将角度写入 chassisProtoMsg
    chassisProtoMsg.set_arm_angle(static_cast<float>(current_arm_angle_));
    chassisProtoMsg.set_shovel_angle(static_cast<float>(current_bucket_angle_));

    // 转向角（已注释：现在使用GNSS diff_age替代编码器数据）
    if (steer_encoder_available_) {
        current_steer_angle_ = angle_sensors_.steering_angle_deg + angle_sensors_.steer_zero_offset_deg;
        chassisProtoMsg.set_steering_percentage(static_cast<float>(current_steer_angle_));
    }

    // 通过标准输出打印角度（相对/绝对）
    // std::cout << "Angle Sensors: "
    //     << ", current_arm_angle_=" << std::fixed << std::setprecision(2) << current_arm_angle_
    //     << ", current_bucket_angle_=" << std::fixed << std::setprecision(2) << current_bucket_angle_ 
    //     << ", current_steer_angle_=" << std::fixed << std::setprecision(2) << current_steer_angle_ 
    //     << ", body_roll_x_deg=" << std::fixed << std::setprecision(2) << angle_sensors_.body_roll_x_deg
    //     << ", boom_roll_x_deg=" << std::fixed << std::setprecision(2) << angle_sensors_.boom_roll_x_deg
    //     << ", bucket_roll_x_deg=" << std::fixed << std::setprecision(2) << angle_sensors_.bucket_roll_x_deg
    //     << std::endl;
}

// 解析 0x00000581 铲斗倾角（X_roll）
void ChanganCANParser::handle0x00000581(struct Canframe *recvCanFrame) {
    if (recvCanFrame->frame.can_dlc != 8) { return; }
    {
        std::lock_guard<std::mutex> lk(vehicleStatusMutex);
        angle_sensors_.bucket_roll_x_deg = parseXRollDegFromSenseFrame(recvCanFrame->frame.data);
        updateRelativeAnglesAndChassis();
    }
}

// 解析 0x00000582 大臂倾角（X_roll）
void ChanganCANParser::handle0x00000582(struct Canframe *recvCanFrame) {
    if (recvCanFrame->frame.can_dlc != 8) { return; }
    {
        std::lock_guard<std::mutex> lk(vehicleStatusMutex);
        double raw_angle = parseXRollDegFromSenseFrame(recvCanFrame->frame.data);
        if(raw_angle > 0.0) {raw_angle = raw_angle - 360.0;}
        angle_sensors_.boom_roll_x_deg = raw_angle;
        updateRelativeAnglesAndChassis();
    }
}

// 解析 0x00000583 车身倾角（X_roll）
void ChanganCANParser::handle0x00000583(struct Canframe *recvCanFrame) {
    if (recvCanFrame->frame.can_dlc != 8) { return; }
    {
        std::lock_guard<std::mutex> lk(vehicleStatusMutex);
        angle_sensors_.body_roll_x_deg = parseXRollDegFromSenseFrame(recvCanFrame->frame.data);
        updateRelativeAnglesAndChassis();
    }
}

// 解析 0x18FF0015 转向角编码器（byte1..2/100.0 度）
void ChanganCANParser::handle0x18FF0015(struct Canframe *recvCanFrame) {
    if (recvCanFrame->frame.can_dlc != 8) { return; }
    {
        std::lock_guard<std::mutex> lk(vehicleStatusMutex);
        // byte[1..2] 拼接为角度*100（假定大端字节序），单位：度
        uint16_t raw = static_cast<uint16_t>((recvCanFrame->frame.data[1] << 8) |
                                             recvCanFrame->frame.data[2]);
        

        // 角度编码器，向左打方向，角度从0增加至45左右，向右打角度从359下降至325左右。
        double deg = static_cast<double>(raw) / 100.0;
        if (deg > 180.0) {    deg = deg - 360.0; }

        angle_sensors_.steering_angle_deg = deg;
        // std::cout << "角度编码器数据deg: " << std::fixed << std::setprecision(2) << angle_sensors_.steering_angle_deg << std::endl;
        steer_encoder_available_ = true;
        updateRelativeAnglesAndChassis();
    }
}



// 静态断言校验各结构体大小为 8 字节，保证与 DBC 对齐
void ChanganCANParser::checkDataStructure() {
    // 确保结构体大小正确
    static_assert(sizeof(CA_ChassisCtrl) == 8, "Invalid CA_ChassisCtrl size");
    static_assert(sizeof(CA_HydraulicCtrl) == 8, "Invalid CA_HydraulicCtrl size");
    static_assert(sizeof(CA_WalkingCtrl) == 8, "Invalid CA_WalkingCtrl size");
    static_assert(sizeof(CA_ActuatorCtrl) == 8, "Invalid CA_ActuatorCtrl size");
    static_assert(sizeof(CA_StatusFB01) == 8, "Invalid CA_StatusFB01 size");
    static_assert(sizeof(CA_MotorStatus02) == 8, "Invalid CA_MotorStatus02 size");
    static_assert(sizeof(CA_WalkingStatus) == 8, "Invalid CA_WalkingStatus size");
    static_assert(sizeof(CA_DanfossSteering) == 8, "Invalid CA_DanfossSteering size");
    
    // std::cout << "[ChanganCANParser] 数据结构检查通过，所有结构体大小为8字节" << std::endl;
}


} // namespace cannode
