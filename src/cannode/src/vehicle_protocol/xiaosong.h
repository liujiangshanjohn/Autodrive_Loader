#pragma once

#include <atomic>
#include <cstdint>
#include <linux/can.h>
#include <chrono>
#include "../chassis_driver.h"
// #include "../low_level_controller/acc_to_speed_controller.h"  // 新协议中不使用，暂时注释
// #include "../low_level_controller/valve_to_angle_controller.h"  // 所有控制改为直接电流控制，不再需要角度控制器
// #include "../low_level_controller/pid_config_manager.h"  // 所有控制改为直接电流控制，不再需要PID配置管理器

namespace cannode {

// CAN消息ID定义（根据excavator协议）
enum XiaosongCanProtoId {
    // 上位机→VCU 控制命令
    XS_ACTUATOR_CTRL_1  = 0x18210001,  // 机构电磁阀控制消息1（动臂下降、动臂抬升、斗杆伸出、斗杆收回）
    XS_ACTUATOR_CTRL_2  = 0x18210002,  // 机构电磁阀控制消息2（铲斗翻斗、铲斗收斗、回转左、回转右）
    XS_WALKING_CTRL     = 0x18210003,  // 行走电机控制消息（左前进、左后退、右前进、右后退）
    XS_RESERVE_CTRL     = 0x18210004,  // 备用比例阀控制（备用比例阀1-4）
    XS_SWITCH_CTRL      = 0x18210005,  // 开关量映射控制（26个开关量）
    XS_ENGINE_THROTTLE_CTRL = 0x18FD0821, // 发动机油门控制器 gear 报文 (起始位48, 长度8bit, 0.05V/bit；1~7档->4~1V)
    
    // VCU→上位机 状态反馈
    XS_STATUS_FB_01     = 0x18200001,  // 状态反馈消息1
    XS_MOTOR_STATUS_02  = 0x18200002,  // 电机状态消息2
    XS_WALKING_STATUS_L = 0x18200003,  // 左行走电机状态
    XS_WALKING_STATUS_R = 0x18200004,  // 右行走电机状态
    XS_PRESSURE_01      = 0x18200005,  // 压力信息1（大臂/斗杆）
    XS_PRESSURE_02      = 0x18200006,  // 压力信息2（铲斗/回转）
    XS_ANGLE_INFO       = 0x18200007,  // 角度信息
    XS_VALVE_CURRENT_1  = 0x18200008,  // 阀电流反馈1
    XS_VALVE_CURRENT_2  = 0x18200009,  // 阀电流反馈2
    XS_INPUT_STATUS     = 0x1820000A,  // 输入状态反馈
    XS_VCU_FEEDBACK     = 0x18FF1002   // VCU反馈消息（开关量输入和状态）
};

#pragma pack(push, 1)

// ================================
// 上位机→VCU 控制命令定义
// ================================

// 机构电磁阀控制消息1 (0x18210001) - 动臂/斗杆
typedef struct XiaosongActuatorControl1 {
    // byte 0-1 (16位动臂下降电磁阀)
    uint16_t arm_down_current;          // 动臂下降电磁阀电流 (0~700mA) 死区约250mA
    
    // byte 2-3 (16位动臂抬升电磁阀)
    uint16_t arm_up_current;            // 动臂抬升电磁阀电流 (0~700mA) 死区约250mA
    
    // byte 4-5 (16位斗杆伸出电磁阀)
    uint16_t stick_extend_current;       // 斗杆伸出电磁阀电流 (0~700mA) 死区约250mA
    
    // byte 6-7 (16位斗杆收回电磁阀)
    uint16_t stick_retract_current;      // 斗杆收回电磁阀电流 (0~700mA) 死区约250mA
} XS_ActuatorCtrl1;

// 机构电磁阀控制消息2 (0x18210002) - 铲斗/回转
typedef struct XiaosongActuatorControl2 {
    // byte 0-1 (16位铲斗翻斗电磁阀)
    uint16_t bucket_dump_current;       // 铲斗翻斗电磁阀电流 (0~700mA) 死区约250mA
    
    // byte 2-3 (16位铲斗收斗电磁阀)
    uint16_t bucket_close_current;      // 铲斗收斗电磁阀电流 (0~700mA) 死区约250mA
    
    // byte 4-5 (16位回转左电磁阀)
    uint16_t rotate_left_current;       // 回转左电磁阀电流 (0~700mA)
    
    // byte 6-7 (16位回转右电磁阀)
    uint16_t rotate_right_current;      // 回转右电磁阀电流 (0~700mA)
} XS_ActuatorCtrl2;

// 行走电机控制消息 (0x18210003) - 左前进、左后退、右前进、右后退
typedef struct XiaosongWalkingControl {
    // byte 0-1 (16位左前进电磁阀)
    uint16_t left_forward_current;      // 左前进电磁阀电流 (0~700mA) 死区约250mA
    
    // byte 2-3 (16位左后退电磁阀)
    uint16_t left_backward_current;     // 左后退电磁阀电流 (0~700mA) 死区约250mA
    
    // byte 4-5 (16位右前进电磁阀)
    uint16_t right_forward_current;     // 右前进电磁阀电流 (0~700mA) 死区约250mA
    
    // byte 6-7 (16位右后退电磁阀)
    uint16_t right_backward_current;    // 右后退电磁阀电流 (0~700mA) 死区约250mA
} XS_WalkingCtrl;

// 备用比例阀控制消息 (0x18210004)
typedef struct XiaosongReserveControl {
    // byte 0-1 (16位备用比例阀1)
    uint16_t reserve_valve1;            // 备用比例阀1
    
    // byte 2-3 (16位备用比例阀2)
    uint16_t reserve_valve2;            // 备用比例阀2
    
    // byte 4-5 (16位备用比例阀3)
    uint16_t reserve_valve3;            // 备用比例阀3
    
    // byte 6-7 (16位备用比例阀4)
    uint16_t reserve_valve4;            // 备用比例阀4
} XS_ReserveCtrl;

// 开关量映射控制消息 (0x18210005) - 根据 excavator-can-host2vcu_utf8.csv
typedef struct XiaosongSwitchControl {
    // byte 0
    uint8_t switch_21 : 1;              // bit 0: 车梯收起电磁阀, DOH_OUT17_125(125#线)
    uint8_t switch_22 : 1;              // bit 1: 车梯放下电磁阀, DOH_OUT18_129(129#线)
    uint8_t switch_23 : 1;              // bit 2: 车梯防落电磁阀
    uint8_t switch_24 : 1;              // bit 3: 先导安全锁定电磁阀, OUT_24_DOH(103#线)
    uint8_t switch_25 : 1;              // bit 4: 机器上推电磁阀
    uint8_t switch_26 : 1;              // bit 5: 直线行走电磁阀
    uint8_t switch_27 : 1;              // bit 6: 回转优先电磁阀
    uint8_t switch_28 : 1;              // bit 7: 回转停车制动电磁阀
    
    // byte 1
    uint8_t switch_29 : 1;              // bit 8: 动臂卸载电磁阀
    uint8_t switch_30 : 1;              // bit 9: 开斗油缸电磁阀, OUT_26_DOH(193#线)
    uint8_t switch_31 : 1;              // bit 10: 关斗油缸电磁阀, OUT_37_DOL(183#线)
    uint8_t switch_19 : 1;              // bit 11: 喇叭, OUT_28_DOH(241#线)
    uint8_t switch_17 : 1;              // bit 12: 灯光1, OUT_19_DOH(128#线)
    uint8_t switch_18 : 1;              // bit 13: 灯光2, OUT_29_DOH(242#线)
    uint8_t switch_32 : 1;              // bit 14: 备用1
    uint8_t switch_20 : 1;              // bit 15: 备用2
    
    // byte 2
    uint8_t switch_38 : 1;              // bit 16: OUT_38_DOL(184#线)
    uint8_t switch_39 : 1;              // bit 17: OUT_39_DOL(185#线)
    uint8_t switch_40 : 1;              // bit 18: OUT_40_DOL(186#线)
    uint8_t switch_42 : 1;              // bit 19: OUT_42_DOL(180#线)
    uint8_t switch_43 : 1;              // bit 20: OUT_43_DOL(181#线)
    uint8_t switch_44 : 1;              // bit 21: OUT_44_DOL(182#线)
    uint8_t switch_45 : 1;              // bit 22: OUT_45_DOL(188#线)
    uint8_t switch_46 : 1;              // bit 23: OUT_46_DOL(187#线)
    
    // byte 3
    uint8_t switch_37 : 1;              // bit 24: OUT_37_DOL(183#线)
    uint8_t switch_41 : 1;              // bit 25: OUT_41_DOL(179#线)
    uint8_t reserved : 6;               // bit 26-31: 预留
    
    // byte 4: 扩展开关低 8 位（CAN bit32~39）
    // byte 5-6: 发动机转速 0~8000 rpm，小端 uint16（CAN 起始位 40，长度 16），见 excavator-can-host2vcu_utf8.csv
    // byte 7: 与 switch_ext_bits 高字节对齐（扩展位仅用低 5 bit 时常为 0）
    // 以下成员与 byte4~7 内存布局等价于：byte4..7 = uint32_le(switch_ext_bits)，发送前 byte5~6 会被覆盖为转速
    // bit 32: 远程申请进入遥控操作 (1是遥控驾驶有效)
    // bit 33: 远程申请进入急停状态 (1是急停有效)
    // bit 34: 上电 (1是有效)
    // bit 35: 发动机启动 (1是有效)
    // bit 36: ACC 指令(DOH_OUT25_194) (1是有效)
    uint32_t switch_ext_bits;
} XS_SwitchCtrl;

// ================================
// VCU→上位机 状态反馈定义
// ================================

// 状态反馈消息1 (0x18200001)
typedef struct XiaosongStatusFeedback01 {
    // byte 0
    uint8_t high_voltage_sts : 1;       // bit 0: 上高压状态
    uint8_t parking_brake : 1;          // bit 1: 停车制动状态
    uint8_t soft_estop_sts : 1;        // bit 2: 软急停状态
    uint8_t external_estop : 1;        // bit 3: 外部急停状态
    uint8_t vehicle_mode : 1;           // bit 4: 整车模式
    uint8_t heartbeat_error : 1;       // bit 5: 上位机心跳状态
    uint8_t body_rotation_lock : 1;    // bit 6: 车体回转锁定状态
    uint8_t reserved1 : 1;              // bit 7: 预留
    
    // byte 1
    uint8_t heartbeat : 1;              // bit 8: 心跳信号
    uint8_t low_beam_fb : 1;           // bit 9: 近光灯反馈
    uint8_t high_beam_fb : 1;          // bit 10: 远光灯反馈
    uint8_t work_light_fb : 1;         // bit 11: 工作灯反馈
    uint8_t horn_fb : 1;               // bit 12: 喇叭反馈
    uint8_t seat_belt_sts : 1;        // bit 13: 座椅/安全带状态
    uint8_t left_parking_lock : 1;    // bit 14: 左履带驻车锁状态
    uint8_t right_parking_lock : 1;   // bit 15: 右履带驻车锁状态
    
    // byte 2 (8位设备电量)
    uint8_t battery_soc;               // 设备电量 (分辨率0.4%)
    
    // byte 3
    uint8_t fault_level : 4;           // bit 24-27: 本体故障等级
    uint8_t alarm_flags : 4;           // bit 28-31: 通用告警标志
    
    // byte 4-7 (32位预留)
    uint32_t reserved2;                // 预留
} XS_StatusFB01;

// 电机状态消息2 (0x18200002)
typedef struct XiaosongMotorStatus02 {
    // byte 0-1 (16位液压泵电机实际转速)
    uint16_t hyd_actual_speed;         // 液压泵电机实际转速
    
    // byte 2-3 (16位液压泵电机实际扭矩)
    uint16_t hyd_actual_torque;        // 液压泵电机实际扭矩
    
    // byte 4-5 (16位液压泵电机电流)
    uint16_t hyd_current;              // 液压泵电机电流
    
    // byte 6
    uint8_t hyd_motor_en : 1;          // bit 48: 液压泵电机使能信号
    uint8_t reserved1 : 2;             // bit 49-50: 预留
    uint8_t hyd_work_mode : 2;        // bit 51-52: 液压泵电机工作模式
    uint8_t hyd_econ_mode : 2;         // bit 53-54: 液压泵电机经济模式
    uint8_t reserved2 : 1;             // bit 55: 预留
    
    // byte 7 (8位预留)
    uint8_t reserved3;                 // 预留
} XS_MotorStatus02;

// 左行走电机状态 (0x18200003)
typedef struct XiaosongWalkingStatusL {
    // byte 0-1 (16位左行走电机电流)
    uint16_t left_current;             // 左行走电机电流
    
    // byte 2-3 (16位左行走电机实际扭矩)
    uint16_t left_actual_torque;       // 左行走电机实际扭矩
    
    // byte 4-5 (16位左行走电机转速)
    uint16_t left_speed;               // 左行走电机转速
    
    // byte 6-7 (16位左履带线速度)
    uint16_t left_line_speed;          // 左履带线速度 (分辨率0.01m/s)
} XS_WalkingStatusL;

// 右行走电机状态 (0x18200004)
typedef struct XiaosongWalkingStatusR {
    // byte 0-1 (16位右行走电机电流)
    uint16_t right_current;            // 右行走电机电流
    
    // byte 2-3 (16位右行走电机实际扭矩)
    uint16_t right_actual_torque;      // 右行走电机实际扭矩
    
    // byte 4-5 (16位右行走电机转速)
    uint16_t right_speed;              // 右行走电机转速
    
    // byte 6-7 (16位右履带线速度)
    uint16_t right_line_speed;         // 右履带线速度 (分辨率0.01m/s)
} XS_WalkingStatusR;

// 压力信息1 (0x18200005) - 大臂/斗杆
typedef struct XiaosongPressure01 {
    // byte 0-1 (16位大臂缸大腔压力)
    uint16_t arm_big_pressure;         // 大臂缸大腔压力 (分辨率0.1MPa)
    
    // byte 2-3 (16位大臂缸小腔压力)
    uint16_t arm_small_pressure;       // 大臂缸小腔压力 (分辨率0.1MPa)
    
    // byte 4-5 (16位斗杆缸大腔压力)
    uint16_t stick_big_pressure;       // 斗杆缸大腔压力 (分辨率0.1MPa)
    
    // byte 6-7 (16位斗杆缸小腔压力)
    uint16_t stick_small_pressure;     // 斗杆缸小腔压力 (分辨率0.1MPa)
} XS_Pressure01;

// 压力信息2 (0x18200006) - 铲斗/回转
typedef struct XiaosongPressure02 {
    // byte 0-1 (16位铲斗缸大腔压力)
    uint16_t bucket_big_pressure;      // 铲斗缸大腔压力 (分辨率0.1MPa)
    
    // byte 2-3 (16位铲斗缸小腔压力)
    uint16_t bucket_small_pressure;    // 铲斗缸小腔压力 (分辨率0.1MPa)
    
    // byte 4-5 (16位回转马达高压压力)
    uint16_t rotate_high_pressure;     // 回转马达高压压力 (分辨率0.1MPa)
    
    // byte 6-7 (16位预留)
    uint16_t reserved;                // 预留
} XS_Pressure02;

// 角度信息 (0x18200007)
// 注意：根据协议，此消息周期为50ms，包含多个16位角度字段
// 实际CAN帧只有8字节，所以前4个角度字段在一个帧中
typedef struct XiaosongAngleInfo {
    // byte 0-1 (16位车体回转角度)
    uint16_t body_rotation_angle;     // 车体回转角度 (0~3600对应0~360.0°)
    
    // byte 2-3 (16位大臂角度)
    uint16_t arm_angle;                // 大臂角度 (0~1800对应0~180.0°)
    
    // byte 4-5 (16位斗杆角度)
    uint16_t stick_angle;              // 斗杆角度 (0~1800对应0~180.0°)
    
    // byte 6-7 (16位铲斗角度)
    uint16_t bucket_angle;             // 铲斗角度 (0~1800对应0~180.0°)
    
    // 注意：车体横滚角和俯仰角在协议中位于bit 64-95，超出8字节范围
    // 可能需要另一个CAN ID或下一个周期发送
} XS_AngleInfo;

// 阀电流反馈1 (0x18200008) - 大臂/斗杆
// 注意：反馈消息的字段顺序需要根据实际协议确认，这里假设与控制消息顺序一致
typedef struct XiaosongValveCurrent1 {
    // byte 0-1 (16位大臂降电磁阀实际电流)
    uint16_t arm_down_current;         // 大臂降电磁阀实际电流 (0~700mA)
    
    // byte 2-3 (16位大臂抬电磁阀实际电流)
    uint16_t arm_up_current;           // 大臂抬电磁阀实际电流 (0~700mA)
    
    // byte 4-5 (16位斗杆伸电磁阀实际电流)
    uint16_t stick_extend_current;     // 斗杆伸电磁阀实际电流 (0~700mA)
    
    // byte 6-7 (16位斗杆收电磁阀实际电流)
    uint16_t stick_retract_current;    // 斗杆收电磁阀实际电流 (0~700mA)
} XS_ValveCurrent1;

// 阀电流反馈2 (0x18200009) - 铲斗/回转
// 注意：反馈消息的字段顺序需要根据实际协议确认，这里假设与控制消息顺序一致
typedef struct XiaosongValveCurrent2 {
    // byte 0-1 (16位铲斗翻斗电磁阀实际电流)
    uint16_t bucket_dump_current;      // 铲斗翻斗电磁阀实际电流 (0~700mA)
    
    // byte 2-3 (16位铲斗收斗电磁阀实际电流)
    uint16_t bucket_close_current;     // 铲斗收斗电磁阀实际电流 (0~700mA)
    
    // byte 4-5 (16位回转左电磁阀实际电流)
    uint16_t rotate_left_current;      // 回转左电磁阀实际电流 (0~700mA)
    
    // byte 6-7 (16位回转右电磁阀实际电流)
    uint16_t rotate_right_current;     // 回转右电磁阀实际电流 (0~700mA)
} XS_ValveCurrent2;

// 输入状态反馈 (0x1820000A)
typedef struct XiaosongInputStatus {
    // byte 0
    uint8_t remote_switch : 1;         // bit 0: 远控切换信号状态
    uint8_t estop_input : 1;           // bit 1: 急停输入状态
    uint8_t remote_start : 1;          // bit 2: 远程启动输入状态
    uint8_t ladder_retract_ok : 1;     // bit 3: 车梯收起到位信号
    uint8_t throttle_low : 4;          // bit 4-7: 油门信号低4位
    
    // byte 1-2 (16位油门信号)
    uint16_t throttle_signal;          // bit 8-23: 油门信号 (16bits，实际bit 4-19)
    
    // byte 3
    uint8_t imu_power_fuse : 1;        // bit 24: 惯导供电保险状态
    uint8_t reserved_input1 : 1;       // bit 25: 备用输入1
    uint8_t reserved_input2 : 1;       // bit 26: 备用输入2
    uint8_t reserved_input3 : 1;       // bit 27: 备用输入3
    uint8_t reserved : 4;             // bit 28-31: 预留
    
    // byte 4-7 (32位预留)
    uint32_t reserved2;                // 预留
} XS_InputStatus;

// VCU反馈消息 (0x18FF1002) - VCU→上位机，根据 excavator-can-vcu2host_utf8.csv
typedef struct XiaosongVCUFeedback {
    // byte 0
    uint8_t in_43_di : 1;              // bit 0: 车梯收起到位信号, IN_43_DI(115#)
    uint8_t in_44_di : 1;              // bit 1: 开关量输入, IN_44_DI(140#)
    uint8_t in_45_di : 1;              // bit 2: 开关量输入, IN_45_DI(155#)
    uint8_t in_46_di : 1;              // bit 3: 开关量输入, IN_46_DI(156#)
    uint8_t in_47_di : 1;              // bit 4: 开关量输入, IN_47_DI(172#)
    uint8_t in_48_di : 1;              // bit 5: 开关量输入, IN_48_DI(191#)
    uint8_t in_49_di : 1;              // bit 6: 开关量输入, IN_49_DI(192#)
    uint8_t in_50_di : 1;              // bit 7: 开关量输入, IN_50_DI(196#)
    
    // byte 1
    uint8_t in_51_di : 1;              // bit 8: 开关量输入, IN_51_DI(249#)
    uint8_t reserved_bits : 7;         // bit 9-15: 预留
    
    // byte 2（与 CSV 起始位 16–23 对齐：起始位 N 对应本字节第 N-16 位）
    uint8_t is_remote : 1;               // bit 16 BIT0: is_remote (1是遥控驾驶有效)
    uint8_t is_emergency : 1;            // bit 17 BIT1: is_emergency (1是急停有效)
    uint8_t vcu_domain_comm_timeout : 1; // bit 18 BIT2: VCU和域控通讯超时 (1是超时)
    uint8_t hydraulic_lock_status : 1;   // bit 19 BIT3: 液压锁状态 (1为可以动作; 0为不可以动作)
    uint8_t reserved_byte2_hi : 4;       // bit 20–23: 预留
    
    // byte 3-7 (预留)
    uint8_t reserved_byte3;
    uint32_t reserved_byte4_7;
} XS_VCUFeedback;

#pragma pack(pop)

    // CAN消息联合体
union XiaosongCanFrameMsg {
    // 控制命令
    XS_ActuatorCtrl1  actuator_ctrl_1;
    XS_ActuatorCtrl2  actuator_ctrl_2;
    XS_WalkingCtrl    walking_ctrl;
    XS_ReserveCtrl    reserve_ctrl;
    XS_SwitchCtrl     switch_ctrl;
    
    // 状态反馈
    XS_StatusFB01        status_fb_01;
    XS_MotorStatus02     motor_status_02;
    XS_WalkingStatusL    walking_status_l;
    XS_WalkingStatusR    walking_status_r;
    XS_Pressure01        pressure_01;
    XS_Pressure02        pressure_02;
    XS_AngleInfo         angle_info;
    XS_ValveCurrent1    valve_current_1;
    XS_ValveCurrent2    valve_current_2;
    XS_InputStatus       input_status;
    XS_VCUFeedback       vcu_feedback;
    
    uint8_t canFrameData[8];
};

/**
 * @brief 小松车辆 CAN 协议解析器
 *
 * 角色：
 *  - 订阅上位机控制指令（由 MsgManager 注入的 protobuf），周期发送控制报文
 *  - 监听 VCU 状态反馈，更新 chassisProtoMsg 与内部反馈变量
 *  - 维护闭环控制器：车速（加速度→速度）、大臂角、铲斗角
 */
class XiaosongCANParser : public ChassisCan {
public:
    XiaosongCANParser(std::string canPort, const std::string& config_file_path = "");
    ~XiaosongCANParser();
    
    /**
     * @brief 注册 CAN ID→处理函数 的回调映射
     */
    void initFuncMap() override;
    
    /**
     * @brief 周期发送控制与计算 dt 的入口（由底层线程定时调用）
     */
    void SendCmdFunc() override;
    
    // 发送控制命令
    void sendActuatorControl1(double dt);  // 动臂/斗杆 (0x18210001)
    void sendActuatorControl2(double dt);  // 铲斗/回转 (0x18210002)
    void sendWalkingControl(double dt);    // 行走控制 (0x18210003)
    void sendReserveControl();             // 备用比例阀 (0x18210004)
    void sendSwitchControl();              // 开关量映射 (0x18210005)
    void sendEngineThrottleControl();      // 发动机油门电压控制 (0x18FD0821)
    
    // 处理接收消息
    void handle0x18200001(struct Canframe *recvCanFrame);  // 状态反馈
    void handle0x18200002(struct Canframe *recvCanFrame);  // 电机状态
    void handle0x18200003(struct Canframe *recvCanFrame);  // 左行走状态
    void handle0x18200004(struct Canframe *recvCanFrame);  // 右行走状态
    void handle0x18200005(struct Canframe *recvCanFrame);  // 压力信息1
    void handle0x18200006(struct Canframe *recvCanFrame);  // 压力信息2
    void handle0x18200007(struct Canframe *recvCanFrame);  // 角度信息
    void handle0x18200008(struct Canframe *recvCanFrame);  // 阀电流反馈1
    void handle0x18200009(struct Canframe *recvCanFrame);  // 阀电流反馈2
    void handle0x1820000A(struct Canframe *recvCanFrame);  // 输入状态
    void handle0x18FF1002(struct Canframe *recvCanFrame);  // VCU反馈消息
    
    // 获取 VCU 反馈数据接口
    bool get_vcu_is_remote() const { return vcu_is_remote_; }
    bool get_vcu_is_emergency() const { return vcu_is_emergency_; }
    /// VCU 0x18FF1002 bit19：true=允许动作，false=不允许动作（与 CSV 一致）
    bool get_vcu_hydraulic_lock_status() const { return vcu_hydraulic_lock_status_; }
    const uint8_t* get_vcu_digital_inputs() const { return vcu_digital_inputs_; }
    // 检查VCU通讯是否超时（1秒内没有收到反馈数据）
    bool get_vcu_communication_timeout(double timeout_sec = 1.0) const;

    /// 原车 J1939 等解析得到的发动机转速（rpm），填入上位机→VCU 0x18210005 的 byte5~byte6
    void setHostEngineSpeedRpm(uint16_t rpm) override;
    
    /**
     * @brief 静态断言校验各 CAN 结构体大小是否为 8 字节
     */
    void checkDataStructure();
    
    // ======= 外部配置接口 =======
    // 所有控制改为直接电流控制，不再需要PID控制器配置接口
    // void setSpeedPid(double kp, double ki, double kd,
    //                  double max_output, double min_output, double max_integral);
    // void setArmPid(double kp, double ki, double kd,
    //                double max_output, double min_output, double max_integral, double deadzone);
    // void setBucketPid(double kp, double ki, double kd,
    //                   double max_output, double min_output, double max_integral, double deadzone);
    // void setZeroOffsets(double boom_zero_deg, double bucket_zero_deg);
    // bool reloadPIDConfig();

private:
    // PID控制器实例（所有控制改为直接电流控制，不再需要角度控制器）
    // AccToSpeedController speed_controller_;      // 车速控制器（新协议中不使用，暂时注释）
    // ValveToAngleController arm_controller_;      // 大臂角度控制器（改为直接电流控制，暂时注释）
    // ValveToAngleController bucket_controller_;   // 铲斗角度控制器（改为直接电流控制，暂时注释）
    
            // 当前反馈状态（从CAN读取）
            double current_speed_;         // 当前车速 (m/s)
            double current_arm_angle_;     // 当前大臂角度 (度)
            double current_bucket_angle_;   // 当前铲斗角度 (度)
            double current_stick_angle_;    // 当前斗杆角度 (度)
            double current_rotate_angle_;   // 当前回转角度 (度)
            uint8_t current_gear_;         // 当前档位 (0=空档,1=前进,2=后退)
            
            // 电流反馈值（从CAN反馈中读取）
            uint16_t feedback_arm_up_current_;        // 动臂抬升电流反馈 (mA)
            uint16_t feedback_arm_down_current_;      // 动臂下降电流反馈 (mA)
            uint16_t feedback_stick_retract_current_; // 斗杆收回电流反馈 (mA)
            uint16_t feedback_stick_extend_current_; // 斗杆伸出电流反馈 (mA)
            uint16_t feedback_bucket_close_current_; // 铲斗收斗电流反馈 (mA)
            uint16_t feedback_bucket_dump_current_;   // 铲斗翻斗电流反馈 (mA)
            uint16_t feedback_rotate_left_current_;   // 回转左电流反馈 (mA)
            uint16_t feedback_rotate_right_current_;  // 回转右电流反馈 (mA)
            uint16_t feedback_left_forward_current_;  // 左前进电流反馈 (mA)
            uint16_t feedback_left_backward_current_; // 左后退电流反馈 (mA)
            uint16_t feedback_right_forward_current_; // 右前进电流反馈 (mA)
            uint16_t feedback_right_backward_current_; // 右后退电流反馈 (mA)
    
    // VCU反馈状态（0x18FF1002）
    bool vcu_is_remote_;              // 遥控驾驶有效
    bool vcu_is_emergency_;           // 急停有效
    bool vcu_hydraulic_lock_status_;  // bit19 液压锁：true=可以动作
    uint8_t vcu_digital_inputs_[8];   // 数字输入状态 (IN_43_DI 到 IN_50_DI)
    std::chrono::steady_clock::time_point last_vcu_feedback_time_;  // 最后收到VCU反馈的时间
    bool vcu_feedback_timeout_initialized_;  // VCU反馈时间戳是否已初始化
    /// 0x18210005 byte5~6：发动机转速 0~8000（来自 /cannode/engine_feedback）
    std::atomic<uint16_t> host_engine_speed_rpm_{0};
    
    // 心跳计数器
    uint8_t heartbeat_counter_;
    
    // 日志输出频率控制
    static constexpr int log_output_interval_ = 10;
    
    int sequence_num_ = -1;
    int sequence_cnt = -1;
    
    // PID配置管理器（所有控制改为直接电流控制，不再需要）
    // std::unique_ptr<PIDConfigManager> pid_config_manager_;
    
    // 初始化PID参数（所有控制改为直接电流控制，不再需要）
    void initPIDControllers();
    
    // 控制周期时间戳（用于计算真实 dt）
    std::chrono::steady_clock::time_point last_ctrl_time_;
    bool last_ctrl_time_inited_ = false;
};

} // namespace cannode

