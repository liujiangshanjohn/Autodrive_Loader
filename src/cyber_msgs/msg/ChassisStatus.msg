# CAN ID 0x18FF0001 数据 (VCUToHost1)
std_msgs/Header header
bool high_voltage_status
bool parking_brake
bool horn_status
bool left_turn_signal
bool right_turn_signal
uint8 walk_motor_mode
bool wet_brake_alarm
bool emergency_stop
uint8 gear_signal
uint8 rotation_alarm
bool heartbeat_status
uint16 brake_control
float32 front_rear_angle
uint8 battery_level
bool charging_status
bool hydraulic_lock
uint8 fault_level
bool turtle_rabbit_gear
bool work_light
uint8 vehicle_mode

# CAN ID 0x18FF0002 数据 (VCUToHost2)
uint16 boom_lift_current
uint16 boom_lower_current
uint16 bucket_close_current
uint16 bucket_open_current

# CAN ID 0x18FF0003 数据 (VCUToHost3)
uint16 boom_big_pressure
uint16 boom_small_pressure
uint16 bucket_big_pressure
uint16 bucket_small_pressure

# CAN ID 0x18FF0004 数据 (VCUToHost4)
uint16 hydraulic_motor_speed
uint16 hydraulic_motor_torque
uint16 hydraulic_motor_current
bool hydraulic_motor_enable

# CAN ID 0x18FF0005 数据 (VCUToHost5)
uint16 walk_motor_current
uint16 walk_motor_torque
float32 walk_motor_speed
bool walk_motor_enable

# CAN ID 0x18FF0006 数据 (VCUToHost6)
uint8 throttle_opening
uint8 brake_opening
bool heartbeat_signal
bool can_loss_1
bool can_loss_2
bool can_loss_3
bool can_loss_4
bool walk_motor_fault
bool hydraulic_motor_fault
bool boom_lift_valve_fault
bool boom_lower_valve_fault
bool bucket_close_valve_fault
bool bucket_open_valve_fault
bool foot_brake_valve_fault
bool turn_valve_fault
bool low_beam
bool high_beam
uint8 hydraulic_motor_voltage
uint16 turn_valve_current
uint8 hydraulic_motor_mode
uint8 vehicle_mode_2

# CAN ID 0x18FF0009 数据 (VCUToHost9)
uint16 rear_motor_current
uint8 rear_motor_voltage
uint16 front_motor_current
uint8 front_motor_voltage
uint16 hydraulic_motor_current_2

# CAN ID 0x18FF000A 数据 (VCUToHostA)
float32 boom_angle
float32 bucket_angle