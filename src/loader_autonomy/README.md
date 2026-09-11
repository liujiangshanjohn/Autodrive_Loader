# loader_autonomy — 无人装载机车端自主主路径骨架

对齐架构：传感驱动 → 感知/定位 → 任务 V 循环 → 规划 → 控制仲裁 → `/vehicle_command` → `cannode` ↔ VCU。

## 与仓库现有包的关系

| 模块 | 状态 | 包 |
|------|------|-----|
| 雷达驱动 | 已有 | `robosense_lidar_driver` |
| GNSS/INS | 已有 | `cgi430_driver` |
| CAN / VCU 桥 | 已有（装载机协议需落地） | `cannode` |
| 公共消息 | 已有 | `cyber_msgs` |
| MQTT 上行 | 已有 | `mqtt_updater` |
| 遥控调试旁路 | 已有（非主路径） | `teleoptocantransformer` |
| 挖机点云投影 | 已有（装载机需旁路/改造） | `lidar_perception` |
| 相机驱动 | **本包旁路骨架** | `camera_driver` |
| 感知 / 定位 / 任务 / 规划 / 控制 | **本包骨架** | `loader_autonomy` |

## 话题约定（骨架）

```
/camera/driver_status          camera_driver → (heartbeat)
/loader/perception             perception
/loader/localization           localization (PoseStamped)
/loader/mission_state          mission
/loader/mission_goal           mission → planning
/loader/plan                   planning → control
/loader/manual_takeover        Bool → control
/loader/estop                  Bool → control
/loader/control_status         control
/loader/vehicle_command_debug  control (JSON stub)
/vehicle_command               实车指令；骨架默认不发布（enable_vehicle_command:=false）
```

## 编译与启动

```bash
cd /home/Autodrive_Loader
colcon build --packages-select camera_driver loader_autonomy
source install/setup.bash

# 仅自主骨架
ros2 launch loader_autonomy autonomy.launch.py

# 骨架 + 相机占位
ros2 launch loader_autonomy loader_stack.launch.py
```

实车闭环另开（已有包）：

```bash
ros2 launch cannode cannode.launch.py
# 可选监控
ros2 launch mqtt_updater chassis_to_mqtt.launch.py
# 可选调试旁路（与自主争用 /vehicle_command，勿同时使能）
ros2 launch teleoptocantransformer teleop_converter.launch.py
```

## 后续细化优先级

1. `control_node`：序列化 `ControlCommand` → `/vehicle_command`（唯一写者；与 teleop 仲裁）
2. `localization_node`：订 `/chcnav/devpvt` + 轮速/铰接角（替代挖机回转语义）
3. `perception_node`：订 `/rslidar_points` + 相机图，输出障碍/料堆/卸料点
4. `mission_node`：真实状态机转移条件
5. `planning_node`：路径/速度/臂斗目标
6. `camera_driver`：GMSL 实采集
7. `cannode`：装载机 CAN 协议表落地；铰接角接入
