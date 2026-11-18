# Autodrive Loader

上海交大无人驾驶装载机上位机软件。按《装载机_上位机通讯协议_20261102》键盘遥控：前进/后退、转向、大臂和铲斗。

协议文件：`docs/protocol/装载机_上位机通讯协议_20261102.xlsx`。实现：`src/cannode/src/vehicle_protocol/loader.h`。使用说明：[src/teleoptocantransformer/README.md](src/teleoptocantransformer/README.md)。

## 仓库结构

| 目录 | 说明 |
|------|------|
| `src/cannode` | CAN 驱动；装载机协议解析 `LoaderCANParser`（`0x100`–`0x102` 下发，`0x200`–`0x205` 反馈） |
| `src/teleoptocantransformer` | 键盘/JSON 遥控 → `/vehicle_command` |
| `src/loader_autonomy` | 感知/定位/规划/控制骨架 |
| `src/camera_driver` | 相机驱动骨架 |
| `src/robosense_lidar_driver` | 速腾激光雷达驱动 |

## 传感器布置（当前约定）

| 传感器 | 安装位置 | 朝向 / 用途 |
|--------|----------|-------------|
| 补盲雷达 | **前挡泥板上方**，或 **动臂根部** | **前向**，补主雷达被铲斗/动臂挡住的近场 |

不是后车轮上方。后轮翼子板不作为补盲雷达安装点。

## CAN 协议（20261102）

CAN：**标准帧**，**250 kbps**，发送周期 **50 ms**。

心跳在 `0x100` byte7 bit7，每 **500 ms** 翻转一次；车端连续 **2 s** 收不到心跳则停机。

| 方向 | CAN ID | 内容 |
|------|--------|------|
| 上位机→车 | 0x100 | 前进/后退/喇叭/照明/心跳 |
| 上位机→车 | 0x101 | 油门/刹车/左转/右转（0~1000） |
| 上位机→车 | 0x102 | 大臂升落/铲斗升落（0~1000） |
| 车→上位机 | 0x200 | 开关量/输出/故障/心跳反馈 |
| 车→上位机 | 0x201 | 大臂/铲斗倾角 |
| 车→上位机 | 0x202 | 转向/大臂压力 |
| 车→上位机 | 0x203 | 铲斗压力 + 油门/刹车输出 |
| 车→上位机 | 0x204 | 转向/大臂阀电流 |
| 车→上位机 | 0x205 | 铲斗阀电流 + 系统电压 + 转向角 |
| 车→上位机 | 0x206 | 行驶速度 + 控制器故障码 |
| 车→上位机 | 0x207 | 倾角原始值 |
| 车→上位机 | 0x208 | 压力传感器原始值 |
| 车→上位机 | 0x209 | 铲斗压力/车速/转向电子尺原始值 |

比例量均为 **小端 uint16**。`0~1000` 对应 `0~100.0%`。同一轴的两个方向互斥（前进/后退、左转/右转、升/落）；同时为 1 时车端无动作。

### 0x100 开关量 + 心跳

| 字段 | 位置 | 说明 |
|------|------|------|
| 前进 | byte0 bit0 | 与后退互斥 |
| 后退 | byte0 bit1 | |
| 喇叭 | byte0 bit2 | |
| 照明 | byte0 bit3 | |
| 心跳 | byte7 bit7 | 500 ms 翻转 |

### 0x101 踏板与转向

| 字段 | 字节 | 范围 |
|------|------|------|
| 油门 | 0–1 | 0~1000 |
| 刹车 | 2–3 | 0~1000 |
| 左转 | 4–5 | 0~1000，与右转互斥 |
| 右转 | 6–7 | 0~1000 |

### 0x102 大臂 / 铲斗

| 字段 | 字节 | 范围 |
|------|------|------|
| 大臂升 | 0–1 | 0~1000 |
| 大臂落 | 2–3 | 0~1000 |
| 铲斗升 | 4–5 | 0~1000 |
| 铲斗落 | 6–7 | 0~1000 |

### 0x200~0x205 反馈摘要

| ID | 主要字段 |
|----|----------|
| 0x200 | 前进/后退/喇叭/照明设定反馈；自动模式；输出（前进/后退/喇叭/灯/臂斗电机等）；传感器/阀故障；心跳 |
| 0x201 | 大臂 X/Y、铲斗 X/Y 倾角。原始值按有符号 16 位，精度 0.01°（0~65535 表示 -32768~32767） |
| 0x202 | 左转/右转压力、大臂升/落压力。0~400 → 0~40.0 MPa |
| 0x203 | 铲斗升/落压力；油门/刹车输出 0~1000 |
| 0x204 | 左转/右转/大臂升/落阀电流（mA） |
| 0x205 | 铲斗升/落阀电流（mA）；系统电压 0~32000 → 0~32.000 V；转向角 0~36000 → -180~+180°（正=左摆） |
| 0x206 | 行驶速度 0~255 → 0~25.5 km/h；控制器故障码 |
| 0x207~0x209 | 倾角/压力/车速/电子尺原始值 |

### ControlCommand → CAN 映射

| 遥控 / `ControlCommand` | CAN |
|-------------------------|-----|
| `gear` D / R / N | 0x100 前进 / 后退 / 空档 |
| `throttle` / `brake` [0,100]% | 0x101 油门 / 刹车 0~1000 |
| `steering_target` [-100,100]% | 0x101 左转或右转（互斥） |
| 大臂升/落电流设定 | 0x102 大臂升/落 0~1000 |
| 铲斗：`close`=升，`dump`=落 | 0x102 铲斗升/落 0~1000 |
| `VehicleSignal.horn` | 0x100 喇叭 |
| `light_code` bit0 或近光 | 0x100 照明 |
| `estop` | 强制空档，关喇叭和照明 |

确认 `src/cannode/config/vehicle_param.pb.txt`：

```text
brand: 14  # SJTU_LOADER
can_port: "can0"
```

## 最快演示（无需编译、无需 CAN 卡）

依赖：Python 3、tkinter、Pillow。WSL 需能弹 GUI 窗口。

```bash
cd /home/Autodrive_Loader
python3 src/teleoptocantransformer/scripts/loader_keyboard_gui.py
```

1. 弹出窗口后，先用鼠标点一下窗口，再按键。
2. 左侧俯视图看整车移动，右上侧视图看大臂/铲斗，右侧看将要下发的 CAN 预览。

可选：同时发布 ROS 话题 `/controls/teleop`：

```bash
python3 src/teleoptocantransformer/scripts/loader_keyboard_gui.py --ros
```

### 按键

| 键 | 作用 |
|----|------|
| W / S | 前进 / 后退 |
| A / D | 左转 / 右转 |
| 空格 | 刹车 |
| I / K | 大臂升 / 落 |
| J / L | 铲斗升 / 落 |
| H / T | 喇叭 / 灯（开关） |
| N | 空档复位 |
| E | 急停 |
| Esc / Q | 急停并退出（终端键盘节点） |

幅度可用参数调低，例如：

```bash
ros2 run teleoptocantransformer keyboard_loader_teleop.py --ros-args \
  -p throttle_level:=0.3 -p steer_level:=0.4
```

## 接 CAN 卡控制实车

```bash
cd /home/Autodrive_Loader
colcon build --packages-select cannode teleoptocantransformer
source install/setup.bash

# 终端 1：CAN + 协议转换 + 监控
ros2 launch teleoptocantransformer loader_keyboard_stack.launch.py

# 终端 2：可视化键盘遥控（发布 /controls/teleop）
python3 src/teleoptocantransformer/scripts/loader_keyboard_gui.py --ros
# 或纯终端键盘（必须交互式 tty）
ros2 run teleoptocantransformer keyboard_loader_teleop.py
```

也可分开启动：

```bash
ros2 launch cannode cannode.launch.py
ros2 launch teleoptocantransformer teleop_converter_loader.launch.py
ros2 run teleoptocantransformer loader_can_monitor.py
```

数据流：

```text
键盘 → /controls/teleop
     → teleop2can_transformer_loader
     → /vehicle_command
     → cannode LoaderCANParser
     → can0  0x100 / 0x101 / 0x102
车端 → 0x200~0x209
     → /cannode/vcu_feedback、/chassis_status
```

实车前建议把 GUI 或键盘节点的油门幅度调低（例如 `throttle_level:=0.2`），并确认控制器已切到自动模式。

### 验证 CAN

监控节点会打印：

- `[TX 0x100/0x101/0x102]`：实际下发的前进后退、油门、转向等
- `[FB]`：车端反馈的前进/后退输出、转向阀电流等

也可：

```bash
ros2 topic echo /can_frame_sent
ros2 topic echo /cannode/vcu_feedback
ros2 topic echo /vehicle_command_debug
```
