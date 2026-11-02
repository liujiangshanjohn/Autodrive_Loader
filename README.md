# Autodrive Loader

上海交大无人驾驶装载机上位机软件。当前重点是按《装载机_上位机通讯协议_20260721》用键盘遥控线控底盘：前进/后退、转向、大臂和铲斗。

## 仓库结构

| 目录 | 说明 |
|------|------|
| `src/cannode` | CAN 驱动；装载机协议解析 `LoaderCANParser`（`0x100`–`0x102` 下发，`0x200`–`0x205` 反馈） |
| `src/teleoptocantransformer` | 键盘/JSON 遥控 → `/vehicle_command` |
| `src/loader_autonomy` | 感知/定位/规划/控制骨架 |
| `src/camera_driver` | 相机驱动骨架 |
| `src/robosense_lidar_driver` | 速腾激光雷达驱动 |

更细的协议字段与 CAN 映射见 [src/teleoptocantransformer/README_LOADER.md](src/teleoptocantransformer/README_LOADER.md)。

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
| E | 急停 |
| N | 空档复位 |

## 接 CAN 卡控制实车

CAN：标准帧，250 kbps，周期 50 ms。确认 `src/cannode/config/vehicle_param.pb.txt` 中：

```text
brand: 14  # SJTU_LOADER
can_port: "can0"
```

```bash
cd /home/Autodrive_Loader
colcon build --packages-select cannode teleoptocantransformer
source install/setup.bash

# 终端 1：CAN + 协议转换 + 监控
ros2 launch teleoptocantransformer loader_keyboard_stack.launch.py

# 终端 2：可视化键盘遥控（发布 /controls/teleop）
python3 src/teleoptocantransformer/scripts/loader_keyboard_gui.py --ros
```

数据流：

```text
键盘 → /controls/teleop
     → teleop2can_transformer_loader
     → /vehicle_command
     → cannode LoaderCANParser
     → can0  0x100 / 0x101 / 0x102
车端 → 0x200~0x205
     → /cannode/vcu_feedback、/chassis_status
```

实车前建议把 GUI 或键盘节点的油门幅度调低（例如 `throttle_level:=0.2`），并确认控制器已切到自动模式。
