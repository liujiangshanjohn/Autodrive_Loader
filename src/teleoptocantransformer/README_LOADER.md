# 上海交大无人装载机线控（协议 20261102）

完整说明见 [README.md](README.md)。协议文件：`docs/protocol/装载机_上位机通讯协议_20261102.xlsx`。

基于《装载机_上位机通讯协议_20261102》：

| 方向 | CAN ID | 内容 |
|------|--------|------|
| 上位机→车 | 0x100 | 前进/后退/喇叭/照明/心跳 |
| 上位机→车 | 0x101 | 油门/刹车/左转/右转 (0~1000) |
| 上位机→车 | 0x102 | 大臂升落/铲斗升落 (0~1000) |
| 车→上位机 | 0x200~0x209 | 状态/倾角/压力/电流/转向角/车速 |

CAN：**标准帧**，**250 kbps**，周期 **50 ms**。心跳 500ms 翻转；连续 2s 无心跳则车端停机。

## 编译

```bash
cd /home/Autodrive_Loader
colcon build --packages-select cannode teleoptocantransformer
source install/setup.bash
```

确认 `cannode/config/vehicle_param.pb.txt` 中：

```text
brand: 14  # SJTU_LOADER
can_port: "can0"
```

## 运行（推荐分终端）

```bash
# 终端1：CAN 驱动 + 协议转换 + 监控
ros2 launch teleoptocantransformer loader_keyboard_stack.launch.py

# 终端2：键盘遥控（必须交互式 tty）
ros2 run teleoptocantransformer keyboard_loader_teleop.py
# 或
python3 src/teleoptocantransformer/scripts/keyboard_loader_teleop.py
```

也可分开启动：

```bash
ros2 launch cannode cannode.launch.py
ros2 launch teleoptocantransformer teleop_converter_loader.launch.py
ros2 run teleoptocantransformer loader_can_monitor.py
```

## 键盘映射

| 键 | 动作 |
|----|------|
| W / S | 前进档+油门 / 后退档+油门 |
| A / D | 左转 / 右转 |
| 空格 | 刹车 |
| I / K | 大臂升 / 落 |
| J / L | 铲斗升 / 落 |
| H / T | 喇叭 / 照明（开关） |
| N | 空档清零 |
| E | 急停 |
| Esc / Q | 急停并退出 |

油门等幅度可用参数调整，例如：

```bash
ros2 run teleoptocantransformer keyboard_loader_teleop.py --ros-args \
  -p throttle_level:=0.3 -p steer_level:=0.4
```

## 验证 CAN

监控节点会打印：

- `[TX 0x100/0x101/0x102]`：实际下发的前进后退、油门、转向等
- `[FB]`：车端反馈的前进/后退输出、转向阀电流等

也可：

```bash
ros2 topic echo /can_frame_sent
ros2 topic echo /cannode/vcu_feedback
ros2 topic echo /vehicle_command_debug
```

## 数据流

```
键盘 → /controls/teleop (JSON)
     → teleop2can_transformer_loader
     → /vehicle_command (ControlCommand protobuf)
     → cannode LoaderCANParser
     → CAN0  0x100/0x101/0x102
车端 → CAN  0x200~0x209
     → /cannode/vcu_feedback + /chassis_status
```



## 离线可视化演示（无需 CAN、无需编译），直接跑

```bash
cd /home/Autodrive_Loader
python3 src/teleoptocantransformer/scripts/loader_keyboard_gui.py
```

1. 弹出窗口后，**先用鼠标点一下窗口**（让键盘焦点进窗口）  
2. 按住按键操作：

| 键 | 作用 |
|----|------|
| **W / S** | 前进 / 后退 |
| **A / D** | 左转 / 右转 |
| **空格** | 刹车 |
| **I / K** | 大臂升 / 落 |
| **J / L** | 铲斗升 / 落 |
| **H / T** | 喇叭 / 灯（开关） |
| **E** | 急停 |
| **N** | 空档复位 |

3. 左侧俯视图看整车移动，右上侧视图看大臂铲斗，右侧看 CAN 预览（`0x100/0x101/0x102`）  
4. 关掉窗口即退出  

可选：同时往 ROS 发话题时加 `--ros`：

```bash
python3 src/teleoptocantransformer/scripts/loader_keyboard_gui.py --ros
```

---

## 接上 CAN 卡实车时（以后）

```bash
cd /home/Autodrive_Loader
colcon build --packages-select cannode teleoptocantransformer
source install/setup.bash

# 终端1
ros2 launch teleoptocantransformer loader_keyboard_stack.launch.py

# 终端2
python3 src/teleoptocantransformer/scripts/loader_keyboard_gui.py --ros
```