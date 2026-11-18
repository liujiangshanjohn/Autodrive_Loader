# teleoptocantransformer

装载机遥控：键盘/JSON → `/vehicle_command` → cannode → CAN。

协议：[docs/protocol/装载机_上位机通讯协议_20261102.xlsx](../../docs/protocol/装载机_上位机通讯协议_20261102.xlsx)  
CAN 标准帧，250 kbps，周期 50 ms。心跳 500 ms 翻转；连续 2 s 无心跳则车端停机。

## 仿真（无需编译、无需 CAN）

WSL 的 Tk 窗口在任务栏里是企鹅图标，经常点不开。默认改到 **Windows 浏览器**：

```bash
cd /home/Autodrive_Loader
python3 src/teleoptocantransformer/scripts/loader_keyboard_gui.py
```

会弹出 Edge/Chrome。**点一下网页**，再按键。不要点任务栏企鹅。

| 键 | 作用 |
|----|------|
| W / S | 前进 / 后退 |
| A / D | 左转 / 右转 |
| 空格 | 刹车 |
| I / K | 大臂关节角目标升 / 落 |
| J / L | 铲斗关节角目标升 / 落 |
| H / T | 喇叭 / 灯 |
| N | 空档 |
| E | 急停 |

同时发 `/controls/teleop`：

```bash
python3 src/teleoptocantransformer/scripts/loader_keyboard_gui.py --ros
```

### 液压闭环（仿真 = 实车逻辑）

倾角计读数换成关节角（仿真里用油缸行程线性换角，实车换成几何即可）。

控制量是 **阀开度**，不是棱柱关节位置：

- `0`：阀关，油缸速度为 0
- **400~700**：有效开度，线性对应那根油缸的线速度（700 最大）
- 小于 400 视为死区，油缸不动

键盘改的是关节角目标。P 用角度误差出阀开度（0 或 400~700），油缸积分行程，再换成关节角反馈，到误差死区后阀关。`0x102` 下发的就是这对阀值。模型见 `scripts/loader_hydro.py`。

离线核对 CAN 编码：

```bash
python3 src/teleoptocantransformer/scripts/test_loader_keyboard_offline.py
```

## 协议

上位机 → 车（本包编码这些帧）：

| ID | 内容 |
|----|------|
| 0x100 | 前进/后退/喇叭/照明（byte0 bit0–3）；心跳 byte7 bit7 |
| 0x101 | 油门/刹车/左转/右转，小端 uint16，0~1000 |
| 0x102 | 大臂/铲斗比例阀，有效带 400~700（油缸速度）；0 为关 |
| 0x103~0x105 | 预留 |

同轴互斥：前进与后退、左转与右转、升与落，同时有效则无动作。

车 → 上位机（cannode 解析）：

| ID | 内容 |
|----|------|
| 0x200 | 设定/开关/输出/故障/心跳；含无线急停、行程开关 |
| 0x201 | 大臂/铲斗倾角，int16 × 0.01° |
| 0x202 | 转向/大臂压力 0~400 → 0~40.0 MPa |
| 0x203 | 铲斗压力；油门/刹车输出 0~1000 |
| 0x204 | 转向/大臂阀电流 mA |
| 0x205 | 铲斗阀电流；电压 0~32000 → 0~32.000 V；转向角 0~36000 → -180~+180°（正=左摆） |
| 0x206 | 车速 0~255 → 0~25.5 km/h；控制器故障码 |
| 0x207 | 倾角原始值 |
| 0x208 | 压力传感器原始值 μA |
| 0x209 | 铲斗压力原始值；车速原始值；转向电子尺 mV |

JSON `/controls/teleop`：

```json
{
  "steering": 0.2,
  "throttle": 0.6,
  "brake": 0.0,
  "gear": "D",
  "boom": 0.5,
  "bucket": 0.4,
  "horn": false,
  "light": false,
  "emergency_stop": false
}
```

## 实车

`src/cannode/config/vehicle_param.pb.txt`：`brand: 14`，`can_port: "can0"`。

```bash
cd /home/Autodrive_Loader
colcon build --packages-select cannode teleoptocantransformer
source install/setup.bash

# 终端 1
ros2 launch teleoptocantransformer loader_keyboard_stack.launch.py

# 终端 2
python3 src/teleoptocantransformer/scripts/loader_keyboard_gui.py --ros
```

```text
键盘 → /controls/teleop → teleop2can_transformer_loader
     → /vehicle_command → cannode → can0  0x100/0x101/0x102
车端 0x200~0x209 → /cannode/vcu_feedback
```
