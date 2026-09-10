# Logitech F710 手柄操作说明

用 F710（背面拨到 **D** / DirectInput）通过摇杆和按键发 `/vehicle_command`，控制大臂、小臂、铲斗、回转、履带。电流规则与调试 UI 一致：非零为 **200～700 mA**，回中或松开为 **0**。

手柄在 Jetson 上识别为 `Logitech Cordless RumblePad 2`。

## 启动前

1. **cannode 已在跑**（例如 `ros2 launch cannode cannode.launch.py` 或 `./run.sh`）。
2. **不要同时开调试界面** `./run_xiaosong_controller.sh`，两边都会发 `/vehicle_command`，会抢控制。
3. 手柄背面拨到 **D**。X 档在本机没有 `xpad` 驱动，打不开。
4. 确认有手柄设备：

```bash
ls /dev/input/js0
```

若没有 `js0`（插上后仍没有），可临时切到 hid-generic 后再拔插或重新授权 USB：

```bash
echo 1 | sudo tee /sys/module/hid/parameters/ignore_special_drivers
```

该设置**重启后失效**，需要再做一次。

## 启动

脚本会自行启动 `joy_node`（若已有则跳过）：

```bash
cd /media/nvidia/00code/cannode_ws/src/cannode/scripts
./run_f710_joy_controller.sh
```

Ctrl+C 退出。若本脚本拉起了 `joy_node`，退出时会一并停掉。

## 操作

实车先按 **Start** 申请遥控（日志出现 `request_remote_control=True`），再动手臂/履带。再按一次 Start 取消申请。

| 操作 | 动作 | 说明 |
|---|---|---|
| 左摇杆上下 | 大臂抬 / 降 | 必须按住 **RB** |
| 右摇杆上下 | 小臂伸 / 收 | 必须按住 **RB** |
| **RB** | 大臂/小臂死人手 | 松开后臂电流立刻归零 |
| **LB** | 铲斗内收 | 按住缓加电流，松开归零 |
| **LT** | 铲斗外翻 | 同上 |
| **X** | 左回转 | 按住缓加，松开归零 |
| **B** | 右回转 | 同上 |
| **Y** | 右履带前进 | 同上 |
| **A** | 右履带后退 | 同上 |
| 十字 **上** | 左履带前进 | 同上 |
| 十字 **下** | 左履带后退 | 同上 |
| **Start** | 申请 / 取消遥控 | 点按切换 |

同一对里两个一起按（例如 X+B、Y+A、十字上下、LB+LT）两边都清零，避免对拉。

按键类（铲斗 / 回转 / 履带）**不用按 RB**。按住约 2 秒从 200 mA 加到 700 mA（约 250 mA/s）。

摇杆回中、按键松开、手柄超时（约 0.3 s 无 `/joy`）都会把对应电流打成 0。

## 看是否在发

终端日志示例：

```text
remote=1 boom u/d=... stick e/r=... bucket c/d=... rot l/r=... L f/b=... R f/b=... mA
```

`remote=0` 表示还没按 Start。也可以：

```bash
ros2 topic echo /vehicle_command_debug_text
ros2 topic echo /joy --once
```

## 常见问题

**调试 UI 显示「已禁用控制数据发送」**  
那是 UI 自己的发送开关，和手柄 RB 无关。用手柄时请关掉 UI。

**按了键没电流**  
确认 `joy_node` 已打开手柄（日志有 `Opened joystick: Logitech Cordless RumblePad 2`）。大臂/小臂必须按住 RB 再推摇杆。

**十字前后反了**  
启动时加：`--ros-args -p invert_hat_y:=true`

**大臂/小臂方向反了**

```bash
./run_f710_joy_controller.sh --ros-args -p invert_boom:=false -p invert_stick:=false
```

**LB/LT 对调（内收外翻反了）**

```bash
./run_f710_joy_controller.sh --ros-args -p bucket_close_button:=6 -p bucket_dump_button:=4
```

**加电流太慢/太快**  
改 `hold_ramp_ma_per_sec`（默认 250）。例如 400 大约 1.25 秒到满：

```bash
./run_f710_joy_controller.sh --ros-args -p hold_ramp_ma_per_sec:=400.0
```

## 相关文件

- `run_f710_joy_controller.sh`：一键启动
- `f710_joy_controller.py`：映射与发令
- `xiaosong_controller.py`：调试 UI（不要和本节点同时发令）
