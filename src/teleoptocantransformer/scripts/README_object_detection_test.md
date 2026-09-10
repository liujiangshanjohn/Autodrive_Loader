# 多摄像头目标检测测试脚本

## 脚本说明

`test_object_detection_publisher.py` - 模拟发布多摄像头目标检测结果，用于测试 `/cannode/chassis_feedback` 中的检测结果拼接功能。

## 功能特性

- 发布话题：`/perception/object_detection`
- 消息类型：`cyber_msgs/msg/MultiCameraObjectDetection`
- 发布频率：10Hz (每 0.1 秒)
- 模拟场景：
  - 场景1：所有摄像头都有检测结果（75% 概率）
  - 场景2：仅前摄像头有检测结果（15% 概率）
  - 场景3：无检测结果（10% 概率）

## 使用方法

### 1. 确保环境已设置

```bash
cd ~/cannode_ws
source install/setup.bash
```

### 2. 运行测试脚本

```bash
python3 src/teleoptocantransformer/scripts/test_object_detection_publisher.py
```

或者：

```bash
ros2 run teleoptocantransformer test_object_detection_publisher.py
```

### 3. 查看输出

在另一个终端中，可以查看 `/cannode/chassis_feedback` 话题，验证检测结果是否已正确拼接：

```bash
source install/setup.bash
ros2 topic echo /cannode/chassis_feedback
```

## 测试数据说明

脚本会生成以下类型的测试数据：

- **前摄像头 (front)**：通常检测车辆和行人
- **左摄像头 (left)**：可能检测车辆、卡车或交通锥
- **右摄像头 (right)**：可能检测车辆、自行车或行人
- **后摄像头 (rear)**：通常检测较少，主要是车辆

每个检测目标包含：
- `id`: 跟踪ID
- `class_name`: 类别名称（person, car, truck, bicycle 等）
- `confidence`: 置信度 (0.0-1.0)
- `center_x`, `center_y`: 归一化中心坐标 (0-1)
- `width`, `height`: 归一化尺寸 (0-1)

## 验证功能

1. 启动 `teleop2can_transformer_xiaosong` 节点
2. 运行测试脚本
3. 检查 `/cannode/chassis_feedback` 话题中的 `object_detection` 字段
4. 确认检测结果已正确拼接到 JSON 中

## 注意事项

- 确保已编译 `cyber_msgs` 包
- 确保 `teleoptocantransformer` 节点正在运行
- 测试脚本会持续运行，按 Ctrl+C 停止
