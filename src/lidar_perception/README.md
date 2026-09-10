# lidar_perception 使用说明

本文档基于当前代码实现，重点说明：

- 数据流程（从传感器到 `/cyber_msgs/point_grid`）
- 现有节点如何启动
- 两个主要 YAML 的关键参数含义

---

## 1. 代码入口与组件

当前包里和点阵投影相关的主要可执行程序：

- `bucket_point_grid_publisher`
  - 入口：`src/main_bucket_point_grid_publisher.cpp`
  - 实现：`src/bucket_point_grid_publisher_clean.cpp`
  - 作用：结合机械臂角度/运动学和外参，发布像素点阵到 `/cyber_msgs/point_grid`
- `lidar_cloud_to_point_grid`
  - 实现：`src/lidar_cloud_to_point_grid.cpp`
  - 作用：直接把激光点云投影到图像像素（用于外参调试）

启动文件：

- `launch/point_grid_publisher.launch.py`
  - 启动 `bucket_point_grid_publisher`
  - 默认参数文件：`config/bucket_point_grid_params.yaml`

---

## 2. 数据流程

### 2.1 `bucket_point_grid_publisher` 流程

1. 订阅输入：
   - 姿态/关节：`/tilt_boom`、`/tilt_stick`、`/tilt_bucket`、`/tilt_body`
   - 回转：`/rotary_encoder_angle`
2. 读取参数：
   - 连杆长度、铰点、bucket_open 偏移（`kinematics.*`）
   - 静态外参（`static_tfs.0`、`static_tfs.1`）
   - 相机内参与投影约定（`bucket_open_tracker.point_grid.*`）
3. 在定时器回调中：
   - 根据传感器角度计算 `base->upper->boom->stick->bucket->bucket_open` 动态 TF 链
   - 用 `upper_link->rslidar` + `rslidar->camera_main` 组合出 `cam_T_base` / `cam_T_upper`
   - 生成一组 3D 点（当前配置可为地面/像素网格/调试点模式）
   - 通过针孔模型投影到像素坐标
4. 发布：
   - 话题：`/cyber_msgs/point_grid`
   - 消息：`cyber_msgs/msg/PointGrid`（`x,y` 作为像素坐标）

### 2.2 `lidar_cloud_to_point_grid` 流程

1. 订阅点云：`/rslidar_points_fixed`
2. 读取内参、外参与投影约定（`config/lidar_cloud_to_point_grid.yaml`）
3. 点云过滤（步长、距离、ROI）
4. 将点从 lidar 坐标变换到 camera 坐标，再投影到像素
5. 发布到 `/cyber_msgs/point_grid`（用于快速外参检查）

---

## 3. 启动方式

先进入工作区并加载环境：

```bash
cd /media/nvidia/00code/cannode_ws
source install/setup.bash
```

### 3.0 单独编译本包

```bash
cd /media/nvidia/00code/cannode_ws
colcon build --packages-select lidar_perception
source install/setup.bash
```

### 3.1 启动 bucket 方案（主流程）

```bash
ros2 launch lidar_perception point_grid_publisher.launch.py
```

说明：

- 使用 `config/bucket_point_grid_params.yaml`
- 节点名：`/bucket_point_grid_publisher`

### 3.2 启动激光直投调试节点（外参调试）

```bash
ros2 run lidar_perception lidar_cloud_to_point_grid --ros-args --params-file src/lidar_perception/config/lidar_cloud_to_point_grid.yaml
```

### 3.3 查看输出

```bash
ros2 topic echo /cyber_msgs/point_grid
```

---

## 4. 关键 YAML 参数说明

## 4.1 `config/bucket_point_grid_params.yaml`

根键：`bucket_point_grid_publisher.ros__parameters`

- `frames.*`
  - 机械臂各 TF 帧名，需与系统约定一致
- `tilt_topics.*` / `tilt_axes.*` / `tilt_zeros_deg.*`
  - 倾角输入源、取轴方式、零点补偿
- `rotary_encoder.*`
  - 回转角来源和零点
- `input_qos.*`
  - 订阅可靠性，回放时通常使用 `false`（best effort）

`bucket_open_tracker` 下：

- `pointcloud_topic`
  - 点云话题名（当前配置为 `/rslidar_points_fixed`）
- `cloud_qos_reliable`
  - 点云订阅可靠性

`bucket_open_tracker.point_grid` 下（最关键）：

- 内参：`fx/fy/cx/cy`
- 图像尺寸：`img_width/img_height`
- 投影约定：
  - `pixel_origin`（`top_left` 或 `bottom_right`）
  - `forward_axis`（`z` 或 `x`）
  - `forward_sign`、`down_sign`
- 像素偏移：`u_offset_px/v_offset_px`
- 模式开关：
  - `use_ground_quad`
  - `use_pixel_quad`
  - `use_pixel_grid`
- `clamp_to_image`
  - 是否把像素裁剪到图像范围内

`angle_model.*`：

- 是否用“相对角”模型，以及各级关节方向符号

`kinematics.*`：

- 几何尺寸参数，直接影响 bucket_open 三维位置

`static_tfs.0` / `static_tfs.1`：

- `static_tfs.0`: `upper_link -> rslidar`
- `static_tfs.1`: `rslidar -> camera_main`
- 这是点阵是否对齐画面的核心外参

## 4.2 `config/lidar_cloud_to_point_grid.yaml`

根键：`lidar_cloud_to_point_grid.ros__parameters`

- `cloud_topic` / `output_topic`
- 相机内参：`fx/fy/cx/cy`、`img_width/img_height`
- 投影约定：`pixel_origin`、`forward_axis`、`forward_sign`、`down_sign`
- `extrinsic_direction`
  - `lidar_to_camera`：`static_tfs.1` 按 `lidar->camera` 解释
  - `camera_to_lidar`：`static_tfs.1` 按 `camera->lidar` 解释
- `static_tfs.1.xyz_m` / `static_tfs.1.rpy_deg`
  - 调试节点使用的雷达到相机外参
- 点云过滤：
  - `max_points`、`stride`
  - `min_range_m`、`max_range_m`
  - `use_aabb_roi` + `roi_center_xyz_m` + `roi_size_xyz_m`

---

## 5. 常见问题排查

- 现象：看不到点
  - 检查是否有发布 `/rslidar_points_fixed`
  - 检查 `forward_axis/forward_sign/down_sign` 是否匹配当前坐标约定
  - 检查 `static_tfs.1` 是否与当前节点读取的参数文件一致
- 现象：点整体偏移
  - 优先检查 `static_tfs.1.rpy_deg`（角度）
  - 再检查 `static_tfs.1.xyz_m`（平移）
- 现象：回放有话题但节点收不到
  - 检查 `input_qos.*` 和 `cloud_qos_reliable` 是否与发布端匹配

