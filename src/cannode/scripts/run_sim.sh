#!/bin/bash

# 统一日志：写入 /media/nvidia/00code/logs，文件名包含日期时间
LOG_ROOT="/media/nvidia/00code/logs"
mkdir -p "$LOG_ROOT"
TS="$(date +"%Y%m%d_%H%M%S")"
LOG_FILE="$LOG_ROOT/cannode_${TS}.log"
echo "===== START $(date -Is) =====" | tee -a "$LOG_FILE"
AWK_BIN="${AWK_BIN:-}"
if [ -z "$AWK_BIN" ]; then
    if command -v gawk >/dev/null 2>&1; then
        AWK_BIN="gawk"
    else
        AWK_BIN="awk"
    fi
fi
exec > >("$AWK_BIN" '{ print strftime("[%Y-%m-%dT%H:%M:%S%z]"), $0; fflush(); }' | tee -a "$LOG_FILE") 2>&1

# 1) 配置 CAN（根据你实际情况改）
run_privileged() {
    if [ "$(id -u)" -eq 0 ]; then
        "$@"
    else
        # 与 run.sh 一致：非 root 时用 echo nvidia | sudo -S
        echo nvidia | sudo -S "$@"
    fi
}

run_privileged modprobe can can_raw mttcan
if ! ip link show can0 >/dev/null 2>&1; then
    run_privileged ip link add dev can0 type can
fi
run_privileged ip link set can0 down
run_privileged ip link set can0 type can bitrate 250000
run_privileged ip link set can0 up

# 配置 can1，与 can0 类似（如果不存在则创建）
if ! ip link show can1 >/dev/null 2>&1; then
    run_privileged ip link add dev can1 type can
fi
run_privileged ip link set can1 down
run_privileged ip link set can1 type can bitrate 250000
run_privileged ip link set can1 up

# 2) ROS2 环境 & 日志目录

source /media/nvidia/00code/cannode_ws/install/setup.bash

# 仿真 bag 仅回放指定话题（其余话题不发布，减轻仿真负载）
BAG_PATH="${BAG_PATH:-/media/nvidia/00code/record_20260512T004913Z_load_truck_auto_lidar_0-360degree/record_20260512T004913Z/ros2_bag}"
BAG_TOPICS=(
    /can_frame_sent
    /cannode/engine_feedback
    /cannode/vcu_feedback
    /chassis/status
    /chassis_status
    /chcnav/devimu
    /chcnav/devpvt
    /rslidar_points
    /shm_cam_0/image_compressed
    /shm_cam_1/image_compressed
    /shm_cam_2/image_compressed
    /shm_cam_3/image_compressed
    /shm_cam_4/image_compressed
    /shm_cam_5/image_compressed
    /shm_cam_6/image_compressed
    /shm_cam_7/image_compressed
    /tilt_body
    /tilt_boom
    /tilt_bucket
    /tilt_stick
)
ros2 bag play --loop "$BAG_PATH" --topics "${BAG_TOPICS[@]}" &

cd /media/nvidia/00code/cannode_ws

# 设置 ROS2 日志目录（如果外部没指定，就用工作空间下的 log）
export ROS_LOG_DIR=${ROS_LOG_DIR:-/media/nvidia/00code/cannode_ws/log}
mkdir -p "$ROS_LOG_DIR"

source install/setup.bash

# 3) 启动所有节点（在后台运行）
# echo "启动 cannode..."
# ros2 launch cannode cannode.launch.py &
# CANNODE_PID=$!

# echo "启动 multi_tilt_imu_node..."
# ros2 launch cannode multi_tilt_imu.launch.py &
# TILT_IMU_PID=$!

echo "启动 teleop2can_transformer..."
ros2 launch teleoptocantransformer teleop_converter_xiaosong.launch.py &
TELEOP_PID=$!

# echo "启动 Danfoss 编码器节点..."
# ros2 launch cannode danfoss_rotary_encoder.launch.py &
# DANFOSS_PID=$!

# echo "启动 chassis_to_mqtt 节点..."
# ros2 run mqtt_updater chassis_to_mqtt &
# MQTT_UPDATER_PID=$!

# echo "启动 速腾 Airy 雷达（rslidar_sdk Humble）..."
# ros2 launch rslidar_sdk humble_start.py &
# RSLIDAR_PID=$!

# echo "启动 CGI-430 CAN 驱动..."
# ros2 launch chcnav demo_0.xml &
# CGI430_PID=$!

# echo "启动 engine_feedback_node（发动机/VCU 反馈解析节点）..."
# ros2 run cannode engine_feedback_node &
# ENGINE_FEEDBACK_PID=$!

# echo "启动 lidar_perception 节点..."
# ros2 launch lidar_perception point_grid_publisher.launch.py &
# LIDAR_PERCEPTION_PID=$!

cd /media/nvidia/00code/cannode_ws
source install/setup.bash

# 4) 等待所有进程
echo "所有节点已启动："
echo "  - cannode (PID: $CANNODE_PID)"
echo "  - multi_tilt_imu_node (PID: $TILT_IMU_PID)"
echo "  - teleop2can_transformer (PID: $TELEOP_PID)"
echo "  - danfoss_rotary_encoder (PID: $DANFOSS_PID)"
echo "  - chassis_to_mqtt (PID: $MQTT_UPDATER_PID)"
# echo "  - rslidar_sdk (Airy 雷达，PID: $RSLIDAR_PID)"
echo "  - CGI-430 (PID: $CGI430_PID)"
echo "  - engine_feedback_node (PID: $ENGINE_FEEDBACK_PID)"
echo ""
echo "按 Ctrl+C 停止所有节点"

# 捕获 Ctrl+C 信号，清理后台进程
trap "echo '正在停止所有节点...'; kill $CANNODE_PID $TILT_IMU_PID $TELEOP_PID $DANFOSS_PID $RSLIDAR_PID $CGI430_PID $ENGINE_FEEDBACK_PID 2>/dev/null; rm -f \"$TMPDIR\"/launch_params_* /media/nvidia/00code/cannode_ws/launch_params_* 2>/dev/null; exit" INT TERM

# 等待所有后台进程
wait

