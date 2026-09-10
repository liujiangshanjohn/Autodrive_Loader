#!/bin/bash
# 统一日志：写入 /media/nvidia/00code/logs，文件名包含日期时间
LOG_ROOT="/media/nvidia/00code/logs"
mkdir -p "$LOG_ROOT"
TS="$(date +"%Y%m%d_%H%M%S")"
LOG_FILE="$LOG_ROOT/${TS}_cannode.log"
echo "===== START $(date -Is) =====" | tee -a "$LOG_FILE"

# 图形会话自启动 / tmux 往往早于移动硬盘挂载或 mttcan 就绪，这里先等再重定向日志，避免 source 失败或 CAN 命令挂死
CANNODE_WS="/media/nvidia/00code/cannode_ws"
SETUP_BASH="${CANNODE_WS}/install/setup.bash"
WAIT_SETUP_MAX="${CANNODE_WAIT_SETUP_SEC:-120}"
WAIT_SETUP_SLICE="${CANNODE_WAIT_SETUP_SLICE:-2}"
if [ ! -f "$SETUP_BASH" ]; then
    echo "[run.sh] 等待 ${SETUP_BASH}（最多 ${WAIT_SETUP_MAX}s）..." | tee -a "$LOG_FILE"
    _waited=0
    while [ ! -f "$SETUP_BASH" ] && [ "$_waited" -lt "$WAIT_SETUP_MAX" ]; do
        sleep "$WAIT_SETUP_SLICE"
        _waited=$((_waited + WAIT_SETUP_SLICE))
    done
    if [ ! -f "$SETUP_BASH" ]; then
        echo "[run.sh] 错误：仍未找到 setup.bash，退出" | tee -a "$LOG_FILE"
        exit 1
    fi
fi
if [ -n "${CANNODE_BOOT_DELAY_SEC:-}" ] && [ "${CANNODE_BOOT_DELAY_SEC}" -gt 0 ] 2>/dev/null; then
    echo "[run.sh] CANNODE_BOOT_DELAY_SEC=${CANNODE_BOOT_DELAY_SEC}，延迟后再继续..." | tee -a "$LOG_FILE"
    sleep "${CANNODE_BOOT_DELAY_SEC}"
fi

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
# sudo 方式与 camera_code/excavator/scripts/start_all.sh 一致：非 root 时用 echo nvidia | sudo -S
run_privileged() {
    if [ "$(id -u)" -eq 0 ]; then
        "$@"
    else
        echo nvidia | sudo -S "$@"
    fi
}

run_privileged modprobe can can_raw mttcan
if [ "${CANNODE_AFTER_MODPROBE_SEC:-0}" -gt 0 ] 2>/dev/null; then
    sleep "${CANNODE_AFTER_MODPROBE_SEC}"
fi
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
cd /media/nvidia/00code/cannode_ws

export ROS_LOG_DIR=${ROS_LOG_DIR:-/media/nvidia/00code/cannode_ws/log}
mkdir -p "$ROS_LOG_DIR"

source install/setup.bash

# 清掉可能带着错误 RMW/域信息的 daemon，减轻「首次自启动异常、重开 tmux 才好」的现象（本窗口最先起，不影响后续窗口里 ros2）
ros2 daemon stop >/dev/null 2>&1 || true
sleep "${CANNODE_AFTER_DAEMON_STOP_SEC:-1}"

# --- DDS 策略（默认单机全集控）---
# local（默认）：ROS_LOCALHOST_ONLY + Fast DDS，所有进程走本机 loopback 发现；速腾 humble_start
#   同样不再绑 Cyclone+eth0，可避免 Jetson 上「节点一多就 rmw handle invalid / echo 不到雷达」。
# eth：恢复 Cyclone + CYCLONEDDS_URI 绑网卡（跨机或必须用 eth 组播时再设 CANNODE_DDS_MODE=eth）。
CANNODE_DDS_MODE="${CANNODE_DDS_MODE:-local}"

if [ "$CANNODE_DDS_MODE" = "eth" ]; then
    CANNODE_DDS_IF="${CANNODE_DDS_IF:-eth0}"
    CANNODE_CYCLONE_XML="${CANNODE_CYCLONE_XML:-/tmp/cyclonedds_vehicle_runsh.xml}"
    cat > "$CANNODE_CYCLONE_XML" <<EOF
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain Id="any">
    <General>
      <Interfaces>
        <NetworkInterface name="${CANNODE_DDS_IF}" multicast="default" priority="default"/>
      </Interfaces>
    </General>
  </Domain>
</CycloneDDS>
EOF
    CYCLONEDDS_URI="file://${CANNODE_CYCLONE_XML}"
    export CYCLONEDDS_URI
    export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
    unset ROS_LOCALHOST_ONLY
    echo "[run.sh] DDS 模式: eth（Cyclone + ${CANNODE_DDS_IF}）"
else
    unset CYCLONEDDS_URI
    export ROS_LOCALHOST_ONLY=1
    export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
    echo "[run.sh] DDS 模式: local（ROS_LOCALHOST_ONLY=1 + ${RMW_IMPLEMENTATION}）"
fi

# 降低「同时创建 DDS participant」失败概率
ulimit -n 1048576 2>/dev/null || ulimit -n 65536 2>/dev/null || true

# 通用间隔（秒）；最后一批三个节点前可额外等待 RUN_SH_LAST_GAP_SEC
RUN_SH_STAGGER_SEC="${RUN_SH_STAGGER_SEC:-2}"
RUN_SH_LAST_GAP_SEC="${RUN_SH_LAST_GAP_SEC:-3}"

# 3) 启动所有节点（后台）。顺序：先雷达 → 其它 → 最后再起 mqtt/cgi/engine（曾对 RMW 压力最大）
if [ "$CANNODE_DDS_MODE" = "eth" ]; then
    echo "启动 速腾 Airy 雷达（rslidar_sdk，dds_interface=${CANNODE_DDS_IF}）..."
    ros2 launch rslidar_sdk humble_start.py "dds_interface:=${CANNODE_DDS_IF}" &
else
    echo "启动 速腾 Airy 雷达（rslidar_sdk，本机 DDS）..."
    ros2 launch rslidar_sdk humble_start.py &
fi
RSLIDAR_PID=$!
sleep "${RUN_SH_STAGGER_SEC}"

echo "启动 lidar_perception（bucket_point_grid_publisher）..."
ros2 launch lidar_perception point_grid_publisher.launch.py &
LIDAR_PERCEPTION_PID=$!
sleep "${RUN_SH_STAGGER_SEC}"

echo "启动 lidar_perception（lidar_arm_bucket_2d_grid）..."
ros2 launch lidar_perception lidar_arm_bucket_2d_grid.launch.py &
LIDAR_ARM_BUCKET_2D_GRID_PID=$!
sleep "${RUN_SH_STAGGER_SEC}"

echo "启动 cannode..."
ros2 launch cannode cannode.launch.py &
CANNODE_PID=$!
sleep "${RUN_SH_STAGGER_SEC}"

echo "启动 multi_tilt_imu_node..."
ros2 launch cannode multi_tilt_imu.launch.py &
TILT_IMU_PID=$!
sleep "${RUN_SH_STAGGER_SEC}"

echo "启动 teleop2can_transformer..."
ros2 launch teleoptocantransformer teleop_converter_xiaosong.launch.py &
TELEOP_PID=$!
sleep "${RUN_SH_STAGGER_SEC}"

echo "启动 Danfoss 编码器节点..."
ros2 launch cannode danfoss_rotary_encoder.launch.py &
DANFOSS_PID=$!
sleep "${RUN_SH_LAST_GAP_SEC}"

echo "启动 chassis_to_mqtt 节点..."
ros2 launch mqtt_updater chassis_to_mqtt.launch.py &
MQTT_UPDATER_PID=$!
sleep "${RUN_SH_STAGGER_SEC}"

echo "启动 CGI-430 CAN 驱动..."
ros2 launch chcnav demo_0.launch.py &
CGI430_PID=$!
sleep "${RUN_SH_STAGGER_SEC}"

echo "启动 engine_feedback_node（发动机/VCU 反馈解析节点）..."
ros2 launch cannode engine_feedback.launch.py &
ENGINE_FEEDBACK_PID=$!

# 4) 等待所有进程
echo ""
if [ "$CANNODE_DDS_MODE" = "eth" ]; then
    echo "全车 DDS: Cyclone + ${CANNODE_DDS_IF}"
else
    echo "全车 DDS: 本机（ROS_LOCALHOST_ONLY=1 + ${RMW_IMPLEMENTATION}）"
fi
echo "所有节点已启动："
echo "  - rslidar_sdk (PID: $RSLIDAR_PID)"
echo "  - lidar_perception / point_grid (PID: $LIDAR_PERCEPTION_PID)"
echo "  - lidar_perception / lidar_arm_bucket_2d_grid (PID: $LIDAR_ARM_BUCKET_2D_GRID_PID)"
echo "  - cannode (PID: $CANNODE_PID)"
echo "  - multi_tilt_imu_node (PID: $TILT_IMU_PID)"
echo "  - teleop2can_transformer (PID: $TELEOP_PID)"
echo "  - danfoss_rotary_encoder (PID: $DANFOSS_PID)"
echo "  - chassis_to_mqtt (PID: $MQTT_UPDATER_PID)"
echo "  - CGI-430 (PID: $CGI430_PID)"
echo "  - engine_feedback_node (PID: $ENGINE_FEEDBACK_PID)"
echo ""
echo "其它终端查看 /rslidar_points 请先执行（与本次 run.sh 一致）:"
if [ "$CANNODE_DDS_MODE" = "eth" ]; then
    echo "  export CYCLONEDDS_URI=${CYCLONEDDS_URI}"
else
    echo "  export ROS_LOCALHOST_ONLY=1"
    echo "  unset CYCLONEDDS_URI"
fi
echo "  export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
echo "  ros2 daemon stop   # 建议：避免 daemon 用错 DDS 缓存"
echo "  ros2 topic hz /rslidar_points"
echo ""
echo "按 Ctrl+C 停止所有节点"

trap "echo '正在停止所有节点...'; kill $CANNODE_PID $TILT_IMU_PID $TELEOP_PID $DANFOSS_PID $MQTT_UPDATER_PID $RSLIDAR_PID $LIDAR_PERCEPTION_PID $LIDAR_ARM_BUCKET_2D_GRID_PID $CGI430_PID $ENGINE_FEEDBACK_PID 2>/dev/null; rm -f \"$TMPDIR\"/launch_params_* /media/nvidia/00code/cannode_ws/launch_params_* 2>/dev/null; exit" INT TERM

wait
