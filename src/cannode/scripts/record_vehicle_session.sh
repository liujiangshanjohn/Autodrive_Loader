#!/usr/bin/env bash
# 录制整车相关 ROS2 话题到 bag，并并行抓取 can0/can1 上的 CAN 文本日志（内核可见的收/发帧，含本机 TX 回显时也会出现在 candump 中）。
# 数据目录：/media/nvidia/data/record_<UTC时间戳>/
#
# 用法（先 source 工作空间）:
#   source /media/nvidia/00code/cannode_ws/install/setup.bash
#   /media/nvidia/00code/cannode_ws/src/cannode/scripts/record_vehicle_session.sh
# 或安装后:
#   bash $(ros2 pkg prefix cannode)/share/cannode/scripts/record_vehicle_session.sh
#
# 环境变量:
#   DATA_ROOT   根目录，默认 /media/nvidia/data
#   SUDO_CAN=1  若当前用户无权限读 CAN，则用 sudo 启动 candump（需免密或终端输入密码）
#   EXTRA_BAG_ARGS  追加传给 ros2 bag record 的参数（如 -s mcap）

set -euo pipefail

DATA_ROOT="${DATA_ROOT:-/media/nvidia/data}"
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
OUT="${DATA_ROOT}/record_${STAMP}"
mkdir -p "${OUT}"

# 若未 source ROS，尝试常见工作空间
if ! command -v ros2 &>/dev/null; then
  for setup in \
    "/media/nvidia/00code/cannode_ws/install/setup.bash" \
    "${CANNODE_WS:-}/install/setup.bash"
  do
    if [[ -f "${setup}" ]]; then
      # shellcheck source=/dev/null
      source "${setup}"
      break
    fi
  done
fi
if ! command -v ros2 &>/dev/null; then
  echo "错误: 未找到 ros2，请先 source install/setup.bash" >&2
  exit 1
fi

TOPICS=(
  /can_frame_sent
  /cannode/chassis_feedback
  /cannode/engine_feedback
  /cannode/vcu_feedback
  /chassis_status
  /chcnav/devimu
  /chcnav/devpvt
  /controls/teleop
  /cyber_msgs/lidar_2d_points
  /cyber_msgs/object2d
  /cyber_msgs/point_grid
  /excavator/hud_layout_cmd
  /excavator/network_qos
  /excavator_joint_angles
  /lidar_arm_bucket/xz_profile
  /parameter_events
  /perception/object_detection
  /rotary_encoder_angle
  /rslidar_points
  /shm_cam_0/image_compressed
  /shm_cam_1/image_compressed
  /shm_cam_2/image_compressed
  /shm_cam_3/image_compressed
  /shm_cam_4/image_compressed
  /shm_cam_5/image_compressed
  /shm_cam_6/image_compressed
  /shm_cam_7/image_compressed
  /tf
  /tf_static
  /tilt_body
  /tilt_boom
  /tilt_bucket
  /tilt_stick
)

CAN0_PID=""
CAN1_PID=""

_can_dump() {
  local iface="$1"
  local logf="$2"
  if ! ip link show "${iface}" &>/dev/null; then
    echo "[${iface}] 接口不存在，跳过 candump" >>"${OUT}/can_record.log"
    echo "[${iface}] 接口不存在，跳过 candump" >&2
    return 0
  fi
  if ! command -v candump &>/dev/null; then
    echo "未安装 can-utils 的 candump，跳过 CAN 记录。安装: sudo apt install can-utils" >>"${OUT}/can_record.log"
    echo "未安装 can-utils 的 candump，跳过 CAN 记录" >&2
    return 0
  fi
  # -t A: 绝对时间戳；-L: 可解析日志格式（一行一帧）
  if [[ "${SUDO_CAN:-0}" == "1" ]]; then
    sudo candump -t A -L "${iface}" >"${logf}" 2>>"${OUT}/can_record.log" &
  else
    candump -t A -L "${iface}" >"${logf}" 2>>"${OUT}/can_record.log" &
  fi
  echo $!
}

cleanup() {
  if [[ -n "${CAN0_PID}" ]]; then kill "${CAN0_PID}" 2>/dev/null || true; wait "${CAN0_PID}" 2>/dev/null || true; fi
  if [[ -n "${CAN1_PID}" ]]; then kill "${CAN1_PID}" 2>/dev/null || true; wait "${CAN1_PID}" 2>/dev/null || true; fi
}
trap cleanup EXIT INT TERM

{
  echo "开始时间(本地): $(date -Is)"
  echo "输出目录: ${OUT}"
  echo "ROS_DISTRO=${ROS_DISTRO:-}"
  echo "话题数: ${#TOPICS[@]}"
  echo "candump: can0->can0.log can1->can1.log（需接口 UP；TX 是否出现取决于接口 loopback 等）"
} | tee "${OUT}/session_info.txt"

CAN0_PID="$(_can_dump can0 "${OUT}/can0.log")"
CAN1_PID="$(_can_dump can1 "${OUT}/can1.log")"

echo "按 Ctrl+C 结束录制（将停止 ros2 bag 与 candump）" | tee -a "${OUT}/session_info.txt"

# shellcheck disable=SC2086
ros2 bag record -o "${OUT}/ros2_bag" ${EXTRA_BAG_ARGS:-} "${TOPICS[@]}"

echo "已结束。bag 前缀: ${OUT}/ros2_bag" | tee -a "${OUT}/session_info.txt"
