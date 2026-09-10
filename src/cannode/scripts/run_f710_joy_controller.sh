#!/usr/bin/env bash
# F710 摇杆控大臂/小臂/铲斗：本脚本会启动 joy_node + 控制节点。需 cannode 在跑。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

if [ -z "${ROS_DISTRO:-}" ]; then
  export ROS_DISTRO=humble
fi
# ROS setup.bash 会读未定义的 AMENT_TRACE_SETUP_FILES，与 set -u 冲突
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [ -f "${WS_DIR}/install/setup.bash" ]; then
  source "${WS_DIR}/install/setup.bash"
fi
set -u

export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION="${PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION:-python}"

JOY_PID=""
cleanup() {
  if [ -n "${JOY_PID}" ] && kill -0 "${JOY_PID}" 2>/dev/null; then
    kill "${JOY_PID}" 2>/dev/null || true
    wait "${JOY_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "F710：摇杆=臂  LB/LT=铲斗  X/B=回转  Y/A=右履带  十字上/下=左履带  RB=臂死人手  Start=遥控"

if pgrep -f '[j]oy_node' >/dev/null 2>&1; then
  echo "检测到已有 joy_node，不再重复启动"
else
  echo "启动 joy_node..."
  ros2 run joy joy_node &
  JOY_PID=$!
  sleep 0.5
  if ! kill -0 "${JOY_PID}" 2>/dev/null; then
    echo "joy_node 启动失败"
    exit 1
  fi
fi

cd "${SCRIPT_DIR}"
python3 f710_joy_controller.py "$@"
