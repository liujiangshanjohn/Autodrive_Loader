#!/usr/bin/env bash
# tmux 仿真一键启动：cannode(run_sim) + cam_geac(run_sim) + excavator(run_sim) + yolo（可选）
# 与 start_all_tmux.sh 用法一致，默认使用独立 tmux 会话名，避免与实车会话冲突。
#
# 用法：
#   ./sim_start_all_tmux.sh start
#   ./sim_start_all_tmux.sh start_with_delay
#   ./sim_start_all_tmux.sh attach
#   ./sim_start_all_tmux.sh stop
#   ./sim_start_all_tmux.sh status
#
# 可选环境变量：
#   SESSION=excavator_sim          # 默认 excavator_sim
#   FORCE_RESTART=0|1
#   TMUX_CANNODE_START_DELAY_SEC
#   TMUX_ALL_BOOT_DELAY_SEC        # 仅 start_with_delay，默认 30
#   ENABLE_YOLO=0|1              # 默认 1；设为 0 不启 yolo 窗口
#   EXCAVATOR_ROS_BRIDGE=0|1     # 默认 0。若 excavator 侧已 colcon 出 ros2_ws，可设 1 并设 EXCAVATOR_ROS2_WS

set -euo pipefail

SESSION="${SESSION:-excavator_sim}"
FORCE_RESTART="${FORCE_RESTART:-0}"
TMUX_CANNODE_START_DELAY_SEC="${TMUX_CANNODE_START_DELAY_SEC:-0}"
ENABLE_YOLO="${ENABLE_YOLO:-1}"
EXCAVATOR_ROS_BRIDGE="${EXCAVATOR_ROS_BRIDGE:-0}"
EXCAVATOR_ROS2_WS="${EXCAVATOR_ROS2_WS:-/media/nvidia/00code/excavator/ros2_ws}"
ACTION="${1:-start}"

CANNODE_SCRIPTS="/media/nvidia/00code/cannode_ws/src/cannode/scripts"
EXC_SCRIPTS="/media/nvidia/00code/excavator/scripts"
CAM_SCRIPTS="/media/nvidia/00code/cam_geac/scripts"
YOLO_WS="/media/nvidia/00code/yolo_detection_ws"

run_privileged() {
  if [ "$(id -u)" -eq 0 ]; then
    "$@"
  else
    echo nvidia | sudo -S "$@"
  fi
}

need_tmux() {
  if ! command -v tmux >/dev/null 2>&1; then
    echo "❌ 未找到 tmux，请先安装：sudo apt-get update && sudo apt-get install -y tmux" >&2
    exit 1
  fi
}

has_session() {
  tmux has-session -t "${SESSION}" >/dev/null 2>&1
}

attach() {
  tmux attach -t "${SESSION}"
}

case "${ACTION}" in
  status)
    need_tmux
    if has_session; then
      echo "✅ tmux session '${SESSION}' 存在"
      tmux ls | sed -n "1,5p" || true
      exit 0
    fi
    echo "ℹ️ tmux session '${SESSION}' 不存在"
    exit 1
    ;;
  stop)
    need_tmux
    if has_session; then
      tmux kill-session -t "${SESSION}"
      echo "✅ 已停止 tmux session '${SESSION}'"
    else
      echo "ℹ️ tmux session '${SESSION}' 不存在，无需停止"
    fi
    exit 0
    ;;
  attach)
    need_tmux
    if ! has_session; then
      echo "❌ tmux session '${SESSION}' 不存在，先执行：./sim_start_all_tmux.sh start 或 start_with_delay" >&2
      exit 1
    fi
    attach
    ;;
  start_with_delay)
    _TMUX_BOOT_DELAY_DEFAULT=30
    ;&
  start|"")
    _TMUX_BOOT_DELAY_DEFAULT="${_TMUX_BOOT_DELAY_DEFAULT:-0}"
    TMUX_ALL_BOOT_DELAY_SEC="${TMUX_ALL_BOOT_DELAY_SEC:-${_TMUX_BOOT_DELAY_DEFAULT}}"
    need_tmux
    if has_session; then
      if [ "${FORCE_RESTART}" = "1" ]; then
        tmux kill-session -t "${SESSION}"
      else
        echo "ℹ️ tmux session '${SESSION}' 已存在，直接 attach（如需重建：FORCE_RESTART=1 ./sim_start_all_tmux.sh start）"
        attach
        exit 0
      fi
    fi

    if [ "${TMUX_ALL_BOOT_DELAY_SEC}" -gt 0 ] 2>/dev/null; then
      echo "⏳ TMUX_ALL_BOOT_DELAY_SEC=${TMUX_ALL_BOOT_DELAY_SEC}，等待后再创建 tmux 会话（仿真）..."
      sleep "${TMUX_ALL_BOOT_DELAY_SEC}"
    fi

    run_privileged -v >/dev/null 2>&1 || true

    # 仿真：cam_geac run_sim 会起 ros_socket_bridge，与 excavator_bridge 二选一；默认不启 excavator_bridge
    if [ "${EXCAVATOR_ROS_BRIDGE}" = "1" ]; then
      run_privileged pkill -f ros_socket_bridge_node >/dev/null 2>&1 || true
      run_privileged pkill -f ros_gst_bridge_node >/dev/null 2>&1 || true
      run_privileged rm -f \
        /tmp/excavator_telemetry.sock \
        /tmp/excavator_control.sock \
        >/dev/null 2>&1 || true
    fi

    # Window 0: cannode 仿真（bag + run_sim）
    tmux new-session -d -s "${SESSION}" -n "cannode_sim" \
      "bash -lc 'sleep \"${TMUX_CANNODE_START_DELAY_SEC}\"; cd \"${CANNODE_SCRIPTS}\" && bash -lc \"./run_sim.sh; exec bash\"'"

    sleep 1
    # Window 1: Excavator 网关仿真（信令见 excavator/scripts/run_sim.sh）
    tmux new-window -t "${SESSION}" -n "excavator_gw_sim" \
      "bash -lc 'cd \"${EXC_SCRIPTS}\" && ./run_sim.sh; exec bash'"

    sleep 1
    # Window 2: cam_geac（勿在反斜杠续行与命令字符串之间插入整行注释，否则 tmux 收不到命令、下一行会被当可执行文件名）
    # YOLO 读 /dev/shm/excavator_cam_*_raw，需 CAM_GEAC_SHM_RAW=1，否则行人检测无图
    tmux new-window -t "${SESSION}" -n "cam_geac_sim" \
      "bash -lc 'export CAM_GEAC_SHM_RAW=\"\${CAM_GEAC_SHM_RAW:-1}\"; cd \"${CAM_SCRIPTS}\" && ./run_isx031.sh; exec bash'"
    # if [ "${EXCAVATOR_ROS_BRIDGE}" = "1" ] && [ -f "${EXCAVATOR_ROS2_WS}/install/setup.bash" ]; then
    #   sleep 1
    #   tmux new-window -t "${SESSION}" -n "excavator_ros_bridge" \
    #     "bash -lc 'export ROS_LOG_DIR=\"\${ROS_LOG_DIR:-/media/nvidia/00code/cannode_ws/log}\" && source \"${EXCAVATOR_ROS2_WS}/install/setup.bash\" && ros2 run excavator_bridge ros_gst_bridge_node --ros-args -p device_id:=excavator-001; exec bash'"
    # elif [ "${EXCAVATOR_ROS_BRIDGE}" = "1" ]; then
    #   echo "⚠️ EXCAVATOR_ROS_BRIDGE=1 但未找到 ${EXCAVATOR_ROS2_WS}/install/setup.bash，跳过 ros_gst_bridge_node 窗口" >&2
    # fi

    if [ "${ENABLE_YOLO}" != "0" ]; then
      sleep 1
      tmux new-window -t "${SESSION}" -n "yolo_det" \
        "bash -lc 'cd \"${YOLO_WS}\" && ./run_test1_online_detection.sh; exec bash'"
    fi

    tmux select-window -t "${SESSION}:cannode_sim" || true

    echo "✅ 已启动仿真 tmux session '${SESSION}'"
    echo "   attach: ./sim_start_all_tmux.sh attach"
    echo "   stop:   ./sim_start_all_tmux.sh stop"
    attach
    ;;
  *)
    echo "用法: $0 {start|start_with_delay|attach|stop|status}" >&2
    exit 2
    ;;
esac
