#!/usr/bin/env bash
# tmux 一键启动：cannode + 相机驱动 + Excavator 网关 + yolo 检测 + 雷达点云时间戳对齐
#
# 用法：
#   ./start_all_tmux.sh start              # 立即创建/复用会话并启动各窗口
#   ./start_all_tmux.sh start_with_delay   # 自启动用：先等待再创建会话（默认 30s，见 TMUX_ALL_BOOT_DELAY_SEC）
#   ./start_all_tmux.sh attach             # 仅 attach
#   ./start_all_tmux.sh stop    # 停止整个会话（等价 kill 所有窗口进程）
#   ./start_all_tmux.sh status  # 查看会话是否存在
#
# 可选环境变量：
#   SESSION=excavator_all   # tmux 会话名（默认 excavator_all）
#   FORCE_RESTART=0|1       # start 时若会话已存在，1=先 kill 再重建；0=直接 attach（默认 0）
#   TMUX_CANNODE_START_DELAY_SEC  # 仅推迟 cannode 窗口内 run.sh（秒，默认 0；自启动可试 5～15）
#   TMUX_ALL_BOOT_DELAY_SEC       # 仅 start_with_delay：创建会话前等待秒数（未设置时默认 30）；start 不使用此项

set -euo pipefail

SESSION="${SESSION:-excavator_all}"
FORCE_RESTART="${FORCE_RESTART:-0}"
# 自启动时可设为非 0：仅推迟 cannode 窗口里 run.sh 的开始，给挂载/网络/mttcan 一点时间
TMUX_CANNODE_START_DELAY_SEC="${TMUX_CANNODE_START_DELAY_SEC:-0}"
ACTION="${1:-start}"

# 统一 sudo 处理方式：与 cannode/scripts/run.sh 一致
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
      echo "❌ tmux session '${SESSION}' 不存在，先执行：./start_all_tmux.sh start 或 start_with_delay" >&2
      exit 1
    fi
    attach
    ;;
  start_with_delay)
    # 与 start 共用逻辑；默認先等 30s（可用 TMUX_ALL_BOOT_DELAY_SEC 覆蓋）
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
        echo "ℹ️ tmux session '${SESSION}' 已存在，直接 attach（如需重建：FORCE_RESTART=1 ./start_all_tmux.sh start）"
        attach
        exit 0
      fi
    fi

    if [ "${TMUX_ALL_BOOT_DELAY_SEC}" -gt 0 ] 2>/dev/null; then
      echo "⏳ TMUX_ALL_BOOT_DELAY_SEC=${TMUX_ALL_BOOT_DELAY_SEC}，等待后再创建 tmux 会话..."
      sleep "${TMUX_ALL_BOOT_DELAY_SEC}"
    fi

    # cam_geac/isx031 需要 sudo 操作 /sys/class/tz_gpio 与 /dev/ttyTHS*
    # 预先刷新 sudo ticket，避免在 tmux 窗口里卡住等待密码
    run_privileged -v >/dev/null 2>&1 || true

    # 避免旧进程占用 /tmp/excavator_telemetry.sock 导致 ros_gst_bridge_node 启动失败
    # cam_geac 的 ros_socket_bridge_node 与 excavator_bridge 的 ros_gst_bridge_node 会争用同一组 /tmp/excavator_*.sock
    run_privileged pkill -f ros_socket_bridge_node >/dev/null 2>&1 || true
    run_privileged pkill -f ros_gst_bridge_node >/dev/null 2>&1 || true
    # 清理 domain socket，避免 “Failed to bind Unix socket: Address already in use”
    run_privileged rm -f \
      /tmp/excavator_telemetry.sock \
      /tmp/excavator_control.sock \
      /tmp/excavator_audio_up.sock \
      /tmp/excavator_audio_down.sock \
      /tmp/excavator_video_control.sock \
      /tmp/excavator_video_hud_composite.sock \
      >/dev/null 2>&1 || true

    # Window 0: cannode
    tmux new-session -d -s "${SESSION}" -n "cannode" \
      "bash -lc 'sleep \"${TMUX_CANNODE_START_DELAY_SEC}\"; cd /media/nvidia/00code/cannode_ws/src/cannode/scripts && bash -lc \"./run.sh; exec bash\"'"
    
    sleep 1
    # Window 2: excavator gateway
    tmux new-window -t "${SESSION}" -n "excavator_gw" \
      "cd /media/nvidia/00code/excavator/scripts && bash -lc './run.sh; exec bash'"

    sleep 1
    # Window 1: camera driver (isx031 + socket bridge)
    tmux new-window -t "${SESSION}" -n "cam_ar0233" \
      "cd /media/nvidia/00code/cam_geac/scripts && bash -lc 'export CAM_GEAC_SHM_RAW=\"\${CAM_GEAC_SHM_RAW:-1}\"; ./run_ar0233.sh; exec bash'"

    sleep 1
    # # Window 3: excavator ROS bridge (ROS -> /tmp/excavator_{telemetry,control}.sock -> Go -> WebRTC DataChannel)
    # tmux new-window -t "${SESSION}" -n "excavator_ros_bridge" \
    #   "bash -lc 'LOG_ROOT=/media/nvidia/00code/logs; mkdir -p \"\$LOG_ROOT\"; TS=\$(date +\"%Y%m%d_%H%M%S\"); LOG_FILE=\"\$LOG_ROOT/excavator_ros_bridge_\${TS}.log\"; echo \"===== START \$(date -Is) =====\" | tee -a \"\$LOG_FILE\"; AWK_BIN=\"${AWK_BIN:-}\"; if [ -z \"\$AWK_BIN\" ]; then if command -v gawk >/dev/null 2>&1; then AWK_BIN=gawk; else AWK_BIN=awk; fi; fi; cd /media/nvidia/00code/camera_code/excavator && export ROS_LOG_DIR=\"${ROS_LOG_DIR:-/media/nvidia/00code/cannode_ws/log}\" && source ros2_ws/install/setup.bash && ros2 run excavator_bridge ros_gst_bridge_node --ros-args -p device_id:=excavator-001 2>&1 | \"\$AWK_BIN\" '{ print strftime(\"[%Y-%m-%dT%H:%M:%S%z]\"), \$0; fflush(); }' | tee -a \"\$LOG_FILE\"; exec bash'"

    # Window 3: yolo online detection
    tmux new-window -t "${SESSION}" -n "yolo_det" \
      "cd /media/nvidia/00code/yolo_detection_ws && bash -lc './run_test1_online_detection.sh; exec bash'"

    # Window 4: 雷达点云 header 时间戳对齐（/rslidar_points -> /rslidar_points_fixed）
   # tmux new-window -t "${SESSION}" -n "lidar_ts_fix" \
    #  "bash -lc 'source /media/nvidia/00code/cannode_ws/install/setup.bash && ros2 run lidar_perception; exec bash'"

    # 让默认落在 cannode 窗口
    tmux select-window -t "${SESSION}:cannode" || true

    echo "✅ 已启动 tmux session '${SESSION}'（当前脚本创建的各窗口）"
    echo "   attach: ./start_all_tmux.sh attach"
    echo "   stop:   ./start_all_tmux.sh stop"
    attach
    ;;
  *)
    echo "用法: $0 {start|start_with_delay|attach|stop|status}" >&2
    exit 2
    ;;
esac

