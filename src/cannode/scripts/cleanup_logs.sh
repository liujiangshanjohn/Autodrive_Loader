#!/usr/bin/env bash
set -euo pipefail

# 清理 ROS2 节点/进程产生的日志文件：
# 默认清理以下目录中最近 N 天的“普通文件”（regular file）：
#   - /media/nvidia/00code/cannode_ws/log
#   - /media/nvidia/00code/logs
#   - $HOME/.ros/log（当前用户；若经 sudo -n 提权则取调用用户的主目录）
# 规则：
# - 删除目录内所有类型为 file 且修改时间超过 N 天的文件
# - 删除删除后留下的且“修改时间超过 N 天”的空目录
#
# 用法示例：
#   ./cleanup_logs.sh --dry-run
#   ./cleanup_logs.sh
#

# 若命令行未指定 --log-dir，则使用 DEFAULT_LOG_DIRS；指定一次或多次则只清理所列目录
LOG_DIRS=()
DAYS=3
MODE="days" # days=按天数保留；today=仅保留当天（00:00 起）
DRY_RUN=0
SUDO_BIN="$(command -v sudo 2>/dev/null || true)"
# 与 run.sh（echo nvidia | sudo -S）一致：默认密码 nvidia；可用 export CANNODE_SUDO_PASSWORD 覆盖
CANNODE_SUDO_PASSWORD="${CANNODE_SUDO_PASSWORD:-nvidia}"

# 非 root 时整脚本提权为 root：优先 sudo -n；否则 sudo -S（与 run.sh 的 echo … | sudo -S 等价）
# 这样 root 属主日志、find -delete 均可处理，且 SUDO_USER 仍指向原用户供 ~/.ros/log 路径
if [[ "$(id -u)" -ne 0 ]] && [[ -n "${SUDO_BIN}" ]]; then
  if sudo -n true 2>/dev/null; then
    exec sudo -n /usr/bin/env bash "$0" "$@"
  else
    exec sudo -S /usr/bin/env bash "$0" "$@" <<< "${CANNODE_SUDO_PASSWORD}"
  fi
fi

# sudo 提权后 HOME 常为 /root；~/.ros/log 应对齐实际调用用户的主目录
EFFECTIVE_HOME="${HOME}"
if [[ "$(id -u)" -eq 0 ]] && [[ -n "${SUDO_USER:-}" ]]; then
  EFFECTIVE_HOME="$(getent passwd "$SUDO_USER" | cut -d: -f6)"
  [[ -z "${EFFECTIVE_HOME}" ]] && EFFECTIVE_HOME="${HOME}"
fi

DEFAULT_LOG_DIRS=(
  "/media/nvidia/00code/cannode_ws/log"
  "/media/nvidia/00code/logs"
  "${EFFECTIVE_HOME}/.ros/log"
)

usage() {
  cat <<EOF
Usage: $0 [--log-dir DIR] ... [--days N] [--dry-run] [-h|--help]

Options:
  --log-dir DIR   日志目录（可重复；默认 cannode_ws/log、00code/logs、~/.ros/log）
  --days N        保留最近 N 天（默认：$DAYS）
  --keep-today    仅保留当天（默认：$MODE）
  --dry-run       仅打印将删除的文件，不实际删除
  -h, --help      显示帮助
EOF
}

can_sudo_non_interactive() {
  [[ -n "${SUDO_BIN}" ]] && sudo -n true 2>/dev/null
}

rm_rf_maybe_sudo() {
  local path="$1"
  if [[ "$DRY_RUN" -eq 1 ]]; then
    echo "$path"
    return 0
  fi

  # 先直接删除（权限足够时）
  rm -rf -- "$path" 2>/dev/null && return 0

  # 权限不足：先 sudo -n，再与 run.sh 相同用 sudo -S
  if [[ -n "${SUDO_BIN}" ]] && [[ "$(id -u)" -ne 0 ]]; then
    if can_sudo_non_interactive; then
      sudo -n rm -rf -- "$path" 2>/dev/null && return 0
    fi
    echo "${CANNODE_SUDO_PASSWORD}" | sudo -S rm -rf -- "$path" 2>/dev/null && return 0
  fi

  echo "WARN: 无权限删除，请检查 sudo 或修正属主：$path" >&2
  return 0
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --log-dir)
      LOG_DIRS+=("${2:-}")
      shift 2
      ;;
    --days)
      DAYS="${2:-}"
      MODE="days"
      shift 2
      ;;
    --keep-today)
      MODE="today"
      shift
      ;;
    --dry-run)
      DRY_RUN=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage
      exit 2
      ;;
  esac
done

if [[ ${#LOG_DIRS[@]} -eq 0 ]]; then
  LOG_DIRS=("${DEFAULT_LOG_DIRS[@]}")
fi

if ! [[ "$DAYS" =~ ^[0-9]+$ ]] || [[ "$DAYS" -lt 0 ]]; then
  echo "Error: --days 需要非负整数，当前：$DAYS" >&2
  exit 1
fi

cleanup_one_dir() {
  local LOG_DIR="$1"

  echo "----------"
  echo "准备清理日志目录：$LOG_DIR"
  if [[ "$DRY_RUN" -eq 1 ]]; then
    echo "模式：DRY RUN（不删除，只打印）"
  else
    echo "模式：实际删除"
  fi

  if [[ "$MODE" == "today" ]]; then
    TODAY="$(date +%F)" # yyyy-mm-dd（隐含 00:00:00）
    echo "保留当天（${TODAY} 00:00 起）（删除早于当天的普通文件）"

    if [[ "$DRY_RUN" -eq 1 ]]; then
      find "$LOG_DIR" -type f ! -newermt "$TODAY" -print
    else
      find "$LOG_DIR" -type f ! -newermt "$TODAY" -print -delete
    fi
  else
    echo "保留最近 $DAYS 天（删除修改时间超过 $DAYS 天的普通文件）"

    # 先删除“旧目录树”，避免先删文件导致目录 mtime 被刷新而残留
    # 只处理 LOG_DIR 的直接子目录（ROS2 常见是一层日期目录），并且不跟随 symlink。
    while IFS= read -r d; do
      rm_rf_maybe_sudo "$d"
    done < <(find "$LOG_DIR" -mindepth 1 -maxdepth 1 -type d -mtime +"$DAYS" -print)

    while IFS= read -r f; do
      rm_rf_maybe_sudo "$f"
    done < <(find "$LOG_DIR" -type f -mtime +"$DAYS" -print)
  fi

  # 清理老的空目录（避免删除活跃目录；同时保证不会动到 symlink）
  if [[ "$MODE" == "today" ]]; then
    # 仅保留当天时，如果目录已经是“空的”，直接清掉即可；
    # 目录的 mtime 可能在日志删除/创建后被更新过，因此不再以 mtime 作为空目录删除条件。
    if [[ "$DRY_RUN" -eq 1 ]]; then
      find "$LOG_DIR" -mindepth 1 -type d -empty -print
    else
      find "$LOG_DIR" -mindepth 1 -type d -empty -print -delete
    fi
  else
    # days 模式下：空目录直接清理，不再依赖目录 mtime（因为删文件会刷新 mtime）
    while IFS= read -r d; do
      rm_rf_maybe_sudo "$d"
    done < <(find "$LOG_DIR" -mindepth 1 -type d -empty -print)
  fi

  echo "目录处理完毕：$LOG_DIR"
}

ANY_OK=0
for LOG_DIR in "${LOG_DIRS[@]}"; do
  [[ -z "$LOG_DIR" ]] && continue
  if [[ ! -d "$LOG_DIR" ]]; then
    echo "WARN: 跳过不存在的目录：$LOG_DIR" >&2
    continue
  fi
  ANY_OK=1
  cleanup_one_dir "$LOG_DIR"
done

if [[ "$ANY_OK" -eq 0 ]]; then
  echo "Error: 没有可清理的目录（请检查路径或创建默认目录）。" >&2
  exit 1
fi

echo "全部清理完成。"

