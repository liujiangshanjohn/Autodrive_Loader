#!/bin/bash
# ============================================
# 兜底清理：停止 run_sim.sh 启动的 ROS2 进程
# 使用: ./kill_sim.sh
# ============================================

set -u

echo "🛑 正在停止 run_sim.sh 相关进程..."

echo "   [1/4] 停止 ros2 bag play（回放）..."
pkill -f "ros2 bag play" 2>/dev/null && echo "        ✅ 已停止" || echo "        ⏭️  未运行"

echo "   [2/4] 停止 cannode launch..."
pkill -f "ros2 launch cannode cannode\\.launch\\.py" 2>/dev/null && echo "        ✅ 已停止" || echo "        ⏭️  未运行"

echo "   [3/4] 停止 teleop2can_transformer launch..."
pkill -f "ros2 launch teleoptocantransformer teleop_converter_xiaosong\\.launch\\.py" 2>/dev/null && echo "        ✅ 已停止" || echo "        ⏭️  未运行"

echo "   [4/4] 停止 mqtt_updater（chassis_to_mqtt）..."
pkill -f "ros2 run mqtt_updater chassis_to_mqtt" 2>/dev/null && echo "        ✅ 已停止" || echo "        ⏭️  未运行"

echo ""
echo "🧹 清理 launch 参数临时文件..."
rm -f "${TMPDIR:-/tmp}"/launch_params_* /media/nvidia/00code/cannode_ws/launch_params_* 2>/dev/null \
  && echo "   ✅ 已清理" || echo "   ⏭️  无需清理"

echo ""
echo "✅ 清理完成"

