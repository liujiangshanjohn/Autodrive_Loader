#!/bin/bash

# 小松挖掘机控制界面启动脚本

# 获取脚本所在目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WS_DIR="$( cd "$SCRIPT_DIR/../../.." && pwd )"

echo "=========================================="
echo "小松挖掘机控制界面启动脚本"
echo "=========================================="
echo "工作空间: $WS_DIR"
echo ""

# 检查 ROS2 环境
if [ -z "$ROS_DISTRO" ]; then
    echo "⚠️  警告: ROS_DISTRO 未设置，尝试设置为 humble"
    export ROS_DISTRO=humble
fi

# Source ROS2
if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    echo "✅ Source ROS2: /opt/ros/$ROS_DISTRO/setup.bash"
    source /opt/ros/$ROS_DISTRO/setup.bash
else
    echo "❌ 错误: 找不到 ROS2 安装目录 /opt/ros/$ROS_DISTRO/"
    echo "请先安装 ROS2 或设置正确的 ROS_DISTRO 环境变量"
    exit 1
fi

# Source 工作空间
if [ -f "$WS_DIR/install/setup.bash" ]; then
    echo "✅ Source 工作空间: $WS_DIR/install/setup.bash"
    source "$WS_DIR/install/setup.bash"
else
    echo "⚠️  警告: 工作空间未编译，尝试编译..."
    cd "$WS_DIR"
    colcon build --packages-select cannode
    if [ -f "$WS_DIR/install/setup.bash" ]; then
        source "$WS_DIR/install/setup.bash"
        echo "✅ 编译完成并 source"
    else
        echo "❌ 错误: 编译失败，请手动编译工作空间"
        exit 1
    fi
fi

echo ""
echo "=========================================="
echo "启动小松挖掘机控制界面..."
echo "=========================================="
echo ""

# 与 cannode 自带的 protobuf 生成代码兼容：~/.local 里 protobuf 4+ 会报
# "Descriptors cannot be created directly"，改用纯 Python 解析器即可。
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION="${PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION:-python}"

# 运行控制界面
cd "$SCRIPT_DIR"
python3 xiaosong_controller.py

