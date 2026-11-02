from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """启动 cannode + 装载机 teleop 转换 + CAN 监控。

    键盘遥控请另开终端：
      python3 src/teleoptocantransformer/scripts/keyboard_loader_teleop.py
    """
    teleop_share = get_package_share_directory('teleoptocantransformer')
    cannode_share = get_package_share_directory('cannode')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(cannode_share, 'launch', 'cannode.launch.py')
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(teleop_share, 'launch', 'teleop_converter_loader.launch.py')
            )
        ),
        Node(
            package='teleoptocantransformer',
            executable='loader_can_monitor.py',
            name='loader_can_monitor',
            output='screen',
        ),
    ])
