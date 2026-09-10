from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 通过 YAML 配置文件灵活配置 CAN 通道、波特率以及编码器数量
    pkg_share = get_package_share_directory('cannode')
    config_file = os.path.join(pkg_share, 'config', 'danfoss_rotary_encoder', 'danfoss_rotary_encoder.yaml')

    return LaunchDescription([
        Node(
            package='cannode',
            executable='danfoss_rotary_encoder_node',
            name='danfoss_rotary_encoder_node',
            output='screen',
            # 直接加载 YAML 参数文件
            parameters=[config_file],
        )
    ])

