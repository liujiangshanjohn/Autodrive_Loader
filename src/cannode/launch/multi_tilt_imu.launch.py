from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 通过 YAML 配置文件灵活配置 CAN 通道、波特率以及倾角仪数量
    pkg_share = get_package_share_directory('cannode')
    config_file = os.path.join(pkg_share, 'config', 'multi_tilt_imu', 'multi_tilt_imu.yaml')

    return LaunchDescription([
        Node(
            package='cannode',
            executable='multi_tilt_imu_node',
            name='multi_tilt_imu_node',
            output='screen',
            # 直接加载 YAML 参数文件
            parameters=[config_file],
        )
    ])

