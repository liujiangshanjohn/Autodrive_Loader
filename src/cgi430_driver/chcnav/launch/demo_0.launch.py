"""包装 demo_0.xml，便于与其它模块统一使用 ros2 launch pkg xxx.launch.py。"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource


def generate_launch_description():
    pkg_share = get_package_share_directory('chcnav')
    xml_path = os.path.join(pkg_share, 'launch', 'demo_0.xml')
    return LaunchDescription([
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(xml_path),
        ),
    ])
