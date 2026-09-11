from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory('camera_driver'),
        'config',
        'camera_driver.yaml',
    )
    return LaunchDescription([
        Node(
            package='camera_driver',
            executable='camera_driver_node',
            name='camera_driver_node',
            output='screen',
            parameters=[cfg],
        ),
    ])
