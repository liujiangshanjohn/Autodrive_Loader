import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('lidar_perception')
    params = os.path.join(pkg, 'config', 'lidar_arm_bucket_2d_grid.yaml')
    return LaunchDescription([
        Node(
            package='lidar_perception',
            executable='lidar_arm_bucket_2d_grid',
            name='lidar_arm_bucket_2d_grid',
            output='screen',
            parameters=[params],
        ),
    ])
