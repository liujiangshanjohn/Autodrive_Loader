import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('cannode')
    params = os.path.join(pkg, 'config', 'engine_feedback.yaml')
    return LaunchDescription([
        Node(
            package='cannode',
            executable='engine_feedback_node',
            name='engine_feedback_node',
            output='screen',
            parameters=[params],
        ),
    ])
