from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('deadman_button', default_value='5'),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
        ),
        Node(
            package='cannode',
            executable='f710_joy_controller.py',
            name='f710_joy_controller',
            output='screen',
            parameters=[{
                'deadman_button': LaunchConfiguration('deadman_button'),
            }],
        ),
    ])
