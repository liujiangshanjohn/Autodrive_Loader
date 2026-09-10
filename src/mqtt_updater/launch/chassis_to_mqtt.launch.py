from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mqtt_updater',
            executable='chassis_to_mqtt',
            name='chassis_to_mqtt',
            output='screen',
        ),
    ])
