from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleoptocantransformer',
            executable='teleop2can_transformer_loader',
            name='teleop2can_transformer_loader',
            output='screen',
            parameters=[{
                'steering_deadzone': 0.05,
                'throttle_deadzone': 0.05,
                'brake_deadzone': 0.05,
                'boom_deadzone': 0.05,
                'bucket_deadzone': 0.05,
                'teleop_timeout_sec': 0.5,
                'verbose_log': True,
            }],
        )
    ])
