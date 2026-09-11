from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('loader_autonomy')
    cfg = os.path.join(pkg, 'config', 'autonomy.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_vehicle_command',
            default_value='false',
            description='If true, control may publish /vehicle_command (not implemented in stub)',
        ),
        Node(
            package='loader_autonomy',
            executable='perception_node',
            name='loader_perception_node',
            output='screen',
            parameters=[cfg],
        ),
        Node(
            package='loader_autonomy',
            executable='localization_node',
            name='loader_localization_node',
            output='screen',
            parameters=[cfg],
        ),
        Node(
            package='loader_autonomy',
            executable='mission_node',
            name='loader_mission_node',
            output='screen',
            parameters=[cfg],
        ),
        Node(
            package='loader_autonomy',
            executable='planning_node',
            name='loader_planning_node',
            output='screen',
            parameters=[cfg],
        ),
        Node(
            package='loader_autonomy',
            executable='control_node',
            name='loader_control_node',
            output='screen',
            parameters=[
                cfg,
                {'enable_vehicle_command': LaunchConfiguration('enable_vehicle_command')},
            ],
        ),
    ])
