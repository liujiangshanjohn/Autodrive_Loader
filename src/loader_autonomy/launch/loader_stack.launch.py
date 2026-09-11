from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    autonomy_share = get_package_share_directory('loader_autonomy')
    camera_share = get_package_share_directory('camera_driver')

    return LaunchDescription([
        DeclareLaunchArgument('with_camera_stub', default_value='true'),
        DeclareLaunchArgument('with_mqtt', default_value='false'),
        DeclareLaunchArgument('with_teleop_bypass', default_value='false'),
        DeclareLaunchArgument('enable_vehicle_command', default_value='false'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(autonomy_share, 'launch', 'autonomy.launch.py')
            ),
            launch_arguments={
                'enable_vehicle_command': LaunchConfiguration('enable_vehicle_command'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(camera_share, 'launch', 'camera_driver.launch.py')
            ),
            condition=IfCondition(LaunchConfiguration('with_camera_stub')),
        ),
        # Existing stacks (optional):
        #   ros2 launch cannode cannode.launch.py
        #   ros2 launch mqtt_updater chassis_to_mqtt.launch.py
        #   ros2 launch teleoptocantransformer teleop_converter.launch.py
    ])
