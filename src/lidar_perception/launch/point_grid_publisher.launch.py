from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("lidar_perception")
    params = os.path.join(pkg_share, "config", "bucket_point_grid_params.yaml")

    return LaunchDescription(
        [
            Node(
                package="lidar_perception",
                executable="bucket_point_grid_publisher",
                name="bucket_point_grid_publisher",
                output="screen",
                parameters=[params],
            ),
        ]
    )

