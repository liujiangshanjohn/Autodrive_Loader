from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix, get_package_share_directory
import os

def generate_launch_description():

    log_dir = os.getenv('ROS_LOG_DIR')
    if log_dir is None:
        log_dir = os.path.join(get_package_prefix('cannode'), 'log')
    # print("Task Management Configuration File:", config_file)
    # 获取 cannode 包的共享目录，config 文件安装在这里
    package_share_dir = get_package_share_directory('cannode')
    config_path = os.path.join(package_share_dir, 'config')
    return LaunchDescription([
        # 启动C++版本的节点
        Node(
            package='cannode',
            executable='cannode_exe',
            name='cannode',
            output='screen',
            arguments=[ '-l', "4", '-o', log_dir ],
            cwd=config_path,
        ),

    ]) 
