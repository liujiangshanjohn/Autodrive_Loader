from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 默认参数文件（使用已安装的包路径；仅改 src 未 build 时 install 下 yaml 仍是旧内容）
    pkg_share = get_package_share_directory('teleoptocantransformer')
    xiaosong_params_default = os.path.join(pkg_share, 'config', 'teleop_xiaosong.yaml')

    xiaosong_params_arg = DeclareLaunchArgument(
        'xiaosong_params_file',
        default_value=xiaosong_params_default,
        description=(
            'teleop_xiaosong.yaml 绝对路径。默认用 install/share 下副本；调试可直接指向 '
            '工作空间 src/teleoptocantransformer/config/teleop_xiaosong.yaml'
        ),
    )

    return LaunchDescription([
        xiaosong_params_arg,
        Node(
            package='teleoptocantransformer',
            executable='teleop2can_transformer_xiaosong',
            name='teleop2can_transformer_xiaosong',
            output='screen',
            parameters=[
                {
                    # 死区参数（用于过滤小的输入抖动）
                    'arm_deadzone': 0.05,
                    'stick_deadzone': 0.05,
                    'bucket_deadzone': 0.05,
                    'swing_deadzone': 0.05,
                    'track_deadzone': 0.05,

                    # 电流全局上限 (mA)；各方向 *_current_input_min/max & output_min/max 为死区后 |x| 映射
                    'max_current': 700.0,

                    # 大臂 boom>0 抬升 / boom<0 下降
                    # 抬升快一些 300~700，下降慢一些 200~600 
                    'arm_up_current_input_min': 0.05,
                    'arm_up_current_input_max': 1.0,
                    'arm_up_current_output_min': 300.0,
                    'arm_up_current_output_max': 700.0,
                    'arm_down_current_input_min': 0.05,
                    'arm_down_current_input_max': 1.0,
                    'arm_down_current_output_min': 300.0,
                    'arm_down_current_output_max': 700.0,

                    # 斗杆 stick>0 伸出 / stick<0 收回
                    # 伸出快一些 300~700，收回慢一些 200~600 
                    'stick_extend_current_input_min': 0.05,
                    'stick_extend_current_input_max': 1.0,
                    'stick_extend_current_output_min': 300.0,
                    'stick_extend_current_output_max': 700.0,
                    'stick_retract_current_input_min': 0.05,
                    'stick_retract_current_input_max': 1.0,
                    'stick_retract_current_output_min': 300.0,
                    'stick_retract_current_output_max': 700.0,

                    # 铲斗 bucket>0 收斗 / bucket<0 放斗(翻斗)
                    # 收斗快一些 300~700，放斗慢一些 200~550 
                    'bucket_close_current_input_min': 0.05,
                    'bucket_close_current_input_max': 1.0,
                    'bucket_close_current_output_min': 300.0,
                    'bucket_close_current_output_max': 700.0,
                    'bucket_dump_current_input_min': 0.05,
                    'bucket_dump_current_input_max': 1.0,
                    'bucket_dump_current_output_min': 200.0,
                    'bucket_dump_current_output_max': 700.0,

                    # 回转 swing>0 右转 / swing<0 左转
                    'swing_right_current_input_min': 0.05,
                    'swing_right_current_input_max': 1.0,
                    'swing_right_current_output_min': 250.0,
                    'swing_right_current_output_max': 650.0,
                    'swing_left_current_input_min': 0.05,
                    'swing_left_current_input_max': 1.0,
                    'swing_left_current_output_min': 250.0,
                    'swing_left_current_output_max': 650.0,

                    # 左履带 前进 / 后退
                    'left_track_forward_current_input_min': 0.05,
                    'left_track_forward_current_input_max': 1.0,
                    'left_track_forward_current_output_min': 300.0,
                    'left_track_forward_current_output_max': 700.0,
                    'left_track_backward_current_input_min': 0.05,
                    'left_track_backward_current_input_max': 1.0,
                    'left_track_backward_current_output_min': 300.0,
                    'left_track_backward_current_output_max': 700.0,

                    # 右履带 前进 / 后退
                    'right_track_forward_current_input_min': 0.05,
                    'right_track_forward_current_input_max': 1.0,
                    'right_track_forward_current_output_min': 300.0,
                    'right_track_forward_current_output_max': 700.0,
                    'right_track_backward_current_input_min': 0.05,
                    'right_track_backward_current_input_max': 1.0,
                    'right_track_backward_current_output_min': 300.0,
                    'right_track_backward_current_output_max': 700.0,
                },
                LaunchConfiguration('xiaosong_params_file'),
            ],
        ),
    ])
