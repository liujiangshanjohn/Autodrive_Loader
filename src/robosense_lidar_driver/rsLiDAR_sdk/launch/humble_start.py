import os
import subprocess
import sys
from getpass import getpass
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def install_cyclone_dds():
    result = subprocess.run(
        ['dpkg', '-s', 'ros-humble-rmw-cyclonedds-cpp'],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    if result.returncode == 0:
        print('DDS is already installed.')
    else:
        print('DDS not installed. Installing now...')
        password = getpass('Enter your sudo password to install DDS: ')
        try:
            subprocess.run(['sudo', '-S', 'apt-get', 'update'], input=password.encode(), check=True)
            subprocess.run(
                ['sudo', '-S', 'apt-get', 'install', '-y', 'ros-humble-rmw-cyclonedds-cpp'],
                input=password.encode(),
                check=True,
            )
            print('DDS installed successfully.')
        except subprocess.CalledProcessError as e:
            print(f'Failed to install DDS: {str(e)}')
            sys.exit(1)


def _cyclonedds_env(dds_if: str) -> dict:
    """写入 CycloneDDS XML，用 file:// 传给 CYCLONEDDS_URI。"""
    if not dds_if:
        return {}
    xml_body = (
        '<?xml version="1.0" encoding="UTF-8" ?>\n'
        '<CycloneDDS xmlns="https://cdds.io/config">\n'
        '  <Domain Id="any">\n'
        '    <General>\n'
        '      <Interfaces>\n'
        f'        <NetworkInterface name="{dds_if}" multicast="default" priority="default"/>\n'
        '      </Interfaces>\n'
        '    </General>\n'
        '  </Domain>\n'
        '</CycloneDDS>\n'
    )
    cfg_path = '/tmp/cyclonedds_rslidar_launch.xml'
    try:
        with open(cfg_path, 'w', encoding='utf-8') as f:
            f.write(xml_body)
    except OSError as e:
        print(f'[humble_start] 警告: 无法写入 {cfg_path} ({e})，改用内联 XML')
        compact = ''.join(xml_body.split())
        return {'CYCLONEDDS_URI': compact}

    file_uri = Path(cfg_path).resolve().as_uri()
    print(f'[humble_start] CYCLONEDDS_URI=file -> NetworkInterface name={dds_if} ({cfg_path})')
    print(
        '[humble_start] 其它终端执行 ros2 前请: '
        f'export CYCLONEDDS_URI={file_uri}'
    )
    return {'CYCLONEDDS_URI': file_uri}


def _detect_default_network_interface() -> str:
    try:
        out = subprocess.run(
            ['ip', 'route', 'get', '8.8.8.8'],
            capture_output=True,
            text=True,
            timeout=3,
            check=False,
        )
        if out.returncode == 0 and out.stdout:
            parts = out.stdout.split()
            if 'dev' in parts:
                idx = parts.index('dev')
                if idx + 1 < len(parts):
                    return parts[idx + 1]
    except (subprocess.TimeoutExpired, FileNotFoundError, ValueError, IndexError):
        pass
    try:
        out = subprocess.run(
            ['ip', 'route', 'show', 'default'],
            capture_output=True,
            text=True,
            timeout=3,
            check=False,
        )
        if out.returncode == 0 and out.stdout:
            for line in out.stdout.strip().splitlines():
                parts = line.split()
                if 'dev' in parts:
                    idx = parts.index('dev')
                    if idx + 1 < len(parts):
                        return parts[idx + 1]
    except (subprocess.TimeoutExpired, FileNotFoundError, ValueError, IndexError):
        pass
    try:
        out = subprocess.run(
            ['ip', '-br', 'link', 'show', 'up'],
            capture_output=True,
            text=True,
            timeout=3,
            check=False,
        )
        if out.returncode == 0 and out.stdout:
            for line in out.stdout.strip().splitlines():
                parts = line.split(None, 2)
                if len(parts) >= 2 and parts[1] == 'UP':
                    name = parts[0].split('@')[0]
                    if name != 'lo':
                        return name
    except (subprocess.TimeoutExpired, FileNotFoundError, ValueError, IndexError):
        pass
    return ''


def _launch_setup(rviz_config_path: str):
    def launch_setup(context, *args, **kwargs):
        # 与整车 run.sh 统一 DDS 时：外层已 export CYCLONEDDS_URI，此处不再写第二份 XML，
        # 避免双配置 / 启动瞬间与多 participant 叠加带来的不稳定。
        localhost_only = os.environ.get('ROS_LOCALHOST_ONLY', '').strip().lower() in ('1', 'true')
        force_local = LaunchConfiguration('force_local_dds').perform(context).strip().lower() in (
            'true',
            '1',
            'yes',
        )
        existing_uri = os.environ.get('CYCLONEDDS_URI', '').strip()

        dds_if = LaunchConfiguration('dds_interface').perform(context).strip()
        if not dds_if:
            dds_if = os.environ.get('ROS_DDS_NETWORK_INTERFACE', '').strip()
        if not dds_if:
            dds_if = _detect_default_network_interface()
            if dds_if:
                print(f'[humble_start] dds_interface 未指定，自动使用默认路由网卡: {dds_if}')

        if localhost_only:
            extra_env = {}
            print(
                '[humble_start] ROS_LOCALHOST_ONLY=1：本机 DDS 走 loopback，不生成 Cyclone 网卡 XML'
            )
        elif not force_local and existing_uri:
            extra_env = {}
            print(
                '[humble_start] 检测到已设置 CYCLONEDDS_URI，沿用整车/外层 DDS（不写 '
                '/tmp/cyclonedds_rslidar_launch.xml）'
            )
        else:
            if force_local and existing_uri:
                print('[humble_start] force_local_dds=true，忽略外层 CYCLONEDDS_URI，生成本地 Cyclone XML')
            extra_env = _cyclonedds_env(dds_if)
            if extra_env:
                os.environ.update(extra_env)

        ur = LaunchConfiguration('use_rviz').perform(context).strip().lower()
        if ur in ('auto', ''):
            use_rviz = bool(os.environ.get('DISPLAY', '').strip())
            print(
                f'[humble_start] use_rviz=auto -> {"启动 rviz（检测到 DISPLAY）" if use_rviz else "跳过 rviz（无 DISPLAY）"}'
            )
        else:
            use_rviz = ur in ('true', '1', 'yes')

        actions = [
            Node(
                namespace='rslidar_sdk',
                package='rslidar_sdk',
                executable='rslidar_sdk_node',
                output='screen',
                additional_env=extra_env,
            ),
        ]
        if use_rviz:
            actions.append(
                Node(
                    namespace='rviz2',
                    package='rviz2',
                    executable='rviz2',
                    arguments=['-d', rviz_config_path],
                    additional_env=extra_env,
                )
            )
        elif ur not in ('auto', ''):
            print('[humble_start] use_rviz=false — skipping rviz2')
        return actions

    return launch_setup


def generate_launch_description():
    localhost_only = os.environ.get('ROS_LOCALHOST_ONLY', '').strip().lower() in ('1', 'true')
    if os.getenv('ROS_DISTRO') == 'humble':
        if localhost_only:
            print('[humble_start] ROS_LOCALHOST_ONLY=1：单机模式 — 不强制安装/使用 Cyclone（减轻多 participant 崩 rmw）')
            if not os.environ.get('RMW_IMPLEMENTATION'):
                os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
                print(
                    f'Environment Variable Set: RMW_IMPLEMENTATION={os.environ.get("RMW_IMPLEMENTATION")}'
                )
            else:
                print(
                    f'RMW_IMPLEMENTATION 已由外部环境指定: {os.environ.get("RMW_IMPLEMENTATION")}'
                )
        else:
            print('Detected ROS 2 Humble. Checking DDS...')
            install_cyclone_dds()
            if not os.environ.get('RMW_IMPLEMENTATION'):
                os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
                print(f'Environment Variable Set: RMW_IMPLEMENTATION={os.environ.get("RMW_IMPLEMENTATION")}')
            else:
                print(
                    f'RMW_IMPLEMENTATION 已由外部环境指定: {os.environ.get("RMW_IMPLEMENTATION")}'
                )

    rviz_config = get_package_share_directory('rslidar_sdk') + '/rviz/rviz2.rviz'

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_rviz',
                default_value='false',
                description='true / false / auto：无 DISPLAY 时不启动 rviz',
            ),
            DeclareLaunchArgument(
                'dds_interface',
                default_value='eth0',
                description='CycloneDDS 绑定网卡（雷达数据口）；默认 eth0。改为其它口可传 dds_interface:=eth1',
            ),
            DeclareLaunchArgument(
                'force_local_dds',
                default_value='false',
                description='true：始终在本 launch 内生成 /tmp/cyclonedds_rslidar_launch.xml；'
                'false：若环境已有 CYCLONEDDS_URI 则沿用（推荐与整车 run.sh 一致）',
            ),
            OpaqueFunction(function=_launch_setup(rviz_config)),
        ]
    )
