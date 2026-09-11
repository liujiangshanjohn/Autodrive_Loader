import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'loader_autonomy'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='无人装载机自主主路径骨架（感知/定位/任务/规划/控制）',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'perception_node = loader_autonomy.perception_node:main',
            'localization_node = loader_autonomy.localization_node:main',
            'mission_node = loader_autonomy.mission_node:main',
            'planning_node = loader_autonomy.planning_node:main',
            'control_node = loader_autonomy.control_node:main',
        ],
    },
)
