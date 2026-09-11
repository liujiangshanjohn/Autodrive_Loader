import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'camera_driver'

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
    description='装载机相机驱动骨架（GMSL 采集待接入）',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'camera_driver_node = camera_driver.camera_driver_node:main',
        ],
    },
)
