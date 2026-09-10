import os
from glob import glob

from setuptools import find_packages
from setuptools import setup

package_name = 'mqtt_updater'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'paho-mqtt'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='订阅 cannode/chassis_feedback 并打印消息',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'chassis_to_mqtt = mqtt_updater.chassis_to_mqtt:main',
        ],
    },
)
