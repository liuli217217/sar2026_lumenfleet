from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'go2_camera_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='unitree',
    maintainer_email='unitree@todo.todo',
    description='Bridge Unitree Go2 front video stream to ROS2 image topics',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'go2_camera_bridge = go2_camera_bridge.go2_camera_bridge:main',
        ],
    },
    
)
