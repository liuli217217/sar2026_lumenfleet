from glob import glob

from setuptools import setup

package_name = 'go2_real_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ll',
    maintainer_email='liuli217217@gmail.com',
    description='Real Unitree Go2 adapter for the four SAR algorithm packages',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'go2_bridge = go2_real_bridge.bridge_node:main',
            'fake_go2 = go2_real_bridge.fake_go2_node:main',
            'sport_cli = go2_real_bridge.sport_cli:main',
            'check_sensors = go2_real_bridge.check_sensors:main',
        ],
    },
)
