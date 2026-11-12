from glob import glob
from setuptools import setup

package_name = 'go2_sar_scout'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SAR 2026 LumenFleet',
    maintainer_email='liuli217217@gmail.com',
    license='MIT',
    entry_points={
        'console_scripts': [
            'sar_scout = go2_sar_scout.scout_node:main',
            'status_referee_adapter = go2_sar_scout.status_referee_adapter_node:main',
        ],
    },
)
