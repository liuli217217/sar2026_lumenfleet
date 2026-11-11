from glob import glob
from setuptools import find_packages, setup

setup(
    name='lumen_sensor_tests', version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/lumen_sensor_tests']),
        ('share/lumen_sensor_tests', ['package.xml']),
        ('share/lumen_sensor_tests/launch', glob('launch/*.launch.py')),
        ('share/lumen_sensor_tests/config', glob('config/*.yaml') + glob('config/*.rviz')),
    ],
    install_requires=['setuptools'], tests_require=['pytest'], zip_safe=True,
    maintainer='Lumen Fleet', maintainer_email='lumen-fleet@example.com',
    description='Read-only sensor diagnostics', license='Apache-2.0',
    entry_points={'console_scripts': [
        'lidar_diagnostic = lumen_sensor_tests.lidar_diagnostic:main',
        'lidar_orientation_check = lumen_sensor_tests.lidar_orientation_check:main',
        'imu_diagnostic = lumen_sensor_tests.imu_diagnostic:main',
        'timestamp_check = lumen_sensor_tests.timestamp_check:main',
        'receive_time_adapter = lumen_sensor_tests.receive_time_adapter:main',
    ]},
)
