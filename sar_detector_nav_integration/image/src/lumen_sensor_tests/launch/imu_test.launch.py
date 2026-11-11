from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    config = os.path.join(get_package_share_directory('lumen_sensor_tests'), 'config', 'sensors.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('imu_topic', description='Discovered sensor_msgs/msg/Imu topic'),
        DeclareLaunchArgument('stationary_check', default_value='false'),
        DeclareLaunchArgument('params_file', default_value=config),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        # Explicit name AND namespace scope overrides above node-specific YAML.
        Node(package='lumen_sensor_tests', executable='imu_diagnostic',
             name='imu_diagnostic', namespace='/', output='screen',
             parameters=[LaunchConfiguration('params_file'), {
                 'imu_topic': LaunchConfiguration('imu_topic'),
                 'stationary_check': ParameterValue(
                     LaunchConfiguration('stationary_check'), value_type=bool),
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
             }]),
    ])
