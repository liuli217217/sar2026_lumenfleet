import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(get_package_share_directory('lumen_sensor_tests'), 'config', 'retime.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=config),
        DeclareLaunchArgument('enabled', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='lumen_sensor_tests', executable='receive_time_adapter',
             name='receive_time_adapter', namespace='/', output='screen',
             parameters=[LaunchConfiguration('params_file'), {
                 'enabled': ParameterValue(LaunchConfiguration('enabled'), value_type=bool),
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
             }]),
    ])
