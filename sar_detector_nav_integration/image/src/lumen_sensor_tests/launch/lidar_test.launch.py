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
        DeclareLaunchArgument('cloud_topic', description='Discovered original PointCloud2 topic'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('params_file', default_value=config),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        # Explicit name AND namespace scope overrides above node-specific YAML.
        Node(package='lumen_sensor_tests', executable='lidar_diagnostic',
             name='lidar_diagnostic', namespace='/', output='screen',
             parameters=[LaunchConfiguration('params_file'), {
                 'cloud_topic': LaunchConfiguration('cloud_topic'),
                 'base_frame': LaunchConfiguration('base_frame'),
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
             }]),
    ])
