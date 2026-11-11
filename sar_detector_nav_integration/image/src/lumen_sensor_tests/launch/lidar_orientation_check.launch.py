import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('lumen_sensor_tests')
    config = os.path.join(share, 'config', 'sensors.yaml')
    rviz_config = os.path.join(share, 'config', 'lidar_orientation_check.rviz')
    return LaunchDescription([
        # Assumes receive_time_adapter (retime.launch.py) is already running
        # and publishing the retimed cloud; this launch does not start it.
        DeclareLaunchArgument('cloud_topic', default_value='/lumen/retimed/cloud_base'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('expected_direction',
                              description='Known obstacle placement: front, left, right or rear'),
        DeclareLaunchArgument('params_file', default_value=config),
        DeclareLaunchArgument('rviz_config', default_value=rviz_config),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='lumen_sensor_tests', executable='lidar_orientation_check',
             name='lidar_orientation_check', namespace='/', output='screen',
             parameters=[LaunchConfiguration('params_file'), {
                 'cloud_topic': LaunchConfiguration('cloud_topic'),
                 'base_frame': LaunchConfiguration('base_frame'),
                 'expected_direction': LaunchConfiguration('expected_direction'),
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
             }]),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', LaunchConfiguration('rviz_config')]),
    ])
