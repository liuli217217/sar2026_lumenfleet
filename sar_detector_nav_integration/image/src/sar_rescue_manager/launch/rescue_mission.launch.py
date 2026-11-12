"""DETECTOR_NAV_INTEGRATION: managed explorer + detector + mock rescue.

Mapping and Nav2 remain separate. No goals are sent until set_enabled service.
Do not start the upstream hybrid_explorer/explore_lite alongside this workflow.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arguments = {
        'rgb_topic': '/lumen/retimed/rgb', 'depth_topic': '/lumen/retimed/depth',
        'camera_info_topic': '/lumen/retimed/rgb_info',
        'odom_topic': '/utlidar/robot_odom_retimed',
        'device': 'cpu', 'max_rate_hz': '5.0',
        'detector_python': '/opt/detector-venv/bin/python3',
        'output_file': '/tmp/sar_targets.json',
        'ledger_file': '/tmp/sar_rescue_ledger.json',
        'mission_id': 'test-map-001', 'standoff_m': '1.0',
        'rescue_executor': 'real',   # real | mock
        'sit_duration': '3.0',
    }
    mapping = os.path.join(get_package_share_directory('sar_target_mapping'), 'launch', 'detector_mapping.launch.py')
    return LaunchDescription([
        *[DeclareLaunchArgument(k, default_value=v) for k, v in arguments.items()],
        IncludeLaunchDescription(PythonLaunchDescriptionSource(mapping), launch_arguments={
            k: LaunchConfiguration(k) for k in
            ('rgb_topic', 'depth_topic', 'camera_info_topic', 'device', 'max_rate_hz', 'detector_python', 'output_file')
        }.items()),
        Node(package='sar_rescue_manager', executable='managed_explorer', output='screen'),
        Node(package='sar_rescue_manager', executable='rescue_manager', output='screen', parameters=[{
            'odom_topic': LaunchConfiguration('odom_topic'),
            'ledger_file': LaunchConfiguration('ledger_file'),
            'mission_id': LaunchConfiguration('mission_id'),
            'standoff_m': ParameterValue(LaunchConfiguration('standoff_m'), value_type=float),
        }]),
        # The rescue executor. 'real' sits the robot down for real (Sit/RiseSit
        # sport requests, the same posture change the two reactive algorithms
        # make) and is the default, because the task is to reach the victim AND
        # sit beside them -- scoring a mock completion against algorithms that
        # really sit would not be an honest comparison. 'mock' keeps the
        # original answer-only behaviour for dry runs where no posture change
        # is wanted. Only one of the two runs.
        Node(package='sar_rescue_manager', executable='mock_rescue', output='screen',
             condition=LaunchConfigurationEquals('rescue_executor', 'mock')),
        Node(package='sar_rescue_manager', executable='real_rescue', output='screen',
             condition=LaunchConfigurationEquals('rescue_executor', 'real'),
             parameters=[{'sit_duration': ParameterValue(
                 LaunchConfiguration('sit_duration'), value_type=float)}]),
        # Translates rescue_manager's ledger into the /sar/events strings
        # mission_monitor scores on, so this workflow can be compared against
        # the two reactive algorithms by the same referee. Listener only --
        # see referee_adapter.py. Harmless outside simulation: with no
        # mission_monitor running, nothing subscribes to /sar/events.
        Node(package='sar_rescue_manager', executable='referee_adapter', output='screen'),
    ])
