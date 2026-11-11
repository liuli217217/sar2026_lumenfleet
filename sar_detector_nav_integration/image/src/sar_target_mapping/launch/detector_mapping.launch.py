"""DETECTOR_NAV_INTEGRATION: detector + mapper only; start navigation separately."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = {
        'rgb_topic': '/lumen/retimed/rgb',
        'depth_topic': '/lumen/retimed/depth',
        'camera_info_topic': '/lumen/retimed/rgb_info',
        'weights': os.path.join(get_package_share_directory('victim_detector_integrated'), 'models', 'sar_3class.pt'),
        'device': 'cpu',
        'detector_python': '/opt/detector-venv/bin/python3',
        'max_rate_hz': '5.0',
        'output_file': '/tmp/sar_targets.json',
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()],
        Node(package='victim_detector_integrated', executable='detector_node',
             name='sar_detector_integrated', output='screen',
             prefix=[LaunchConfiguration('detector_python')],
             parameters=[{
                 'image_topic': LaunchConfiguration('rgb_topic'),
                 'weights': LaunchConfiguration('weights'),
                 'device': LaunchConfiguration('device'),
                 'conf': .25, 'class_conf': 'mannequin:0.10,cone:0.20,first_aid:0.20',
                 'track_enable': True, 'track_bridge': False,
                 'publish_annotated': False, 'best_effort_qos': True,
                 'max_rate_hz': ParameterValue(LaunchConfiguration('max_rate_hz'), value_type=float),
             }]),
        Node(package='sar_target_mapping', executable='target_mapper', output='screen',
             parameters=[{
                 'detections_topic': '/sar_detector_integrated/detections_confirmed',
                 'depth_topic': LaunchConfiguration('depth_topic'),
                 'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                 'output_file': LaunchConfiguration('output_file'),
             }]),
    ])
