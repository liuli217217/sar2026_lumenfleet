#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    database_path = LaunchConfiguration('database_path')

    common_params = {
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'map_frame_id': 'map',
        'database_path': database_path,

        'subscribe_rgb': False,
        'subscribe_depth': False,
        'subscribe_rgbd': False,
        'subscribe_scan': False,
        'subscribe_scan_cloud': True,
        'subscribe_odom': True,
        'subscribe_odom_info': False,

        'approx_sync': True,
        'sync_queue_size': 20,
        'topic_queue_size': 20,
        'wait_for_transform': 0.2,

        'Reg/Strategy': '1',

        'Grid/3D': 'true',
        'Grid/CellSize': '0.05',
        'Grid/RangeMin': '0.3',
        'Grid/RangeMax': '8.0',
        'Grid/GroundIsObstacle': 'false',
        'Grid/NormalsSegmentation': 'false',
        'Grid/MinGroundHeight': '-0.10',
        'Grid/MaxGroundHeight': '0.05',
        'Grid/MaxObstacleHeight': '1.2',
        'Grid/NoiseFilteringRadius': '0.10',
        'Grid/NoiseFilteringMinNeighbors': '2',
        'Grid/FootprintLength': '0.70',
        'Grid/FootprintWidth': '0.35',
        'Grid/FootprintHeight': '0.25',
        'Grid/RayTracing': 'true',

        'Icp/VoxelSize': '0.10',
        'Icp/Iterations': '12',
        'Icp/MaxCorrespondenceDistance': '0.3',

        # Localization mode
        'Mem/IncrementalMemory': 'false',
        'Mem/InitWMWithAllNodes': 'true'
    }

    retime_utlidar = Node(
        package='go2_bringup',
        executable='retime_utlidar.py',
        name='retime_utlidar',
        output='screen'
    )

    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[common_params],
        remappings=[
            ('scan_cloud', '/utlidar/cloud_deskewed_retimed'),
            ('odom', '/utlidar/robot_odom_retimed')
        ]
    )

    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[{
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',
            'subscribe_rgb': False,
            'subscribe_depth': False,
            'subscribe_rgbd': False,
            'subscribe_scan': False,
            'subscribe_scan_cloud': True,
            'subscribe_odom': True,
            'subscribe_odom_info': False,
            'approx_sync': True,
            'queue_size': 20
        }],
        remappings=[
            ('scan_cloud', '/utlidar/cloud_deskewed_retimed'),
            ('odom', '/utlidar/robot_odom_retimed')
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'database_path',
            default_value='/home/unitree/.ros/go2_rtabmap.db',
            description='Path to RTAB-Map database'
        ),
        retime_utlidar,
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=retime_utlidar,
                on_start=[rtabmap, rtabmap_viz]
            )
        )
    ])