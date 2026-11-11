#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    database_path = LaunchConfiguration('database_path')

    common_params = {
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',

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
        'Grid/RangeMin': '0.3',
        'Grid/RangeMax': '8.0',
        'Grid/GroundIsObstacle': 'false',
        'Grid/NormalsSegmentation': 'false',
        'Grid/MinGroundHeight': '-0.10',
        'Grid/MaxGroundHeight': '0.10',
        'Grid/MaxObstacleHeight': '1.2',
        'Grid/NoiseFilteringRadius': '0.20',
        'Grid/NoiseFilteringMinNeighbors': '5',
        'Grid/FootprintLength': '0.70',
        'Grid/FootprintWidth': '0.35',
        'Grid/FootprintHeight': '0.25',
        'Grid/RayTracing': 'true',

        'Icp/VoxelSize': '0.15',
        'Icp/Iterations': '12',
        'Icp/MaxCorrespondenceDistance': '0.6',
    

        # Mapping mode
        'Mem/IncrementalMemory': 'true',
        'Mem/InitWMWithAllNodes': 'false'
    }

    delete_db_cmd = ExecuteProcess(
        cmd=['bash', '-c',
             'rm -f /home/unitree/.ros/go2_rtabmap.db /home/unitree/.ros/rtabmap.db'],
        output='screen'
    )

    go2_online_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('go2_bringup'),
                'launch',
                'go2_online.launch.py'
            )
        )
    )

    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[common_params, {'database_path': database_path}],
        remappings=[
            ('scan_cloud', '/utlidar/cloud_deskewed_retimed'),
            ('odom', '/utlidar/robot_odom_retimed')
        ]
    )

    map_saver_server = Node(
        package='nav2_map_server',
        executable='map_saver_server',
        name='map_saver_server',
        output='screen',
        parameters=[{
            'save_map_timeout': 5.0,
            'free_thresh_default': 0.25,
            'occupied_thresh_default': 0.65,
        }]
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
        }, {'database_path': database_path}],
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
        delete_db_cmd,
        go2_online_cmd,
        rtabmap,
        map_saver_server,
        rtabmap_viz
    ])
