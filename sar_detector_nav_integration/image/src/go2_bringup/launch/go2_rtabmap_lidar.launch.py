#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
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
            'Icp/MaxCorrespondenceDistance': '0.6'
        }],
        remappings=[
            ('scan_cloud', '/utlidar/cloud_deskewed_retimed'),
            ('odom', '/utlidar/robot_odom_retimed')
        ],
        arguments=['-d']
    )

    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[{
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
            'queue_size': 20
        }],
        remappings=[
            ('scan_cloud', '/utlidar/cloud_deskewed_retimed'),
            ('odom', '/utlidar/robot_odom_retimed')
        ]
    )

    return LaunchDescription([
        rtabmap,
        rtabmap_viz
    ])