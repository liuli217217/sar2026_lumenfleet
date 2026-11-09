#!/usr/bin/env python3
#
# Local/test mapping pipeline -- bundles the 4 pieces that are always started
# together for RGB-D RTAB-Map mapping on this test rig (companion-Jetson or
# consolidated-Jetson camera, clock not trusted): lidar/odom retiming, camera
# TF, camera receive-time retiming, and RTAB-Map itself.
#
# Deliberately NOT included here (stay their own terminals):
#   - the RealSense driver itself -- runs on whatever host physically has the
#     camera, outside this workspace's launch tree
#   - go2_navigation_rtabmap.launch.py -- toggled (enable_motion) independently
#   - keyboard_control.py / hybrid_explorer.py -- started manually, on purpose
#
# For the actual competition rig, use go2_mapping_pipeline_competition.launch.py
# instead -- no receive_time_adapter (no documented camera-clock fault there,
# per TEAM_GUIDE.md), and camera TF read from the organizers' per-robot
# calibration file instead of this file's hardcoded "seek" defaults.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    lumen_dir = get_package_share_directory('lumen_sensor_tests')

    retime_utlidar = Node(
        package='go2_bringup',
        executable='retime_utlidar.py',
        name='retime_utlidar',
        output='screen',
    )

    camera_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lumen_dir, 'launch', 'camera_tf.launch.py')
        )
    )

    camera_retime = Node(
        package='lumen_sensor_tests',
        executable='receive_time_adapter',
        name='receive_time_adapter',
        output='screen',
        parameters=[{
            'enabled': True,
            'input_reliability': LaunchConfiguration('camera_input_reliability'),
            'rgb_input': LaunchConfiguration('rgb_input'),
            'rgb_output': LaunchConfiguration('rgb_output'),
            'depth_input': LaunchConfiguration('depth_input'),
            'depth_output': LaunchConfiguration('depth_output'),
            'rgb_info_input': LaunchConfiguration('rgb_info_input'),
            'rgb_info_output': LaunchConfiguration('rgb_info_output'),
        }],
    )

    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'approx_sync': True,
            'approx_sync_max_interval': 0.1,
            'database_path': LaunchConfiguration('database_path'),
            'Mem/IncrementalMemory': LaunchConfiguration('mem_incremental'),
        }],
        remappings=[
            ('rgb/image', LaunchConfiguration('rgb_output')),
            ('depth/image', LaunchConfiguration('depth_output')),
            ('rgb/camera_info', LaunchConfiguration('rgb_info_output')),
            ('odom', '/utlidar/robot_odom_retimed'),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('camera_input_reliability', default_value='reliable'),
        DeclareLaunchArgument('rgb_input', default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('rgb_output', default_value='/lumen/retimed/rgb'),
        DeclareLaunchArgument('depth_input', default_value='/camera/aligned_depth_to_color/image_raw'),
        DeclareLaunchArgument('depth_output', default_value='/lumen/retimed/depth'),
        DeclareLaunchArgument('rgb_info_input', default_value='/camera/color/camera_info'),
        DeclareLaunchArgument('rgb_info_output', default_value='/lumen/retimed/rgb_info'),
        DeclareLaunchArgument('database_path', default_value=[
            '/mnt/sdcard/rtab_map_data/rtabmap_', Command(['date', '-u', '+%Y%m%dT%H%M%SZ']), '.db',
        ]),
        DeclareLaunchArgument('mem_incremental', default_value='true',
                              description="'false' localizes in an existing database (point database_path "
                                          "at it) without modifying it -- for repeated nav-goal trials against "
                                          "one fixed map. 'true' (default) builds/extends the map."),

        retime_utlidar,
        camera_tf,
        camera_retime,
        rtabmap,
    ])
