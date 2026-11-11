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
from launch_ros.parameter_descriptions import ParameterValue


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
            'Mem/IncrementalMemory': ParameterValue(LaunchConfiguration('mem_incremental'), value_type=str),
            # Grid/* occupancy-grid tuning ported 2026-11-11 from
            # go2_mapping.launch.py / go2_navigation.launch.py (the lidar/
            # scan_cloud variants) -- this file had NONE of it, running
            # RTAB-Map's bare defaults, which is the direct cause of noisy
            # "occupied" blobs in /map false-blocking real doorways (live
            # observation, same session). Only the grid-generation params
            # are ported, not Reg/Strategy or Icp/* -- those are specific
            # to the lidar/scan_cloud registration pipeline and not
            # applicable to this RGB-D variant's registration.
            'Grid/3D': 'true',
            'Grid/RangeMin': '0.3',
            'Grid/RangeMax': '8.0',
            'Grid/GroundIsObstacle': 'false',
            'Grid/NormalsSegmentation': 'false',
            'Grid/MinGroundHeight': '-0.10',
            'Grid/MaxGroundHeight': '0.10',
            'Grid/MaxObstacleHeight': '1.2',
            # The two params directly targeting noisy/isolated false-occupied
            # cells: a cell only counts as an obstacle if it has at least
            # NoiseFilteringMinNeighbors other occupied cells within
            # NoiseFilteringRadius -- an isolated depth-noise speckle fails
            # this and gets dropped instead of becoming a permanent "wall".
            'Grid/NoiseFilteringRadius': '0.20',
            'Grid/NoiseFilteringMinNeighbors': '5',
            'Grid/FootprintLength': '0.70',
            'Grid/FootprintWidth': '0.35',
            'Grid/FootprintHeight': '0.25',
            'Grid/RayTracing': 'true',
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
