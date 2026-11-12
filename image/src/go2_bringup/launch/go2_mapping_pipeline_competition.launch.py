#!/usr/bin/env python3
#
# Competition mapping pipeline. Two differences from go2_mapping_pipeline.launch.py:
#
#   - No receive_time_adapter for the camera. This rig's camera-clock fault
#     (see docs/DIAGNOSTIC_DEPTH_STALL_2026-11-06.md) has no documented
#     equivalent on the competition robot in TEAM_GUIDE.md. If the
#     competition camera turns out to need it too, add it back the same way
#     the local pipeline does, and point RTAB-Map at its retimed output
#     instead of the raw topics below.
#   - Camera TF comes from the organizers' per-robot calibration file
#     (TEAM_GUIDE.md 3.7, /opt/sar/calibration/go2_camera_realsense.yaml),
#     not this repo's hardcoded "seek" defaults in
#     lumen_sensor_tests/camera_tf.launch.py. The math below is adapted from
#     their examples/native_ros/src/sar_go2_example/launch/camera_tf.launch.py
#     (that package is example code, not a build dependency of this workspace).
#
# Still assumes the camera driver and retime_utlidar's lidar prerequisites
# are started separately, same as the local pipeline.

import math

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def quaternion(R):
    """3x3 rotation matrix -> quaternion [x, y, z, w]."""
    tr = R[0][0] + R[1][1] + R[2][2]
    if tr > 0:
        s = 2 * math.sqrt(1 + tr)
        return [(R[2][1] - R[1][2]) / s, (R[0][2] - R[2][0]) / s, (R[1][0] - R[0][1]) / s, s / 4]
    i = max(range(3), key=lambda a: R[a][a])
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2 * math.sqrt(1 + R[i][i] - R[j][j] - R[k][k])
    q = [0.0] * 4
    q[i], q[j], q[k], q[3] = s / 4, (R[j][i] + R[i][j]) / s, (R[k][i] + R[i][k]) / s, (R[k][j] - R[j][k]) / s
    return q


def camera_tf(context):
    with open(LaunchConfiguration('calibration').perform(context)) as f:
        cal = yaml.safe_load(f)
    T = cal['T_go2_camera_link']  # p_go2 = T * p_camera_link
    x, y, z = (T[r][3] for r in range(3))
    qx, qy, qz, qw = quaternion([row[:3] for row in T[:3]])
    args = {'x': x, 'y': y, 'z': z, 'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw}
    return [Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='go2_camera_to_realsense', output='screen',
        arguments=[a for k, v in args.items() for a in (f'--{k}', f'{v:.9f}')]
        + ['--frame-id', cal['frames']['go2'], '--child-frame-id', cal['frames']['link']],
    )]


def generate_launch_description():
    retime_utlidar = Node(
        package='go2_bringup',
        executable='retime_utlidar.py',
        name='retime_utlidar',
        output='screen',
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
            # scan_cloud variants) -- this file had none of it, running
            # RTAB-Map's bare defaults, the direct cause of noisy
            # "occupied" blobs in /map false-blocking real doorways (live
            # observation, same session). Only the grid-generation params
            # are ported, not Reg/Strategy or Icp/* -- those are specific
            # to the lidar/scan_cloud registration pipeline, not this
            # RGB-D variant's.
            #
            # 2026-11-12: confirmed live on the robot that this port never
            # actually took effect -- RTAB-Map had been running on bare
            # defaults in production. Also, the height bands below were
            # never correctly anchored to this robot's actual geometry in
            # frame_id (base_link): a recorded bag showed the real floor
            # sits at z ~= -0.288m in base_link, not near 0 as the
            # original -0.10/0.10/1.2 assumed, a plausible direct cause of
            # phantom "occupied" blobs at real, open doorways. Shifted by
            # -0.288 to preserve the original band width / height-above-
            # floor while correctly centering on the measured floor -- see
            # go2_mapping_pipeline.launch.py's matching comment for detail.
            'Grid/3D': 'true',
            'Grid/RangeMin': '0.3',
            'Grid/RangeMax': '8.0',
            'Grid/GroundIsObstacle': 'false',
            'Grid/NormalsSegmentation': 'false',
            'Grid/MinGroundHeight': '-0.39',
            'Grid/MaxGroundHeight': '-0.19',
            'Grid/MaxObstacleHeight': '0.91',
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
            ('rgb/image', LaunchConfiguration('rgb_input')),
            ('depth/image', LaunchConfiguration('depth_input')),
            ('rgb/camera_info', LaunchConfiguration('rgb_info_input')),
            ('odom', '/utlidar/robot_odom_retimed'),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('calibration', default_value='/opt/sar/calibration/go2_camera_realsense.yaml'),
        DeclareLaunchArgument('rgb_input', default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('depth_input', default_value='/camera/aligned_depth_to_color/image_raw'),
        DeclareLaunchArgument('rgb_info_input', default_value='/camera/color/camera_info'),
        DeclareLaunchArgument('database_path', default_value=[
            '/mnt/sdcard/rtab_map_data/rtabmap_', Command(['date', '-u', '+%Y%m%dT%H%M%SZ']), '.db',
        ]),
        DeclareLaunchArgument('mem_incremental', default_value='true',
                              description="'false' localizes in an existing database without modifying it."),

        OpaqueFunction(function=camera_tf),
        retime_utlidar,
        rtabmap,
    ])
