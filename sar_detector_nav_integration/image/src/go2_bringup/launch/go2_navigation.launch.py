#!/usr/bin/env python3
#
# Navigation stack for Unitree Go2
#
# Pipeline:
#   go2_online (driver + retime_utlidar + robot_state_publisher)
#   RTAB-Map localization  →  map → odom → base_link TF
#   nav2_map_server        →  /map  (pre-saved .pgm/.yaml)
#   Nav2 stack             →  global/local planner + MPPI controller
#   collision_monitor      →  safety stop / slowdown zones
#   velocity_smoother      →  smooth acceleration, hard 0.30 m/s cap
#   go2_cmd_vel_bridge     →  /cmd_vel → /api/sport/request → robot

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    bringup_dir = get_package_share_directory('go2_bringup')

    map_yaml        = LaunchConfiguration('map')
    database_path   = LaunchConfiguration('database_path')
    nav2_params_file = LaunchConfiguration('params_file')

    # ── Robot bringup ────────────────────────────────────────────────────────
    # Launches: robot_state_publisher, go2_driver, retime_utlidar
    # RViz is suppressed here — the navigation-specific RViz below replaces it
    go2_online_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'go2_online.launch.py')
        ),
        launch_arguments={'rviz': 'false'}.items()
    )

    # ── Navigation RViz ──────────────────────────────────────────────────────
    # Displays: map, global/local costmaps, paths, collision zones, Nav2 panel
    # Fixed Frame = map  |  Top-down view following base_link
    # Tools: "2D Pose Estimate" → set initial localisation pose
    #        "2D Nav Goal"      → send navigation goal to bt_navigator
    rviz_nav = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(bringup_dir, 'config', 'nav2_rviz.rviz')]
    )

    # ── RTAB-Map localization ────────────────────────────────────────────────
    # Loads existing .db, matches current scans → publishes map → odom TF.
    # Its /map output is remapped to /rtabmap/map to avoid conflicting with
    # nav2_map_server which serves the pre-saved static map on /map.
    rtabmap_localization = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
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
            'Grid/RangeMin': '0.3',
            'Grid/RangeMax': '8.0',
            'Grid/NormalsSegmentation': 'false',
            'Grid/MinGroundHeight': '-0.10',
            'Grid/MaxGroundHeight': '0.10',
            'Grid/MaxObstacleHeight': '1.2',
            'Grid/NoiseFilteringRadius': '0.20',
            'Grid/NoiseFilteringMinNeighbors': '5',
            'Grid/RayTracing': 'true',
            'Icp/VoxelSize': '0.15',
            'Icp/Iterations': '12',
            'Icp/MaxCorrespondenceDistance': '0.6',

            # Localization mode — do NOT add new nodes to the map
            'Mem/IncrementalMemory': 'false',
            'Mem/InitWMWithAllNodes': 'true',

            # Publish last known pose immediately at startup even before
            # a loop closure is found — allows map→odom TF to appear without
            # waiting for a successful scan match
            # 'Mem/UseOdomFeatures': 'false',
            # 'pub_loc_pose_only_when_localizing': 'false',
        }],
        remappings=[
            ('scan_cloud', '/utlidar/cloud_deskewed_retimed'),
            ('odom',       '/utlidar/robot_odom_retimed'),
            ('map',        '/rtabmap/map'),
        ]
    )

    # ── Nav2 map server ──────────────────────────────────────────────────────
    # Serves the pre-saved .pgm/.yaml map on /map for the global costmap.
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[nav2_params_file, {'yaml_filename': map_yaml}]
    )

    # ── Nav2 core nodes ──────────────────────────────────────────────────────

    # cmd_vel from controller is sent to cmd_vel_nav so that it passes through
    # velocity_smoother → collision_monitor before reaching the bridge.
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_file],
        remappings=[('cmd_vel', 'cmd_vel_nav')]
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[nav2_params_file]
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file]
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params_file]
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params_file]
    )

    # Subscribes cmd_vel_nav, enforces 0.30 m/s cap, smooth acceleration,
    # publishes cmd_vel_smoothed.
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params_file],
        remappings=[
            ('cmd_vel',         'cmd_vel_nav'),
            ('cmd_vel_smoothed', 'cmd_vel_smoothed'),
        ]
    )

    # Safety: stops robot immediately if obstacle enters StopZone (~0.55m),
    # halves speed in SlowZone (~0.80m). Reads cmd_vel_smoothed, outputs cmd_vel.
    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[nav2_params_file]
    )

    # Manages lifecycle (configure → activate) of all Nav2 nodes.
    lifecycle_manager_nav = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[nav2_params_file]
    )

    # ── UTLidar → LaserScan for costmaps ────────────────────────────────────
    # Converts the retimed UTLidar 3D cloud to a 2D scan in the base_link plane.
    # target_frame: base_link (robot body frame, ~35cm above ground)
    # min_height: -0.1  → captures from near ground level
    # max_height:  1.0  → captures obstacles up to 1m above base_link
    # Publishes /utlidar/scan used by both local and global costmaps.
    utlidar_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='utlidar_to_laserscan',
        output='screen',
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.05,
            'min_height': -0.10,
            'max_height': 1.0,
            'angle_min': -3.14159,
            'angle_max':  3.14159,
            'angle_increment': 0.0087,
            'scan_time': 0.1,
            'range_min': 0.10,
            'range_max': 10.0,
            'use_inf': True,
        }],
        remappings=[
            ('cloud_in', '/utlidar/cloud_deskewed_retimed'),
            ('scan',     '/utlidar/scan'),
        ]
    )

    # ── Go2 cmd_vel bridge ───────────────────────────────────────────────────
    # Converts /cmd_vel (geometry_msgs/Twist) → /api/sport/request (Unitree API).
    # Built-in safety: 0.25 s cmd_vel timeout → StopMove command.
    # Hard velocity clamp enforced here as final defence.
    go2_cmd_vel_bridge = Node(
        package='go2_cmd_vel_bridge',
        executable='go2_cmd_vel_bridge',
        name='go2_cmd_vel_bridge',
        output='screen',
        parameters=[{
            'max_vx': 0.40,
            'max_vy': 0.25,
            'max_wz': 0.80,
            'cmd_timeout_s': 0.25,  # sends StopMove if no cmd_vel for 250 ms
            'deadband_linear': 0.06,
            'deadband_angular': 0.08,
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value='/home/unitree/unitree_ros2/cyclonedds_ws/maps/map.yaml',
            description='Full path to the saved occupancy grid map (.yaml)'
        ),
        DeclareLaunchArgument(
            'database_path',
            default_value='/home/unitree/.ros/go2_rtabmap.db',
            description='Full path to RTAB-Map database for localization'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(bringup_dir, 'config', 'nav2_params.yaml'),
            description='Full path to Nav2 parameters file'
        ),

        go2_online_cmd,
        rviz_nav,
        rtabmap_localization,
        map_server,
        controller_server,
        smoother_server,
        planner_server,
        behavior_server,
        bt_navigator,
        velocity_smoother,
        collision_monitor,
        lifecycle_manager_nav,
        utlidar_to_laserscan,
        go2_cmd_vel_bridge,
    ])
