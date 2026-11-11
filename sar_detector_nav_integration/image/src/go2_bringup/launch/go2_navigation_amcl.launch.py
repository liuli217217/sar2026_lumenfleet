#!/usr/bin/env python3
#
# Navigation stack for Unitree Go2 — AMCL-based localization
#
# Pipeline:
#   go2_online (driver + retime_utlidar + robot_state_publisher)
#   utlidar_to_laserscan   →  /utlidar/scan  (360° 2D scan from UTLidar cloud)
#   AMCL localization      →  map → odom TF  (particle filter on slam_toolbox map)
#   nav2_map_server        →  /map  (pre-saved .pgm/.yaml from go2_mapping_slam_toolbox)
#   Nav2 stack             →  global/local planner + MPPI controller
#   collision_monitor      →  safety stop / slowdown zones
#   velocity_smoother      →  smooth acceleration, hard 0.30 m/s cap
#   go2_cmd_vel_bridge     →  /cmd_vel → /api/sport/request → robot
#
# Workflow:
#   1. Run go2_mapping_slam_toolbox.launch.py and drive the robot to build a map
#   2. Save the map:  ros2 run nav2_map_server map_saver_cli -f maps/map
#   3. Launch this file — set initial pose in RViz with "2D Pose Estimate"

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    bringup_dir = get_package_share_directory('go2_bringup')

    map_yaml         = LaunchConfiguration('map')
    nav2_params_file = LaunchConfiguration('params_file')

    # ── Robot bringup ────────────────────────────────────────────────────────
    go2_online_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'go2_online.launch.py')
        ),
        launch_arguments={'rviz': 'false'}.items()
    )

    # ── Navigation RViz ──────────────────────────────────────────────────────
    rviz_nav = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(bringup_dir, 'config', 'nav2_rviz.rviz')]
    )

    # ── UTLidar → LaserScan ──────────────────────────────────────────────────
    # Converts the retimed UTLidar 3D cloud to a 2D scan in base_link frame.
    # Used by AMCL, costmaps, and collision_monitor.
    utlidar_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='utlidar_to_laserscan',
        output='screen',
        parameters=[{
            'target_frame':        'base_link',
            'transform_tolerance':  0.05,
            'min_height':          -0.35,
            'max_height':           1.0,
            'angle_min':           -3.14159,
            'angle_max':            3.14159,
            'angle_increment':      0.00873,
            'scan_time':            0.1,
            'range_min':            0.10,
            'range_max':           20.0,
            'use_inf':              True,
        }],
        remappings=[
            ('cloud_in', '/utlidar/cloud_deskewed_retimed'),
            ('scan',     '/utlidar/scan'),
        ]
    )

    # ── AMCL localization ────────────────────────────────────────────────────
    # Particle-filter localization against the pre-saved slam_toolbox map.
    # Publishes map → odom TF.
    # After launch: use "2D Pose Estimate" in RViz to set the initial pose.
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params_file]
    )

    # ── Nav2 map server ──────────────────────────────────────────────────────
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[nav2_params_file, {'yaml_filename': map_yaml}]
    )

    # ── Nav2 core nodes ──────────────────────────────────────────────────────

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

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params_file],
        remappings=[
            ('cmd_vel',          'cmd_vel_nav'),
            ('cmd_vel_smoothed', 'cmd_vel_smoothed'),
        ]
    )

    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[nav2_params_file]
    )

    lifecycle_manager_nav = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[nav2_params_file]
    )

    # ── Go2 cmd_vel bridge ───────────────────────────────────────────────────
    go2_cmd_vel_bridge = Node(
        package='go2_cmd_vel_bridge',
        executable='go2_cmd_vel_bridge',
        name='go2_cmd_vel_bridge',
        output='screen',
        parameters=[{
            'max_vx': 0.30,
            'max_vy': 0.25,
            'max_wz': 0.60,
            'cmd_timeout_s': 0.25,
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value='/home/unitree/unitree_ros2/cyclonedds_ws/maps/map.yaml',
            description='Full path to the slam_toolbox saved occupancy grid (.yaml)'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(bringup_dir, 'config', 'nav2_params_amcl.yaml'),
            description='Full path to Nav2 parameters file'
        ),

        go2_online_cmd,
        rviz_nav,
        utlidar_to_laserscan,
        amcl,
        map_server,
        controller_server,
        smoother_server,
        planner_server,
        behavior_server,
        bt_navigator,
        velocity_smoother,
        collision_monitor,
        lifecycle_manager_nav,
        go2_cmd_vel_bridge,
    ])
