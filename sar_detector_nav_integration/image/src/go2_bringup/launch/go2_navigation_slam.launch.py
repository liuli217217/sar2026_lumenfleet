#!/usr/bin/env python3
#
# Navigation stack for Unitree Go2 -- SLAM-concurrent variant.
#
# Unlike go2_navigation.launch.py (RTAB-Map localization against a pre-saved
# map), this navigates WHILE building the map, not against an existing one:
#
#   go2_mapping_slam_toolbox  →  go2_online + body_filter + utlidar_to_laserscan
#                                 + slam_toolbox  →  live /map, map->odom TF
#   Nav2 stack                →  global/local planner + MPPI controller,
#                                 global costmap's static_layer subscribes to
#                                 slam_toolbox's live /map (no map_server)
#   collision_monitor         →  safety stop / slowdown zones
#   velocity_smoother         →  smooth acceleration, hard 0.30 m/s cap
#   go2_cmd_vel_bridge        →  /cmd_vel → /api/sport/request → robot

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    bringup_dir = get_package_share_directory('go2_bringup')
    nav2_params_file = LaunchConfiguration('params_file')
    use_rviz = LaunchConfiguration('use_rviz')
    enable_motion = LaunchConfiguration('enable_motion')

    # ── Mapping + sensor bringup ─────────────────────────────────────────────
    # Launches: go2_online (driver + retime_utlidar + robot_state_publisher),
    # body_filter, utlidar_to_laserscan, slam_toolbox, map_saver_server.
    # Its own RViz is suppressed -- the navigation RViz below replaces it.
    go2_mapping_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'go2_mapping_slam_toolbox.launch.py')
        ),
        launch_arguments={'rviz': 'false'}.items()
    )

    # ── Navigation RViz ──────────────────────────────────────────────────────
    # Gated off for headless robot-side docker runs (no display attached);
    # run RViz separately on a team laptop on the same ROS_DOMAIN_ID instead,
    # loading config/nav2_rviz.rviz from this package.
    rviz_nav = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(bringup_dir, 'config', 'nav2_rviz.rviz')],
        condition=IfCondition(use_rviz)
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

    # Subscribes cmd_vel_nav, enforces velocity caps, smooth acceleration,
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

    # Safety: stops robot immediately if obstacle enters StopZone, halves
    # speed in SlowZone. Reads cmd_vel_smoothed, outputs cmd_vel.
    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[nav2_params_file]
    )

    # Manages lifecycle (configure → activate) of the Nav2 nodes. Does NOT
    # manage map_server -- there isn't one; /map comes from slam_toolbox.
    lifecycle_manager_nav = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[nav2_params_file]
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
            # The bridge is disabled unless enable_motion:=true, and its own
            # defaults point at test topics, so wire it explicitly here.
            'enabled': ParameterValue(enable_motion, value_type=bool),
            'cmd_vel_topic': '/cmd_vel',
            'request_topic': '/api/sport/request',
            'max_vx': 0.40,
            'max_vy': 0.25,
            'max_wz': 0.80,
            'cmd_timeout_s': 0.25,
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(bringup_dir, 'config', 'nav2_params_slam.yaml'),
            description='Full path to Nav2 parameters file'
        ),
        DeclareLaunchArgument(
            'enable_motion',
            default_value='false',
            description='Let the bridge command the robot. false = Nav2 plans but nothing moves'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch the navigation RViz (set false for headless robot-side runs)'
        ),

        go2_mapping_cmd,
        rviz_nav,
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
