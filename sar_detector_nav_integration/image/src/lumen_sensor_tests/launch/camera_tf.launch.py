from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Defaults are robot 'seek''s captured values (2026-10-26): the direct
    # base_link -> go2_front_camera_optical shortcut (URDF-derived, bypasses
    # Head_upper/Head_lower) and the organizers' go2_front_camera_optical ->
    # camera_link calibration, read live off /tf_static. Each competition
    # robot has its own calibration -- override these, don't assume they
    # transfer. camera_link -> camera_depth/color_frame and their optical
    # children are published automatically by realsense2_camera itself; do
    # not duplicate them here.
    return LaunchDescription([
        DeclareLaunchArgument('cam_x', default_value='0.33'),
        DeclareLaunchArgument('cam_y', default_value='0.0'),
        DeclareLaunchArgument('cam_z', default_value='0.04'),
        DeclareLaunchArgument('cam_roll', default_value='-1.5708'),
        DeclareLaunchArgument('cam_pitch', default_value='0.0'),
        DeclareLaunchArgument('cam_yaw', default_value='-1.5708'),
        DeclareLaunchArgument('rs_x', default_value='-0.020514283'),
        DeclareLaunchArgument('rs_y', default_value='-0.044518822'),
        DeclareLaunchArgument('rs_z', default_value='0.009181702'),
        DeclareLaunchArgument('rs_qx', default_value='0.494388503'),
        DeclareLaunchArgument('rs_qy', default_value='-0.489520126'),
        DeclareLaunchArgument('rs_qz', default_value='0.508376422'),
        DeclareLaunchArgument('rs_qw', default_value='0.507447984'),

        Node(package='tf2_ros', executable='static_transform_publisher',
             name='base_link_to_go2_front_camera_optical', output='screen',
             arguments=[
                 '--x', LaunchConfiguration('cam_x'),
                 '--y', LaunchConfiguration('cam_y'),
                 '--z', LaunchConfiguration('cam_z'),
                 '--roll', LaunchConfiguration('cam_roll'),
                 '--pitch', LaunchConfiguration('cam_pitch'),
                 '--yaw', LaunchConfiguration('cam_yaw'),
                 '--frame-id', 'base_link',
                 '--child-frame-id', 'go2_front_camera_optical',
             ]),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='go2_front_camera_optical_to_camera_link', output='screen',
             arguments=[
                 '--x', LaunchConfiguration('rs_x'),
                 '--y', LaunchConfiguration('rs_y'),
                 '--z', LaunchConfiguration('rs_z'),
                 '--qx', LaunchConfiguration('rs_qx'),
                 '--qy', LaunchConfiguration('rs_qy'),
                 '--qz', LaunchConfiguration('rs_qz'),
                 '--qw', LaunchConfiguration('rs_qw'),
                 '--frame-id', 'go2_front_camera_optical',
                 '--child-frame-id', 'camera_link',
             ]),
    ])
