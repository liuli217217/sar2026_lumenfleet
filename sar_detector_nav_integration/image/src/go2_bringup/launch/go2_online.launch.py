#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rviz = LaunchConfiguration('rviz')
    driver = LaunchConfiguration('driver')
    camera = LaunchConfiguration('camera')
    camera_interface = LaunchConfiguration('camera_interface')
    camera_tf = LaunchConfiguration('camera_tf')
    camera_x = LaunchConfiguration('camera_x')
    camera_y = LaunchConfiguration('camera_y')
    camera_z = LaunchConfiguration('camera_z')
    camera_roll = LaunchConfiguration('camera_roll')
    camera_pitch = LaunchConfiguration('camera_pitch')
    camera_yaw = LaunchConfiguration('camera_yaw')

    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz',
        default_value='True',
        description='Launch RViz'
    )
    declare_driver_cmd = DeclareLaunchArgument(
        'driver',
        default_value='True',
        description='Launch go2_driver. Set false when /utlidar/* is already published '
                    'by the robot, so only one stack can command it'
    )
    declare_camera_cmd = DeclareLaunchArgument(
        'camera',
        default_value='True',
        description='Launch Go2 front camera bridge'
    )
    declare_camera_interface_cmd = DeclareLaunchArgument(
        'camera_interface',
        default_value='eth1',
        description='Network interface used by the Go2 front camera stream'
    )
    declare_camera_tf_cmd = DeclareLaunchArgument(
        'camera_tf',
        default_value='True',
        description='Publish static base_link to camera_face transform'
    )
    declare_camera_x_cmd = DeclareLaunchArgument(
        'camera_x',
        default_value='0.30',
        description='Camera x offset from base_link in meters'
    )
    declare_camera_y_cmd = DeclareLaunchArgument(
        'camera_y',
        default_value='0.0',
        description='Camera y offset from base_link in meters'
    )
    declare_camera_z_cmd = DeclareLaunchArgument(
        'camera_z',
        default_value='0.05',
        description='Camera z offset from base_link in meters'
    )
    declare_camera_roll_cmd = DeclareLaunchArgument(
        'camera_roll',
        default_value='-1.57079632679',
        description='Camera optical-frame roll relative to base_link in radians'
    )
    declare_camera_pitch_cmd = DeclareLaunchArgument(
        'camera_pitch',
        default_value='0.0',
        description='Camera optical-frame pitch relative to base_link in radians'
    )
    declare_camera_yaw_cmd = DeclareLaunchArgument(
        'camera_yaw',
        default_value='-1.57079632679',
        description='Camera optical-frame yaw relative to base_link in radians'
    )

    # Robot description + robot_state_publisher
    robot_description_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('go2_description'),
                'launch',
                'robot.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'False'
        }.items()
    )

    # Real robot driver
    driver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('go2_driver'),
                'launch',
                'go2_driver.launch.py'
            )
        ),
        condition=IfCondition(driver)
    )

    retime_cmd = Node(
        package='go2_bringup',
        executable='retime_utlidar.py',
        name='retime_utlidar',
        output='screen'
    )

    camera_cmd = Node(
        package='go2_camera_bridge',
        executable='go2_camera_bridge',
        name='go2_camera_node',
        output='screen',
        parameters=[{
            'interface': camera_interface,
        }],
        additional_env={
            'PYTHONNOUSERSITE': '1',
        },
        condition=IfCondition(camera)
    )

    camera_tf_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_face_static_tf',
        output='screen',
        arguments=[
            '--x', camera_x,
            '--y', camera_y,
            '--z', camera_z,
            '--roll', camera_roll,
            '--pitch', camera_pitch,
            '--yaw', camera_yaw,
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_face',
        ],
        condition=IfCondition(camera_tf)
    )

    # RViz
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('go2_rviz'),
                'launch',
                'rviz.launch.py'
            )
        ),
        condition=IfCondition(rviz)
    )

    ld = LaunchDescription()
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_driver_cmd)
    ld.add_action(declare_camera_cmd)
    ld.add_action(declare_camera_interface_cmd)
    ld.add_action(declare_camera_tf_cmd)
    ld.add_action(declare_camera_x_cmd)
    ld.add_action(declare_camera_y_cmd)
    ld.add_action(declare_camera_z_cmd)
    ld.add_action(declare_camera_roll_cmd)
    ld.add_action(declare_camera_pitch_cmd)
    ld.add_action(declare_camera_yaw_cmd)
    ld.add_action(robot_description_cmd)
    ld.add_action(retime_cmd)
    ld.add_action(driver_cmd)
    ld.add_action(camera_cmd)
    ld.add_action(camera_tf_cmd)
    ld.add_action(rviz_cmd)

    return ld
