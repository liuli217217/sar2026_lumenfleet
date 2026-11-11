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
    gui = LaunchConfiguration('gui')

    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz',
        default_value='True',
        description='Launch RViz'
    )

    declare_gui_cmd = DeclareLaunchArgument(
        'gui',
        default_value='True',
        description='Launch joint_state_publisher_gui'
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

    # Joint sliders for offline testing
    joint_state_publisher_gui_cmd = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(gui)
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
    ld.add_action(declare_gui_cmd)
    ld.add_action(robot_description_cmd)
    ld.add_action(joint_state_publisher_gui_cmd)
    ld.add_action(rviz_cmd)

    return ld