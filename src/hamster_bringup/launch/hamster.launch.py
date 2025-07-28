#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'enable_camera',
            default_value='true',
            description='Enable camera processing'
        ),
        
        DeclareLaunchArgument(
            'enable_teleop',
            default_value='false',
            description='Enable keyboard teleoperation'
        ),
        
        DeclareLaunchArgument(
            'enable_object_following',
            default_value='false',
            description='Enable automatic object following'
        ),
        
        # Hamster Driver Node (always start)
        Node(
            package='hamster_bringup',
            executable='hamster_driver_node',
            name='hamster_driver',
            output='screen',
            parameters=[],
        ),
        
        # Camera Node (conditional)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('hamster_camera'),
                    'launch',
                    'camera.launch.py'
                ])
            ]),
            launch_arguments={
                'enable_object_following': LaunchConfiguration('enable_object_following')
            }.items(),
            condition=IfCondition(LaunchConfiguration('enable_camera'))
        ),
        
        # Teleop Node (conditional)
        Node(
            package='hamster_teleop',
            executable='teleop_node',
            name='hamster_teleop',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_teleop'))
        ),
    ])
