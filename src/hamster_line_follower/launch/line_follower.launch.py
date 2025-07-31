#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'linear_speed',
            default_value='0.2',
            description='Linear speed for line following'
        ),
        DeclareLaunchArgument(
            'angular_gain',
            default_value='0.005',
            description='Angular velocity gain'
        ),
        DeclareLaunchArgument(
            'kp',
            default_value='0.8',
            description='PID proportional gain'
        ),
        DeclareLaunchArgument(
            'ki',
            default_value='0.0',
            description='PID integral gain'
        ),
        DeclareLaunchArgument(
            'kd',
            default_value='0.1',
            description='PID derivative gain'
        ),
        DeclareLaunchArgument(
            'brightness_threshold',
            default_value='200',
            description='Brightness threshold for white line detection'
        ),
        
        # Line Follower Node
        Node(
            package='hamster_line_follower',
            executable='line_follower_node',
            name='line_follower',
            output='screen',
            parameters=[{
                'linear_speed': LaunchConfiguration('linear_speed'),
                'angular_gain': LaunchConfiguration('angular_gain'),
                'kp': LaunchConfiguration('kp'),
                'ki': LaunchConfiguration('ki'),
                'kd': LaunchConfiguration('kd'),
                'roi_height_ratio': 0.3,
                'roi_y_offset': 0.7,
                'blur_kernel_size': 5,
                'canny_low': 40,
                'canny_high': 120,
                'min_line_length': 8,
                'max_line_gap': 15,
                'use_contour_detection': True,
                'white_line_detection': True,
                'brightness_threshold': LaunchConfiguration('brightness_threshold'),
                'adaptive_threshold': True,
                'contrast_enhancement': True,
                'color_filtering': True,
                'saturation_max': 30,
                'line_thinning': True,
                'merge_close_lines': True,
            }]
        ),
    ])