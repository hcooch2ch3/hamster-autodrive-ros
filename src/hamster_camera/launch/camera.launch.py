#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # Launch arguments
            DeclareLaunchArgument(
                "enable_object_following",
                default_value="false",
                description="Enable automatic object following",
            ),
            DeclareLaunchArgument(
                "detection_threshold",
                default_value="500",
                description="Minimum area for object detection",
            ),
            DeclareLaunchArgument(
                "canny_low",
                default_value="50",
                description="Canny edge detection low threshold",
            ),
            DeclareLaunchArgument(
                "canny_high",
                default_value="150",
                description="Canny edge detection high threshold",
            ),
            # Camera Node
            Node(
                package="hamster_camera",
                executable="camera_node",
                name="hamster_camera",
                output="screen",
                parameters=[
                    {
                        "enable_object_following": LaunchConfiguration(
                            "enable_object_following"
                        ),
                        "detection_threshold": LaunchConfiguration(
                            "detection_threshold"
                        ),
                        "canny_low": LaunchConfiguration("canny_low"),
                        "canny_high": LaunchConfiguration("canny_high"),
                    }
                ],
            ),
        ]
    )
