#!/usr/bin/env python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("orion_joy_arm_bridge")
    default_cfg = pkg + "/config/joy_manipulator.yaml"

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "joy_params_file",
                default_value=default_cfg,
                description="orion_joy_arm_bridge 参数文件路径",
            ),
            Node(
                package="orion_joy_arm_bridge",
                executable="joy_manipulator_node",
                name="joy_manipulator_node",
                output="screen",
                parameters=[LaunchConfiguration("joy_params_file")],
            ),
        ]
    )
