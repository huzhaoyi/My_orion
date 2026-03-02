#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("orion_description")
    urdf_path = os.path.join(pkg_share, "urdf", "orion.urdf")
    meshes_abs = os.path.join(pkg_share, "meshes")

    with open(urdf_path, "r") as f:
        urdf_content = f.read()
    urdf_content = urdf_content.replace("../meshes/", meshes_abs + "/")

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": urdf_content}],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                output="screen",
            ),
        ]
    )
