#!/usr/bin/env python3
"""
Orion URDF 可视化（无 MoveIt）：robot_state_publisher 读 urdf 字符串，joint_state_publisher_gui 送关节角。

mesh 路径将 package 相对路径替换为 share 目录绝对路径，便于 RViz 加载 STL。
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """返回仅含 RSP 与 joint_state_publisher_gui 的 LaunchDescription。"""
    pkg_share = get_package_share_directory("sealien_ctrlpilot_manipulator_orion_description")
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
