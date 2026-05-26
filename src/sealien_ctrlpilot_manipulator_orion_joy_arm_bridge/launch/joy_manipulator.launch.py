#!/usr/bin/env python3
"""
启动 sealien_ctrlpilot_manipulator_orion_joy_arm_bridge::joy_manipulator_node：双路 sensor_msgs/Joy、joint_states，
通过参数文件配置映射到 /manipulator/* 服务与 FollowJointTrajectory。

Launch 参数 joy_params_file 默认指向包内 config/joy_manipulator.yaml，可按现场手柄改键位与话题名。
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """DeclareLaunchArgument + Node(joy_manipulator_node)。"""
    pkg = get_package_share_directory("sealien_ctrlpilot_manipulator_orion_joy_arm_bridge")
    default_cfg = pkg + "/config/joy_manipulator.yaml"

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "joy_params_file",
                default_value=default_cfg,
                description="sealien_ctrlpilot_manipulator_orion_joy_arm_bridge 参数文件路径",
            ),
            Node(
                package="sealien_ctrlpilot_manipulator_orion_joy_arm_bridge",
                executable="joy_manipulator_node",
                name="joy_manipulator_node",
                output="screen",
                parameters=[LaunchConfiguration("joy_params_file")],
            ),
        ]
    )
