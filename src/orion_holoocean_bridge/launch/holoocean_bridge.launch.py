#!/usr/bin/env python3
"""启动 HoloOcean ArmSensor -> joint_states 桥接节点（单臂 right_arm 6DOF+夹爪）。"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("orion_holoocean_bridge")
    params_path = os.path.join(pkg_share, "config", "holoocean_bridge_params.yaml")
    use_params = os.path.isfile(params_path)

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "arm_sensor_topic",
                default_value="/holoocean/rov0/ArmSensor",
                description="HoloOcean ArmSensor topic to subscribe.",
            ),
            Node(
                package="orion_holoocean_bridge",
                executable="arm_sensor_to_joint_state",
                name="arm_sensor_to_joint_state",
                output="screen",
                parameters=[params_path] if use_params else [],
            ),
        ]
    )
