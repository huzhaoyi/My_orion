#!/usr/bin/env python3
"""启动 PyBullet 仿真控制器 + MoveIt（无 joint_state_publisher_gui），支持 Plan & Execute。"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    demo_launch = os.path.join(
        get_package_share_directory("orion_moveit_config"), "launch", "demo.launch.py"
    )
    use_gui = LaunchConfiguration("use_gui", default="false")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_gui", default_value="false", description="Open PyBullet GUI window."),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(demo_launch),
                launch_arguments=[("use_joint_state_gui", "false")],
            ),
            Node(
                package="orion_pybullet_sim",
                executable="pybullet_controller",
                name="pybullet_controller",
                output="screen",
                parameters=[{"use_gui": use_gui}],
            ),
        ]
    )
