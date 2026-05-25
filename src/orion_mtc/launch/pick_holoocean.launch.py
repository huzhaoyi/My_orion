#!/usr/bin/env python3
"""
兼容入口：转发至 sealien_ctrlpilot_manipulator_orion/sealien_ctrlpilot_manipulator_orion.launch.py。

请优先使用：
  ros2 launch sealien_ctrlpilot_manipulator_orion sealien_ctrlpilot_manipulator_orion.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    share = get_package_share_directory("sealien_ctrlpilot_manipulator_orion")
    main_launch = os.path.join(share, "launch", "sealien_ctrlpilot_manipulator_orion.launch.py")

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(main_launch),
        ),
    ])
