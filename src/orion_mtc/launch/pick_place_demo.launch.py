#!/usr/bin/env python3

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    orion_desc_share = get_package_share_directory("orion_description")
    orion_moveit_share = get_package_share_directory("orion_moveit_config")

    urdf_path = os.path.join(orion_desc_share, "urdf", "orion.urdf")
    srdf_path = os.path.join(orion_moveit_share, "config", "orion.srdf")
    joint_limits_path = os.path.join(orion_moveit_share, "config", "joint_limits.yaml")
    ompl_path = os.path.join(orion_moveit_share, "config", "ompl_planning.yaml")
    kinematics_path = os.path.join(orion_moveit_share, "config", "kinematics.yaml")
    controllers_path = os.path.join(orion_moveit_share, "config", "moveit_controllers.yaml")

    with open(urdf_path, "r") as f:
        urdf_content = f.read()
    urdf_content = urdf_content.replace("../meshes/", "package://orion_description/meshes/")

    with open(srdf_path, "r") as f:
        srdf_content = f.read()

    move_group_params = {
        "robot_description": urdf_content,
        "robot_description_semantic": srdf_content,
        "publish_robot_description": True,
        "publish_robot_description_semantic": True,
    }

    with open(joint_limits_path, "r") as f:
        move_group_params.update(yaml.safe_load(f))
    with open(ompl_path, "r") as f:
        ompl_config = yaml.safe_load(f)
    move_group_params["planning_pipelines"] = ["move_group"]
    move_group_params["default_planning_pipeline"] = "move_group"
    move_group_params["move_group"] = ompl_config
    with open(kinematics_path, "r") as f:
        move_group_params.update(yaml.safe_load(f))
    with open(controllers_path, "r") as f:
        move_group_params.update(yaml.safe_load(f))

    demo_launch_dir = os.path.join(get_package_share_directory("orion_moveit_config"), "launch")
    demo_launch = os.path.join(demo_launch_dir, "demo.launch.py")

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(demo_launch),
            ),
            Node(
                package="orion_mtc",
                executable="mtc_node",
                name="orion_mtc_node",
                output="screen",
                parameters=[move_group_params],
            ),
        ]
    )
