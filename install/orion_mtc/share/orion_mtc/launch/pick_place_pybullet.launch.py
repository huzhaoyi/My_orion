#!/usr/bin/env python3
"""
Pick-and-place 测试：在 PyBullet 仿真中运行 MTC 任务。
会启动 MoveIt + RViz + PyBullet 控制器，然后 MTC 节点自动执行一次抓放。
"""
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    orion_desc_share = get_package_share_directory("orion_description")
    orion_moveit_share = get_package_share_directory("orion_moveit_config")
    orion_pybullet_share = get_package_share_directory("orion_pybullet_sim")
    orion_mtc_share = get_package_share_directory("orion_mtc")

    urdf_path = os.path.join(orion_desc_share, "urdf", "orion.urdf")
    srdf_path = os.path.join(orion_moveit_share, "config", "orion.srdf")
    joint_limits_path = os.path.join(orion_moveit_share, "config", "joint_limits.yaml")
    ompl_path = os.path.join(orion_moveit_share, "config", "ompl_planning.yaml")
    pilz_path = os.path.join(orion_moveit_share, "config", "pilz_industrial_motion_planner_planning.yaml")
    pilz_cartesian_path = os.path.join(orion_moveit_share, "config", "pilz_cartesian_limits.yaml")
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
        joint_limits_cfg = yaml.safe_load(f)
    move_group_params.update(joint_limits_cfg)
    with open(pilz_cartesian_path, "r") as f:
        pilz_cartesian_cfg = yaml.safe_load(f)
    move_group_params["robot_description_planning"] = {
        "joint_limits": joint_limits_cfg.get("joint_limits", joint_limits_cfg),
        "cartesian_limits": pilz_cartesian_cfg["robot_description_planning"]["cartesian_limits"],
    }
    with open(ompl_path, "r") as f:
        ompl_config = yaml.safe_load(f)
    with open(pilz_path, "r") as f:
        pilz_config = yaml.safe_load(f)
    move_group_params["planning_pipelines"] = ["move_group", "pilz"]
    move_group_params["default_planning_pipeline"] = "move_group"
    move_group_params["move_group"] = ompl_config
    move_group_params["pilz"] = pilz_config
    with open(kinematics_path, "r") as f:
        move_group_params.update(yaml.safe_load(f))
    with open(controllers_path, "r") as f:
        move_group_params.update(yaml.safe_load(f))

    # 合并 MTC pick-place 参数（approach/lift/place/支撑面等，与官方 demo 对应）
    pick_place_params_path = os.path.join(orion_mtc_share, "config", "pick_place_params.yaml")
    if os.path.isfile(pick_place_params_path):
        with open(pick_place_params_path, "r") as f:
            pick_place_params = yaml.safe_load(f)
        if pick_place_params:
            move_group_params.update(pick_place_params)

    pybullet_launch = os.path.join(orion_pybullet_share, "launch", "pybullet_sim.launch.py")

    # MTC 节点延迟 12 秒启动，等待 move_group 与 PyBullet 控制器就绪
    mtc_node = Node(
        package="orion_mtc",
        executable="mtc_node",
        name="orion_mtc_node",
        output="screen",
        parameters=[move_group_params],
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(pybullet_launch),
            ),
            TimerAction(period=12.0, actions=[mtc_node]),
        ]
    )
