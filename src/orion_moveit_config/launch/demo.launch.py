#!/usr/bin/env python3

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    use_joint_state_gui = LaunchConfiguration("use_joint_state_gui", default="true")
    orion_desc_share = get_package_share_directory("orion_description")
    orion_moveit_share = get_package_share_directory("orion_moveit_config")

    urdf_path = os.path.join(orion_desc_share, "urdf", "orion.urdf")
    rviz_path = os.path.join(orion_moveit_share, "config", "moveit.rviz")
    srdf_path = os.path.join(orion_moveit_share, "config", "orion.srdf")
    joint_limits_path = os.path.join(orion_moveit_share, "config", "joint_limits.yaml")
    ompl_path = os.path.join(orion_moveit_share, "config", "ompl_planning.yaml")
    pilz_path = os.path.join(orion_moveit_share, "config", "pilz_industrial_motion_planner_planning.yaml")
    pilz_cartesian_path = os.path.join(orion_moveit_share, "config", "pilz_cartesian_limits.yaml")
    kinematics_path = os.path.join(orion_moveit_share, "config", "kinematics.yaml")
    controllers_path = os.path.join(orion_moveit_share, "config", "moveit_controllers.yaml")

    with open(urdf_path, "r") as f:
        urdf_content = f.read()
    # 使用 package:// 便于 MoveIt resource_retriever 通过 ament 解析 mesh
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
        kinematics_config = yaml.safe_load(f)
    move_group_params.update(kinematics_config)

    with open(controllers_path, "r") as f:
        controllers_config = yaml.safe_load(f)
    move_group_params.update(controllers_config)

    # capabilities 为单字符串：默认能力与 MTC 执行能力，供 orion_mtc 的 execute_task_solution 使用
    # 注：MoveGroupExecuteService 在 Humble 中无对应插件，已省略
    move_group_params["capabilities"] = (
        "move_group/MoveGroupCartesianPathService "
        "move_group/MoveGroupExecuteTrajectoryAction "
        "move_group/MoveGroupKinematicsService "
        "move_group/MoveGroupMoveAction "
        "move_group/MoveGroupPlanService "
        "move_group/MoveGroupQueryPlannersService "
        "move_group/MoveGroupStateValidationService "
        "move_group/MoveGroupGetPlanningSceneService "
        "move_group/ApplyPlanningSceneService "
        "move_group/ClearOctomapService "
        "move_group/TfPublisher "
        "move_group/ExecuteTaskSolutionCapability"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_joint_state_gui",
                default_value="true",
                description="If true, start joint_state_publisher_gui; if false, joint_states come from external (e.g. PyBullet).",
            ),
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
                condition=IfCondition(use_joint_state_gui),
            ),
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                name="move_group",
                output="screen",
                parameters=[move_group_params],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_path],
                output="screen",
                parameters=[
                    {"robot_description": urdf_content},
                    {"robot_description_semantic": srdf_content},
                    {"robot_description_kinematics": kinematics_config},
                ],
            ),
        ]
    )
