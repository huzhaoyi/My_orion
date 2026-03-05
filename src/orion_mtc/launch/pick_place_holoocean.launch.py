#!/usr/bin/env python3
"""
Pick-and-place 与 HoloOcean 联调：关节状态来自 /holoocean/rov0/ArmSensor（right_arm 6DOF+夹爪）。
启动 MoveIt + RViz + HoloOcean 桥接节点，MTC 使用 HoloOcean 机械臂当前状态进行规划。
注意：轨迹执行需 HoloOcean 或其它控制器提供 FollowJointTrajectory action。
需能导入 holoocean_interfaces：通过环境变量 HOLOOCEAN_ROS_INSTALL 指定 holoocean-ros 的 install 目录，
或先 source 该工作区的 setup.bash，本 launch 会为桥接节点注入其 Python 路径。
"""
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _holoocean_interfaces_pythonpath():
    """返回用于导入 holoocean_interfaces 的 PYTHONPATH 前缀（holoocean-ros install 下的 Python 包路径）。"""
    install_dir = os.environ.get("HOLOOCEAN_ROS_INSTALL", "/home/huzy/holoocean-ros/install")
    # 常见 colcon 布局：local/lib/python3.10/dist-packages 或 lib/python3.10/site-packages
    candidates = [
        os.path.join(install_dir, "holoocean_interfaces", "local", "lib", "python3.10", "dist-packages"),
        os.path.join(install_dir, "holoocean_interfaces", "lib", "python3.10", "site-packages"),
    ]
    for candidate in candidates:
        if os.path.isdir(candidate):
            existing = os.environ.get("PYTHONPATH", "")
            return candidate + (os.pathsep + existing if existing else "")
    # 未找到则仍返回第一候选路径，便于 source 后或路径存在时可用
    existing = os.environ.get("PYTHONPATH", "")
    return candidates[0] + (os.pathsep + existing if existing else "")


def generate_launch_description():
    orion_desc_share = get_package_share_directory("orion_description")
    orion_moveit_share = get_package_share_directory("orion_moveit_config")
    orion_holoocean_share = get_package_share_directory("orion_holoocean_bridge")
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

    pick_place_params_path = os.path.join(orion_mtc_share, "config", "pick_place_params.yaml")
    if os.path.isfile(pick_place_params_path):
        with open(pick_place_params_path, "r") as f:
            pick_place_params = yaml.safe_load(f)
        if pick_place_params:
            move_group_params.update(pick_place_params)

    demo_launch = os.path.join(orion_moveit_share, "launch", "demo.launch.py")
    bridge_params = os.path.join(orion_holoocean_share, "config", "holoocean_bridge_params.yaml")

    bridge_node = Node(
        package="orion_holoocean_bridge",
        executable="arm_sensor_to_joint_state",
        name="arm_sensor_to_joint_state",
        output="screen",
        parameters=[bridge_params] if os.path.isfile(bridge_params) else [],
        additional_env={"PYTHONPATH": _holoocean_interfaces_pythonpath()},
    )
    trajectory_bridge_node = Node(
        package="orion_holoocean_bridge",
        executable="trajectory_to_agent_bridge",
        name="trajectory_to_agent_bridge",
        output="screen",
        additional_env={"PYTHONPATH": _holoocean_interfaces_pythonpath()},
    )
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
                PythonLaunchDescriptionSource(demo_launch),
                launch_arguments=[("use_joint_state_gui", "false")],
            ),
            bridge_node,
            trajectory_bridge_node,
            TimerAction(period=12.0, actions=[mtc_node]),
        ]
    )
