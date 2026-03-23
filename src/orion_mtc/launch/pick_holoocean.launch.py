#!/usr/bin/env python3
"""
抓取（MTC）与 HoloOcean 联调：关节状态来自 /holoocean/rov0/ArmSensor（right_arm 6DOF+夹爪）。
启动 MoveIt + RViz + HoloOcean 桥接节点、rosbridge（网页上位机用）、MTC。
MTC 执行：orion_mtc_node 将规划得到的轨迹发送到 arm_controller / hand_controller 的
FollowJointTrajectory action，由 trajectory_to_agent_bridge 接收并转为 AgentCommand 发布到
/holoocean/command/agent/arm，在 HoloOcean 中驱动机械臂（顺序：0=左臂，1=右臂）。
需能导入 holoocean_interfaces：通过环境变量 HOLOOCEAN_ROS_INSTALL 指定 holoocean-ros 的 install 目录，
或先 source 该工作区的 setup.bash，本 launch 会为桥接节点注入其 Python 路径。
"""
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
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

    pick_params_path = os.path.join(orion_mtc_share, "config", "pick_params.yaml")
    if os.path.isfile(pick_params_path):
        with open(pick_params_path, "r") as f:
            pick_params = yaml.safe_load(f)
        if pick_params:
            move_group_params.update(pick_params)
    runtime_policy_path = os.path.join(orion_mtc_share, "config", "runtime_policy.yaml")
    if os.path.isfile(runtime_policy_path):
        with open(runtime_policy_path, "r") as f:
            rp = yaml.safe_load(f)
        if rp and "orion_mtc_node" in rp and "ros__parameters" in rp:
            move_group_params.update(rp["orion_mtc_node"]["ros__parameters"])
        elif rp and "runtime_policy" in rp:
            move_group_params["runtime_policy"] = rp["runtime_policy"]

    demo_launch = os.path.join(orion_moveit_share, "launch", "demo.launch.py")
    bridge_params = os.path.join(orion_holoocean_share, "config", "holoocean_bridge_params.yaml")

    traj_bridge_params = {}
    if os.path.isfile(bridge_params):
        with open(bridge_params, "r") as f:
            bridge_cfg = yaml.safe_load(f)
        traj_section = (bridge_cfg or {}).get("trajectory_to_agent_bridge", {})
        if isinstance(traj_section, dict):
            traj_bridge_params = traj_section.get("ros__parameters", traj_section) or {}

    # 安全干净退出：延长 SIGINT 后等待时间再升级 SIGTERM/SIGKILL（launch 要求为可迭代/字符串）
    shutdown_timeouts = {"sigterm_timeout": "15", "sigkill_timeout": "5"}

    bridge_node = Node(
        package="orion_holoocean_bridge",
        executable="arm_sensor_to_joint_state",
        name="arm_sensor_to_joint_state",
        output="screen",
        parameters=[bridge_params] if os.path.isfile(bridge_params) else [],
        additional_env={"PYTHONPATH": _holoocean_interfaces_pythonpath()},
        **shutdown_timeouts,
    )
    trajectory_bridge_node = Node(
        package="orion_holoocean_bridge",
        executable="trajectory_to_agent_bridge",
        name="trajectory_to_agent_bridge",
        output="screen",
        parameters=[traj_bridge_params] if traj_bridge_params else [],
        additional_env={"PYTHONPATH": _holoocean_interfaces_pythonpath()},
        **shutdown_timeouts,
    )
    cable_sensor_to_pose_node = Node(
        package="orion_holoocean_bridge",
        executable="cable_sensor_to_object_pose",
        name="cable_sensor_to_object_pose",
        output="screen",
        parameters=[bridge_params] if os.path.isfile(bridge_params) else [],
        additional_env={"PYTHONPATH": _holoocean_interfaces_pythonpath()},
        **shutdown_timeouts,
    )
    # prefix 使用 stdbuf 无缓冲(0)，确保 MTC 任务树与 Failing stage(s) 等全部实时输出
    mtc_node = Node(
        package="orion_mtc",
        executable="mtc_node",
        name="orion_mtc_node",
        output="screen",
        parameters=[move_group_params],
        prefix="stdbuf -o 0 -e 0",
        **shutdown_timeouts,
    )

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(demo_launch),
            launch_arguments=[
                ("use_joint_state_gui", "false"),
                ("tf_under_manipulator", "true"),
            ],
        ),
        bridge_node,
        trajectory_bridge_node,
        cable_sensor_to_pose_node,
        mtc_node,
    ]

    try:
        from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
        rosbridge_share = get_package_share_directory("rosbridge_server")
        rosbridge_launch = os.path.join(rosbridge_share, "launch", "rosbridge_websocket_launch.xml")
        if os.path.isfile(rosbridge_launch):
            actions.insert(0, IncludeLaunchDescription(XMLLaunchDescriptionSource(rosbridge_launch)))
    except Exception:
        pass

    return LaunchDescription(actions)
