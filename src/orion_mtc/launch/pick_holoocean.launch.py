#!/usr/bin/env python3
"""
抓取（MTC）与 HoloOcean 联调：关节状态来自 /holoocean/rov0/ArmSensor（right_arm 6DOF+夹爪）。
启动 MoveIt + RViz + HoloOcean 桥接节点、rosbridge（网页上位机用）、MTC。
默认同时包含 keypoint_to_arm_tf（融合抓取话题 object_pose_fused），可用 start_keypoint_to_arm_tf:=false 关闭。
MTC 执行：orion_mtc_node 将规划得到的轨迹发送到 arm_controller / hand_controller 的
FollowJointTrajectory action，由 trajectory_to_agent_bridge 接收并转为 AgentCommand 发布到
/holoocean/command/agent/arm，在 HoloOcean 中驱动机械臂（顺序：0=左臂，1=右臂）。
需能导入 holoocean_interfaces：通过环境变量 HOLOOCEAN_ROS_INSTALL 指定 holoocean-ros 的 install 目录，
或先 source 该工作区的 setup.bash，本 launch 会为桥接节点注入其 Python 路径。
是否默认启动手柄桥接 / RViz：见 orion_mtc/config/orion_mtc_params.yaml 中 use_joy_manipulator、start_rviz（launch 参数仍可覆盖）。
"""
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
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


def _yaml_bool_to_launch_arg_string(value) -> str:
    """将 YAML 中的布尔语义转为 launch 参数 true/false 字符串。"""
    if value is True:
        return "true"
    if value is False:
        return "false"
    if isinstance(value, str):
        s = value.strip().lower()
        if s in ("true", "1", "yes", "on"):
            return "true"
        if s in ("false", "0", "no", "off"):
            return "false"
    return "false"


_LAUNCH_ONLY_PARAM_KEYS = frozenset({"use_joy_manipulator", "start_rviz"})


def generate_launch_description():
    """
    组合 MoveIt demo、HoloOcean 四桥接节点、rosbridge、MTC、可选 joy_manipulator 与 RViz。

    从 orion_mtc_params.yaml 读取默认 use_joy_manipulator/start_rviz；为 trajectory 节点注入 holoocean_interfaces 的 PYTHONPATH。
    """
    orion_desc_share = get_package_share_directory("orion_description")
    orion_moveit_share = get_package_share_directory("orion_moveit_config")
    orion_holoocean_share = get_package_share_directory("orion_holoocean_bridge")
    orion_mtc_share = get_package_share_directory("orion_mtc")
    orion_joy_share = get_package_share_directory("orion_joy_arm_bridge")
    joy_params_path = os.path.join(orion_joy_share, "config", "joy_manipulator.yaml")

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

    mtc_app_params_path = os.path.join(orion_mtc_share, "config", "orion_mtc_params.yaml")
    mtc_app_raw = {}
    if os.path.isfile(mtc_app_params_path):
        with open(mtc_app_params_path, "r") as f:
            loaded_mtc = yaml.safe_load(f)
        if isinstance(loaded_mtc, dict):
            mtc_app_raw = loaded_mtc
    use_joy_default = _yaml_bool_to_launch_arg_string(mtc_app_raw.get("use_joy_manipulator", False))
    start_rviz_default = _yaml_bool_to_launch_arg_string(mtc_app_raw.get("start_rviz", False))
    mtc_for_node = {k: v for k, v in mtc_app_raw.items() if k not in _LAUNCH_ONLY_PARAM_KEYS}
    if mtc_for_node:
        move_group_params.update(mtc_for_node)

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

    joy_node = Node(
        package="orion_joy_arm_bridge",
        executable="joy_manipulator_node",
        name="joy_manipulator_node",
        output="screen",
        parameters=[joy_params_path] if os.path.isfile(joy_params_path) else [],
        condition=IfCondition(LaunchConfiguration("use_joy_manipulator")),
        **shutdown_timeouts,
    )

    arg_use_joy = DeclareLaunchArgument(
        "use_joy_manipulator",
        default_value=use_joy_default,
        description=(
            "true：启动 orion_joy_arm_bridge；默认取自 share/orion_mtc/config/orion_mtc_params.yaml"
        ),
    )
    arg_start_rviz = DeclareLaunchArgument(
        "start_rviz",
        default_value=start_rviz_default,
        description=(
            "true：启动 RViz2；默认取自 share/orion_mtc/config/orion_mtc_params.yaml 中 start_rviz"
        ),
    )

    keypoint_launch = os.path.join(orion_mtc_share, "launch", "keypoint_arm_tf.launch.py")

    arg_tf_under = DeclareLaunchArgument(
        "tf_under_manipulator",
        default_value="true",
        description="与 MoveIt demo 一致；true 时 TF 在 /manipulator/tf，keypoint 同步重映射",
    )
    arg_start_keypoint = DeclareLaunchArgument(
        "start_keypoint_to_arm_tf",
        default_value="true",
        description="true：一并启动 keypoint_arm_tf.launch（默认订阅真实 /perception/sonar/keypoints；离线可 keypoint_use_mock_keypoints:=true）",
    )
    arg_keypoint_mock = DeclareLaunchArgument(
        "keypoint_use_mock_keypoints",
        default_value="false",
        description="false：订阅真实 Keypoints；true：定时器注入假数据（离线调 TF）",
    )
    arg_keypoint_platform = DeclareLaunchArgument(
        "keypoint_use_platform_tf",
        default_value="false",
        description="真机平台 URDF（sensor_camera1 等）时设 true，与 keypoint_arm_tf 一致",
    )
    arg_keypoint_preset = DeclareLaunchArgument(
        "keypoint_mock_preset",
        default_value="sonar_cable_9",
        description="mock 预设，同 keypoint_arm_tf.launch",
    )

    keypoint_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(keypoint_launch),
        launch_arguments=[
            ("use_mock_keypoints", LaunchConfiguration("keypoint_use_mock_keypoints")),
            ("use_platform_tf", LaunchConfiguration("keypoint_use_platform_tf")),
            ("mock_preset", LaunchConfiguration("keypoint_mock_preset")),
            ("tf_under_manipulator", LaunchConfiguration("tf_under_manipulator")),
        ],
        condition=IfCondition(LaunchConfiguration("start_keypoint_to_arm_tf")),
    )

    actions = [
        arg_use_joy,
        arg_start_rviz,
        arg_tf_under,
        arg_start_keypoint,
        arg_keypoint_mock,
        arg_keypoint_platform,
        arg_keypoint_preset,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(demo_launch),
            launch_arguments=[
                ("use_joint_state_gui", "false"),
                ("tf_under_manipulator", LaunchConfiguration("tf_under_manipulator")),
                ("start_rviz", LaunchConfiguration("start_rviz")),
            ],
        ),
        bridge_node,
        trajectory_bridge_node,
        cable_sensor_to_pose_node,
        joy_node,
        mtc_node,
        keypoint_include,
    ]

    try:
        from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
        rosbridge_share = get_package_share_directory("rosbridge_server")
        rosbridge_launch = os.path.join(rosbridge_share, "launch", "rosbridge_websocket_launch.xml")
        if os.path.isfile(rosbridge_launch):
            actions.insert(1, IncludeLaunchDescription(XMLLaunchDescriptionSource(rosbridge_launch)))
    except Exception:
        pass

    return LaunchDescription(actions)
