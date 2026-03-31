#!/usr/bin/env python3
"""
Keypoints 经 TF 变换到左右臂基座相关坐标系并打印（keypoint_to_arm_tf_node）。

两种模式（use_platform_tf）：

- false（默认）：本地感知调试。分两种情况：
  * tf_under_manipulator=false（默认）：独立小 TF 树 camera→sensor_link→left_arm_base/right_arm_base；
    与 **仅** 含 base_link→Link* 的 URDF **不连通**，勿与 pick_holoocean 同时使用。
  * tf_under_manipulator=true（如 pick_holoocean）：**不把** left_arm_base 挂在 sensor 下（避免与 URDF 链重名或断树），
    改为 base_link→camera→sensor_link，中心线在 **base_link** 下拟合，可与 MTC 的 base_link 规划一致。

- true：对接 sealien_ctrlpilot_location（rov.urdf_simulate.xml 等）。仅启动 keypoint 节点；帧名与
  robot_state_publisher 一致：sensor_camera1、sensor_left_roboticarm、sensor_right_roboticarm。
  需已运行机体 URDF + odom→base_link 等，使 TF 树完整。

启动参数 use_mock_keypoints:=true 时不订阅 /perception/sonar/keypoints，按参数注入假数据。

tf_under_manipulator:=true 时节点与 static_transform_publisher 将 tf/tf_static 重映射到
/manipulator/tf、/manipulator/tf_static，与 orion_moveit_config demo.launch（MTC 联调）一致。

generate_launch_description 经 OpaqueFunction 按 use_platform_tf/use_mock_keypoints 组合 static 与 keypoint 节点参数。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *_args, **_kwargs):
    """根据 LaunchConfiguration 组装 static TF 列表与 keypoint_to_arm_tf 节点（平台模式省略 static）。"""
    use_mock = LaunchConfiguration("use_mock_keypoints").perform(context).lower() in ("true", "1", "yes")
    use_platform = LaunchConfiguration("use_platform_tf").perform(context).lower() in ("true", "1", "yes")
    mock_preset = LaunchConfiguration("mock_preset").perform(context)
    tf_under = LaunchConfiguration("tf_under_manipulator").perform(context).lower() in ("true", "1", "yes")
    tf_remappings = (
        [("tf", "/manipulator/tf"), ("tf_static", "/manipulator/tf_static")] if tf_under else []
    )

    if use_platform:
        keypoint_params = {
            "input_topic": "/perception/sonar/keypoints",
            "source_frame_override": "sensor_camera1",
            "left_arm_frame": "sensor_left_roboticarm",
            "right_arm_frame": "sensor_right_roboticarm",
            "tf_timeout_sec": 0.5,
            "tf_use_latest_timestamp": True,
            "qos_best_effort": False,
            "qos_depth": 10,
            "use_mock_keypoints": use_mock,
            "mock_frame_id": "sensor_camera1",
            "mock_kp_x": 0.0,
            "mock_kp_y": 2.9,
            "mock_kp_z": 0.0,
            "mock_period_sec": 1.0,
            "mock_preset": mock_preset,
        }
    elif tf_under and not use_platform:
        # pick_holoocean / MTC：URDF 为 world→base_link→Link*，无 left_arm_base；将感知树挂到 base_link。
        keypoint_params = {
            "input_topic": "/perception/sonar/keypoints",
            "source_frame_override": "sensor_link",
            "left_arm_frame": "base_link",
            "right_arm_frame": "base_link",
            "tf_timeout_sec": 0.5,
            "tf_use_latest_timestamp": True,
            "qos_best_effort": False,
            "qos_depth": 10,
            "use_mock_keypoints": use_mock,
            "mock_frame_id": "camera",
            "mock_kp_x": 0.0,
            "mock_kp_y": 2.9,
            "mock_kp_z": 0.0,
            "mock_period_sec": 1.0,
            "mock_preset": mock_preset,
        }
    else:
        keypoint_params = {
            "input_topic": "/perception/sonar/keypoints",
            "source_frame_override": "sensor_link",
            "left_arm_frame": "left_arm_base",
            "right_arm_frame": "right_arm_base",
            "tf_timeout_sec": 0.5,
            "tf_use_latest_timestamp": True,
            "qos_best_effort": False,
            "qos_depth": 10,
            "use_mock_keypoints": use_mock,
            "mock_frame_id": "camera",
            "mock_kp_x": 0.0,
            "mock_kp_y": 2.9,
            "mock_kp_z": 0.0,
            "mock_period_sec": 1.0,
            "mock_preset": mock_preset,
        }

    keypoint_node = Node(
        package="orion_mtc",
        executable="keypoint_to_arm_tf_node",
        name="keypoint_to_arm_tf",
        output="screen",
        parameters=[keypoint_params],
        remappings=tf_remappings,
    )

    if use_platform:
        return [keypoint_node]

    camera_to_sensor = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_camera_to_sensor_link",
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0",
            "--qx",
            "0.5",
            "--qy",
            "0.5",
            "--qz",
            "0.5",
            "--qw",
            "0.5",
            "--frame-id",
            "camera",
            "--child-frame-id",
            "sensor_link",
        ],
        output="screen",
        remappings=tf_remappings,
    )

    if tf_under:
        base_to_camera = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_base_link_to_camera",
            arguments=[
                "--x",
                "0",
                "--y",
                "0",
                "--z",
                "0",
                "--qx",
                "0",
                "--qy",
                "0",
                "--qz",
                "0",
                "--qw",
                "1",
                "--frame-id",
                "base_link",
                "--child-frame-id",
                "camera",
            ],
            output="screen",
            remappings=tf_remappings,
        )
        return [base_to_camera, camera_to_sensor, keypoint_node]

    left_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_sensor_link_to_left_arm_base",
        arguments=[
            "--x",
            "1.55",
            "--y",
            "0.5653",
            "--z",
            "-0.283628",
            "--qx",
            "0",
            "--qy",
            "0",
            "--qz",
            "0",
            "--qw",
            "1",
            "--frame-id",
            "sensor_link",
            "--child-frame-id",
            "left_arm_base",
        ],
        output="screen",
        remappings=tf_remappings,
    )
    right_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_sensor_link_to_right_arm_base",
        arguments=[
            "--x",
            "1.55",
            "--y",
            "-0.5653",
            "--z",
            "-0.283628",
            "--qx",
            "0",
            "--qy",
            "0",
            "--qz",
            "0",
            "--qw",
            "1",
            "--frame-id",
            "sensor_link",
            "--child-frame-id",
            "right_arm_base",
        ],
        output="screen",
        remappings=tf_remappings,
    )
    return [camera_to_sensor, left_tf, right_tf, keypoint_node]


def generate_launch_description():
    """声明 use_mock_keypoints / use_platform_tf，由 OpaqueFunction 展开具体 Node 列表。"""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_mock_keypoints",
                default_value="false",
                description=(
                    "true：不订阅 /perception/sonar/keypoints，用定时器注入假 Keypoints（与 ros2 echo 样例一致，可调参数 mock_kp_*）"
                ),
            ),
            DeclareLaunchArgument(
                "use_platform_tf",
                default_value="false",
                description=(
                    "true：仅启动 keypoint 节点，帧名对齐 sealien_ctrlpilot_location URDF（sensor_camera1、"
                    "sensor_left_roboticarm、sensor_right_roboticarm），不启动本地 static_transform_publisher"
                ),
            ),
            DeclareLaunchArgument(
                "mock_preset",
                default_value="sonar_cable_9",
                description=(
                    "use_mock_keypoints 时注入方式：sonar_cable_9=内置 9 点（与 centerline 测试）；legacy_single=单点 mock_kp_*"
                ),
            ),
            DeclareLaunchArgument(
                "tf_under_manipulator",
                default_value="false",
                description=(
                    "true：tf/tf_static 重映射到 /manipulator/*，与 pick_holoocean / MTC demo 一致"
                ),
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
