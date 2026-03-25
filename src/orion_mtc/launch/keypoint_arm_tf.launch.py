#!/usr/bin/env python3
"""
静态 TF（camera=视觉 keypoints 的 frame_id，sensor_link 与臂安装 TF 衔接）。

- camera -> sensor_link：
  p_sensor = R_{v->r} * p_camera。采用循环置换使典型点 (0, ~2.9, 0)_camera -> (~2.9,0,0)_sensor，
  再减左臂安装得 (~1.35,-0.565,0.283)_left_arm；旧矩阵会把 Y 映成 -X 导致整条链符号错。
  R_{v->r}=[[0,1,0],[0,0,1],[1,0,0]]，R_{r->v}=R_{v->r}^T=[[0,0,1],[1,0,0],[0,1,0]]。
  四元数 (qx,qy,qz,qw)=(0.5, 0.5, 0.5, 0.5)，平移暂为 0（需与实物再标定）。
- sensor_link -> left_arm_base / right_arm_base：仅平移（安装位姿）。

启动参数 use_mock_keypoints:=true 时不订阅 /keypoints，按 echo 样例注入假数据（可调 mock_kp_*）。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *_args, **_kwargs):
    use_mock = LaunchConfiguration("use_mock_keypoints").perform(context).lower() in ("true", "1", "yes")
    keypoint_params = {
        "input_topic": "/keypoints",
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
        "mock_kp_y": 2.9054482685810803,
        "mock_kp_z": -9.536743164059724e-05,
        "mock_period_sec": 1.0,
    }
    keypoint_node = Node(
        package="orion_mtc",
        executable="keypoint_to_arm_tf_node",
        name="keypoint_to_arm_tf",
        output="screen",
        parameters=[keypoint_params],
    )
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
    )
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
    )
    return [camera_to_sensor, left_tf, right_tf, keypoint_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_mock_keypoints",
                default_value="false",
                description="true：不订阅 /keypoints，用定时器注入假 Keypoints（与 ros2 echo 样例一致，可调参数 mock_kp_*）",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
