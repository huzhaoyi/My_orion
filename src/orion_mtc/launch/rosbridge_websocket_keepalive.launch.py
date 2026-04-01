#!/usr/bin/env python3
"""
网页上位机用 rosbridge + rosapi。相对官方 rosbridge_websocket_launch.xml 增加：
- websocket_ping_interval / websocket_ping_timeout：Tornado 层保活。

注意：Humble 的 rosbridge 将 ping 间隔默认值设为整数 0，参数类型被定为 INTEGER；若用 launch parameters 传入 25.0 会触发 DOUBLE/INTEGER 冲突。
因此这两项通过 **可执行文件命令行** `--websocket_ping_interval` / `--websocket_ping_timeout` 传入（ argparse 在 declare 前得到 float）。

pick_holoocean 默认包含本 launch。也可单独：ros2 launch orion_mtc rosbridge_websocket_keepalive.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *_args, **_kwargs):
    port = int(LaunchConfiguration("port").perform(context))
    ping_iv = float(LaunchConfiguration("websocket_ping_interval").perform(context))
    ping_to = float(LaunchConfiguration("websocket_ping_timeout").perform(context))
    return [
        Node(
            package="rosbridge_server",
            executable="rosbridge_websocket",
            name="rosbridge_websocket",
            output="screen",
            arguments=[
                "--websocket_ping_interval",
                str(ping_iv),
                "--websocket_ping_timeout",
                str(ping_to),
            ],
            parameters=[
                {
                    "port": port,
                    "fragment_timeout": 600,
                    "max_message_size": 10000000,
                    "unregister_timeout": 10.0,
                    "delay_between_messages": 0.0,
                    "use_compression": False,
                    "call_services_in_new_thread": False,
                    "default_call_service_timeout": 0.0,
                    "send_action_goals_in_new_thread": False,
                    "topics_glob": "",
                    "services_glob": "",
                    "params_glob": "",
                    "bson_only_mode": False,
                }
            ],
        ),
        Node(
            package="rosapi",
            executable="rosapi_node",
            name="rosapi",
            output="screen",
            parameters=[
                {
                    "topics_glob": "",
                    "services_glob": "",
                    "params_glob": "",
                    "params_timeout": 5.0,
                }
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("port", default_value="9090", description="rosbridge WebSocket 端口"),
            DeclareLaunchArgument(
                "websocket_ping_interval",
                default_value="25.0",
                description="秒；>0 时 Tornado 周期性 WebSocket ping（官方默认可为 0）",
            ),
            DeclareLaunchArgument(
                "websocket_ping_timeout",
                default_value="120.0",
                description="秒；等待 pong",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
