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
    address = LaunchConfiguration("address").perform(context)
    port = int(LaunchConfiguration("port").perform(context))
    ping_iv = float(LaunchConfiguration("websocket_ping_interval").perform(context))
    ping_to = float(LaunchConfiguration("websocket_ping_timeout").perform(context))
    unregister_to = float(LaunchConfiguration("unregister_timeout").perform(context))
    rosbridge_log_level = LaunchConfiguration("rosbridge_log_level").perform(context)
    if ping_iv < 0.0:
        ping_iv = 0.0
    if ping_to < 0.0:
        ping_to = 0.0
    if unregister_to < 0.5:
        unregister_to = 0.5
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
                "--ros-args",
                "--log-level",
                "rosbridge_websocket:={}".format(rosbridge_log_level),
            ],
            parameters=[
                {
                    "port": port,
                    "address": address,
                    "fragment_timeout": 600,
                    "max_message_size": 10000000,
                    # 失效连接更快注销，减少“closed websocket”告警持续时长。
                    "unregister_timeout": unregister_to,
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
            DeclareLaunchArgument(
                "address",
                default_value="0.0.0.0",
                description="rosbridge 绑定地址（默认跨机可访问；仅本机安全模式可改为 127.0.0.1）",
            ),
            DeclareLaunchArgument(
                "port",
                default_value="9091",
                description="rosbridge WebSocket 端口（默认 9091，避免与同事常用 9090 冲突；网页默认 ws 需一致或用 ?ws=）",
            ),
            DeclareLaunchArgument(
                "websocket_ping_interval",
                default_value="5.0",
                description="秒；>0 时 Tornado 周期性 WebSocket ping（僵尸连接快速回收默认 5）",
            ),
            DeclareLaunchArgument(
                "websocket_ping_timeout",
                default_value="10.0",
                description="秒；等待 pong（默认 10）",
            ),
            DeclareLaunchArgument(
                "unregister_timeout",
                default_value="0.5",
                description="秒；失效订阅/连接注销超时，默认 0.5 以缩短 closed websocket 告警窗口",
            ),
            DeclareLaunchArgument(
                "rosbridge_log_level",
                default_value="warn",
                description="rosbridge_websocket 日志级别（debug/info/warn/error/fatal）",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
