#!/usr/bin/env python3
"""
将高频 /joint_states 中继为 Web 低频话题，降低 rosbridge 与浏览器长期负载。

默认：
- 输入：/joint_states
- 输出：/manipulator/web/joint_states
- 发布频率：15Hz
"""

import copy

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateWebRelayNode(Node):
    """订阅高频 JointState，定时发布最新一帧到 Web 专用话题。"""

    def __init__(self) -> None:
        super().__init__("joint_state_web_relay")
        self.declare_parameter("input_topic", "/joint_states")
        self.declare_parameter("output_topic", "/manipulator/web/joint_states")
        self.declare_parameter("publish_rate_hz", 15.0)

        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        publish_rate_hz = self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        if publish_rate_hz < 1.0:
            publish_rate_hz = 1.0

        self._latest_joint_state = None
        self._sub = self.create_subscription(
            JointState,
            input_topic,
            self._on_joint_state,
            10,
        )
        self._pub = self.create_publisher(JointState, output_topic, 10)
        self._timer = self.create_timer(1.0 / float(publish_rate_hz), self._on_timer)

        self.get_logger().info(
            "joint_state_web_relay: sub=%s pub=%s rate=%.1fHz"
            % (input_topic, output_topic, publish_rate_hz)
        )

    def _on_joint_state(self, msg: JointState) -> None:
        # 缓存最新状态，定时器按固定频率发出，避免输入突发频率直接透传到 Web。
        self._latest_joint_state = copy.deepcopy(msg)

    def _on_timer(self) -> None:
        if self._latest_joint_state is None:
            return
        self._pub.publish(self._latest_joint_state)


def main(args=None):
    """节点入口：spin JointStateWebRelayNode。"""
    rclpy.init(args=args)
    node = JointStateWebRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
