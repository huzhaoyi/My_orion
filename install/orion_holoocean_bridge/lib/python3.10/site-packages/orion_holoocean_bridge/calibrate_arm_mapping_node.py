#!/usr/bin/env python3
"""
校准脚本：依次只驱动 HoloOcean 右臂的一个 command 下标（command[15]～command[21]），
方便你观察 RViz 里哪个 Orion 关节在动，从而确定真实对应关系。

用法：
  1. 先启动 HoloOcean 仿真 + arm_sensor_to_joint_state（或整个 pick_place_holoocean）。
  2. 在另一个终端：source 好 holoocean-ros 与 My_orion 后运行：
     ros2 run orion_holoocean_bridge calibrate_arm_mapping
  3. 脚本会每 8 秒把 right_arm 的一个下标设为 30 度，其余为 0，并打印当前驱动的下标。
  4. 看 RViz 里 joint1～joint6（及夹爪）哪个在动，记下：command[15+?] -> Orion joint?。
  5. 根据记录更新 trajectory_to_agent_bridge 与 arm_sensor_to_joint_state 中的映射常量。
"""

import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header

from holoocean_interfaces.msg import AgentCommand


# command[0:8] 推进器, [8:15] 左臂, [15:22] 右臂
RIGHT_ARM_START = 15
RIGHT_ARM_LEN = 7
DEG_TEST = 30.0
SEC_PER_INDEX = 8.0


class CalibrateArmMappingNode(Node):
    def __init__(self):
        super().__init__("calibrate_arm_mapping")
        self.declare_parameter("agent_command_topic", "/holoocean/command/agent")
        self.declare_parameter("agent_frame_id", "rov0")
        self.declare_parameter("deg_test", DEG_TEST)
        self.declare_parameter("sec_per_index", SEC_PER_INDEX)

        topic = self.get_parameter("agent_command_topic").get_parameter_value().string_value
        self._frame_id = self.get_parameter("agent_frame_id").get_parameter_value().string_value
        self._deg = self.get_parameter("deg_test").get_parameter_value().double_value
        self._sec = self.get_parameter("sec_per_index").get_parameter_value().double_value

        self._pub = self.create_publisher(AgentCommand, topic, 10)
        self._index = 0
        self._timer = self.create_timer(self._sec, self._on_timer)
        self.get_logger().info(
            "Calibration: will set right_arm index 0..6 to %.1f deg one by one (%.1f s each). "
            "Watch RViz and note which Orion joint moves for each index."
            % (self._deg, self._sec)
        )
        self._print_once()

    def _print_once(self):
        i = self._index
        if i < RIGHT_ARM_LEN:
            name = "gripper" if i == 6 else "arm joint %d" % (i + 1)
            self.get_logger().info(
                ">>> Setting right_arm[%d] (command[%d]) = %.1f deg (%s) - which Orion joint moves in RViz?"
                % (i, RIGHT_ARM_START + i, self._deg, name)
            )

    def _on_timer(self):
        msg = AgentCommand()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        cmd = [0.0] * 8 + [0.0] * 7 + [0.0] * RIGHT_ARM_LEN
        if self._index < RIGHT_ARM_LEN:
            cmd[RIGHT_ARM_START + self._index] = self._deg
        msg.command = cmd
        self._pub.publish(msg)
        self._index += 1
        if self._index <= RIGHT_ARM_LEN:
            self._print_once()
        if self._index >= RIGHT_ARM_LEN:
            self.get_logger().info(
                ">>> One full cycle done. Restarting from index 0 (Ctrl+C to exit)."
            )
            self._index = 0


def main(args=None):
    rclpy.init(args=args)
    node = CalibrateArmMappingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
