#!/usr/bin/env python3
"""
将 MoveIt FollowJointTrajectory 转为 HoloOcean AgentCommand 的桥接节点。
- 弧度转度数；左臂 6 关节 + 夹爪（2 个手部关节合并为 1 个自由度）。
- 发布到 /holoocean/command/agent，话题。
"""

import math
import time
from typing import List, Tuple

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, GoalResponse
from rclpy.node import Node

from holoocean_interfaces.msg import AgentCommand
from std_msgs.msg import Header


RAD_TO_DEG = 180.0 / math.pi
# AgentCommand.command 共 22 维: [0:8] 推进器, [8:15] 左臂(7: 6关节+夹爪), [15:22] 右臂(7)，单位度；左臂 command[8..13]=Joint1..6，command[14]=夹爪
LEFT_ARM_START = 8
LEFT_ARM_LEN = 7
GRIPPER_CMD_INDEX = 14  # 左臂夹爪在 command 数组中的下标
PUBLISH_RATE_HZ = 50.0
# Orion joint1..6 -> command[8..13]，夹爪 -> command[14]（与 left_arm_joints[0..6] 定义一致）
ORION_TO_HOLOOCEAN_LEFT_ARM = (0, 1, 2, 3, 4, 5)
ORION_TO_HOLOOCEAN_LEFT_ARM_SIGN = (1.0, -1.0, -1.0, -1.0, -1.0, -1.0)

# 与 orion_moveit_config / arm_sensor 一致的关节名，用于按名从 trajectory 中取位置（RViz 下发顺序可能不同）
ARM_JOINT_NAMES = [
    "joint_base_link_Link1",
    "joint_Link1_Link2",
    "joint_Link2_Link3",
    "joint_LinkVirtual_Link4",
    "joint_Link4_Link5",
    "joint_Link5_Link6",
]
HAND_JOINT_NAMES = [
    "joint_Link6_Link7",
    "joint_Link6_Link8",
]
# Orion SRDF: close=(0,0) rad, open=(0.4,-0.4) rad；HoloOcean 夹爪单值：0°=闭合，-90°=完全打开
ORION_OPEN_RAD = 0.4
HOLOOCEAN_GRIPPER_CLOSED_DEG = 0.0
HOLOOCEAN_GRIPPER_OPEN_DEG = -90.0


def _interpolate_point(
    points: List,
    t: float,
) -> Tuple[List[float], bool]:
    """按时间 t 从 trajectory.points 插值出位置列表，返回 (positions, done)。"""
    if not points:
        return [], True
    t_end = points[-1].time_from_start.sec + points[-1].time_from_start.nanosec * 1e-9
    if t >= t_end:
        pos = list(points[-1].positions) if points[-1].positions else []
        return pos, True
    for i in range(len(points) - 1):
        t0 = points[i].time_from_start.sec + points[i].time_from_start.nanosec * 1e-9
        t1 = points[i + 1].time_from_start.sec + points[i + 1].time_from_start.nanosec * 1e-9
        if t0 <= t <= t1:
            if t1 <= t0:
                frac = 1.0
            else:
                frac = (t - t0) / (t1 - t0)
            p0 = list(points[i].positions) if points[i].positions else []
            p1 = list(points[i + 1].positions) if points[i + 1].positions else []
            if len(p0) == len(p1):
                pos = [p0[j] + frac * (p1[j] - p0[j]) for j in range(len(p0))]
                return pos, False
            return p1, False
    return list(points[0].positions) if points[0].positions else [], False


class TrajectoryToAgentBridgeNode(Node):
    """提供 FollowJointTrajectory action，将轨迹转为 AgentCommand（度、夹爪合并）发布到 /holoocean/command/agent。"""

    def __init__(self) -> None:
        super().__init__("trajectory_to_agent_bridge")
        self.declare_parameter("agent_command_topic", "/holoocean/command/agent")
        self.declare_parameter("agent_frame_id", "rov0")
        self.declare_parameter("publish_rate_hz", PUBLISH_RATE_HZ)

        topic = self.get_parameter("agent_command_topic").get_parameter_value().string_value
        self._frame_id = self.get_parameter("agent_frame_id").get_parameter_value().string_value
        rate_hz = self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        self._dt = 1.0 / rate_hz if rate_hz > 0.0 else 0.02

        self._command_pub = self.create_publisher(AgentCommand, topic, 10)
        # 当前左臂 7 维（度）：6 关节 + 1 夹爪（合并）
        self._left_arm_deg: List[float] = [0.0] * LEFT_ARM_LEN

        self._arm_action = ActionServer(
            self,
            FollowJointTrajectory,
            "arm_controller/follow_joint_trajectory",
            self._execute_arm_callback,
            goal_callback=self._goal_callback,
        )
        self._hand_action = ActionServer(
            self,
            FollowJointTrajectory,
            "hand_controller/follow_joint_trajectory",
            self._execute_hand_callback,
            goal_callback=self._goal_callback,
        )
        self.get_logger().info(
            "trajectory_to_agent_bridge: publishing to %s (left arm in deg, gripper merged)"
            % topic
        )
        self._log_mapping_and_indices()

    def _log_mapping_and_indices(self) -> None:
        """打印发送规矩与 command 数组下标含义。"""
        self.get_logger().info(
            "AgentCommand.command: [0:8] 推进器, [8:15] 左臂(7), [15:22] 右臂(7)，左臂夹爪=command[%d]"
            % GRIPPER_CMD_INDEX
        )
        self.get_logger().info(
            "Orion joint1..6 -> command[8..13]（左臂），符号 ORION_TO_HOLOOCEAN_LEFT_ARM_SIGN"
        )
        for orion_i in range(6):
            holo_i = ORION_TO_HOLOOCEAN_LEFT_ARM[orion_i]
            sign = ORION_TO_HOLOOCEAN_LEFT_ARM_SIGN[orion_i]
            cmd_idx = LEFT_ARM_START + holo_i
            self.get_logger().info(
                "  Orion joint%d -> command[%d], sign=%.1f" % (orion_i + 1, cmd_idx, sign)
            )
        self.get_logger().info("  夹爪(合并) -> command[%d]，0°=闭合 -90°=打开" % GRIPPER_CMD_INDEX)

    def _goal_callback(self, _goal_request) -> GoalResponse:
        return GoalResponse.ACCEPT

    def _publish_agent_command(self) -> None:
        """发布当前 self._left_arm_deg 到 AgentCommand；左臂占 command[8:15]，夹爪=command[14]。"""
        msg = AgentCommand()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        # 共 22 元素：8 推进器 + 7 左臂(6关节+夹爪) + 7 右臂；左臂夹爪在 command[14]
        cmd = [0.0] * 8 + list(self._left_arm_deg) + [0.0] * 7
        msg.command = cmd
        self._command_pub.publish(msg)

    def _execute_arm_callback(self, goal_handle):
        """执行 arm 轨迹：按 joint_names 取位置，按 ARM_JOINT_NAMES 顺序写入 left_arm[0:6]，夹爪保持当前。"""
        result = FollowJointTrajectory.Result()
        try:
            trajectory = goal_handle.request.trajectory
            joint_names = list(trajectory.joint_names) if trajectory.joint_names else []
            name_to_idx = {name: i for i, name in enumerate(joint_names)}
            points = list(trajectory.points)
            if not points:
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                goal_handle.succeed()
                return result
            start_time = time.monotonic()
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                    goal_handle.canceled()
                    return result
                elapsed = time.monotonic() - start_time
                pos_rad, done = _interpolate_point(points, elapsed)
                if len(pos_rad) == len(joint_names):
                    for orion_i in range(6):
                        idx = name_to_idx.get(ARM_JOINT_NAMES[orion_i])
                        if idx is not None:
                            sign = ORION_TO_HOLOOCEAN_LEFT_ARM_SIGN[orion_i]
                            self._left_arm_deg[orion_i] = sign * float(pos_rad[idx]) * RAD_TO_DEG
                self._publish_agent_command()
                if done:
                    break
                time.sleep(self._dt)
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            goal_handle.succeed()
            return result
        except Exception as e:
            self.get_logger().error("arm execute error: %s" % str(e))
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            goal_handle.abort()
            return result

    def _orion_hand_to_holoocean_gripper_deg(self, link7_rad: float, link8_rad: float) -> float:
        """Orion 双关节 (Link7, Link8) 弧度 -> HoloOcean 夹爪度：0°=闭合，-90°=完全打开。"""
        # 张开量 (rad)：close=(0,0)->0，open=(0.4,-0.4)->0.4
        opening_rad = 0.5 * (float(link7_rad) - float(link8_rad))
        if ORION_OPEN_RAD <= 1e-9:
            return HOLOOCEAN_GRIPPER_CLOSED_DEG
        ratio = opening_rad / ORION_OPEN_RAD
        ratio = max(0.0, min(1.0, ratio))
        return HOLOOCEAN_GRIPPER_CLOSED_DEG + ratio * (
            HOLOOCEAN_GRIPPER_OPEN_DEG - HOLOOCEAN_GRIPPER_CLOSED_DEG
        )

    def _execute_hand_callback(self, goal_handle):
        """执行 hand 轨迹：按 joint_names 取两夹爪关节弧度，映射为 HoloOcean 单值（0°=闭合，-90°=打开）写入 left_arm[6]。"""
        result = FollowJointTrajectory.Result()
        try:
            trajectory = goal_handle.request.trajectory
            points = list(trajectory.points)
            joint_names = list(trajectory.joint_names) if trajectory.joint_names else []
            name_to_idx = {name: i for i, name in enumerate(joint_names)}
            if not points:
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                goal_handle.succeed()
                return result
            start_time = time.monotonic()
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                    goal_handle.canceled()
                    return result
                elapsed = time.monotonic() - start_time
                pos_rad, done = _interpolate_point(points, elapsed)
                if len(pos_rad) == len(joint_names):
                    vals = []
                    for name in HAND_JOINT_NAMES:
                        idx = name_to_idx.get(name)
                        if idx is not None:
                            vals.append(float(pos_rad[idx]))
                    if len(vals) >= 2:
                        self._left_arm_deg[6] = self._orion_hand_to_holoocean_gripper_deg(
                            vals[0], vals[1]
                        )
                    elif len(vals) == 1:
                        self._left_arm_deg[6] = self._orion_hand_to_holoocean_gripper_deg(
                            vals[0], vals[0]
                        )
                self._publish_agent_command()
                if done:
                    break
                time.sleep(self._dt)
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            goal_handle.succeed()
            return result
        except Exception as e:
            self.get_logger().error("hand execute error: %s" % str(e))
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            goal_handle.abort()
            return result


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryToAgentBridgeNode()
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
