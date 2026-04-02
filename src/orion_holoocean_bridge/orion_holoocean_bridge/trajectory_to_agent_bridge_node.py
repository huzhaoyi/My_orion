#!/usr/bin/env python3
"""
将 MoveIt FollowJointTrajectory 转为 HoloOcean 臂指令的桥接节点。
- 弧度转度数；左臂 6 关节 + 夹爪（2 个手部关节合并为 1 个自由度）。
- 发布到 /holoocean/command/agent/arm：仅 14 维（无推进器），[0:7] 左臂 7 关节，[7:14] 右臂 7 关节。

提供 arm_controller 与 hand_controller 两个 FollowJointTrajectory ActionServer，
按关节名从轨迹插值映射到 Holoocean 左臂度数与单值夹爪。

MTC/orion_mtc 可能对两臂控制器并行发送 goal；默认单线程 spin 会导致第二个 execute
长期得不到调度。此处使用 MultiThreadedExecutor，并对 _left_arm_deg 与发布加互斥锁。

调试：enable_trajectory_debug_log=true 时 INFO 打印 arm/hand 轨迹点（°）及节流打印 AgentCommand。
"""

import math
import threading
import time
from typing import List, Tuple

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from holoocean_interfaces.msg import AgentCommand
from std_msgs.msg import Header


RAD_TO_DEG = 180.0 / math.pi
# /holoocean/command/agent/arm 仅 14 维（无推进器）：[0:7] 左臂，[7:14] 右臂，单位度；左臂 command[0..5]=Joint1..6，command[6]=夹爪
ARM_LEN = 7
LEFT_ARM_GRIPPER_INDEX = 6  # 左臂夹爪在左臂 7 维中的下标
PUBLISH_RATE_HZ = 50.0
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
    """提供 FollowJointTrajectory action，将轨迹转为 AgentCommand（度、夹爪合并）发布到 /holoocean/command/agent/arm，顺序：0=左臂、1=右臂。"""

    def __init__(self) -> None:
        super().__init__("trajectory_to_agent_bridge")
        self.declare_parameter("agent_command_topic", "/holoocean/command/agent/arm")
        self.declare_parameter("agent_frame_id", "rov0")
        self.declare_parameter("publish_rate_hz", PUBLISH_RATE_HZ)
        # true：INFO 打印 MoveIt 下发的轨迹点 + 节流打印发往 HoloOcean 的 AgentCommand（°）
        self.declare_parameter("enable_trajectory_debug_log", False)
        self.declare_parameter("log_agent_command_throttle_sec", 0.25)

        topic = self.get_parameter("agent_command_topic").get_parameter_value().string_value
        self._frame_id = self.get_parameter("agent_frame_id").get_parameter_value().string_value
        rate_hz = self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        self._dt = 1.0 / rate_hz if rate_hz > 0.0 else 0.02
        self._enable_trajectory_debug_log = (
            self.get_parameter("enable_trajectory_debug_log").get_parameter_value().bool_value
        )
        self._log_cmd_throttle = max(
            0.05,
            self.get_parameter("log_agent_command_throttle_sec").get_parameter_value().double_value,
        )

        self._command_pub = self.create_publisher(AgentCommand, topic, 10)
        # 当前左臂 7 维（度）：6 关节 + 1 夹爪（合并），对应 command[0:7]
        self._left_arm_deg: List[float] = [0.0] * ARM_LEN
        self._arm_state_lock = threading.Lock()

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
            "trajectory_to_agent_bridge: publishing to %s (14 dim: [0:7] left arm, [7:14] right arm, no thrusters)"
            % topic
        )
        self._log_mapping_and_indices()

    def _log_mapping_and_indices(self) -> None:
        """打印发送规矩与 command 数组下标含义（14 维臂指令，无推进器）。"""
        self.get_logger().info(
            "command: 14 dim, [0:7] left arm, [7:14] right arm; left gripper = command[%d]"
            % LEFT_ARM_GRIPPER_INDEX
        )
        self.get_logger().info(
            "Orion joint1..6 -> command[0..5] (left), sign ORION_TO_HOLOOCEAN_LEFT_ARM_SIGN"
        )
        for orion_i in range(6):
            holo_i = ORION_TO_HOLOOCEAN_LEFT_ARM[orion_i]
            sign = ORION_TO_HOLOOCEAN_LEFT_ARM_SIGN[orion_i]
            self.get_logger().info(
                "  Orion joint%d -> command[%d], sign=%.1f" % (orion_i + 1, holo_i, sign)
            )
        self.get_logger().info(
            "  gripper (merged) -> command[%d], 0 deg=closed -90 deg=open" % LEFT_ARM_GRIPPER_INDEX
        )

    def _goal_callback(self, _goal_request) -> GoalResponse:
        return GoalResponse.ACCEPT

    def _publish_agent_command_locked(self) -> None:
        """在已持有 _arm_state_lock 时调用：发布 14 维 AgentCommand。"""
        msg = AgentCommand()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        cmd = list(self._left_arm_deg) + [0.0] * ARM_LEN
        msg.command = cmd
        self._command_pub.publish(msg)
        if self._enable_trajectory_debug_log:
            self.get_logger().info(
                "→仿真 AgentCommand (°) 左[0:7]=%s 右[7:14]=%s"
                % (
                    " ".join("%.2f" % x for x in cmd[:ARM_LEN]),
                    " ".join("%.2f" % x for x in cmd[ARM_LEN:]),
                ),
                throttle_duration_sec=self._log_cmd_throttle,
            )

    def _log_trajectory_deg(self, joint_names, points, label: str) -> None:
        """打印即将转为 AgentCommand 的 FollowJointTrajectory（关节名 + 各点时间 + 位姿 °）。"""
        if not self._enable_trajectory_debug_log:
            return
        if not joint_names or not points:
            return
        t_end = points[-1].time_from_start.sec + points[-1].time_from_start.nanosec * 1.0e-9
        names_line = ", ".join("%s[%d]" % (joint_names[i], i) for i in range(len(joint_names)))
        self.get_logger().info(
            "%s →仿真: points=%d duration=%.3fs | %s" % (label, len(points), t_end, names_line)
        )
        n = len(points)
        head_n, tail_n = 24, 24
        if n > head_n + tail_n + 1:
            indices = list(range(head_n)) + list(range(n - tail_n, n))
            self.get_logger().info(
                "%s →仿真: 仅打印前 %d / 后 %d 点（共 %d 点）" % (label, head_n, tail_n, n)
            )
        else:
            indices = list(range(n))
        for p in indices:
            pt = points[p]
            if not pt.positions:
                continue
            t_sec = pt.time_from_start.sec + pt.time_from_start.nanosec * 1.0e-9
            deg_str = " ".join(
                "%.2f" % (float(x) * RAD_TO_DEG) for x in pt.positions[: len(joint_names)]
            )
            self.get_logger().info("%s →仿真: point[%d] t=%.3fs ° | %s" % (label, p, t_sec, deg_str))

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
            self._log_trajectory_deg(joint_names, points, "arm_controller")
            start_time = time.monotonic()
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                    goal_handle.canceled()
                    return result
                elapsed = time.monotonic() - start_time
                pos_rad, done = _interpolate_point(points, elapsed)
                with self._arm_state_lock:
                    if len(pos_rad) == len(joint_names):
                        for orion_i in range(6):
                            idx = name_to_idx.get(ARM_JOINT_NAMES[orion_i])
                            if idx is not None:
                                sign = ORION_TO_HOLOOCEAN_LEFT_ARM_SIGN[orion_i]
                                self._left_arm_deg[orion_i] = sign * float(pos_rad[idx]) * RAD_TO_DEG
                    self._publish_agent_command_locked()
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
            self._log_trajectory_deg(joint_names, points, "hand_controller")
            start_time = time.monotonic()
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                    goal_handle.canceled()
                    return result
                elapsed = time.monotonic() - start_time
                pos_rad, done = _interpolate_point(points, elapsed)
                with self._arm_state_lock:
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
                    self._publish_agent_command_locked()
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
    """节点入口：多线程 executor，便于 arm/hand 两个 Action 的 execute 并发运行。"""
    rclpy.init(args=args)
    node = TrajectoryToAgentBridgeNode()
    node.get_logger().info(
        "trajectory_to_agent_bridge: MultiThreadedExecutor(num_threads=4)（并行 arm/hand goal）"
    )
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
