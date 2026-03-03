#!/usr/bin/env python3
"""
PyBullet 仿真控制器：提供 FollowJointTrajectory action server，发布 joint_states。
使 MoveIt Plan & Execute 能在 PyBullet 中驱动 Orion。
"""

import os
import tempfile
import time
from typing import Dict, List, Optional

import pybullet as p
import pybullet_data
import rclpy
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, GoalResponse
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


# 所有可驱动关节（与 moveit_controllers 一致）
ARM_JOINTS = [
    "joint_base_link_Link1",
    "joint_Link1_Link2",
    "joint_Link2_Link3",
    "joint_LinkVirtual_Link4",
    "joint_Link4_Link5",
    "joint_Link5_Link6",
]
HAND_JOINTS = [
    "joint_Link6_Link7",
    "joint_Link6_Link8",
]
ALL_JOINTS = ARM_JOINTS + HAND_JOINTS


def _resolve_urdf_with_meshes() -> str:
    """从 orion_description 读取 URDF，将 mesh 路径改为绝对路径并写入临时文件。"""
    pkg_share = get_package_share_directory("orion_description")
    urdf_path = os.path.join(pkg_share, "urdf", "orion.urdf")
    meshes_abs = os.path.join(pkg_share, "meshes", "stl")
    with open(urdf_path, "r", encoding="utf-8") as f:
        content = f.read()
    content = content.replace("../meshes/stl/", meshes_abs + "/")
    fd, path = tempfile.mkstemp(suffix=".urdf")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            f.write(content)
        return path
    except Exception:
        os.close(fd)
        raise


class PyBulletControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("pybullet_controller")
        self._robot_id: Optional[int] = None
        self._joint_name_to_index: Dict[str, int] = {}
        self._current_positions: Dict[str, float] = {j: 0.0 for j in ALL_JOINTS}

        # 启动 PyBullet（DIRECT 不弹窗，适合与 RViz 配合）
        use_gui_param = self.declare_parameter("use_gui", False).value
        use_gui = use_gui_param if isinstance(use_gui_param, bool) else (use_gui_param == "true")
        if use_gui:
            self._client = p.connect(p.GUI)
        else:
            self._client = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0.0, 0.0, -9.81)

        urdf_path = _resolve_urdf_with_meshes()
        try:
            self._robot_id = p.loadURDF(
                urdf_path,
                basePosition=[0.0, 0.0, 0.0],
                baseOrientation=[0.0, 0.0, 0.0, 1.0],
                useFixedBase=True,
            )
        finally:
            try:
                os.unlink(urdf_path)
            except OSError:
                pass

        if self._robot_id is None:
            raise RuntimeError("Failed to load Orion URDF in PyBullet")

        num_joints = p.getNumJoints(self._robot_id)
        for i in range(num_joints):
            info = p.getJointInfo(self._robot_id, i)
            if info[2] == p.JOINT_REVOLUTE or info[2] == p.JOINT_PRISMATIC:
                name = info[1].decode("utf-8") if isinstance(info[1], bytes) else info[1]
                self._joint_name_to_index[name] = i

        for name in ALL_JOINTS:
            if name not in self._joint_name_to_index:
                self.get_logger().warn("Joint not found in PyBullet: %s" % name)

        self._joint_states_pub = self.create_publisher(
            JointState, "joint_states", 10
        )
        self._timer = self.create_timer(1.0 / 100.0, self._publish_joint_states)
        self._arm_action = ActionServer(
            self,
            FollowJointTrajectory,
            "arm_controller/follow_joint_trajectory",
            self._execute_trajectory_callback,
            goal_callback=self._goal_callback,
        )
        self._hand_action = ActionServer(
            self,
            FollowJointTrajectory,
            "hand_controller/follow_joint_trajectory",
            self._execute_trajectory_callback,
            goal_callback=self._goal_callback,
        )
        self.get_logger().info(
            "PyBullet controller running: arm_controller and hand_controller actions, publishing joint_states"
        )

    def _goal_callback(self, _goal_request) -> GoalResponse:
        return GoalResponse.ACCEPT

    def _execute_trajectory_callback(self, goal_handle):
        """在回调内循环执行轨迹，并持续发布 joint_states，最后返回 Result。"""
        result = FollowJointTrajectory.Result()
        try:
            goal = goal_handle.request
            trajectory = goal.trajectory
            joint_names = list(trajectory.joint_names)
            points = trajectory.points
            if not points:
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                goal_handle.succeed()
                return result
            self.get_logger().info(
                "Executing trajectory: %d points, duration %.2fs"
                % (len(points), points[-1].time_from_start.sec + points[-1].time_from_start.nanosec * 1e-9)
            )
            start_time = time.monotonic()
            end_t = points[-1].time_from_start.sec + points[-1].time_from_start.nanosec * 1e-9
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                    goal_handle.canceled()
                    return result
                elapsed = time.monotonic() - start_time
                for point in points:
                    t = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
                    if elapsed >= t:
                        positions = list(point.positions) if point.positions else []
                        if len(positions) == len(joint_names):
                            for j, name in enumerate(joint_names):
                                if name in self._joint_name_to_index:
                                    self._current_positions[name] = positions[j]
                                    idx = self._joint_name_to_index[name]
                                    p.resetJointState(
                                        self._robot_id, idx, positions[j], 0.0
                                    )
                self._publish_joint_states()
                if elapsed >= end_t:
                    break
                time.sleep(0.02)
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            goal_handle.succeed()
            self.get_logger().info("Trajectory execution finished successfully.")
            return result
        except Exception as e:
            self.get_logger().error("Error in execute callback: %s" % str(e), throttle_duration_sec=1.0)
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            goal_handle.abort(result)
            return result

    def _publish_joint_states(self) -> None:
        if self._robot_id is None:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ALL_JOINTS
        msg.position = [self._current_positions[n] for n in ALL_JOINTS]
        msg.velocity = [0.0] * len(ALL_JOINTS)
        msg.effort = []
        self._joint_states_pub.publish(msg)

    def destroy_node(self) -> None:
        if hasattr(self, "_client"):
            p.disconnect(self._client)
        super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = PyBulletControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
