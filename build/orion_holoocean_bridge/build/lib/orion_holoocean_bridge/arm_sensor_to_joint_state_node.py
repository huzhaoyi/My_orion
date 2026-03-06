#!/usr/bin/env python3
"""
将 HoloOcean /holoocean/rov0/ArmSensor 的 left_arm 数据转为 Orion 的 joint_states，
供 MoveIt/MTC 使用。单臂：6DOF + 夹爪（2 个手部关节映射为同一夹爪值）。
left_arm_gripped 发布到话题供 MTC 动态抓取时等待“抓稳/松开”。

对应关系（以 WorkingClassROVArmSensor 消息定义为准）：
- left_arm_joints[0..5] = Joint1..Joint6，left_arm_joints[6] = Gripper，单位度。
- Orion 臂关节顺序：joint_base_link_Link1..joint_Link5_Link6 即 Joint1..Joint6，夹爪为 Link7/Link8。
- 1:1 映射：left_arm_joints[i] -> Orion 第 i 个臂关节，left_arm_joints[6] -> 夹爪。
"""

import math
from typing import List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from holoocean_interfaces.msg import WorkingClassROVArmSensor


# 与 orion_moveit_config / orion_mtc 一致的关节名
# 臂关节：joint_base_link_Link1..joint_Link5_Link6，AgentCommand 左臂 command[8..13]
ARM_JOINT_NAMES = [
    "joint_base_link_Link1",
    "joint_Link1_Link2",
    "joint_Link2_Link3",
    "joint_LinkVirtual_Link4",
    "joint_Link4_Link5",
    "joint_Link5_Link6",
]
# 夹爪：joint_Link6_Link7/Link8；HoloOcean 中 left_arm_joints[6] / AgentCommand.command[14]
HAND_JOINT_NAMES = [
    "joint_Link6_Link7",
    "joint_Link6_Link8",
]
ALL_JOINT_NAMES = ARM_JOINT_NAMES + HAND_JOINT_NAMES

DEG_TO_RAD = math.pi / 180.0
# WorkingClassROVArmSensor: left_arm_joints[0]=Joint1 .. [5]=Joint6, [6]=Gripper，与 Orion 臂关节顺序 1:1
HOLOOCEAN_TO_ORION_ARM = (0, 1, 2, 3, 4, 5)
LEFT_ARM_SIGN = (1.0, -1.0, -1.0, -1.0, -1.0, -1.0)
# HoloOcean 夹爪：0°=闭合，-90°=完全打开 -> Orion open=(0.4,-0.4) rad（与 orion.srdf 一致）
HOLOOCEAN_GRIPPER_OPEN_DEG = -90.0
ORION_OPEN_RAD = 0.4


def _to_list(val) -> List[float]:
    """将 left_arm_joints（可能为 tuple/list）转为 list，长度 7。"""
    if hasattr(val, "__iter__") and not isinstance(val, (str, bytes)):
        out = list(val)[:7]
        while len(out) < 7:
            out.append(0.0)
        return out
    return [0.0] * 7


class ArmSensorToJointStateNode(Node):
    """订阅 HoloOcean ArmSensor，发布 Orion 的 joint_states（仅左臂：6DOF + 夹爪）。"""

    def __init__(self) -> None:
        super().__init__("arm_sensor_to_joint_state")
        self.declare_parameter("arm_sensor_topic", "/holoocean/rov0/ArmSensor")
        self.declare_parameter("joint_states_topic", "joint_states")
        self.declare_parameter("joints_in_degrees", True)
        self.declare_parameter("publish_frame_id", "base_link")
        self.declare_parameter("gripped_topic", "left_arm_gripped")

        arm_sensor_topic = self.get_parameter("arm_sensor_topic").get_parameter_value().string_value
        joint_states_topic = self.get_parameter("joint_states_topic").get_parameter_value().string_value
        self._joints_in_degrees = self.get_parameter("joints_in_degrees").get_parameter_value().bool_value
        self._frame_id = self.get_parameter("publish_frame_id").get_parameter_value().string_value
        gripped_topic = self.get_parameter("gripped_topic").get_parameter_value().string_value

        self._sub = self.create_subscription(
            WorkingClassROVArmSensor,
            arm_sensor_topic,
            self._on_arm_sensor,
            10,
        )
        self._pub = self.create_publisher(JointState, joint_states_topic, 10)
        self._gripped_pub = self.create_publisher(Float32, gripped_topic, 10)

        self.get_logger().info(
            "arm_sensor_to_joint_state: sub=%s pub=%s degrees=%s"
            % (arm_sensor_topic, joint_states_topic, self._joints_in_degrees)
        )

    def _on_arm_sensor(self, msg: WorkingClassROVArmSensor) -> None:
        left = _to_list(getattr(msg, "left_arm_joints", [0.0] * 7))
        if len(left) < 7:
            left = left + [0.0] * (7 - len(left))

        scale = DEG_TO_RAD if self._joints_in_degrees else 1.0
        arm_positions = [
            LEFT_ARM_SIGN[i] * float(left[HOLOOCEAN_TO_ORION_ARM[i]]) * scale
            for i in range(6)
        ]
        # HoloOcean 夹爪度 0°=闭合、-90°=打开 -> Orion (Link7, Link8) rad，与 SRDF open=(0.4,-0.4) 一致
        gripper_deg = float(left[6])
        ratio = 0.0 if abs(HOLOOCEAN_GRIPPER_OPEN_DEG) < 1e-9 else max(
            0.0, min(1.0, gripper_deg / HOLOOCEAN_GRIPPER_OPEN_DEG)
        )
        link7_rad = ratio * ORION_OPEN_RAD
        link8_rad = -link7_rad
        hand_positions = [link7_rad, link8_rad]

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.header.frame_id = self._frame_id
        js.name = ALL_JOINT_NAMES
        js.position = arm_positions + hand_positions
        js.velocity = [0.0] * len(ALL_JOINT_NAMES)
        js.effort = []
        self._pub.publish(js)
        # 节流打印：当前状态（度），joint1~6 + 夹爪
        arm_deg = [arm_positions[i] * (1.0 / DEG_TO_RAD) for i in range(6)]
        gripper_deg_log = gripper_deg if self._joints_in_degrees else (gripper_deg * (1.0 / DEG_TO_RAD))
        self.get_logger().info(
            "arm_sensor 当前状态 左臂 joint1~6(度): %s, gripper(度): %.2f"
            % (", ".join("%.2f" % arm_deg[i] for i in range(6)), float(gripper_deg_log)),
            throttle_duration_sec=2.0,
        )

        gripped = float(getattr(msg, "left_arm_gripped", 0.0))
        out = Float32()
        out.data = gripped
        self._gripped_pub.publish(out)
        if abs(gripped) > 1e-6:
            self.get_logger().debug(
                "left_arm_gripped=%.3f (collision/grip hint)" % gripped
            )


def main(args=None):
    rclpy.init(args=args)
    node = ArmSensorToJointStateNode()
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
