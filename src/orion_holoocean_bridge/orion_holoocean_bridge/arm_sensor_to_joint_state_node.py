#!/usr/bin/env python3
"""
将 HoloOcean /holoocean/rov0/ArmSensor 的 right_arm 数据转为 Orion 的 joint_states，
供 MoveIt/MTC 使用。单臂：6DOF + 夹爪（2 个手部关节映射为同一夹爪值）。
right_arm_gripped 可作为夹爪/碰撞状态参考（当前仅用于日志）。

对应关系（以 WorkingClassROVArmSensor 消息定义为准）：
- right_arm_joints[0..5] = Joint1..Joint6，right_arm_joints[6] = Gripper，单位度。
- Orion 臂关节顺序：joint_base_link_Link1..joint_Link5_Link6 即 Joint1..Joint6，夹爪为 Link7/Link8。
- 1:1 映射：right_arm_joints[i] -> Orion 第 i 个臂关节，right_arm_joints[6] -> 夹爪。
"""

import math
from typing import List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from holoocean_interfaces.msg import WorkingClassROVArmSensor


# 与 orion_moveit_config / orion_mtc 一致的关节名
# 臂关节：joint_base_link_Link1..joint_Link5_Link6，command[17..22]
ARM_JOINT_NAMES = [
    "joint_base_link_Link1",
    "joint_Link1_Link2",
    "joint_Link2_Link3",
    "joint_LinkVirtual_Link4",
    "joint_Link4_Link5",
    "joint_Link5_Link6",
]
# 夹爪：joint_Link6_Link7/Link8，command[23]
HAND_JOINT_NAMES = [
    "joint_Link6_Link7",
    "joint_Link6_Link8",
]
ALL_JOINT_NAMES = ARM_JOINT_NAMES + HAND_JOINT_NAMES

DEG_TO_RAD = math.pi / 180.0
# WorkingClassROVArmSensor: right_arm_joints[0]=Joint1 .. [5]=Joint6, [6]=Gripper，与 Orion 臂关节顺序 1:1
HOLOOCEAN_TO_ORION_ARM = (0, 1, 2, 3, 4, 5)
RIGHT_ARM_SIGN = (1.0, 1.0, 1.0, 1.0, 1.0, 1.0)


def _to_list(val) -> List[float]:
    """将 right_arm_joints（可能为 tuple/list）转为 list，长度 7。"""
    if hasattr(val, "__iter__") and not isinstance(val, (str, bytes)):
        out = list(val)[:7]
        while len(out) < 7:
            out.append(0.0)
        return out
    return [0.0] * 7


class ArmSensorToJointStateNode(Node):
    """订阅 HoloOcean ArmSensor，发布 Orion 的 joint_states（仅右臂：6DOF + 夹爪）。"""

    def __init__(self) -> None:
        super().__init__("arm_sensor_to_joint_state")
        self.declare_parameter("arm_sensor_topic", "/holoocean/rov0/ArmSensor")
        self.declare_parameter("joint_states_topic", "joint_states")
        self.declare_parameter("joints_in_degrees", True)
        self.declare_parameter("publish_frame_id", "base_link")

        arm_sensor_topic = self.get_parameter("arm_sensor_topic").get_parameter_value().string_value
        joint_states_topic = self.get_parameter("joint_states_topic").get_parameter_value().string_value
        self._joints_in_degrees = self.get_parameter("joints_in_degrees").get_parameter_value().bool_value
        self._frame_id = self.get_parameter("publish_frame_id").get_parameter_value().string_value

        self._sub = self.create_subscription(
            WorkingClassROVArmSensor,
            arm_sensor_topic,
            self._on_arm_sensor,
            10,
        )
        self._pub = self.create_publisher(JointState, joint_states_topic, 10)

        self.get_logger().info(
            "arm_sensor_to_joint_state: sub=%s pub=%s degrees=%s"
            % (arm_sensor_topic, joint_states_topic, self._joints_in_degrees)
        )

    def _on_arm_sensor(self, msg: WorkingClassROVArmSensor) -> None:
        right = _to_list(msg.right_arm_joints)
        if len(right) < 7:
            right = right + [0.0] * (7 - len(right))

        scale = DEG_TO_RAD if self._joints_in_degrees else 1.0
        arm_positions = [
            RIGHT_ARM_SIGN[i] * float(right[HOLOOCEAN_TO_ORION_ARM[i]]) * scale
            for i in range(6)
        ]
        gripper_val = float(right[6]) * scale
        hand_positions = [gripper_val, gripper_val]

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.header.frame_id = self._frame_id
        js.name = ALL_JOINT_NAMES
        js.position = arm_positions + hand_positions
        js.velocity = [0.0] * len(ALL_JOINT_NAMES)
        js.effort = []
        self._pub.publish(js)
        # 节流打印：当前状态（度），顺序与 right_arm_joints[0..5]=Joint1..6 一致
        arm_deg = [arm_positions[i] * (1.0 / DEG_TO_RAD) for i in range(6)]
        self.get_logger().info(
            "arm_sensor 当前状态 joint1~6(度): %s"
            % ", ".join("%.2f" % arm_deg[i] for i in range(6)),
            throttle_duration_sec=2.0,
        )

        gripped = getattr(msg, "right_arm_gripped", 0.0)
        if abs(gripped) > 1e-6:
            self.get_logger().debug(
                "right_arm_gripped=%.3f (collision/grip hint)", float(gripped)
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
        rclpy.shutdown()


if __name__ == "__main__":
    main()
