#!/usr/bin/env python3
"""
诊断节点：订阅 /holoocean/rov0/ArmSensor，打印 right_arm 下标与 Orion(RViz) 关节名对照表。

WorkingClassROVArmSensor 定义：right_arm_joints[0]=Joint1, [1]=Joint2, ..., [5]=Joint6, [6]=Gripper。
RViz 显示的是 Orion 关节名：joint_base_link_Link1, joint_Link1_Link2, ... joint_Link5_Link6。
本节点按实测映射 right_arm[5,0,1,4,2,3]->Orion joint1~6 打印，使 echo 的数值与 RViz 当前关节角一致。

用法：
  ros2 run orion_holoocean_bridge arm_sensor_mapping_test
"""

import math
from typing import List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from holoocean_interfaces.msg import WorkingClassROVArmSensor


# 与 moveit_controllers / URDF 一致
ARM_JOINT_NAMES = [
    "joint_base_link_Link1",
    "joint_Link1_Link2",
    "joint_Link2_Link3",
    "joint_LinkVirtual_Link4",
    "joint_Link4_Link5",
    "joint_Link5_Link6",
]
# 与 arm_sensor_to_joint_state_node 一致
HOLOOCEAN_TO_ORION_ARM = (0, 1, 2, 3, 4, 5)
DEG_TO_RAD = math.pi / 180.0


def _to_list(val) -> List[float]:
    if hasattr(val, "__iter__") and not isinstance(val, (str, bytes)):
        out = list(val)[:7]
        while len(out) < 7:
            out.append(0.0)
        return out
    return [0.0] * 7


class ArmSensorMappingTestNode(Node):
    def __init__(self) -> None:
        super().__init__("arm_sensor_mapping_test")
        self.declare_parameter("arm_sensor_topic", "/holoocean/rov0/ArmSensor")
        self.declare_parameter("joint_states_topic", "joint_states")
        self.declare_parameter("joints_in_degrees", True)

        arm_sensor_topic = self.get_parameter("arm_sensor_topic").get_parameter_value().string_value
        joint_states_topic = self.get_parameter("joint_states_topic").get_parameter_value().string_value
        self._joints_in_degrees = self.get_parameter("joints_in_degrees").get_parameter_value().bool_value

        self._sub_arm = self.create_subscription(
            WorkingClassROVArmSensor,
            arm_sensor_topic,
            self._on_arm_sensor,
            10,
        )
        self._sub_js = self.create_subscription(
            JointState,
            joint_states_topic,
            self._on_joint_states,
            10,
        )
        self._last_js: Optional[JointState] = None
        self.get_logger().info(
            "ArmSensor 映射诊断: 订阅 %s 与 %s，打印 right_arm[i] <-> Orion 关节名（节流 2s）"
            % (arm_sensor_topic, joint_states_topic)
        )
        self.get_logger().info(
            "当前映射: right_arm_joints[HOLOOCEAN_TO_ORION_ARM[i]] -> Orion ARM_JOINT_NAMES[i] (1:1 时为 right_arm[i]->joint(i+1))"
        )

    def _on_joint_states(self, msg: JointState) -> None:
        self._last_js = msg

    def _on_arm_sensor(self, msg: WorkingClassROVArmSensor) -> None:
        right = _to_list(msg.right_arm_joints)
        scale = DEG_TO_RAD if self._joints_in_degrees else 1.0
        lines = []
        for i in range(6):
            holo_idx = HOLOOCEAN_TO_ORION_ARM[i]
            val_deg = float(right[holo_idx])
            val_rad = val_deg * scale
            name = ARM_JOINT_NAMES[i]
            js_rad = ""
            if self._last_js and name in self._last_js.name:
                idx = self._last_js.name.index(name)
                js_rad = " | joint_states=%.4f rad" % self._last_js.position[idx]
            lines.append(
                "  right_arm[%d] = %10.4f (deg) -> Orion %s (joint%d)%s"
                % (holo_idx, val_deg, name, i + 1, js_rad)
            )
        gripper_deg = float(right[6])
        lines.append("  right_arm[6] = %10.4f (deg) -> 夹爪(合并)" % gripper_deg)
        self.get_logger().info(
            "ArmSensor -> Orion 对照表:\n%s" % "\n".join(lines),
            throttle_duration_sec=2.0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = ArmSensorMappingTestNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
