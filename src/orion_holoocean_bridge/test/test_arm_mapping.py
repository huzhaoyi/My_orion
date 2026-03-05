#!/usr/bin/env python3
"""
映射逻辑单元测试：用真实数据验证 ArmSensor -> joint_states 与 trajectory -> command 的对应关系。
不依赖仿真，仅验证常数与换算公式。
"""
import math
import sys
import os

# 允许从源码目录导入（不依赖 install）
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

# 与 arm_sensor_to_joint_state_node 完全一致的常量和公式
DEG_TO_RAD = math.pi / 180.0
HOLOOCEAN_TO_ORION_ARM = (5, 0, 1, 4, 2, 3)
RIGHT_ARM_SIGN = (1.0, 1.0, 1.0, 1.0, 1.0, 1.0)

# 与 trajectory_to_agent_bridge_node 一致
RAD_TO_DEG = 180.0 / math.pi
ORION_TO_HOLOOCEAN_RIGHT_ARM = (5, 0, 1, 4, 2, 3)
ORION_TO_HOLOOCEAN_RIGHT_ARM_SIGN = (1.0, 1.0, 1.0, 1.0, 1.0, 1.0)


def arm_sensor_to_positions(right_deg: list, joints_in_degrees: bool = True) -> list:
    """ArmSensor right_arm_joints(7) -> Orion arm positions(6) in rad."""
    scale = DEG_TO_RAD if joints_in_degrees else 1.0
    return [
        RIGHT_ARM_SIGN[i] * float(right_deg[HOLOOCEAN_TO_ORION_ARM[i]]) * scale
        for i in range(6)
    ]


def trajectory_to_right_arm_deg(pos_rad: list) -> list:
    """Orion trajectory positions(6) rad -> right_arm_deg(7), index 6 = gripper."""
    right = [0.0] * 7
    for orion_i in range(6):
        holo_i = ORION_TO_HOLOOCEAN_RIGHT_ARM[orion_i]
        sign = ORION_TO_HOLOOCEAN_RIGHT_ARM_SIGN[orion_i]
        right[holo_i] = sign * float(pos_rad[orion_i]) * RAD_TO_DEG
    return right


def test_arm_sensor_mapping_with_real_data():
    """用你提供的真实 ArmSensor 数据：right_arm 与 RViz 关节应一致。"""
    # 你之前给的数据：right_arm_joints 顺序为话题原始顺序
    right_arm_deg = [
        -1.97,   # [0]
        -26.17,  # [1]
        19.61,   # [2]
        25.59,   # [3]
        -24.19,  # [4]
        47.11,   # [5]
        0.0,     # [6] gripper
    ]
    # 期望：Orion joint1=47, joint2=-2, joint3=-26, joint4=-24, joint5=20, joint6=26 (度)
    # 映射应为 right[5]->joint1, right[0]->joint2, right[1]->joint3, right[4]->joint4, right[2]->joint5, right[3]->joint6
    positions_rad = arm_sensor_to_positions(right_arm_deg, joints_in_degrees=True)
    positions_deg = [p * (1.0 / DEG_TO_RAD) for p in positions_rad]
    expected_deg = [47.11, -1.97, -26.17, -24.19, 19.61, 25.59]
    for i in range(6):
        assert abs(positions_deg[i] - expected_deg[i]) < 0.02, (
            "Orion joint%d: got %.2f deg, expected %.2f" % (i + 1, positions_deg[i], expected_deg[i])
        )
    print("PASS arm_sensor: right_arm[5,0,1,4,2,3] -> Orion joint1~6 (47,-2,-26,-24,20,26 deg)")


def test_trajectory_to_command_mapping():
    """规划下发：Orion joint1~6(rad) -> command[15:21] 对应 right_arm[5,0,1,4,2,3]。"""
    # Orion 关节弧度 (例如 joint1=47°=0.82rad, joint2=-2°, ...)
    pos_rad = [
        47.0 * DEG_TO_RAD,
        -2.0 * DEG_TO_RAD,
        -26.0 * DEG_TO_RAD,
        -24.0 * DEG_TO_RAD,
        20.0 * DEG_TO_RAD,
        26.0 * DEG_TO_RAD,
    ]
    right_deg = trajectory_to_right_arm_deg(pos_rad)
    # 期望：command[15]=right_arm[0]=?, 我们写的是 orion_i->right_arm[ORION_TO_HOLOOCEAN_RIGHT_ARM[orion_i]]
    # orion0 -> right[5]=47, orion1->right[0]=-2, orion2->right[1]=-26, orion3->right[4]=-24, orion4->right[2]=20, orion5->right[3]=26
    assert abs(right_deg[5] - 47.0) < 0.02, "right_arm[5] should be 47 (Orion joint1)"
    assert abs(right_deg[0] - (-2.0)) < 0.02, "right_arm[0] should be -2 (Orion joint2)"
    assert abs(right_deg[1] - (-26.0)) < 0.02, "right_arm[1] should be -26 (Orion joint3)"
    assert abs(right_deg[4] - (-24.0)) < 0.02, "right_arm[4] should be -24 (Orion joint4)"
    assert abs(right_deg[2] - 20.0) < 0.02, "right_arm[2] should be 20 (Orion joint5)"
    assert abs(right_deg[3] - 26.0) < 0.02, "right_arm[3] should be 26 (Orion joint6)"
    print("PASS trajectory: Orion joint1~6 -> right_arm[5,0,1,4,2,3] (command[15:21])")


if __name__ == "__main__":
    test_arm_sensor_mapping_with_real_data()
    test_trajectory_to_command_mapping()
    print("All mapping tests passed.")
