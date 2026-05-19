/* constants：ARM_JOINTS / HAND_JOINTS 及 allowed collision 链接名列表 */

#include "orion_mtc/core/constants.hpp"

namespace orion_mtc
{

/*
 * 抓取阶段允许与目标缆/柱体接触的连杆集合（ACM 条目来源）。
 */
const std::vector<std::string> OBJECT_GRASP_ALLOWED_LINKS = { "gripper_tcp", "Link6", "Link7", "Link8" };

/*
 * 预抓接近时额外允许与中臂连杆碰擦（避免误报自碰）。
 */
const std::vector<std::string> PREGRASP_OBJECT_ALLOWED_LINKS = {
    "Link2", "Link3", "Link4", "Link5", "gripper_tcp", "Link6", "Link7", "Link8"
};

/*
 * 缆段局部预检：仅放宽手爪链与邻近 segment 的碰撞。
 */
const std::vector<std::string> CABLE_LOCAL_PREGRASP_ALLOWED_LINKS = {
    "gripper_tcp", "Link6", "Link7", "Link8"
};

/*
 * 缆侧接近阶段 ACM 与会话一致的手爪链路。
 */
const std::vector<std::string> CABLE_LOCAL_APPROACH_ALLOWED_LINKS = {
    "gripper_tcp", "Link6", "Link7", "Link8"
};

const std::vector<std::string> TARGET_SENSOR_PEG_ALLOWED_LINKS = {
    "arm_base_link",
    "Link1",
    "Link2",
    "Link3",
    "LinkVirtual",
    "Link4",
    "Link5",
    "gripper_tcp",
    "Link6",
    "Link7",
    "Link8",
};

/*
 * MoveIt group「arm」关节名顺序，与轨迹/自碰判断一致。
 */
const std::vector<std::string> ARM_JOINTS = {
    "joint_arm_base_link_Link1", "joint_Link1_Link2", "joint_Link2_Link3",
    "joint_LinkVirtual_Link4", "joint_Link4_Link5", "joint_Link5_Link6"
};

/*
 * 夹爪并联关节名，供纯手爪段与闭合判定使用。
 */
const std::vector<std::string> HAND_JOINTS = { "joint_Link6_Link7", "joint_Link6_Link8" };

}  // namespace orion_mtc
