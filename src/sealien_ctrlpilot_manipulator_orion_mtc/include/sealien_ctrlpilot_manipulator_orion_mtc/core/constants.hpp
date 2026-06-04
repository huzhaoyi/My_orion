/* 机械臂/夹爪/碰撞相关常量，与 sealien_ctrlpilot_manipulator_orion_moveit_config 一致 */

#ifndef ORION_MTC_CORE_CONSTANTS_HPP
#define ORION_MTC_CORE_CONSTANTS_HPP

#include <string>
#include <vector>

namespace sealien_ctrlpilot_manipulator_orion_mtc
{

constexpr const char* const MANIPULATOR_NS = "/manipulator";

/* 孔位旁静态面板可视化（与 TaskManager 孔位 markers 同命名空间习惯） */
constexpr const char PANEL_OBSTACLES_MARKERS_TOPIC[] = "/manipulator/panel_obstacles_markers";

/* TargetSensor 规划场景中的 peg 网格 id（与缆绳 object 区分） */
constexpr const char TARGET_SENSOR_PEG_COLLISION_ID[] = "targetsensor_peg";

/* 插孔时附着到末端的持物杆体碰撞 id（用于 peg↔面板几何避障） */
constexpr const char INSERT_HELD_PEG_COLLISION_ID[] = "insert_held_peg";

/* 插孔时围绕目标孔心生成的「带孔门板」碰撞 id（仅对 peg 生效，对准从孔过、对不准被挡） */
constexpr const char INSERT_PEG_GATE_PANEL_ID[] = "insert_peg_gate_panel";

extern const std::vector<std::string> OBJECT_GRASP_ALLOWED_LINKS;
extern const std::vector<std::string> PREGRASP_OBJECT_ALLOWED_LINKS;
extern const std::vector<std::string> CABLE_LOCAL_PREGRASP_ALLOWED_LINKS;
extern const std::vector<std::string> CABLE_LOCAL_APPROACH_ALLOWED_LINKS;

/* TargetSensor peg 场景：臂全链路与 peg 网格可碰，避免预抓 PTP 因连杆扫过物体而 NO_IK_SOLUTION */
extern const std::vector<std::string> TARGET_SENSOR_PEG_ALLOWED_LINKS;

extern const std::vector<std::string> ARM_JOINTS;
extern const std::vector<std::string> HAND_JOINTS;

}  // namespace sealien_ctrlpilot_manipulator_orion_mtc

#endif  // ORION_MTC_CORE_CONSTANTS_HPP
