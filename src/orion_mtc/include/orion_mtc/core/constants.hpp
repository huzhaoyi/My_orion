/* 机械臂/夹爪/碰撞相关常量，与 orion_moveit_config 一致 */

#ifndef ORION_MTC_CORE_CONSTANTS_HPP
#define ORION_MTC_CORE_CONSTANTS_HPP

#include <string>
#include <vector>

namespace orion_mtc
{

/* 机械臂话题/服务/action 统一前缀 */
constexpr const char* const MANIPULATOR_NS = "/manipulator";

/* 末端与物体允许接触的 link：仅手腕/夹爪，不放开整臂 */
extern const std::vector<std::string> OBJECT_GRASP_ALLOWED_LINKS;

/* return home 阶段允许与已放置物体接触的 link（仅末端 Link5~8，不放开整臂） */
extern const std::vector<std::string> RETURN_HOME_OBJECT_ALLOWED_LINKS;

/* 控制器对应关节列表（与 orion_moveit_config moveit_controllers.yaml 一致） */
extern const std::vector<std::string> ARM_JOINTS;
extern const std::vector<std::string> HAND_JOINTS;

}  // namespace orion_mtc

#endif  // ORION_MTC_CORE_CONSTANTS_HPP
