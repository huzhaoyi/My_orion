/* 机械臂/夹爪/碰撞相关常量，与 orion_moveit_config 一致 */

#ifndef ORION_MTC_CORE_CONSTANTS_HPP
#define ORION_MTC_CORE_CONSTANTS_HPP

#include <string>
#include <vector>

namespace orion_mtc
{

constexpr const char* const MANIPULATOR_NS = "/manipulator";

extern const std::vector<std::string> OBJECT_GRASP_ALLOWED_LINKS;
extern const std::vector<std::string> PREGRASP_OBJECT_ALLOWED_LINKS;
extern const std::vector<std::string> CABLE_LOCAL_PREGRASP_ALLOWED_LINKS;
extern const std::vector<std::string> CABLE_LOCAL_APPROACH_ALLOWED_LINKS;

extern const std::vector<std::string> ARM_JOINTS;
extern const std::vector<std::string> HAND_JOINTS;

}  // namespace orion_mtc

#endif  // ORION_MTC_CORE_CONSTANTS_HPP
