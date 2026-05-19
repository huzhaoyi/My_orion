/* RoboticArmCmd / RoboticArmRequest 与 orion_mtc 对接常量 */

#ifndef ORION_MTC_INTERFACE_ROBOTIC_ARM_CMD_TYPES_HPP
#define ORION_MTC_INTERFACE_ROBOTIC_ARM_CMD_TYPES_HPP

#include <cstdint>

namespace orion_mtc
{
namespace robotic_arm_cmd
{

constexpr uint8_t REQUEST_TYPE_GRASP = 0U;
constexpr uint8_t REQUEST_TYPE_INSERT = 1U;

constexpr uint8_t RESULT_SUCCESS = 0U;
constexpr uint8_t RESULT_EXEC_FAILED = 1U;
constexpr uint8_t RESULT_REJECTED_STATE = 2U;
constexpr uint8_t RESULT_REJECTED_NO_HELD = 3U;
constexpr uint8_t RESULT_ESTOP = 4U;
constexpr uint8_t RESULT_INVALID = 5U;

}  // namespace robotic_arm_cmd
}  // namespace orion_mtc

#endif  // ORION_MTC_INTERFACE_ROBOTIC_ARM_CMD_TYPES_HPP
