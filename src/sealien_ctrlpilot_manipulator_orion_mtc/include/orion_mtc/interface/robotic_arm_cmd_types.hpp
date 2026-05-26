/* RoboticArmCmd / RoboticArmRequest 与 orion_mtc 对接常量 */

#ifndef ORION_MTC_INTERFACE_ROBOTIC_ARM_CMD_TYPES_HPP
#define ORION_MTC_INTERFACE_ROBOTIC_ARM_CMD_TYPES_HPP

#include <cstdint>

namespace orion_mtc
{
namespace robotic_arm_cmd
{

/*
 * RoboticArmCmd order.type（与 sealien_ctrlpilot_msgmanagement/RoboticArmRequest.type 一致）
 * CancelGoal 非 type：任意进行中 Goal 取消时执行 急停→开爪→ready(关爪)→clear_estop
 */
constexpr uint8_t REQUEST_TYPE_GRASP = 0U;         /* 销钉 / TargetSensor 抓取；TARGET_SENSOR 链；TF 后不做物体系姿态标定 */
constexpr uint8_t REQUEST_TYPE_INSERT = 1U;        /* 插孔；需已持物；order.keypoint 为孔位（常用 odom） */
constexpr uint8_t REQUEST_TYPE_GRASP_CABLE = 2U;   /* 缆绳侧抓；LEGACY 链；TF 后可右乘 orientation_correction */
constexpr uint8_t REQUEST_TYPE_OPEN_GRIPPER = 3U;  /* 仅开夹爪（hand group）；keypoint 可忽略 */
constexpr uint8_t REQUEST_TYPE_CLOSE_GRIPPER = 4U; /* 仅关夹爪（hand group）；keypoint 可忽略 */
constexpr uint8_t REQUEST_TYPE_GO_READY = 5U;      /* 回 SRDF ready 并关爪；keypoint 可忽略；busy 时可能拒绝 */

inline bool isPickRequestType(uint8_t request_type)
{
  return request_type == REQUEST_TYPE_GRASP || request_type == REQUEST_TYPE_GRASP_CABLE;
}

inline bool isMaintenanceRequestType(uint8_t request_type)
{
  return request_type == REQUEST_TYPE_OPEN_GRIPPER || request_type == REQUEST_TYPE_CLOSE_GRIPPER ||
         request_type == REQUEST_TYPE_GO_READY;
}

constexpr uint8_t RESULT_SUCCESS = 0U;
constexpr uint8_t RESULT_EXEC_FAILED = 1U;
constexpr uint8_t RESULT_REJECTED_STATE = 2U;
constexpr uint8_t RESULT_REJECTED_NO_HELD = 3U;
constexpr uint8_t RESULT_ESTOP = 4U;
constexpr uint8_t RESULT_INVALID = 5U;

}  // namespace robotic_arm_cmd
}  // namespace orion_mtc

#endif  // ORION_MTC_INTERFACE_ROBOTIC_ARM_CMD_TYPES_HPP
