/* 机器人任务状态：纯数据与状态判断，不依赖 ROS 业务流程 */

#ifndef ORION_MTC_CORE_TASK_STATE_HPP
#define ORION_MTC_CORE_TASK_STATE_HPP

#include <string>

namespace orion_mtc
{

enum class RobotTaskMode
{
  IDLE,
  PICKING,
  HOLDING_TRACKED,
  HOLDING_UNTRACKED,
  ERROR
};

inline bool isHolding(RobotTaskMode m)
{
  return m == RobotTaskMode::HOLDING_TRACKED || m == RobotTaskMode::HOLDING_UNTRACKED;
}

inline bool isIdleOrError(RobotTaskMode m)
{
  return m == RobotTaskMode::IDLE || m == RobotTaskMode::ERROR;
}

inline bool canAcceptPick(RobotTaskMode m)
{
  return isIdleOrError(m) && !isHolding(m);
}

inline std::string toStateString(RobotTaskMode m)
{
  switch (m)
  {
    case RobotTaskMode::IDLE:
      return "IDLE";
    case RobotTaskMode::PICKING:
      return "PICKING";
    case RobotTaskMode::HOLDING_TRACKED:
      return "HOLDING_TRACKED";
    case RobotTaskMode::HOLDING_UNTRACKED:
      return "HOLDING_UNTRACKED";
    case RobotTaskMode::ERROR:
      return "ERROR";
    default:
      return "UNKNOWN";
  }
}

}  // namespace orion_mtc

#endif  // ORION_MTC_CORE_TASK_STATE_HPP
