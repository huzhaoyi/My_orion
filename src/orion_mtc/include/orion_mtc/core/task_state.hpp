/* 机器人任务状态：纯数据与状态判断，不依赖 ROS 业务流程 */

#ifndef ORION_MTC_CORE_TASK_STATE_HPP
#define ORION_MTC_CORE_TASK_STATE_HPP

#include <string>

namespace orion_mtc
{

/* HOLDING_TRACKED 可精准放置，HOLDING_UNTRACKED 仅可 release 放置 */
enum class RobotTaskMode
{
  IDLE,
  PICKING,
  HOLDING_TRACKED,   /* 有完整 held_object（来自 pick），可 place_precise */
  HOLDING_UNTRACKED, /* 手里可能有物但无 tcp_to_object，仅可 place_release */
  PLACING,
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
    case RobotTaskMode::PLACING:
      return "PLACING";
    case RobotTaskMode::ERROR:
      return "ERROR";
    default:
      return "UNKNOWN";
  }
}

}  // namespace orion_mtc

#endif  // ORION_MTC_CORE_TASK_STATE_HPP
