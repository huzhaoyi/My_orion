/* recovery_actions：失败后按 policy 清 scene、复位持物、可选 Pilz 回 ready */

#include "orion_mtc/orchestration/recovery_actions.hpp"
#include "orion_mtc/orchestration/task_manager.hpp"
#include "orion_mtc/scene/planning_scene_manager.hpp"
#include <rclcpp/rclcpp.hpp>

namespace orion_mtc
{

static const rclcpp::Logger LOGGER = rclcpp::get_logger("orion_mtc.recovery");

/*
 * 保存 scene 与 task 管理指针；不拥有所有权，与 OrionMTCNode 生命周期对齐。
 */
RecoveryActions::RecoveryActions(PlanningSceneManager* scene_manager, TaskManager* task_manager)
  : scene_manager_(scene_manager), task_manager_(task_manager)
{
}

/*
 * 依次 detach held_unknown、held_tracked、object；任一步失败记 WARN，最终仍返回综合成败。
 */
bool RecoveryActions::clearSceneResiduals()
{
  if (!scene_manager_)
  {
    return true;
  }
  bool ok = true;
  if (!scene_manager_->clearAttachedObjectFromPlanningScene("held_unknown"))
  {
    RCLCPP_WARN(LOGGER, "clearSceneResiduals: clear held_unknown failed");
    ok = false;
  }
  if (!scene_manager_->clearAttachedObjectFromPlanningScene("held_tracked"))
  {
    RCLCPP_WARN(LOGGER, "clearSceneResiduals: clear held_tracked failed");
    ok = false;
  }
  if (!scene_manager_->clearAttachedObjectFromPlanningScene("object"))
  {
    RCLCPP_WARN(LOGGER, "clearSceneResiduals: clear object failed");
    ok = false;
  }
  if (ok)
  {
    RCLCPP_INFO(LOGGER, "clearSceneResiduals: done");
  }
  return ok;
}

/*
 * 委托 TaskManager::handleResetHeldObject 清除内部持物状态；无 task_manager 则视为 no-op 成功。
 */
bool RecoveryActions::resetHeldState()
{
  if (!task_manager_)
  {
    return true;
  }
  std::string msg;
  bool ok = task_manager_->handleResetHeldObject(msg);
  if (!ok)
  {
    RCLCPP_WARN(LOGGER, "resetHeldState: %s", msg.c_str());
  }
  return ok;
}

/*
 * 预留占位：当前不发起回零运动，仅日志说明；避免误称已执行 home。
 */
bool RecoveryActions::goHomeIfSafe()
{
  /* 当前仅做状态与 scene 清理，不发送回 ready 轨迹；后续可接 move_group 或预存 joint state */
  RCLCPP_INFO(LOGGER, "goHomeIfSafe: no motion implemented yet, skip");
  return true;
}

}  // namespace orion_mtc
