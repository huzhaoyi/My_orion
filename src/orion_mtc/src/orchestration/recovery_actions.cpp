#include "orion_mtc/orchestration/recovery_actions.hpp"
#include "orion_mtc/orchestration/task_manager.hpp"
#include "orion_mtc/scene/planning_scene_manager.hpp"
#include <rclcpp/rclcpp.hpp>

namespace orion_mtc
{

static const rclcpp::Logger LOGGER = rclcpp::get_logger("orion_mtc.recovery");

RecoveryActions::RecoveryActions(PlanningSceneManager* scene_manager, TaskManager* task_manager)
  : scene_manager_(scene_manager), task_manager_(task_manager)
{
}

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

bool RecoveryActions::goHomeIfSafe()
{
  /* 当前仅做状态与 scene 清理，不发送回 ready 轨迹；后续可接 move_group 或预存 joint state */
  RCLCPP_INFO(LOGGER, "goHomeIfSafe: no motion implemented yet, skip");
  return true;
}

}  // namespace orion_mtc
