/* 三个原子恢复动作：清理 scene 残留、清业务状态、回 ready（可选），供失败时按 policy 组合调用 */

#ifndef ORION_MTC_ORCHESTRATION_RECOVERY_ACTIONS_HPP
#define ORION_MTC_ORCHESTRATION_RECOVERY_ACTIONS_HPP

namespace orion_mtc
{
class PlanningSceneManager;
class TaskManager;
}

namespace orion_mtc
{

class RecoveryActions
{
public:
  RecoveryActions(PlanningSceneManager* scene_manager, TaskManager* task_manager);

  /** 清理 held_unknown / held_tracked / object 的 attach 残留 */
  bool clearSceneResiduals();

  /** 清业务持物状态并清理 scene attach */
  bool resetHeldState();

  /** 尝试让机械臂回 ready；当前可为空实现或仅做状态清理，后续接 move_group */
  bool goHomeIfSafe();

private:
  PlanningSceneManager* scene_manager_;
  TaskManager* task_manager_;
};

}  // namespace orion_mtc

#endif  // ORION_MTC_ORCHESTRATION_RECOVERY_ACTIONS_HPP
