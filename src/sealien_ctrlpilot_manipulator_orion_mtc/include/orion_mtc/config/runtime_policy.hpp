/* 运行策略：失败后怎么办、是否自动恢复等，与 MTCConfig 几何参数分离 */

#ifndef ORION_MTC_CONFIG_RUNTIME_POLICY_HPP
#define ORION_MTC_CONFIG_RUNTIME_POLICY_HPP

namespace orion_mtc
{

struct RuntimePolicy
{
  bool auto_start_worker = true;
  bool retry_on_plan_failure = false;
  int max_retries = 1;

  bool auto_clear_scene_before_sync = true;
  bool auto_reset_after_execution_failure = true;
  bool auto_go_home_after_failure = false;

  bool reject_new_jobs_while_busy = false;
};

}  // namespace orion_mtc

#endif  // ORION_MTC_CONFIG_RUNTIME_POLICY_HPP
