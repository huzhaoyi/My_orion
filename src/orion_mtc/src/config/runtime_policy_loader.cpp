#include "orion_mtc/config/runtime_policy_loader.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/exceptions.hpp>

namespace orion_mtc
{

void declareRuntimePolicyParameters(const std::shared_ptr<rclcpp::Node>& node)
{
  if (!node)
  {
    return;
  }
  auto declare_bool = [&node](const std::string& name, bool default_val) {
    try
    {
      node->declare_parameter<bool>(name, default_val);
    }
    catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
    {
    }
  };
  auto declare_int = [&node](const std::string& name, int default_val) {
    try
    {
      node->declare_parameter<int>(name, default_val);
    }
    catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
    {
    }
  };
  const std::string prefix = "runtime_policy.";
  declare_bool(prefix + "auto_start_worker", true);
  declare_bool(prefix + "retry_on_plan_failure", false);
  declare_int(prefix + "max_retries", 1);
  declare_bool(prefix + "auto_clear_scene_before_sync", true);
  declare_bool(prefix + "auto_reset_after_execution_failure", true);
  declare_bool(prefix + "auto_go_home_after_failure", false);
  declare_bool(prefix + "reject_new_jobs_while_busy", false);
}

void loadRuntimePolicyFromNode(const std::shared_ptr<rclcpp::Node>& node, RuntimePolicy& policy)
{
  if (!node)
  {
    return;
  }
  const std::string prefix = "runtime_policy.";
  node->get_parameter(prefix + "auto_start_worker", policy.auto_start_worker);
  node->get_parameter(prefix + "retry_on_plan_failure", policy.retry_on_plan_failure);
  node->get_parameter(prefix + "max_retries", policy.max_retries);
  node->get_parameter(prefix + "auto_clear_scene_before_sync", policy.auto_clear_scene_before_sync);
  node->get_parameter(prefix + "auto_reset_after_execution_failure",
                      policy.auto_reset_after_execution_failure);
  node->get_parameter(prefix + "auto_go_home_after_failure", policy.auto_go_home_after_failure);
  node->get_parameter(prefix + "reject_new_jobs_while_busy", policy.reject_new_jobs_while_busy);
}

}  // namespace orion_mtc
