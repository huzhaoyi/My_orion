/* 运行策略参数：声明与从 node 加载，风格与 MTCConfig 一致 */

#ifndef ORION_MTC_CONFIG_RUNTIME_POLICY_LOADER_HPP
#define ORION_MTC_CONFIG_RUNTIME_POLICY_LOADER_HPP

#include "orion_mtc/config/runtime_policy.hpp"
#include <memory>

namespace rclcpp
{
class Node;
}

namespace orion_mtc
{

void declareRuntimePolicyParameters(const std::shared_ptr<rclcpp::Node>& node);
void loadRuntimePolicyFromNode(const std::shared_ptr<rclcpp::Node>& node, RuntimePolicy& policy);

}  // namespace orion_mtc

#endif  // ORION_MTC_CONFIG_RUNTIME_POLICY_LOADER_HPP
