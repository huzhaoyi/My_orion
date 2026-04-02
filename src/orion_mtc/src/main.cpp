/* orion_mtc 入口：规划节点与 action_client 节点共用 MultiThreadedExecutor，主线程 join 前调用 setupPlanningScene */

#include "orion_mtc/app/orion_mtc_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <thread>

/*
 * 初始化 rclcpp，启用参数覆盖；orion_mtc_action_client 与 orion_mtc_node 同挂 MultiThreadedExecutor，
 * 使 FeasibilityChecker 在 orion_mtc_node 上的 joint_states 等回调持续被驱动；主线程在 spin 后调用 setupPlanningScene，再 join。
 */
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<orion_mtc::OrionMTCNode>(options);
  rclcpp::ExecutorOptions exec_opts;
  rclcpp::executors::MultiThreadedExecutor executor(exec_opts, 8U);

  auto spin_thread = std::make_unique<std::thread>([&executor, &node]() {
    executor.add_node(node->getNodeBaseInterface());
    executor.add_node(node->getPlanningNodeBaseInterface());
    executor.spin();
    executor.remove_node(node->getPlanningNodeBaseInterface());
    executor.remove_node(node->getNodeBaseInterface());
  });

  node->setupPlanningScene();
  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
