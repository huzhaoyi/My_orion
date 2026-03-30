/* orion_mtc 入口：规划节点与 action_client 节点共用 MultiThreadedExecutor，主线程 join 前调用 setupPlanningScene */

#include "orion_mtc/app/orion_mtc_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <thread>

/*
 * 初始化 rclcpp，启用参数覆盖；单节点挂 MultiThreadedExecutor 子线程 spin，
 * 主线程在首帧 spin 后调用 setupPlanningScene 同步刷 scene，再 join 退出。
 */
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<orion_mtc::OrionMTCNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &node]() {
    executor.add_node(node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(node->getNodeBaseInterface());
  });

  node->setupPlanningScene();
  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
