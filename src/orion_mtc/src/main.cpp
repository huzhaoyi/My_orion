#include "orion_mtc/app/orion_mtc_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <thread>

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
