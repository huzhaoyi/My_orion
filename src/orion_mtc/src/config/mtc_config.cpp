#include "orion_mtc/config/mtc_config.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/exceptions.hpp>

namespace orion_mtc
{

void declareParameters(rclcpp::Node* node)
{
  if (!node)
  {
    return;
  }
  auto declare_if_not_set = [node](const char* name, double val) {
    try
    {
      node->declare_parameter<double>(name, val);
    }
    catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
    {
      /* 已由 launch 的 params 提供，跳过 */
    }
  };
  auto declare_str_if_not_set = [node](const char* name, const std::string& val) {
    try
    {
      node->declare_parameter<std::string>(name, val);
    }
    catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
    {
    }
  };
  declare_if_not_set("approach_object_min_dist", 0.08);
  declare_if_not_set("approach_object_max_dist", 0.15);
  declare_if_not_set("lift_object_min_dist", 0.05);
  declare_if_not_set("lift_object_max_dist", 0.25);
  declare_str_if_not_set("support_surface_link", "");
  declare_if_not_set("place_pose_x", 0.35);
  declare_if_not_set("place_pose_y", 0.15);
  declare_if_not_set("place_pose_z", 0.4);
  declare_if_not_set("place_pose_qx", 0.0);
  declare_if_not_set("place_pose_qy", 0.0);
  declare_if_not_set("place_pose_qz", 0.0);
  declare_if_not_set("place_pose_qw", 1.0);
  declare_if_not_set("retreat_min_dist", 0.12);
  declare_if_not_set("retreat_max_dist", 0.25);
  declare_if_not_set("lower_to_place_min_dist", 0.05);
  declare_if_not_set("lower_to_place_max_dist", 0.12);
}

void loadFromNode(rclcpp::Node* node, MTCConfig& config)
{
  if (!node)
  {
    return;
  }
  node->get_parameter("approach_object_min_dist", config.approach_object_min_dist);
  node->get_parameter("approach_object_max_dist", config.approach_object_max_dist);
  node->get_parameter("lift_object_min_dist", config.lift_object_min_dist);
  node->get_parameter("lift_object_max_dist", config.lift_object_max_dist);
  node->get_parameter("retreat_min_dist", config.retreat_min_dist);
  node->get_parameter("retreat_max_dist", config.retreat_max_dist);
  node->get_parameter("lower_to_place_min_dist", config.lower_to_place_min_dist);
  node->get_parameter("lower_to_place_max_dist", config.lower_to_place_max_dist);
  node->get_parameter("place_pose_x", config.default_place_x);
  node->get_parameter("place_pose_y", config.default_place_y);
  node->get_parameter("place_pose_z", config.default_place_z);
  node->get_parameter("place_pose_qx", config.default_place_qx);
  node->get_parameter("place_pose_qy", config.default_place_qy);
  node->get_parameter("place_pose_qz", config.default_place_qz);
  node->get_parameter("place_pose_qw", config.default_place_qw);
  node->get_parameter("support_surface_link", config.support_surface_link);
}

}  // namespace orion_mtc
