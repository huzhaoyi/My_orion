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
  declare_if_not_set("approach_min_fraction", 0.85);
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
  declare_if_not_set("retreat_min_dist", 0.07);
  declare_if_not_set("retreat_max_dist", 0.25);
  declare_if_not_set("retreat_short_min_dist", 0.03);
  declare_if_not_set("retreat_short_max_dist", 0.05);
  declare_if_not_set("lower_to_place_min_dist", 0.05);
  declare_if_not_set("lower_to_place_max_dist", 0.12);
  declare_if_not_set("grasp_offset_along_axis", 0.0);
  declare_str_if_not_set("place_transport_pose", "transport");
  try
  {
    node->declare_parameter<std::vector<double>>(
        "cable_side_grasp.approach_dist_candidates",
        std::vector<double>{ 0.10, 0.08, 0.06, 0.04 });
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<std::vector<double>>(
        "cable_side_grasp.axial_shift_candidates",
        std::vector<double>{ -0.05, 0.0, 0.05 });
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<std::vector<double>>(
        "cable_side_grasp.roll_candidates_deg",
        std::vector<double>{ 0.0, 180.0 });
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<double>("cable_side_grasp.retreat_dist", 0.05);
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<double>("cable_side_grasp.cable_total_length", 3.0);
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<double>("cable_side_grasp.cable_segment_length", 0.25);
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<double>("cable_side_grasp.cable_radius", 0.025);
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<int>("cable_side_grasp.grasp_neighbor_segments", 1);
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
}

void loadFromNode(rclcpp::Node* node, MTCConfig& config)
{
  if (!node)
  {
    return;
  }
  node->get_parameter("approach_object_min_dist", config.approach_object_min_dist);
  node->get_parameter("approach_object_max_dist", config.approach_object_max_dist);
  node->get_parameter("approach_min_fraction", config.approach_min_fraction);
  node->get_parameter("lift_object_min_dist", config.lift_object_min_dist);
  node->get_parameter("lift_object_max_dist", config.lift_object_max_dist);
  node->get_parameter("retreat_min_dist", config.retreat_min_dist);
  node->get_parameter("retreat_max_dist", config.retreat_max_dist);
  node->get_parameter("retreat_short_min_dist", config.retreat_short_min_dist);
  node->get_parameter("retreat_short_max_dist", config.retreat_short_max_dist);
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
  node->get_parameter("grasp_offset_along_axis", config.grasp_offset_along_axis);
  node->get_parameter("place_transport_pose", config.place_transport_pose);
  node->get_parameter("cable_side_grasp.approach_dist_candidates",
                      config.cable_grasp.approach_dist_candidates);
  node->get_parameter("cable_side_grasp.axial_shift_candidates",
                      config.cable_grasp.axial_shift_candidates);
  node->get_parameter("cable_side_grasp.roll_candidates_deg",
                      config.cable_grasp.roll_candidates_deg);
  node->get_parameter("cable_side_grasp.retreat_dist", config.cable_grasp.retreat_dist);
  node->get_parameter("cable_side_grasp.cable_total_length", config.cable_grasp.cable_total_length);
  node->get_parameter("cable_side_grasp.cable_segment_length", config.cable_grasp.cable_segment_length);
  node->get_parameter("cable_side_grasp.cable_radius", config.cable_grasp.cable_radius);
  node->get_parameter("cable_side_grasp.grasp_neighbor_segments", config.cable_grasp.grasp_neighbor_segments);
}

}  // namespace orion_mtc
