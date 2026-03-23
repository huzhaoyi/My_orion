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
    }
  };
  declare_if_not_set("lift_object_min_dist", 0.05);
  declare_if_not_set("lift_object_max_dist", 0.25);
  declare_if_not_set("grasp_offset_along_axis", 0.0);
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
        std::vector<double>{ 0.0 });
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<std::vector<double>>(
        "cable_side_grasp.approach_around_axis_candidates_deg",
        std::vector<double>{ 0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0 });
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<std::vector<double>>(
        "cable_side_grasp.grasp_depth_candidates",
        std::vector<double>{ 0.010, 0.015, 0.020 });
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<std::vector<double>>(
        "cable_side_grasp.pregrasp_offset_candidates",
        std::vector<double>{ 0.030, 0.040, 0.050 });
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<std::vector<double>>(
        "cable_side_grasp.tcp_bias_rpy_deg",
        std::vector<double>{ 0.0, 0.0, 0.0 });
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
  try
  {
    node->declare_parameter<double>("cable_side_grasp.approach_lin_velocity_scaling", 0.5);
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<double>("cable_side_grasp.approach_lin_acceleration_scaling", 0.35);
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
  node->get_parameter("lift_object_min_dist", config.lift_object_min_dist);
  node->get_parameter("lift_object_max_dist", config.lift_object_max_dist);
  node->get_parameter("grasp_offset_along_axis", config.grasp_offset_along_axis);
  node->get_parameter("cable_side_grasp.approach_dist_candidates",
                      config.cable_grasp.approach_dist_candidates);
  node->get_parameter("cable_side_grasp.axial_shift_candidates",
                      config.cable_grasp.axial_shift_candidates);
  node->get_parameter("cable_side_grasp.roll_candidates_deg",
                      config.cable_grasp.roll_candidates_deg);
  node->get_parameter("cable_side_grasp.approach_around_axis_candidates_deg",
                      config.cable_grasp.approach_around_axis_candidates_deg);
  node->get_parameter("cable_side_grasp.grasp_depth_candidates",
                      config.cable_grasp.grasp_depth_candidates);
  node->get_parameter("cable_side_grasp.pregrasp_offset_candidates",
                      config.cable_grasp.pregrasp_offset_candidates);
  node->get_parameter("cable_side_grasp.tcp_bias_rpy_deg", config.cable_grasp.tcp_bias_rpy_deg);
  node->get_parameter("cable_side_grasp.retreat_dist", config.cable_grasp.retreat_dist);
  node->get_parameter("cable_side_grasp.cable_total_length", config.cable_grasp.cable_total_length);
  node->get_parameter("cable_side_grasp.cable_segment_length", config.cable_grasp.cable_segment_length);
  node->get_parameter("cable_side_grasp.cable_radius", config.cable_grasp.cable_radius);
  node->get_parameter("cable_side_grasp.grasp_neighbor_segments", config.cable_grasp.grasp_neighbor_segments);
  node->get_parameter("cable_side_grasp.approach_lin_velocity_scaling",
                      config.cable_grasp.approach_lin_velocity_scaling);
  node->get_parameter("cable_side_grasp.approach_lin_acceleration_scaling",
                      config.cable_grasp.approach_lin_acceleration_scaling);
}

}  // namespace orion_mtc
