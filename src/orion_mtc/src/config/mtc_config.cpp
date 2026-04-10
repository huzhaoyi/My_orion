/* mtc_config：自 ROS 参数声明加载抓取几何与工作空间审批阈值 */

#include "orion_mtc/config/mtc_config.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/exceptions.hpp>
#include <string>

namespace orion_mtc
{

/*
 * 声明 cable_side_grasp.* 与 grasp_offset_along_axis 等参数；已声明时吞掉异常，便于重复加载。
 */
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
  try
  {
    node->declare_parameter<double>("cable_side_grasp.approach_normal_sign", 1.0);
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<double>("target_sensor_pick.approach_normal_sign", -1.0);
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<double>("target_sensor_pick.grasp_depth_m", 0.015);
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<std::vector<double>>(
        "target_sensor_pick.pregrasp_distances_m",
        std::vector<double>{ 0.42, 0.34, 0.26, 0.20, 0.14, 0.10, 0.06 });
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<double>("target_sensor_pick.retreat_distance_m", 0.12);
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<double>("peg_insert.pre_offset_m", 0.10);
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<double>("peg_insert.insert_depth_m", 0.04);
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<double>("peg_insert.retreat_m", 0.12);
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<std::vector<double>>("peg_insert.insert_axis_local_xyz",
                                                 std::vector<double>{ 1.0, 0.0, 0.0 });
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<double>("peg_insert.lin_velocity_scaling", 0.2);
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<double>("peg_insert.lin_acceleration_scaling", 0.2);
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<double>("peg_insert.cartesian_velocity_scaling", 0.25);
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<double>("peg_insert.cartesian_acceleration_scaling", 0.25);
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<double>("peg_insert.cartesian_step_size", 0.005);
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<bool>("peg_insert.enable_chamfer_plane_search", false);
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<double>("peg_insert.chamfer_plane_delta_m", 0.002);
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<int>("peg_insert.insert_axial_segments", 1);
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<bool>("peg_insert.use_static_map_to_base_for_target_insert", false);
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<std::vector<double>>("peg_insert.static_transform_map_to_base_link",
                                                 std::vector<double>{});
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<bool>("peg_insert.target_insert_use_configured_hole_positions_map", false);
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  {
    static const double k_slot_def_m[7][3] = {
        { -113.93, 129.1, -132.1 },
        { -113.93, 128.9, -132.1 },
        { -113.93, 129.1, -132.36 },
        { -113.93, 128.9, -132.35 },
        { -113.93, 128.6, -132.36 },
        { -113.93, 127.8, -130.89 },
        { -113.93, 127.8, -131.12 },
    };
    for (int i = 0; i < 7; ++i)
    {
      const std::string key =
          "peg_insert.targetsensor_slot_" + std::to_string(i + 1) + "_position_map";
      try
      {
        node->declare_parameter<std::vector<double>>(
            key, std::vector<double>{ k_slot_def_m[i][0], k_slot_def_m[i][1], k_slot_def_m[i][2] });
      }
      catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
      {
      }
    }
  }
}

/*
 * 从 node 读取已声明参数填入 MTCConfig；node 为空则立即返回。
 */
void loadFromNode(rclcpp::Node* node, MTCConfig& config)
{
  if (!node)
  {
    return;
  }
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
  node->get_parameter("cable_side_grasp.approach_normal_sign", config.cable_grasp.approach_normal_sign);
  node->get_parameter("target_sensor_pick.approach_normal_sign", config.target_sensor_pick.approach_normal_sign);
  node->get_parameter("target_sensor_pick.grasp_depth_m", config.target_sensor_pick.grasp_depth_m);
  node->get_parameter("target_sensor_pick.pregrasp_distances_m", config.target_sensor_pick.pregrasp_distances_m);
  node->get_parameter("target_sensor_pick.retreat_distance_m", config.target_sensor_pick.retreat_distance_m);
  if (config.target_sensor_pick.pregrasp_distances_m.empty())
  {
    config.target_sensor_pick.pregrasp_distances_m = { 0.10 };
  }
  node->get_parameter("peg_insert.pre_offset_m", config.peg_insert.pre_offset_m);
  node->get_parameter("peg_insert.insert_depth_m", config.peg_insert.insert_depth_m);
  node->get_parameter("peg_insert.retreat_m", config.peg_insert.retreat_m);
  node->get_parameter("peg_insert.insert_axis_local_xyz", config.peg_insert.insert_axis_local_xyz);
  if (config.peg_insert.insert_axis_local_xyz.size() != 3u)
  {
    config.peg_insert.insert_axis_local_xyz = { 1.0, 0.0, 0.0 };
  }
  node->get_parameter("peg_insert.lin_velocity_scaling", config.peg_insert.lin_velocity_scaling);
  node->get_parameter("peg_insert.lin_acceleration_scaling", config.peg_insert.lin_acceleration_scaling);
  node->get_parameter("peg_insert.cartesian_velocity_scaling", config.peg_insert.cartesian_velocity_scaling);
  node->get_parameter("peg_insert.cartesian_acceleration_scaling", config.peg_insert.cartesian_acceleration_scaling);
  node->get_parameter("peg_insert.cartesian_step_size", config.peg_insert.cartesian_step_size);
  node->get_parameter("peg_insert.enable_chamfer_plane_search", config.peg_insert.enable_chamfer_plane_search);
  node->get_parameter("peg_insert.chamfer_plane_delta_m", config.peg_insert.chamfer_plane_delta_m);
  node->get_parameter("peg_insert.insert_axial_segments", config.peg_insert.insert_axial_segments);
  if (config.peg_insert.insert_axial_segments < 1)
  {
    config.peg_insert.insert_axial_segments = 1;
  }
  node->get_parameter("peg_insert.use_static_map_to_base_for_target_insert",
                       config.peg_insert.use_static_map_to_base_for_target_insert);
  node->get_parameter("peg_insert.static_transform_map_to_base_link",
                       config.peg_insert.static_transform_map_to_base_link);
  node->get_parameter("peg_insert.target_insert_use_configured_hole_positions_map",
                       config.peg_insert.target_insert_use_configured_hole_positions_map);
  for (int i = 0; i < 7; ++i)
  {
    const std::string key =
        "peg_insert.targetsensor_slot_" + std::to_string(i + 1) + "_position_map";
    node->get_parameter(key, config.peg_insert.targetsensor_slot_position_map[static_cast<std::size_t>(i)]);
  }
}

}  // namespace orion_mtc
