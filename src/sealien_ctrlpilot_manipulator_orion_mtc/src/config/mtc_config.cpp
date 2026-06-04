/* mtc_config：自 ROS 参数声明加载抓取几何与工作空间审批阈值 */

#include "sealien_ctrlpilot_manipulator_orion_mtc/config/mtc_config.hpp"
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/exceptions.hpp>
#include <string>

namespace sealien_ctrlpilot_manipulator_orion_mtc
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
        std::vector<double>{ 0.0, 45.0, 90.0, 135.0, 180.0 });
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
    node->declare_parameter<int>("target_sensor_pick.approach_axis_local", 2);
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<std::vector<int64_t>>(
        "target_sensor_pick.approach_axis_local_candidates",
        std::vector<int64_t>{ 2, 0, 1 });
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<int>("target_sensor_pick.rod_axis_local", 2);
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<std::vector<int64_t>>(
        "target_sensor_pick.hard_forbid_approach_axes_local",
        std::vector<int64_t>{});
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<bool>("target_sensor_pick.dynamic_axis_priority_enable", true);
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<double>("target_sensor_pick.dynamic_axis_near_max_distance_m", 0.55);
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<double>("target_sensor_pick.dynamic_axis_mid_max_distance_m", 1.10);
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<std::vector<int64_t>>(
        "target_sensor_pick.dynamic_axis_near_order",
        std::vector<int64_t>{ 2, 0, 1 });
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<std::vector<int64_t>>(
        "target_sensor_pick.dynamic_axis_mid_order",
        std::vector<int64_t>{ 0, 2, 1 });
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<std::vector<int64_t>>(
        "target_sensor_pick.dynamic_axis_far_order",
        std::vector<int64_t>{ 0, 2, 1 });
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<bool>("target_sensor_pick.dynamic_axis_priority_by_z_enable", true);
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<double>("target_sensor_pick.dynamic_axis_low_z_threshold_m", -0.15);
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<std::vector<int64_t>>(
        "target_sensor_pick.dynamic_axis_low_z_order",
        std::vector<int64_t>{ 0, 2, 1 });
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<bool>("target_sensor_pick.low_z_specialized_candidates_enable", true);
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<int>("target_sensor_pick.low_z_pregrasp_keep_count", 4);
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<std::vector<double>>(
        "target_sensor_pick.low_z_sign_candidates",
        std::vector<double>{});
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<std::vector<double>>(
        "target_sensor_pick.low_z_roll_candidates_deg",
        std::vector<double>{ 0.0, 15.0, -15.0 });
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<double>("target_sensor_pick.surface_backoff_m", 0.0);
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
    node->declare_parameter<std::vector<double>>(
        "target_sensor_pick.fallback_pregrasp_distances_m",
        std::vector<double>{ 0.06, 0.08, 0.10, 0.14, 0.20, 0.26 });
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<std::vector<double>>(
        "target_sensor_pick.approach_sign_candidates",
        std::vector<double>{ -1.0, 1.0 });
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<bool>("target_sensor_pick.dynamic_sign_priority_enable", true);
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<double>("target_sensor_pick.preferred_sign_for_negative_y", 1.0);
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<double>("target_sensor_pick.preferred_sign_for_positive_y", -1.0);
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<std::vector<double>>(
        "target_sensor_pick.tool_roll_candidates_deg",
        std::vector<double>{ 0.0, 45.0, -45.0, 90.0, -90.0 });
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
    node->declare_parameter<std::vector<double>>(
        "target_sensor_pick.insert_axis_hint_base_xyz",
        std::vector<double>{ 0.0, 0.0, 0.0 });
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<int>("target_sensor_pick.insert_axis_hint_target_set_index", -1);
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<double>("target_sensor_pick.max_abs_dot_approach_insert_hint", 1.0);
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<double>("target_sensor_pick.insert_hint_min_abs_dot_rod", 0.0);
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<std::vector<double>>(
        "target_sensor_pick.handle_axis_local_xyz",
        std::vector<double>{});
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<double>("target_sensor_pick.insert_hint_max_abs_dot_handle", 1.0);
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
    node->declare_parameter<bool>("peg_insert.go_ready_before_insert", false);
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<bool>("peg_insert.pre_insert_use_base_x", false);
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<double>("peg_insert.pre_insert_base_x_offset_m", 0.20);
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
    node->declare_parameter<double>("peg_insert.latch_pre_extract_m", 0.0);
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
    node->declare_parameter<double>("peg_insert.insert_motion_axis_sign", 1.0);
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<std::vector<double>>("peg_insert.peg_rod_axis_tcp_xyz",
                                                 std::vector<double>{ 0.0, 1.0, 0.0 });
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<bool>("peg_insert.auto_flip_rod_to_insert", false);
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<bool>("peg_insert.swap_tip_handle_180", false);
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<double>("peg_insert.tool_roll_about_insert_axis_deg", 0.0);
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<std::vector<double>>(
        "peg_insert.tool_rpy_offset_deg",
        std::vector<double>{ 0.0, 0.0, 0.0 });
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
    node->declare_parameter<bool>("peg_insert.enable_align_depth_guard", true);
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<bool>("peg_insert.enable_hole_panel_collision", true);
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<double>("peg_insert.hole_radius_m", 0.03);
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<double>("peg_insert.peg_radius_m", 0.012);
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<double>("peg_insert.align_lateral_tol_m", 0.0);
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<double>("peg_insert.align_stop_margin_m", 0.005);
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<double>("peg_insert.panel_front_offset_m", 0.0);
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<double>("peg_insert.peg_collision_length_m", 0.16);
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<double>("peg_insert.panel_gate_half_size_m", 0.30);
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<double>("peg_insert.panel_gate_thickness_m", 0.02);
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<bool>("panel_obstacles.enable", false);
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<bool>("panel_obstacles.publish_markers", true);
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<std::string>("panel_obstacles.frame_id", "arm_base_link");
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<double>("panel_obstacles.unit_scale", 0.01);
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<double>("panel_obstacles.wall_thickness_m", 0.02);
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<double>("panel_obstacles.aabb_margin_m", 0.005);
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<std::string>("panel_obstacles.panel_1_id", "hole_panel_1");
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<std::vector<double>>("panel_obstacles.panel_1_corners_xyz", std::vector<double>{});
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<std::string>("panel_obstacles.panel_2_id", "hole_panel_2");
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<std::vector<double>>("panel_obstacles.panel_2_corners_xyz", std::vector<double>{});
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<std::string>("manipulator_frame.tf_target_frame", "sensor_left_roboticarm");
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
  }
  try
  {
    node->declare_parameter<std::string>("manipulator_frame.planning_frame_id", "arm_base_link");
  }
  catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&)
  {
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
  node->get_parameter("target_sensor_pick.approach_axis_local", config.target_sensor_pick.approach_axis_local);
  if (config.target_sensor_pick.approach_axis_local < 0 || config.target_sensor_pick.approach_axis_local > 2)
  {
    config.target_sensor_pick.approach_axis_local = 2;
  }
  node->get_parameter(
      "target_sensor_pick.approach_axis_local_candidates",
      config.target_sensor_pick.approach_axis_local_candidates);
  node->get_parameter("target_sensor_pick.rod_axis_local", config.target_sensor_pick.rod_axis_local);
  if (config.target_sensor_pick.rod_axis_local < 0 || config.target_sensor_pick.rod_axis_local > 2)
  {
    config.target_sensor_pick.rod_axis_local = 2;
  }
  node->get_parameter("target_sensor_pick.hard_forbid_approach_axes_local",
                      config.target_sensor_pick.hard_forbid_approach_axes_local);
  std::vector<int64_t> normalized_forbid_axes;
  for (int64_t axis : config.target_sensor_pick.hard_forbid_approach_axes_local)
  {
    if (axis < 0 || axis > 2)
    {
      continue;
    }
    if (std::find(normalized_forbid_axes.begin(), normalized_forbid_axes.end(), axis) ==
        normalized_forbid_axes.end())
    {
      normalized_forbid_axes.push_back(axis);
    }
  }
  config.target_sensor_pick.hard_forbid_approach_axes_local = normalized_forbid_axes;
  std::vector<int64_t> normalized_axis_candidates;
  for (int64_t axis : config.target_sensor_pick.approach_axis_local_candidates)
  {
    if (axis < 0 || axis > 2)
    {
      continue;
    }
    if (std::find(normalized_axis_candidates.begin(), normalized_axis_candidates.end(), axis) ==
        normalized_axis_candidates.end())
    {
      normalized_axis_candidates.push_back(axis);
    }
  }
  if (normalized_axis_candidates.empty())
  {
    normalized_axis_candidates.push_back(config.target_sensor_pick.approach_axis_local);
  }
  config.target_sensor_pick.approach_axis_local_candidates = normalized_axis_candidates;
  node->get_parameter("target_sensor_pick.dynamic_axis_priority_enable",
                      config.target_sensor_pick.dynamic_axis_priority_enable);
  node->get_parameter("target_sensor_pick.dynamic_axis_near_max_distance_m",
                      config.target_sensor_pick.dynamic_axis_near_max_distance_m);
  node->get_parameter("target_sensor_pick.dynamic_axis_mid_max_distance_m",
                      config.target_sensor_pick.dynamic_axis_mid_max_distance_m);
  if (config.target_sensor_pick.dynamic_axis_near_max_distance_m < 0.0)
  {
    config.target_sensor_pick.dynamic_axis_near_max_distance_m = 0.0;
  }
  if (config.target_sensor_pick.dynamic_axis_mid_max_distance_m <
      config.target_sensor_pick.dynamic_axis_near_max_distance_m)
  {
    config.target_sensor_pick.dynamic_axis_mid_max_distance_m =
        config.target_sensor_pick.dynamic_axis_near_max_distance_m;
  }
  node->get_parameter("target_sensor_pick.dynamic_axis_near_order",
                      config.target_sensor_pick.dynamic_axis_near_order);
  node->get_parameter("target_sensor_pick.dynamic_axis_mid_order",
                      config.target_sensor_pick.dynamic_axis_mid_order);
  node->get_parameter("target_sensor_pick.dynamic_axis_far_order",
                      config.target_sensor_pick.dynamic_axis_far_order);
  node->get_parameter("target_sensor_pick.dynamic_axis_priority_by_z_enable",
                      config.target_sensor_pick.dynamic_axis_priority_by_z_enable);
  node->get_parameter("target_sensor_pick.dynamic_axis_low_z_threshold_m",
                      config.target_sensor_pick.dynamic_axis_low_z_threshold_m);
  node->get_parameter("target_sensor_pick.dynamic_axis_low_z_order",
                      config.target_sensor_pick.dynamic_axis_low_z_order);
  node->get_parameter("target_sensor_pick.low_z_specialized_candidates_enable",
                      config.target_sensor_pick.low_z_specialized_candidates_enable);
  node->get_parameter("target_sensor_pick.low_z_pregrasp_keep_count",
                      config.target_sensor_pick.low_z_pregrasp_keep_count);
  if (config.target_sensor_pick.low_z_pregrasp_keep_count < 0)
  {
    config.target_sensor_pick.low_z_pregrasp_keep_count = 0;
  }
  node->get_parameter("target_sensor_pick.low_z_sign_candidates",
                      config.target_sensor_pick.low_z_sign_candidates);
  node->get_parameter("target_sensor_pick.low_z_roll_candidates_deg",
                      config.target_sensor_pick.low_z_roll_candidates_deg);
  for (double& sign : config.target_sensor_pick.low_z_sign_candidates)
  {
    sign = (sign < 0.0) ? -1.0 : 1.0;
  }
  std::sort(config.target_sensor_pick.low_z_sign_candidates.begin(),
            config.target_sensor_pick.low_z_sign_candidates.end());
  config.target_sensor_pick.low_z_sign_candidates.erase(
      std::unique(config.target_sensor_pick.low_z_sign_candidates.begin(),
                  config.target_sensor_pick.low_z_sign_candidates.end()),
      config.target_sensor_pick.low_z_sign_candidates.end());
  auto normalize_axis_order = [&config](const std::vector<int64_t>& src) {
    std::vector<int64_t> out;
    for (int64_t axis : src)
    {
      if (axis < 0 || axis > 2)
      {
        continue;
      }
      if (std::find(out.begin(), out.end(), axis) == out.end())
      {
        out.push_back(axis);
      }
    }
    if (out.empty())
    {
      out = config.target_sensor_pick.approach_axis_local_candidates;
    }
    return out;
  };
  config.target_sensor_pick.dynamic_axis_near_order =
      normalize_axis_order(config.target_sensor_pick.dynamic_axis_near_order);
  config.target_sensor_pick.dynamic_axis_mid_order =
      normalize_axis_order(config.target_sensor_pick.dynamic_axis_mid_order);
  config.target_sensor_pick.dynamic_axis_far_order =
      normalize_axis_order(config.target_sensor_pick.dynamic_axis_far_order);
  config.target_sensor_pick.dynamic_axis_low_z_order =
      normalize_axis_order(config.target_sensor_pick.dynamic_axis_low_z_order);
  node->get_parameter("target_sensor_pick.surface_backoff_m", config.target_sensor_pick.surface_backoff_m);
  if (config.target_sensor_pick.surface_backoff_m < 0.0)
  {
    config.target_sensor_pick.surface_backoff_m = 0.0;
  }
  node->get_parameter("target_sensor_pick.grasp_depth_m", config.target_sensor_pick.grasp_depth_m);
  node->get_parameter("target_sensor_pick.pregrasp_distances_m", config.target_sensor_pick.pregrasp_distances_m);
  node->get_parameter("target_sensor_pick.fallback_pregrasp_distances_m",
                      config.target_sensor_pick.fallback_pregrasp_distances_m);
  node->get_parameter("target_sensor_pick.approach_sign_candidates",
                      config.target_sensor_pick.approach_sign_candidates);
  node->get_parameter("target_sensor_pick.dynamic_sign_priority_enable",
                      config.target_sensor_pick.dynamic_sign_priority_enable);
  node->get_parameter("target_sensor_pick.preferred_sign_for_negative_y",
                      config.target_sensor_pick.preferred_sign_for_negative_y);
  node->get_parameter("target_sensor_pick.preferred_sign_for_positive_y",
                      config.target_sensor_pick.preferred_sign_for_positive_y);
  config.target_sensor_pick.preferred_sign_for_negative_y =
      (config.target_sensor_pick.preferred_sign_for_negative_y < 0.0) ? -1.0 : 1.0;
  config.target_sensor_pick.preferred_sign_for_positive_y =
      (config.target_sensor_pick.preferred_sign_for_positive_y < 0.0) ? -1.0 : 1.0;
  node->get_parameter("target_sensor_pick.tool_roll_candidates_deg",
                      config.target_sensor_pick.tool_roll_candidates_deg);
  node->get_parameter("target_sensor_pick.retreat_distance_m", config.target_sensor_pick.retreat_distance_m);
  if (config.target_sensor_pick.pregrasp_distances_m.empty())
  {
    config.target_sensor_pick.pregrasp_distances_m = { 0.10 };
  }
  if (config.target_sensor_pick.fallback_pregrasp_distances_m.empty())
  {
    config.target_sensor_pick.fallback_pregrasp_distances_m = { 0.06, 0.08, 0.10, 0.14, 0.20, 0.26 };
  }
  if (config.target_sensor_pick.approach_sign_candidates.empty())
  {
    config.target_sensor_pick.approach_sign_candidates = { config.target_sensor_pick.approach_normal_sign };
  }
  if (config.target_sensor_pick.tool_roll_candidates_deg.empty())
  {
    config.target_sensor_pick.tool_roll_candidates_deg = { 0.0 };
  }
  node->get_parameter("target_sensor_pick.insert_axis_hint_base_xyz",
                      config.target_sensor_pick.insert_axis_hint_base_xyz);
  node->get_parameter("target_sensor_pick.insert_axis_hint_target_set_index",
                      config.target_sensor_pick.insert_axis_hint_target_set_index);
  node->get_parameter("target_sensor_pick.max_abs_dot_approach_insert_hint",
                      config.target_sensor_pick.max_abs_dot_approach_insert_hint);
  node->get_parameter("target_sensor_pick.insert_hint_min_abs_dot_rod",
                      config.target_sensor_pick.insert_hint_min_abs_dot_rod);
  node->get_parameter("target_sensor_pick.handle_axis_local_xyz",
                      config.target_sensor_pick.handle_axis_local_xyz);
  node->get_parameter("target_sensor_pick.insert_hint_max_abs_dot_handle",
                      config.target_sensor_pick.insert_hint_max_abs_dot_handle);
  if (config.target_sensor_pick.max_abs_dot_approach_insert_hint < 0.0)
  {
    config.target_sensor_pick.max_abs_dot_approach_insert_hint = 0.0;
  }
  if (config.target_sensor_pick.max_abs_dot_approach_insert_hint > 1.0)
  {
    config.target_sensor_pick.max_abs_dot_approach_insert_hint = 1.0;
  }
  if (config.target_sensor_pick.insert_hint_min_abs_dot_rod < 0.0)
  {
    config.target_sensor_pick.insert_hint_min_abs_dot_rod = 0.0;
  }
  if (config.target_sensor_pick.insert_hint_min_abs_dot_rod > 1.0)
  {
    config.target_sensor_pick.insert_hint_min_abs_dot_rod = 1.0;
  }
  if (config.target_sensor_pick.insert_hint_max_abs_dot_handle < 0.0)
  {
    config.target_sensor_pick.insert_hint_max_abs_dot_handle = 0.0;
  }
  if (config.target_sensor_pick.insert_hint_max_abs_dot_handle > 1.0)
  {
    config.target_sensor_pick.insert_hint_max_abs_dot_handle = 1.0;
  }
  node->get_parameter("peg_insert.pre_offset_m", config.peg_insert.pre_offset_m);
  node->get_parameter("peg_insert.go_ready_before_insert", config.peg_insert.go_ready_before_insert);
  node->get_parameter("peg_insert.pre_insert_use_base_x", config.peg_insert.pre_insert_use_base_x);
  node->get_parameter("peg_insert.pre_insert_base_x_offset_m", config.peg_insert.pre_insert_base_x_offset_m);
  if (config.peg_insert.pre_insert_base_x_offset_m < 0.0)
  {
    config.peg_insert.pre_insert_base_x_offset_m = 0.0;
  }
  node->get_parameter("peg_insert.insert_depth_m", config.peg_insert.insert_depth_m);
  node->get_parameter("peg_insert.latch_pre_extract_m", config.peg_insert.latch_pre_extract_m);
  if (!std::isfinite(config.peg_insert.latch_pre_extract_m))
  {
    config.peg_insert.latch_pre_extract_m = 0.0;
  }
  if (config.peg_insert.latch_pre_extract_m < 0.0)
  {
    config.peg_insert.latch_pre_extract_m = 0.0;
  }
  node->get_parameter("peg_insert.retreat_m", config.peg_insert.retreat_m);
  node->get_parameter("peg_insert.insert_axis_local_xyz", config.peg_insert.insert_axis_local_xyz);
  if (config.peg_insert.insert_axis_local_xyz.size() != 3u)
  {
    config.peg_insert.insert_axis_local_xyz = { 1.0, 0.0, 0.0 };
  }
  node->get_parameter("peg_insert.insert_motion_axis_sign", config.peg_insert.insert_motion_axis_sign);
  if (!std::isfinite(config.peg_insert.insert_motion_axis_sign)
      || std::abs(config.peg_insert.insert_motion_axis_sign) < 1e-9)
  {
    config.peg_insert.insert_motion_axis_sign = 1.0;
  }
  node->get_parameter("peg_insert.peg_rod_axis_tcp_xyz", config.peg_insert.peg_rod_axis_tcp_xyz);
  if (config.peg_insert.peg_rod_axis_tcp_xyz.size() != 3u)
  {
    config.peg_insert.peg_rod_axis_tcp_xyz = { 0.0, 1.0, 0.0 };
  }
  node->get_parameter("peg_insert.auto_flip_rod_to_insert", config.peg_insert.auto_flip_rod_to_insert);
  node->get_parameter("peg_insert.swap_tip_handle_180", config.peg_insert.swap_tip_handle_180);
  node->get_parameter("peg_insert.tool_roll_about_insert_axis_deg",
                      config.peg_insert.tool_roll_about_insert_axis_deg);
  node->get_parameter("peg_insert.tool_rpy_offset_deg", config.peg_insert.tool_rpy_offset_deg);
  if (config.peg_insert.tool_rpy_offset_deg.size() != 3u)
  {
    config.peg_insert.tool_rpy_offset_deg = { 0.0, 0.0, 0.0 };
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
  node->get_parameter("peg_insert.enable_align_depth_guard", config.peg_insert.enable_align_depth_guard);
  node->get_parameter("peg_insert.enable_hole_panel_collision", config.peg_insert.enable_hole_panel_collision);
  node->get_parameter("peg_insert.hole_radius_m", config.peg_insert.hole_radius_m);
  if (!std::isfinite(config.peg_insert.hole_radius_m) || config.peg_insert.hole_radius_m <= 0.0)
  {
    config.peg_insert.hole_radius_m = 0.03;
  }
  node->get_parameter("peg_insert.peg_radius_m", config.peg_insert.peg_radius_m);
  if (!std::isfinite(config.peg_insert.peg_radius_m) || config.peg_insert.peg_radius_m < 0.0)
  {
    config.peg_insert.peg_radius_m = 0.012;
  }
  node->get_parameter("peg_insert.align_lateral_tol_m", config.peg_insert.align_lateral_tol_m);
  if (!std::isfinite(config.peg_insert.align_lateral_tol_m) || config.peg_insert.align_lateral_tol_m < 0.0)
  {
    config.peg_insert.align_lateral_tol_m = 0.0;
  }
  node->get_parameter("peg_insert.align_stop_margin_m", config.peg_insert.align_stop_margin_m);
  if (!std::isfinite(config.peg_insert.align_stop_margin_m) || config.peg_insert.align_stop_margin_m < 0.0)
  {
    config.peg_insert.align_stop_margin_m = 0.0;
  }
  node->get_parameter("peg_insert.panel_front_offset_m", config.peg_insert.panel_front_offset_m);
  if (!std::isfinite(config.peg_insert.panel_front_offset_m))
  {
    config.peg_insert.panel_front_offset_m = 0.0;
  }
  node->get_parameter("peg_insert.peg_collision_length_m", config.peg_insert.peg_collision_length_m);
  if (!std::isfinite(config.peg_insert.peg_collision_length_m) || config.peg_insert.peg_collision_length_m <= 1e-4)
  {
    config.peg_insert.peg_collision_length_m = 0.16;
  }
  node->get_parameter("peg_insert.panel_gate_half_size_m", config.peg_insert.panel_gate_half_size_m);
  if (!std::isfinite(config.peg_insert.panel_gate_half_size_m) || config.peg_insert.panel_gate_half_size_m <= 1e-3)
  {
    config.peg_insert.panel_gate_half_size_m = 0.30;
  }
  node->get_parameter("peg_insert.panel_gate_thickness_m", config.peg_insert.panel_gate_thickness_m);
  if (!std::isfinite(config.peg_insert.panel_gate_thickness_m) || config.peg_insert.panel_gate_thickness_m <= 1e-3)
  {
    config.peg_insert.panel_gate_thickness_m = 0.02;
  }
  node->get_parameter("panel_obstacles.enable", config.panel_obstacles.enable);
  node->get_parameter("panel_obstacles.publish_markers", config.panel_obstacles.publish_markers);
  node->get_parameter("panel_obstacles.frame_id", config.panel_obstacles.frame_id);
  if (config.panel_obstacles.frame_id.empty())
  {
    config.panel_obstacles.frame_id = "arm_base_link";
  }
  node->get_parameter("panel_obstacles.unit_scale", config.panel_obstacles.unit_scale);
  if (!std::isfinite(config.panel_obstacles.unit_scale) || config.panel_obstacles.unit_scale <= 0.0)
  {
    config.panel_obstacles.unit_scale = 1.0;
  }
  node->get_parameter("panel_obstacles.wall_thickness_m", config.panel_obstacles.wall_thickness_m);
  if (!std::isfinite(config.panel_obstacles.wall_thickness_m) || config.panel_obstacles.wall_thickness_m <= 0.0)
  {
    config.panel_obstacles.wall_thickness_m = 0.02;
  }
  node->get_parameter("panel_obstacles.aabb_margin_m", config.panel_obstacles.aabb_margin_m);
  if (!std::isfinite(config.panel_obstacles.aabb_margin_m) || config.panel_obstacles.aabb_margin_m < 0.0)
  {
    config.panel_obstacles.aabb_margin_m = 0.0;
  }
  config.panel_obstacles.panels.clear();
  std::string panel_1_id;
  std::vector<double> panel_1_corners;
  std::string panel_2_id;
  std::vector<double> panel_2_corners;
  node->get_parameter("panel_obstacles.panel_1_id", panel_1_id);
  node->get_parameter("panel_obstacles.panel_1_corners_xyz", panel_1_corners);
  node->get_parameter("panel_obstacles.panel_2_id", panel_2_id);
  node->get_parameter("panel_obstacles.panel_2_corners_xyz", panel_2_corners);
  if (!panel_1_id.empty() && panel_1_corners.size() >= 12u)
  {
    PanelObstacleEntry e;
    e.id = panel_1_id;
    e.corners_xyz = std::move(panel_1_corners);
    config.panel_obstacles.panels.push_back(std::move(e));
  }
  if (!panel_2_id.empty() && panel_2_corners.size() >= 12u)
  {
    PanelObstacleEntry e;
    e.id = panel_2_id;
    e.corners_xyz = std::move(panel_2_corners);
    config.panel_obstacles.panels.push_back(std::move(e));
  }
  node->get_parameter("manipulator_frame.tf_target_frame", config.manipulator_frame.tf_target_frame);
  node->get_parameter("manipulator_frame.planning_frame_id", config.manipulator_frame.planning_frame_id);
  if (config.manipulator_frame.tf_target_frame.empty())
  {
    config.manipulator_frame.tf_target_frame = "sensor_left_roboticarm";
  }
  if (config.manipulator_frame.planning_frame_id.empty())
  {
    config.manipulator_frame.planning_frame_id = "arm_base_link";
  }
}

}  // namespace sealien_ctrlpilot_manipulator_orion_mtc
