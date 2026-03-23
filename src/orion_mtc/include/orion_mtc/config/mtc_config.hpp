/* MTC 参数统一配置：业务逻辑依赖 config 对象，而非随处 get_parameter */

#ifndef ORION_MTC_CONFIG_MTC_CONFIG_HPP
#define ORION_MTC_CONFIG_MTC_CONFIG_HPP

#include <string>
#include <vector>

namespace rclcpp
{
class Node;
}

namespace orion_mtc
{

/** 缆绳侧向抓取候选参数（approach/axial/roll 多候选）+ 分段碰撞参数 */
struct CableGraspConfig
{
  std::vector<double> approach_dist_candidates{ 0.10, 0.08, 0.06, 0.04 };
  std::vector<double> pregrasp_offset_candidates{ 0.030, 0.040, 0.050 };
  std::vector<double> axial_shift_candidates{ -0.05, 0.0, 0.05 };
  std::vector<double> roll_candidates_deg{ 0.0 };
  std::vector<double> approach_around_axis_candidates_deg{ 0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0 };
  std::vector<double> grasp_depth_candidates{ 0.010, 0.015, 0.020 };
  std::vector<double> tcp_bias_rpy_deg{ 0.0, 0.0, 0.0 };
  double retreat_dist = 0.05;
  double cable_total_length = 3.0;
  double cable_segment_length = 0.25;
  double cable_radius = 0.025;
  int grasp_neighbor_segments = 1;
  double approach_lin_velocity_scaling = 0.5;
  double approach_lin_acceleration_scaling = 0.35;
};

struct MTCConfig
{
  double lift_object_min_dist = 0.05f;
  double lift_object_max_dist = 0.25f;
  double grasp_offset_along_axis = 0.0f;
  CableGraspConfig cable_grasp;
};

void declareParameters(rclcpp::Node* node);
void loadFromNode(rclcpp::Node* node, MTCConfig& config);

}  // namespace orion_mtc

#endif  // ORION_MTC_CONFIG_MTC_CONFIG_HPP
