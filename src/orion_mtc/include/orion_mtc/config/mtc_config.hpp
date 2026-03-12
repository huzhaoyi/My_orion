/* MTC 参数统一配置：业务逻辑依赖 config 对象，而非随处 get_parameter */

#ifndef ORION_MTC_CONFIG_MTC_CONFIG_HPP
#define ORION_MTC_CONFIG_MTC_CONFIG_HPP

#include <string>

namespace rclcpp
{
class Node;
}

namespace orion_mtc
{

struct MTCConfig
{
  /* pick approach / lift */
  double approach_object_min_dist = 0.08f;
  double approach_object_max_dist = 0.15f;
  /* approach 段最少走满距离的比例 [0,1]，越大越容易“夹到”（末端更贴近物体再闭合） */
  double approach_min_fraction = 0.85f;
  double lift_object_min_dist = 0.05f;
  double lift_object_max_dist = 0.25f;
  /* place retreat / lower */
  double retreat_min_dist = 0.12f;
  double retreat_max_dist = 0.25f;
  double lower_to_place_min_dist = 0.05f;
  double lower_to_place_max_dist = 0.12f;
  /* 默认放置位姿（无 place_pose 话题时使用） */
  double default_place_x = 0.35f;
  double default_place_y = 0.15f;
  double default_place_z = 0.4f;
  double default_place_qx = 0.0f;
  double default_place_qy = 0.0f;
  double default_place_qz = 0.0f;
  double default_place_qw = 1.0f;
  /* 支撑面 link，空则不加 allow/forbid collision */
  std::string support_surface_link;
};

/* 在 node 上声明参数（若已声明则跳过，避免 ParameterAlreadyDeclaredException） */
void declareParameters(rclcpp::Node* node);

/* 从 node 读取当前参数填充 config */
void loadFromNode(rclcpp::Node* node, MTCConfig& config);

}  // namespace orion_mtc

#endif  // ORION_MTC_CONFIG_MTC_CONFIG_HPP
