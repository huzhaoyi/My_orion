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
  std::vector<double> axial_shift_candidates{ -0.05, 0.0, 0.05 };
  std::vector<double> roll_candidates_deg{ 0.0, 180.0 };
  double retreat_dist = 0.05;
  /* 缆绳分段碰撞：总长、每段长、半径、抓取邻域段数（局部 ACM 的邻域） */
  double cable_total_length = 3.0;
  double cable_segment_length = 0.25;
  double cable_radius = 0.025;
  int grasp_neighbor_segments = 1;
};

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
  double retreat_min_dist = 0.05f;
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
  /* 长杆夹持点沿物体轴相对几何中心的偏移 [m]，正=物体 +Z 方向；0=几何中心，attach 与 pick 目标均用此 */
  double grasp_offset_along_axis = 0.0f;
  /* 放置前运输姿态：SRDF 中 arm 的 group_state 名，非空则在 move to pre-place 前先 move to 此姿态以减小扫掠 */
  std::string place_transport_pose = "transport";
  /* 退离第一段（short clear）距离范围 [m]，先脱离释放区 */
  double retreat_short_min_dist = 0.03f;
  double retreat_short_max_dist = 0.05f;
  /* 退离第二段（long leave）使用 retreat_min_dist / retreat_max_dist */

  /* 缆绳侧向抓取候选参数 */
  CableGraspConfig cable_grasp;
};

/* 在 node 上声明参数（若已声明则跳过，避免 ParameterAlreadyDeclaredException） */
void declareParameters(rclcpp::Node* node);

/* 从 node 读取当前参数填充 config */
void loadFromNode(rclcpp::Node* node, MTCConfig& config);

}  // namespace orion_mtc

#endif  // ORION_MTC_CONFIG_MTC_CONFIG_HPP
