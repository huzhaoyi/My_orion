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
  /** 侧向接近法向符号：+1 为默认几何；仿真/坐标系若表现为从下往上接近可置 -1 改为从另一侧（等效翻转 +Z/-Z 接近侧） */
  double approach_normal_sign = 1.0;
};

/*
 * TargetSensor 抓取（peg）：与缆绳共用 gripper_tcp 约定；接近方向符号与 cable_side_grasp.approach_normal_sign 一致。
 */
struct TargetSensorPickConfig
{
  /** 沿物体局部 +Z 定义的接近方向再乘此符号（<0 取反），与 HoloOcean/缆绳侧抓对齐 */
  double approach_normal_sign = -1.0;
  /** 预抓距候选 [m]，由大到小尝试（更远预抓更易在臂工作空间内求 IK） */
  std::vector<double> pregrasp_distances_m{ 0.42, 0.34, 0.26, 0.20, 0.14, 0.10, 0.06 };
  double retreat_distance_m = 0.12;
};

/*
 * TargetSensor 插孔（peg-in-hole）：几何沿 target_pose 姿态的局部 -Z 为插入轴（与历史 base_link 竖直孔一致）。
 */
struct PegInsertConfig
{
  double pre_offset_m = 0.10;
  double insert_depth_m = 0.04;
  double retreat_m = 0.12;
  double lin_velocity_scaling = 0.2;
  double lin_acceleration_scaling = 0.2;
  double cartesian_velocity_scaling = 0.25;
  double cartesian_acceleration_scaling = 0.25;
  double cartesian_step_size = 0.005;
  bool enable_chamfer_plane_search = false;
  double chamfer_plane_delta_m = 0.002;
  int insert_axial_segments = 1;
  /*
   * TARGET_INSERT：目标在 map/world 时若 TF 树无 map→base_link，可启用此项，用固定外参代替 lookupTransform。
   * 数值与 tf2 lookupTransform(\"base_link\", \"map\") 返回的 Transform 一致：parent=base_link，child=map。
   */
  bool use_static_map_to_base_for_target_insert = false;
  std::vector<double> static_transform_map_to_base_link;
};

struct MTCConfig
{
  double grasp_offset_along_axis = 0.0f;
  CableGraspConfig cable_grasp;
  TargetSensorPickConfig target_sensor_pick;
  PegInsertConfig peg_insert;
};

void declareParameters(rclcpp::Node* node);
void loadFromNode(rclcpp::Node* node, MTCConfig& config);

}  // namespace orion_mtc

#endif  // ORION_MTC_CONFIG_MTC_CONFIG_HPP
