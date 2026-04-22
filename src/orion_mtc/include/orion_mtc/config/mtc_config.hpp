/* MTC 参数统一配置：业务逻辑依赖 config 对象，而非随处 get_parameter */

#ifndef ORION_MTC_CONFIG_MTC_CONFIG_HPP
#define ORION_MTC_CONFIG_MTC_CONFIG_HPP

#include <string>
#include <cstdint>
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
  std::vector<double> approach_around_axis_candidates_deg{ 0.0, 45.0, 90.0, 135.0, 180.0 };
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
  /** 抓取接近轴（物体局部轴）：0->X, 1->Y, 2->Z。默认 2（Z 轴）用于顶部下压。 */
  int approach_axis_local = 2;
  /** 抓取接近轴候选（按顺序优先尝试）：0->X, 1->Y, 2->Z。默认顶部优先。 */
  std::vector<int64_t> approach_axis_local_candidates{ 2, 0, 1 };
  /** 是否按目标距离动态切换接近轴优先级。 */
  bool dynamic_axis_priority_enable = true;
  /** 近距离上限 [m]：<=该值使用 near 顺序。 */
  double dynamic_axis_near_max_distance_m = 0.55;
  /** 中距离上限 [m]：>near_max 且 <=mid_max 使用 mid 顺序；>mid_max 进入 far 顺序。 */
  double dynamic_axis_mid_max_distance_m = 1.10;
  /** 近距离接近轴顺序（默认顶部优先）。 */
  std::vector<int64_t> dynamic_axis_near_order{ 2, 0, 1 };
  /** 中距离接近轴顺序（默认前向与顶部并重，先前向）。 */
  std::vector<int64_t> dynamic_axis_mid_order{ 0, 2, 1 };
  /** 远距离接近轴顺序（默认前向优先）。 */
  std::vector<int64_t> dynamic_axis_far_order{ 0, 2, 1 };
  /** 是否按目标高度 z 动态覆盖接近轴优先级。 */
  bool dynamic_axis_priority_by_z_enable = true;
  /** 低高度阈值 [m]：目标 z <= 该值时，使用 low_z 顺序。 */
  double dynamic_axis_low_z_threshold_m = -0.15;
  /** 低高度接近轴顺序（默认前向优先，其次顶部，再侧向）。 */
  std::vector<int64_t> dynamic_axis_low_z_order{ 0, 2, 1 };
  /** 从目标中心沿接近反方向退让到“表面附近”的距离 [m]，用于避免以中心点规划导致 Link6 先碰。 */
  double surface_backoff_m = 0.0;
  /** 终抓阶段沿接近方向前推的“吃入深度” [m]（参考缆绳 grasp_depth 语义）。 */
  double grasp_depth_m = 0.015;
  /** 预抓距候选 [m]，由大到小尝试（更远预抓更易在臂工作空间内求 IK） */
  std::vector<double> pregrasp_distances_m{ 0.42, 0.34, 0.26, 0.20, 0.14, 0.10, 0.06 };
  /** 额外预抓距回退候选 [m]（与 pregrasp_distances_m 合并去重后排序）。 */
  std::vector<double> fallback_pregrasp_distances_m{ 0.06, 0.08, 0.10, 0.14, 0.20, 0.26 };
  /** 接近方向符号候选（通常包含 +1/-1）；空时回退为 approach_normal_sign。 */
  std::vector<double> approach_sign_candidates{ -1.0, 1.0 };
  /** 是否根据目标横向位置（base_link.y）动态调整 sign 优先顺序。 */
  bool dynamic_sign_priority_enable = true;
  /** 当目标 y < 0 时优先的 sign（常用 +1）。 */
  double preferred_sign_for_negative_y = 1.0;
  /** 当目标 y >= 0 时优先的 sign（常用 -1）。 */
  double preferred_sign_for_positive_y = -1.0;
  /** TargetSensor 顶抓姿态绕接近轴滚转候选 [deg]。 */
  std::vector<double> tool_roll_candidates_deg{ 0.0, 45.0, -45.0, 90.0, -90.0 };
  double retreat_distance_m = 0.12;
};

/*
 * TargetSensor 插孔（peg-in-hole）：插入轴由 target_pose 旋转后的局部轴向定义（insert_axis_local_xyz）。
 */
struct PegInsertConfig
{
  double pre_offset_m = 0.10;
  /* true：插孔前先回 SRDF 命名状态 ready，减少当前姿态导致的绕行。 */
  bool go_ready_before_insert = false;
  /*
   * true：在 move to pre-insert 前，先到孔前方更远处的前置点，再线性推进到 pre-insert，
   * 用于减少从孔后方绕到前方的观感问题。
   */
  bool enable_front_waypoint = false;
  /*
   * true：前置点按 base_link 的 X 轴构造（x = target.x - front_waypoint_base_x_offset_m，y/z 与 target 相同）。
   * false：沿插入轴从 pre-insert 反向退 front_waypoint_offset_m。
   */
  bool front_waypoint_use_base_x = false;
  /* 前置点在 base_link X 方向相对孔位的退让距离 [m]（front_waypoint_use_base_x=true 时生效）。 */
  double front_waypoint_base_x_offset_m = 0.10;
  /*
   * true：pre-insert 按 base_link X 轴保持安全距离（x = target.x - pre_insert_base_x_offset_m），
   * 让对准阶段不直接贴孔。
   */
  bool pre_insert_use_base_x = false;
  /* pre-insert 在 base_link X 方向相对孔位的退让距离 [m]（pre_insert_use_base_x=true 时生效）。 */
  double pre_insert_base_x_offset_m = 0.20;
  /* 孔前置点相对 pre-insert 继续沿 -insert_axis 退让的距离 [m]。 */
  double front_waypoint_offset_m = 0.08;
  double insert_depth_m = 0.04;
  /* 插孔释放后沿 base_link +Z 线性抬离距离 [m]。 */
  double retreat_m = 0.12;
  /* 局部插入轴（hole/slot 局部系），默认 +X；运行时会自动归一化。 */
  std::vector<double> insert_axis_local_xyz{ 1.0, 0.0, 0.0 };
  /* 绕插入轴旋转末端姿态 [deg]（仅改姿态不改插入方向）；常用 180.0 以翻转“杆/孔前后”。 */
  double tool_roll_about_insert_axis_deg = 0.0;
  /* gripper_tcp 局部姿态偏置 [deg]，顺序 roll/pitch/yaw；用于“像 link5 一样”微调插孔朝向。 */
  std::vector<double> tool_rpy_offset_deg{ 0.0, 0.0, 0.0 };
  double lin_velocity_scaling = 0.2;
  double lin_acceleration_scaling = 0.2;
  double cartesian_velocity_scaling = 0.25;
  double cartesian_acceleration_scaling = 0.25;
  double cartesian_step_size = 0.005;
  bool enable_chamfer_plane_search = false;
  double chamfer_plane_delta_m = 0.002;
  int insert_axial_segments = 1;
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
