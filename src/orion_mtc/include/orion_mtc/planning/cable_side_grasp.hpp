/* 缆绳侧向包夹抓取：tool_y=缆绳轴、tool_z=接近方向（与 orion gripper_tcp / Link6 一致） */

#ifndef ORION_MTC_PLANNING_CABLE_SIDE_GRASP_HPP
#define ORION_MTC_PLANNING_CABLE_SIDE_GRASP_HPP

#include "orion_mtc/config/mtc_config.hpp"
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <optional>
#include <rclcpp/time.hpp>
#include <string>
#include <vector>

namespace orion_mtc
{

/** 缆绳检测输入：position/direction 需已在规划系（如 base_link）下 */
struct CableDetection
{
  Eigen::Vector3d position;
  Eigen::Vector3d direction;  // 单位向量，缆绳轴线
};

/** 单条抓取候选：grasp_pose / pregrasp_pose 为 base_link 下 */
struct CableGraspCandidate
{
  Eigen::Isometry3d grasp_pose;
  Eigen::Isometry3d pregrasp_pose;
  Eigen::Vector3d approach_dir;   // 从 pregrasp 指向 grasp（侧向接近）
  Eigen::Vector3d retreat_dir;
  double approach_dist = 0.0;
  double retreat_dist = 0.0;
  double score = 0.0;
  /** 最近段索引（用于局部 ACM） */
  int nearest_segment_index = -1;
  /** 抓取局部邻域段索引（仅对这些段放宽末端碰撞） */
  std::vector<int> local_segment_indices;
};

/**
 * 生成侧向包夹抓取候选（±法向、多接近距离、轴向微移、滚转），按 score 升序排序。
 * 输入 cable 的 position/direction 必须在规划系（base_link）下。
 */
std::vector<CableGraspCandidate> generateCableSideGrasps(const CableDetection& cable,
                                                         const CableGraspConfig& cfg);

/**
 * 生成最优单条侧抓（全部候选排序后取 score 最小）。
 */
std::optional<CableGraspCandidate> generateBestCableSideGrasp(const CableDetection& cable,
                                                              const CableGraspConfig& cfg);

/** Eigen::Isometry3d -> geometry_msgs::Pose */
void isometryToPose(const Eigen::Isometry3d& T,
                    geometry_msgs::msg::Pose& out);

/** 带 frame_id 和 stamp 的 PoseStamped */
geometry_msgs::msg::PoseStamped toPoseStamped(const Eigen::Isometry3d& T,
                                             const std::string& frame_id,
                                             const rclcpp::Time& stamp);

/** 带 frame_id 的 Vector3Stamped（世界系方向，供 MTC MoveRelative 使用） */
geometry_msgs::msg::Vector3Stamped toVector3Stamped(const Eigen::Vector3d& v,
                                                    const std::string& frame_id,
                                                    const rclcpp::Time& stamp);

}  // namespace orion_mtc

#endif  // ORION_MTC_PLANNING_CABLE_SIDE_GRASP_HPP
