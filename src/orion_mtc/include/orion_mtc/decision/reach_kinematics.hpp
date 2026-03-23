/* 从 MoveIt RobotModel 估计 gripper_tcp 相对 base_link 的可达距离上界，并解析 feasibility 参数 */

#ifndef ORION_MTC_DECISION_REACH_KINEMATICS_HPP
#define ORION_MTC_DECISION_REACH_KINEMATICS_HPP

#include <memory>
#include <rclcpp/node.hpp>
#include <string>

namespace moveit
{
namespace core
{
class RobotModel;
}
}  // namespace moveit

namespace orion_mtc
{

/** max_reach_soft 未显式配置时相对 hard 的比例 */
inline constexpr double KINEMATIC_SOFT_REACH_RATIO = 0.92;

/**
 * 沿 URDF 从 tip_link 走向 base_link，累加各子连杆 joint origin 平移范数。
 * 为可达球半径的保守上界（三角不等式）；真实最大可能略小。
 * @return 上界 [m]；链不存在则返回 -1.0
 */
double computeKinematicReachUpperBoundM(const moveit::core::RobotModel& model,
                                        const std::string& base_link,
                                        const std::string& tip_link);

struct ResolvedReachLimits
{
  double hard_m = 0.0;
  double soft_m = 0.0;
  bool used_kinematic = false;
  /** feasibility.max_reach_hard>0 时置位 */
  bool manual_hard = false;
  /** URDF 链长上界 [m]，未计算时为 -1.0 */
  double kinematic_ub_m = -1.0;
};

/**
 * 解析 feasibility.max_reach_hard / max_reach_soft：
 * - max_reach_hard > 0：使用给定值（手动覆盖）
 * - max_reach_hard <= 0：URDF 上界 × max_reach_hard_kinematic_margin（默认 1.08），略大于纯几何上界以便交给规划器尝试
 * - max_reach_soft <= 0：hard × KINEMATIC_SOFT_REACH_RATIO
 * 无模型时 hard 回退为 k_fallback_hard_m（默认 2.0），避免误杀
 */
ResolvedReachLimits resolveFeasibilityReachLimits(
    const rclcpp::Node::SharedPtr& node,
    const std::shared_ptr<const moveit::core::RobotModel>& robot_model,
    const std::string& base_link,
    const std::string& tip_link);

}  // namespace orion_mtc

#endif  // ORION_MTC_DECISION_REACH_KINEMATICS_HPP
