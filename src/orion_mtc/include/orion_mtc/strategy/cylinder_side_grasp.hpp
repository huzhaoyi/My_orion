/* 圆柱抓取策略：TOP_DOWN（正上方下压）+ 碰撞体姿态构造 */

#ifndef ORION_MTC_STRATEGY_CYLINDER_SIDE_GRASP_HPP
#define ORION_MTC_STRATEGY_CYLINDER_SIDE_GRASP_HPP

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <optional>

namespace orion_mtc
{

struct CylinderSideGraspSemantic
{
  geometry_msgs::msg::Quaternion grasp_orientation;
  geometry_msgs::msg::Vector3 approach_axis;
};

/* 正上方下压抓取语义：
 * - grasp Z 轴 = approach_axis（从 grasp 指向 pregrasp 的方向，通常是 base_link 的“上”）
 * - grasp X 轴尽量沿圆柱轴（axis_direction），但会被正交化到垂直于 Z 的平面，避免退化。 */
std::optional<CylinderSideGraspSemantic> buildTopDownCylinderGrasp(
    const geometry_msgs::msg::Vector3& axis_direction,
    const geometry_msgs::msg::Vector3& up_dir);

/* 圆柱碰撞体姿态：MoveIt CYLINDER 的局部 Z 对齐到 axis_direction */
geometry_msgs::msg::Quaternion buildCylinderCollisionOrientationFromAxis(
    const geometry_msgs::msg::Vector3& axis_direction);

}  // namespace orion_mtc

#endif  // ORION_MTC_STRATEGY_CYLINDER_SIDE_GRASP_HPP

