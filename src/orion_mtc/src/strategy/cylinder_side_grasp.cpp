#include "orion_mtc/strategy/cylinder_side_grasp.hpp"
#include <Eigen/Geometry>
#include <cmath>

namespace orion_mtc
{

std::optional<CylinderSideGraspSemantic> buildTopDownCylinderGrasp(
    const geometry_msgs::msg::Vector3& axis_direction,
    const geometry_msgs::msg::Vector3& up_dir)
{
  Eigen::Vector3d up(up_dir.x, up_dir.y, up_dir.z);
  const double un = up.norm();
  if (un < 1e-9)
  {
    up = Eigen::Vector3d(0.0, 0.0, 1.0);
  }
  else
  {
    up = up / un;
  }

  Eigen::Vector3d z = up;  // grasp Z: pregrasp -> grasp 的反方向（MoveRelative 会沿 -Z 下压）
  if (z.z() < 0.0)
  {
    z = -z;
  }

  Eigen::Vector3d x(axis_direction.x, axis_direction.y, axis_direction.z);
  const double xn = x.norm();
  if (xn < 1e-9)
  {
    x = Eigen::Vector3d(1.0, 0.0, 0.0);
  }
  else
  {
    x = x / xn;
  }

  /* 将圆柱轴投影到垂直于下压轴的平面，避免 x 与 z 平行导致退化 */
  x = x - x.dot(z) * z;
  const double xpn = x.norm();
  if (xpn < 1e-6)
  {
    const Eigen::Vector3d fallback(1.0, 0.0, 0.0);
    x = fallback - fallback.dot(z) * z;
  }
  const double xn2 = x.norm();
  if (xn2 < 1e-9)
  {
    return std::nullopt;
  }
  x = x / xn2;

  Eigen::Vector3d y = z.cross(x);
  const double yn = y.norm();
  if (yn < 1e-9)
  {
    return std::nullopt;
  }
  y = y / yn;
  x = y.cross(z);
  const double xn3 = x.norm();
  if (xn3 < 1e-9)
  {
    return std::nullopt;
  }
  x = x / xn3;

  Eigen::Matrix3d R;
  R.col(0) = x;
  R.col(1) = y;
  R.col(2) = z;
  Eigen::Quaterniond q(R);
  q.normalize();

  CylinderSideGraspSemantic out;
  out.grasp_orientation.x = q.x();
  out.grasp_orientation.y = q.y();
  out.grasp_orientation.z = q.z();
  out.grasp_orientation.w = q.w();
  out.approach_axis.x = z.x();
  out.approach_axis.y = z.y();
  out.approach_axis.z = z.z();
  return out;
}

geometry_msgs::msg::Quaternion buildCylinderCollisionOrientationFromAxis(
    const geometry_msgs::msg::Vector3& axis_direction)
{
  Eigen::Vector3d z(axis_direction.x, axis_direction.y, axis_direction.z);
  const double zn = z.norm();
  if (zn < 1e-9)
  {
    geometry_msgs::msg::Quaternion q;
    q.w = 1.0;
    q.x = 0.0;
    q.y = 0.0;
    q.z = 0.0;
    return q;
  }
  z = z / zn;

  Eigen::Vector3d ref(0.0, 0.0, 1.0);
  if (std::fabs(z.dot(ref)) > 0.95)
  {
    ref = Eigen::Vector3d(1.0, 0.0, 0.0);
  }
  Eigen::Vector3d x = ref.cross(z);
  const double xn = x.norm();
  if (xn < 1e-9)
  {
    x = Eigen::Vector3d(1.0, 0.0, 0.0);
  }
  else
  {
    x = x / xn;
  }
  Eigen::Vector3d y = z.cross(x);
  const double yn = y.norm();
  if (yn > 1e-9)
  {
    y = y / yn;
  }

  Eigen::Matrix3d R;
  R.col(0) = x;
  R.col(1) = y;
  R.col(2) = z;
  Eigen::Quaterniond q(R);
  q.normalize();

  geometry_msgs::msg::Quaternion out;
  out.w = q.w();
  out.x = q.x();
  out.y = q.y();
  out.z = q.z();
  return out;
}

}  // namespace orion_mtc

