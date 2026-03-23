#include "orion_mtc/decision/cylinder_side_grasp.hpp"
#include <Eigen/Geometry>
#include <cmath>

namespace orion_mtc
{

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

