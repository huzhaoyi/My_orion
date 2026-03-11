#include "orion_mtc/perception/grasp_generator.hpp"
#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace orion_mtc
{

namespace
{
constexpr double DEG_TO_RAD = 3.14159265358979323846 / 180.0;

void rotationMatrixToQuaternion(const double R[9], double& qx, double& qy, double& qz, double& qw)
{
  double trace = R[0] + R[4] + R[8];
  if (trace > 0.0)
  {
    double s = 0.5 / std::sqrt(trace + 1.0);
    qw = 0.25 / s;
    qx = (R[7] - R[5]) * s;
    qy = (R[2] - R[6]) * s;
    qz = (R[3] - R[1]) * s;
  }
  else if (R[0] > R[4] && R[0] > R[8])
  {
    double s = 2.0 * std::sqrt(1.0 + R[0] - R[4] - R[8]);
    qw = (R[7] - R[5]) / s;
    qx = 0.25 * s;
    qy = (R[1] + R[3]) / s;
    qz = (R[2] + R[6]) / s;
  }
  else if (R[4] > R[8])
  {
    double s = 2.0 * std::sqrt(1.0 + R[4] - R[0] - R[8]);
    qw = (R[2] - R[6]) / s;
    qx = (R[1] + R[3]) / s;
    qy = 0.25 * s;
    qz = (R[5] + R[7]) / s;
  }
  else
  {
    double s = 2.0 * std::sqrt(1.0 + R[8] - R[0] - R[4]);
    qw = (R[3] - R[1]) / s;
    qx = (R[2] + R[6]) / s;
    qy = (R[5] + R[7]) / s;
    qz = 0.25 * s;
  }
}
}  // namespace

GraspGenerator::GraspGenerator(const GraspGeneratorParams& params) : params_(params)
{
}

std::vector<GraspCandidate> GraspGenerator::generate(const TargetObject& target) const
{
  std::vector<GraspCandidate> out;
  const double zx = target.axis_direction.x;
  const double zy = target.axis_direction.y;
  const double zz = target.axis_direction.z;
  double nx = 0.0, ny = 0.0, nz = 1.0;
  double dot = nx * zx + ny * zy + nz * zz;
  if (std::fabs(dot) > 0.99)
  {
    nx = 1.0;
    ny = 0.0;
    nz = 0.0;
    dot = zx;
  }
  double xx = ny * zz - nz * zy;
  double xy = nz * zx - nx * zz;
  double xz = nx * zy - ny * zx;
  double xn = std::sqrt(xx * xx + xy * xy + xz * xz);
  if (xn < 1e-12)
  {
    xx = 1.0;
    xy = 0.0;
    xz = 0.0;
  }
  else
  {
    xx /= xn;
    xy /= xn;
    xz /= xn;
  }
  double yx = zy * xz - zz * xy;
  double yy = zz * xx - zx * xz;
  double yz = zx * xy - zy * xx;

  for (double deg : params_.yaw_degrees)
  {
    double rad = deg * DEG_TO_RAD;
    double c = std::cos(rad);
    double s = std::sin(rad);
    double rx = xx * c + yx * s;
    double ry = xy * c + yy * s;
    double rz = xz * c + yz * s;
    double ux = xx * (-s) + yx * c;
    double uy = xy * (-s) + yy * c;
    double uz = xz * (-s) + yz * c;
    double R[9] = { rx, ux, zx, ry, uy, zy, rz, uz, zz };
    double qx, qy, qz, qw;
    rotationMatrixToQuaternion(R, qx, qy, qz, qw);
    GraspCandidate candidate;
    candidate.header.frame_id = params_.frame_id;
    candidate.header.stamp.sec = 0;
    candidate.header.stamp.nanosec = 0;
    candidate.pose.position.x = target.position.x;
    candidate.pose.position.y = target.position.y;
    candidate.pose.position.z = target.position.z;
    candidate.pose.orientation.x = qx;
    candidate.pose.orientation.y = qy;
    candidate.pose.orientation.z = qz;
    candidate.pose.orientation.w = qw;
    out.push_back(candidate);
  }
  return out;
}

void GraspGenerator::setParams(const GraspGeneratorParams& params)
{
  params_ = params;
}

}  // namespace orion_mtc
