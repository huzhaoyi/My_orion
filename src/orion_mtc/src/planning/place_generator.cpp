#include "orion_mtc/planning/place_generator.hpp"
#include <Eigen/Geometry>
#include <cmath>

namespace orion_mtc
{

namespace
{
constexpr double DEG_TO_RAD = 3.14159265358979323846 / 180.0;

void eigenIsometryToPose(const Eigen::Isometry3d& T, geometry_msgs::msg::Pose& out)
{
  out.position.x = T.translation().x();
  out.position.y = T.translation().y();
  out.position.z = T.translation().z();
  Eigen::Quaterniond q(T.rotation());
  out.orientation.x = q.x();
  out.orientation.y = q.y();
  out.orientation.z = q.z();
  out.orientation.w = q.w();
}

Eigen::Quaterniond yawRotationZ(double degrees)
{
  return Eigen::Quaterniond(Eigen::AngleAxisd(degrees * DEG_TO_RAD, Eigen::Vector3d::UnitZ()));
}
}  // namespace

PlaceGenerator::PlaceGenerator(const PlaceGeneratorParams& params) : params_(params)
{
}

std::vector<PlaceCandidate> PlaceGenerator::generate(
    const geometry_msgs::msg::PoseStamped& target_pose,
    const HeldObjectContext& held) const
{
  std::vector<PlaceCandidate> out;
  if (!held.valid)
  {
    return out;
  }
  const double cx = target_pose.pose.position.x;
  const double cy = target_pose.pose.position.y;
  const double cz = target_pose.pose.position.z;
  Eigen::Quaterniond q_obj(target_pose.pose.orientation.w,
                           target_pose.pose.orientation.x,
                           target_pose.pose.orientation.y,
                           target_pose.pose.orientation.z);
  const double step = params_.position_step_xy;

  std::vector<std::pair<double, double>> xy_offsets;
  xy_offsets.push_back({ 0.0, 0.0 });
  if (params_.enable_position_samples)
  {
    xy_offsets.push_back({ step, 0.0 });
    xy_offsets.push_back({ -step, 0.0 });
    xy_offsets.push_back({ 0.0, step });
    xy_offsets.push_back({ 0.0, -step });
  }

  for (const auto& off : xy_offsets)
  {
    for (double yaw_deg : params_.yaw_degrees)
    {
      Eigen::Quaterniond q = q_obj * yawRotationZ(yaw_deg);
      Eigen::Isometry3d T_base_obj = Eigen::Isometry3d::Identity();
      T_base_obj.translate(Eigen::Vector3d(cx + off.first, cy + off.second, cz));
      T_base_obj.rotate(q);
      Eigen::Isometry3d T_base_tcp = T_base_obj * held.tcp_to_object.inverse();

      PlaceCandidate c;
      c.object_pose.header = target_pose.header;
      c.object_pose.header.frame_id = params_.frame_id.empty() ? target_pose.header.frame_id :
                                                                 params_.frame_id;
      c.object_pose.pose.position.x = T_base_obj.translation().x();
      c.object_pose.pose.position.y = T_base_obj.translation().y();
      c.object_pose.pose.position.z = T_base_obj.translation().z();
      Eigen::Quaterniond qo(T_base_obj.rotation());
      c.object_pose.pose.orientation.x = qo.x();
      c.object_pose.pose.orientation.y = qo.y();
      c.object_pose.pose.orientation.z = qo.z();
      c.object_pose.pose.orientation.w = qo.w();
      eigenIsometryToPose(T_base_tcp, c.tcp_pose);
      c.score = 0.0;
      if (std::abs(off.first) < 1e-9 && std::abs(off.second) < 1e-9 && std::abs(yaw_deg) < 1e-9)
      {
        c.source = "center";
      }
      else
      {
        c.source = "offset_xy_yaw";
      }
      out.push_back(c);
    }
  }
  return out;
}

void PlaceGenerator::setParams(const PlaceGeneratorParams& params)
{
  params_ = params;
}

}  // namespace orion_mtc
