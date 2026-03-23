#include "orion_mtc/planning/cable_side_grasp.hpp"
#include "orion_mtc/planning/cable_segments.hpp"
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <optional>

namespace orion_mtc
{

namespace
{
constexpr double MIN_NORMAL_PROJECTION = 1e-6;
constexpr double M_PI_D = 3.14159265358979323846;
}

/* 在垂直于缆绳轴 d 的平面内构造绕轴方向旋转的侧向法向候选 */
static std::vector<Eigen::Vector3d> makeSideNormals(
    const Eigen::Vector3d& d,
    const std::vector<double>& around_axis_deg)
{
  Eigen::Vector3d axis = d.normalized();
  Eigen::Vector3d base = Eigen::Vector3d::UnitZ().cross(axis);
  if (base.norm() < MIN_NORMAL_PROJECTION)
  {
    base = Eigen::Vector3d::UnitY().cross(axis);
  }
  if (base.norm() < MIN_NORMAL_PROJECTION)
  {
    base = Eigen::Vector3d::UnitX().cross(axis);
  }
  if (base.norm() < MIN_NORMAL_PROJECTION)
  {
    return {};
  }
  base.normalize();

  std::vector<Eigen::Vector3d> normals;
  for (double deg : around_axis_deg)
  {
    double rad = deg * M_PI_D / 180.0;
    Eigen::Quaterniond rot(Eigen::AngleAxisd(rad, axis));
    normals.push_back((rot * base).normalized());
  }
  return normals;
}

/* 抓取系与 URDF gripper_tcp 一致：y=缆绳轴，z=接近方向，x=y×z（右手系） */
static Eigen::Matrix3d makeBaseRotation(const Eigen::Vector3d& axis_d,
                                       const Eigen::Vector3d& approach_n)
{
  Eigen::Vector3d y_axis = axis_d.normalized();
  Eigen::Vector3d z_axis = approach_n.normalized();
  Eigen::Vector3d x_axis = y_axis.cross(z_axis).normalized();

  Eigen::Matrix3d R;
  R.col(0) = x_axis;
  R.col(1) = y_axis;
  R.col(2) = z_axis;
  return R;
}

/* 绕轴 axis 旋转 angle_rad */
static Eigen::Matrix3d rotateAboutAxis(const Eigen::Vector3d& axis, double angle_rad)
{
  Eigen::AngleAxisd aa(angle_rad, axis.normalized());
  return aa.toRotationMatrix();
}

static Eigen::Matrix3d tcpBiasFromRpyDeg(const std::vector<double>& rpy_deg)
{
  if (rpy_deg.size() < 3U)
  {
    return Eigen::Matrix3d::Identity();
  }
  double rx = rpy_deg[0] * M_PI_D / 180.0;
  double ry = rpy_deg[1] * M_PI_D / 180.0;
  double rz = rpy_deg[2] * M_PI_D / 180.0;
  Eigen::Matrix3d R = Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
                      Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY()).toRotationMatrix() *
                      Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX()).toRotationMatrix();
  return R;
}

namespace
{
std::vector<CableGraspCandidate> enumerateCableSideGrasps(const CableDetection& cable,
                                                          const CableGraspConfig& cfg)
{
  std::vector<CableGraspCandidate> out;

  Eigen::Vector3d d = cable.direction.normalized();
  if (d.norm() < 0.9)
  {
    return out;
  }

  const std::vector<double>& pre_offsets =
      !cfg.pregrasp_offset_candidates.empty() ? cfg.pregrasp_offset_candidates : cfg.approach_dist_candidates;
  if (pre_offsets.empty())
  {
    return out;
  }

  Eigen::Matrix3d R_tcp_bias = tcpBiasFromRpyDeg(cfg.tcp_bias_rpy_deg);

  auto normals = makeSideNormals(d, cfg.approach_around_axis_candidates_deg);
  if (normals.empty())
  {
    return out;
  }

  std::vector<CableSegment> segments = buildCableSegments(
      cable.position, d, cfg.cable_total_length, cfg.cable_segment_length, cfg.cable_radius);
  const int num_segments = static_cast<int>(segments.size());

  for (const auto& n : normals)
  {
    Eigen::Matrix3d R0 = makeBaseRotation(d, n);
    Eigen::Matrix3d R_side_bias = R0 * R_tcp_bias;

    for (double shift : cfg.axial_shift_candidates)
    {
      Eigen::Vector3d p_center = cable.position + d * shift;

      for (double roll_deg : cfg.roll_candidates_deg)
      {
        double roll_rad = roll_deg * M_PI_D / 180.0;
        Eigen::Matrix3d R = R_side_bias * rotateAboutAxis(d, roll_rad);

        for (double grasp_depth : cfg.grasp_depth_candidates)
        {
          Eigen::Vector3d p_closure = p_center - n * grasp_depth;

          for (double pre_off : pre_offsets)
          {
            CableGraspCandidate c;
            c.grasp_pose = Eigen::Isometry3d::Identity();
            c.grasp_pose.linear() = R;
            c.grasp_pose.translation() = p_closure;

            c.pregrasp_pose = Eigen::Isometry3d::Identity();
            c.pregrasp_pose.linear() = R;
            c.pregrasp_pose.translation() = p_closure - n * pre_off;

            c.approach_dir = n;
            c.retreat_dir = -n;
            c.approach_dist = pre_off;
            c.retreat_dist = cfg.retreat_dist;

            c.score = std::abs(shift) + 0.1 * grasp_depth + 0.5 * pre_off + (roll_deg == 0.0 ? 0.0 : 0.1);
            if (num_segments > 0)
            {
              c.nearest_segment_index = nearestSegmentIndex(p_closure, segments);
              c.local_segment_indices = localSegmentIndices(
                  c.nearest_segment_index, cfg.grasp_neighbor_segments, num_segments);
            }
            out.push_back(c);
          }
        }
      }
    }
  }

  std::sort(out.begin(), out.end(),
            [](const CableGraspCandidate& a, const CableGraspCandidate& b) {
              return a.score < b.score;
            });
  return out;
}
}  // namespace

std::vector<CableGraspCandidate> generateCableSideGrasps(const CableDetection& cable,
                                                         const CableGraspConfig& cfg)
{
  return enumerateCableSideGrasps(cable, cfg);
}

std::optional<CableGraspCandidate> generateBestCableSideGrasp(const CableDetection& cable,
                                                               const CableGraspConfig& cfg)
{
  std::vector<CableGraspCandidate> v = enumerateCableSideGrasps(cable, cfg);
  if (v.empty())
  {
    return std::nullopt;
  }
  return v.front();
}

void isometryToPose(const Eigen::Isometry3d& T, geometry_msgs::msg::Pose& out)
{
  out.position.x = T.translation().x();
  out.position.y = T.translation().y();
  out.position.z = T.translation().z();
  Eigen::Quaterniond q(T.linear());
  out.orientation.x = q.x();
  out.orientation.y = q.y();
  out.orientation.z = q.z();
  out.orientation.w = q.w();
}

geometry_msgs::msg::PoseStamped toPoseStamped(const Eigen::Isometry3d& T,
                                              const std::string& frame_id,
                                              const rclcpp::Time& stamp)
{
  geometry_msgs::msg::PoseStamped ps;
  ps.header.frame_id = frame_id;
  ps.header.stamp = stamp;
  isometryToPose(T, ps.pose);
  return ps;
}

geometry_msgs::msg::Vector3Stamped toVector3Stamped(const Eigen::Vector3d& v,
                                                     const std::string& frame_id,
                                                     const rclcpp::Time& stamp)
{
  geometry_msgs::msg::Vector3Stamped vs;
  vs.header.frame_id = frame_id;
  vs.header.stamp = stamp;
  vs.vector.x = v.x();
  vs.vector.y = v.y();
  vs.vector.z = v.z();
  return vs;
}

}  // namespace orion_mtc
