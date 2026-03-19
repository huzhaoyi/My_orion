#include "orion_mtc/planning/cable_side_grasp.hpp"
#include "orion_mtc/planning/cable_segments.hpp"
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

namespace orion_mtc
{

namespace
{
constexpr double MIN_NORMAL_PROJECTION = 1e-6;
constexpr double M_PI_D = 3.14159265358979323846;
}

/* 在垂直于缆绳轴 d 的平面内构造侧向法向候选：优先 world Z/Y/X 投影，得到 ±n */
static std::vector<Eigen::Vector3d> makeSideNormals(const Eigen::Vector3d& d)
{
  std::vector<Eigen::Vector3d> refs = {
      Eigen::Vector3d::UnitZ(),
      Eigen::Vector3d::UnitY(),
      Eigen::Vector3d::UnitX(),
  };

  for (const auto& ref : refs)
  {
    Eigen::Vector3d proj = ref - ref.dot(d) * d;
    if (proj.norm() > MIN_NORMAL_PROJECTION)
    {
      Eigen::Vector3d n = proj.normalized();
      return { n, -n };
    }
  }
  return {};
}

/* 抓取系：x=缆绳轴，z=接近方向，y=z×x */
static Eigen::Matrix3d makeBaseRotation(const Eigen::Vector3d& axis_d,
                                       const Eigen::Vector3d& approach_n)
{
  Eigen::Vector3d x = axis_d.normalized();
  Eigen::Vector3d z = approach_n.normalized();
  Eigen::Vector3d y = z.cross(x).normalized();
  x = y.cross(z).normalized();

  Eigen::Matrix3d R;
  R.col(0) = x;
  R.col(1) = y;
  R.col(2) = z;
  return R;
}

/* 绕轴 axis 旋转 angle_rad */
static Eigen::Matrix3d rotateAboutAxis(const Eigen::Vector3d& axis, double angle_rad)
{
  Eigen::AngleAxisd aa(angle_rad, axis.normalized());
  return aa.toRotationMatrix();
}

std::vector<CableGraspCandidate> generateCableSideGrasps(const CableDetection& cable,
                                                          const CableGraspConfig& cfg)
{
  std::vector<CableGraspCandidate> out;

  Eigen::Vector3d d = cable.direction.normalized();
  if (d.norm() < 0.9)
  {
    return out;
  }

  auto normals = makeSideNormals(d);
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

    for (double shift : cfg.axial_shift_candidates)
    {
      Eigen::Vector3d p_shift = cable.position + d * shift;

      for (double roll_deg : cfg.roll_candidates_deg)
      {
        double roll_rad = roll_deg * M_PI_D / 180.0;
        Eigen::Matrix3d R = R0 * rotateAboutAxis(d, roll_rad);

        for (double ad : cfg.approach_dist_candidates)
        {
          CableGraspCandidate c;
          c.grasp_pose = Eigen::Isometry3d::Identity();
          c.grasp_pose.linear() = R;
          c.grasp_pose.translation() = p_shift;

          c.pregrasp_pose = Eigen::Isometry3d::Identity();
          c.pregrasp_pose.linear() = R;
          c.pregrasp_pose.translation() = p_shift - n * ad;

          c.approach_dir = n;
          c.retreat_dir = -n;
          c.approach_dist = ad;
          c.retreat_dist = cfg.retreat_dist;

          c.score = std::abs(shift) + 0.5 * ad + (roll_deg == 0.0 ? 0.0 : 0.1);
          if (num_segments > 0)
          {
            c.nearest_segment_index = nearestSegmentIndex(p_shift, segments);
            c.local_segment_indices = localSegmentIndices(
                c.nearest_segment_index, cfg.grasp_neighbor_segments, num_segments);
          }
          out.push_back(c);
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
