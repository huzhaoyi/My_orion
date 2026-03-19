#include "orion_mtc/planning/cable_segments.hpp"
#include <cmath>
#include <sstream>

namespace orion_mtc
{

std::vector<CableSegment> buildCableSegments(const Eigen::Vector3d& center,
                                             const Eigen::Vector3d& axis,
                                             double total_length,
                                             double segment_length,
                                             double radius)
{
  std::vector<CableSegment> out;
  if (segment_length <= 0.0 || total_length <= 0.0)
  {
    return out;
  }
  Eigen::Vector3d d = axis.normalized();
  const int n = static_cast<int>(std::ceil(total_length / segment_length));
  for (int k = 0; k < n; ++k)
  {
    double offset_k = -total_length / 2.0 + segment_length / 2.0 + static_cast<double>(k) * segment_length;
    CableSegment seg;
    std::ostringstream oss;
    oss << "cable_seg_" << k;
    seg.id = oss.str();
    seg.center = center + d * offset_k;
    seg.axis = d;
    seg.length = segment_length;
    seg.radius = radius;
    out.push_back(seg);
  }
  return out;
}

int nearestSegmentIndex(const Eigen::Vector3d& p_grasp,
                        const std::vector<CableSegment>& segments)
{
  if (segments.empty())
  {
    return -1;
  }
  int best = 0;
  double best_sq = (segments[0].center - p_grasp).squaredNorm();
  for (size_t i = 1; i < segments.size(); ++i)
  {
    double sq = (segments[i].center - p_grasp).squaredNorm();
    if (sq < best_sq)
    {
      best_sq = sq;
      best = static_cast<int>(i);
    }
  }
  return best;
}

std::vector<int> localSegmentIndices(int nearest_index,
                                     int grasp_neighbor_segments,
                                     int num_segments)
{
  std::vector<int> out;
  if (num_segments <= 0 || nearest_index < 0)
  {
    return out;
  }
  int lo = nearest_index - grasp_neighbor_segments;
  int hi = nearest_index + grasp_neighbor_segments;
  if (lo < 0)
  {
    lo = 0;
  }
  if (hi >= num_segments)
  {
    hi = num_segments - 1;
  }
  for (int i = lo; i <= hi; ++i)
  {
    out.push_back(i);
  }
  return out;
}

}  // namespace orion_mtc
