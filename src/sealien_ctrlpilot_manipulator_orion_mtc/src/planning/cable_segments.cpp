/* cable_segments：沿轴线切片生成 CableSegment 列表供碰撞与预检 */

#include "sealien_ctrlpilot_manipulator_orion_mtc/planning/cable_segments.hpp"
#include <cmath>
#include <sstream>

namespace sealien_ctrlpilot_manipulator_orion_mtc
{

/*
 * 沿归一化 axis 在 [-L/2,L/2] 内均分切片，每段长度 segment_length（末段可能略长由 ceil 覆盖总长）；
 * 各段 id 为 cable_seg_k，中心在轴上、axis 拷贝为 d。
 */
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

/*
 * 在 segments 中选与 p_grasp 欧氏距离最小的段下标；空列表返回 -1。
 */
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

/*
 * 以 nearest_index 为中心，向两侧各扩展 grasp_neighbor_segments 格，裁剪到 [0,num_segments-1]。
 * 用于预检/ACM 仅放宽邻近缆段与手爪碰撞。
 */
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

}  // namespace sealien_ctrlpilot_manipulator_orion_mtc
