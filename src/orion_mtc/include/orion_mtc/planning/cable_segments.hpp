/* 缆绳分段碰撞代理：3m 缆绳离散为多段短圆柱，局部放宽、远端严格避碰 */

#ifndef ORION_MTC_PLANNING_CABLE_SEGMENTS_HPP
#define ORION_MTC_PLANNING_CABLE_SEGMENTS_HPP

#include <Eigen/Geometry>
#include <string>
#include <vector>

namespace orion_mtc
{

/** 单段缆绳：短圆柱，局部 Z 对齐缆绳轴线 */
struct CableSegment
{
  std::string id;
  Eigen::Vector3d center;
  Eigen::Vector3d axis;  // 单位向量，圆柱局部 Z
  double length = 0.25;
  double radius = 0.025;
};

/**
 * 将整根缆绳离散为多段短圆柱。
 * center/axis 为缆绳中点与轴线（单位向量），total_length 为总长，segment_length 为每段长。
 * 段中心沿轴分布：offset_k = -L/2 + seg_len/2 + k*seg_len，p_k = center + axis*offset_k
 */
std::vector<CableSegment> buildCableSegments(const Eigen::Vector3d& center,
                                             const Eigen::Vector3d& axis,
                                             double total_length,
                                             double segment_length,
                                             double radius);

/** 抓点 p_grasp 到各段中心距离，返回最近段索引 */
int nearestSegmentIndex(const Eigen::Vector3d& p_grasp,
                        const std::vector<CableSegment>& segments);

/** 抓取局部邻域段索引：[nearest - neighbor, ..., nearest + neighbor]，做边界裁剪 */
std::vector<int> localSegmentIndices(int nearest_index,
                                      int grasp_neighbor_segments,
                                      int num_segments);

}  // namespace orion_mtc

#endif  // ORION_MTC_PLANNING_CABLE_SEGMENTS_HPP
