/* 多目标感知模型：单目标抽象，供 TargetCache / TargetSelector / GraspGenerator 使用 */

#ifndef ORION_MTC_CORE_TARGET_OBJECT_HPP
#define ORION_MTC_CORE_TARGET_OBJECT_HPP

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <string>

namespace orion_mtc
{

struct TargetObject
{
  /** 在当帧 targets 中的下标，-1 表示无效 */
  int index = -1;

  /** 物体中心或参考点（与 TargetSet 同一坐标系，通常 base_link） */
  geometry_msgs::msg::Point position;

  /** 圆柱轴方向，应由源端或解析时归一化 */
  geometry_msgs::msg::Vector3 axis_direction;

  /** 物体类型标识，如 "target"，便于后续扩展 */
  std::string object_type = "target";

  /** 可选：若上游有跟踪，可填稳定 id，便于跳过已抓、去重 */
  int stable_id = -1;

  /** 可选：置信度 [0,1]，默认 1.0 */
  double confidence = 1.0;

  /** 可选：时间戳（纳秒 epoch），用于陈旧过滤、去重窗口 */
  int64_t stamp_ns = 0;
};

}  // namespace orion_mtc

#endif  // ORION_MTC_CORE_TARGET_OBJECT_HPP
