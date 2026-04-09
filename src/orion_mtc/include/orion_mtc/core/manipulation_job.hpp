/* 操纵任务模型：排队执行的最小单元，与“即时 action”语义分离 */

#ifndef ORION_MTC_CORE_MANIPULATION_JOB_HPP
#define ORION_MTC_CORE_MANIPULATION_JOB_HPP

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <cstdint>
#include <optional>
#include <string>

namespace orion_mtc
{

/* PICK 感知链：legacy=原桥接；fused=中心线；target_sensor=TargetSensor 目标抓取链。 */
enum class GraspSource : uint8_t
{
  LEGACY = 0,
  FUSED = 1,
  TARGET_SENSOR = 2,
};

enum class JobType
{
  PICK,
  RESET_HELD_OBJECT,
  SYNC_HELD_OBJECT,
  OPEN_GRIPPER,
  CLOSE_GRIPPER,
  TARGET_INSERT
};

struct ManipulationJob
{
  std::string job_id;
  JobType type = JobType::PICK;

  /* PICK 用 object_pose；SYNC 用 object_pose + tcp_pose */
  std::optional<geometry_msgs::msg::PoseStamped> target_pose;
  std::optional<geometry_msgs::msg::PoseStamped> object_pose;
  std::optional<geometry_msgs::msg::Pose> tcp_pose;

  std::string object_id;
  bool tracked = false;

  /** PICK 选用哪条感知链（executeJob / handlePick 从对应缓存取 pose+axis）。 */
  GraspSource grasp_source = GraspSource::LEGACY;

  /*
   * 优先级：数值越大越先执行。
   * priority < 0 表示未指定，入队时用 getDefaultPriority(type) 填充；
   * 0 为有效值。恢复类 job 可显式覆盖默认，普通业务 job 不建议超过恢复类默认值（如 50）。
   */
  int priority = 0;

  /** 任务来源，便于日志与调试：topic_pick_trigger, submit_job_service, pick_action 等 */
  std::string source;

  /** 创建时间（纳秒，epoch），用于去重窗口、排队延迟、超时丢弃 */
  int64_t created_at_ns = 0;
};

inline const char* graspSourceToCString(GraspSource g)
{
  switch (g)
  {
    case GraspSource::FUSED:
      return "FUSED";
    case GraspSource::TARGET_SENSOR:
      return "TARGET_SENSOR";
    case GraspSource::LEGACY:
    default:
      return "LEGACY";
  }
}

inline const char* jobTypeToCString(JobType t)
{
  switch (t)
  {
    case JobType::PICK:
      return "PICK";
    case JobType::RESET_HELD_OBJECT:
      return "RESET_HELD_OBJECT";
    case JobType::SYNC_HELD_OBJECT:
      return "SYNC_HELD_OBJECT";
    case JobType::OPEN_GRIPPER:
      return "OPEN_GRIPPER";
    case JobType::CLOSE_GRIPPER:
      return "CLOSE_GRIPPER";
    case JobType::TARGET_INSERT:
      return "TARGET_INSERT";
    default:
      return "NONE";
  }
}

/** 推荐默认优先级：数值越大越优先；恢复类优先于普通抓取。 */
inline int getDefaultPriority(JobType t)
{
  switch (t)
  {
    case JobType::RESET_HELD_OBJECT:
      return 100;
    case JobType::SYNC_HELD_OBJECT:
      return 80;
    case JobType::PICK:
    case JobType::OPEN_GRIPPER:
    case JobType::CLOSE_GRIPPER:
    case JobType::TARGET_INSERT:
    default:
      return 0;
  }
}

}  // namespace orion_mtc

#endif  // ORION_MTC_CORE_MANIPULATION_JOB_HPP
