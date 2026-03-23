/* 任务入队去重：与 Worker 当前 job、短时间窗口内的同目标拒绝 */

#ifndef ORION_MTC_ORCHESTRATION_JOB_DEDUPLICATOR_HPP
#define ORION_MTC_ORCHESTRATION_JOB_DEDUPLICATOR_HPP

#include "orion_mtc/core/manipulation_job.hpp"
#include "orion_mtc/core/runtime_status.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <cstdint>
#include <string>

namespace orion_mtc
{

class JobDeduplicator
{
public:
    static bool getJobPoseForDedup(const ManipulationJob& job, geometry_msgs::msg::Pose* out);

    static bool posesNear(const geometry_msgs::msg::Pose& a, const geometry_msgs::msg::Pose& b,
                          double pos_tol_m, double quat_dot_min);

    bool isDuplicate(const ManipulationJob& job,
                     int64_t now_ns,
                     WorkerStatus worker_status,
                     const std::string& current_job_type,
                     bool current_job_has_pose,
                     const geometry_msgs::msg::Pose& current_job_target_pose,
                     JobType last_accepted_type,
                     bool last_accepted_has_pose,
                     const geometry_msgs::msg::Pose& last_accepted_pose,
                     int64_t last_accepted_time_ns,
                     std::string* out_reason) const;

    static constexpr double DEDUP_POS_TOL_M = 0.01;
    static constexpr double DEDUP_QUAT_DOT_MIN = 0.99;
    static constexpr int64_t DEDUP_TIME_WINDOW_NS = 1000000000;
};

}  // namespace orion_mtc

#endif  // ORION_MTC_ORCHESTRATION_JOB_DEDUPLICATOR_HPP
