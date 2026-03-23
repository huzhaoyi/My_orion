#include "orion_mtc/orchestration/job_deduplicator.hpp"
#include <cmath>

namespace orion_mtc
{

bool JobDeduplicator::getJobPoseForDedup(const ManipulationJob& job, geometry_msgs::msg::Pose* out)
{
    if (out == nullptr)
    {
        return false;
    }
    switch (job.type)
    {
        case JobType::PICK:
            if (job.object_pose.has_value())
            {
                *out = job.object_pose->pose;
                return true;
            }
            return false;
        case JobType::SYNC_HELD_OBJECT:
            if (job.object_pose.has_value())
            {
                *out = job.object_pose->pose;
                return true;
            }
            return false;
        case JobType::RESET_HELD_OBJECT:
        case JobType::OPEN_GRIPPER:
        case JobType::CLOSE_GRIPPER:
            return false;
        default:
            return false;
    }
}

bool JobDeduplicator::posesNear(const geometry_msgs::msg::Pose& a, const geometry_msgs::msg::Pose& b,
                                double pos_tol_m, double quat_dot_min)
{
    double dx = a.position.x - b.position.x;
    double dy = a.position.y - b.position.y;
    double dz = a.position.z - b.position.z;
    double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (dist > pos_tol_m)
    {
        return false;
    }
    double dot = a.orientation.w * b.orientation.w + a.orientation.x * b.orientation.x +
                 a.orientation.y * b.orientation.y + a.orientation.z * b.orientation.z;
    if (dot < 0.0)
    {
        dot = -dot;
    }
    return dot >= quat_dot_min;
}

bool JobDeduplicator::isDuplicate(const ManipulationJob& job,
                                  int64_t now_ns,
                                  WorkerStatus worker_status,
                                  const std::string& current_job_type,
                                  bool current_job_has_pose,
                                  const geometry_msgs::msg::Pose& current_job_target_pose,
                                  JobType last_accepted_type,
                                  bool last_accepted_has_pose,
                                  const geometry_msgs::msg::Pose& last_accepted_pose,
                                  int64_t last_accepted_time_ns,
                                  std::string* out_reason) const
{
    if (out_reason != nullptr)
    {
        out_reason->clear();
    }
    const char* type_str = jobTypeToCString(job.type);
    geometry_msgs::msg::Pose job_pose;
    bool job_has_pose = getJobPoseForDedup(job, &job_pose);

    if (worker_status == WorkerStatus::RUNNING_JOB && current_job_type == type_str)
    {
        if (job_has_pose && current_job_has_pose &&
            posesNear(current_job_target_pose, job_pose, DEDUP_POS_TOL_M, DEDUP_QUAT_DOT_MIN))
        {
            if (out_reason != nullptr)
            {
                *out_reason = "duplicate of running job (same type and target)";
            }
            return true;
        }
        if (!job_has_pose && !current_job_has_pose)
        {
            if (out_reason != nullptr)
            {
                *out_reason = "duplicate of running job (same type, no target)";
            }
            return true;
        }
    }

    if ((now_ns - last_accepted_time_ns) < DEDUP_TIME_WINDOW_NS && last_accepted_type == job.type)
    {
        if (job_has_pose && last_accepted_has_pose &&
            posesNear(last_accepted_pose, job_pose, DEDUP_POS_TOL_M, DEDUP_QUAT_DOT_MIN))
        {
            if (out_reason != nullptr)
            {
                *out_reason = "duplicate within time window (same type and target)";
            }
            return true;
        }
        if (!job_has_pose && !last_accepted_has_pose)
        {
            if (out_reason != nullptr)
            {
                *out_reason = "duplicate within time window (same type, no target)";
            }
            return true;
        }
    }

    return false;
}

}  // namespace orion_mtc
