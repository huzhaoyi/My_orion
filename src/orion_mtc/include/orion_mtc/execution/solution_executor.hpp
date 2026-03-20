/* 执行层：Solution -> 分段 apply scene、执行轨迹、等待 gripped/unlock；pick 时提取 held context */

#ifndef ORION_MTC_EXECUTION_SOLUTION_EXECUTOR_HPP
#define ORION_MTC_EXECUTION_SOLUTION_EXECUTOR_HPP

#include "orion_mtc/core/held_object.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_task_constructor_msgs/msg/solution.hpp>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace moveit
{
namespace core
{
class RobotModel;
typedef std::shared_ptr<const RobotModel> RobotModelConstPtr;
}
}

namespace orion_mtc
{
class PlanningSceneManager;
class TrajectoryExecutor;
}

namespace orion_mtc
{

/* expect_gripped: true=等待夹紧, false=等待松开；timeout_sec；返回是否在超时内满足 */
using WaitForGrippedFn = std::function<bool(bool expect_gripped, double timeout_sec)>;

/* 阶段上报：(job_id, task_type, stage_index, stage_name, stage_state, detail)，供 /manipulator/task_stage */
using StageReportFn = std::function<void(const std::string& job_id, const std::string& task_type,
                                        std::size_t stage_index, const std::string& stage_name,
                                        const std::string& stage_state, const std::string& detail)>;

class SolutionExecutor
{
public:
  SolutionExecutor(PlanningSceneManager* scene_manager, TrajectoryExecutor* trajectory_executor);
  ~SolutionExecutor() = default;

  /* 通用 solution 执行：按段 apply scene、发轨迹，手部段后根据闭合/张开调用 wait_for_gripped。
   * 可选 stage_report：每段执行前后回调，stage_names 长度可小于 segment 数，不足用 segment_N */
  bool executeSolution(const moveit_task_constructor_msgs::msg::Solution& solution_msg,
                      WaitForGrippedFn wait_for_gripped,
                      StageReportFn stage_report = nullptr,
                      const std::string& job_id = "",
                      const std::string& task_type = "",
                      const std::vector<std::string>& stage_names = {});

  /* Pick 专用：执行中在 attach 段后根据末端 FK 填充 held_context_out。可选 stage_report 同上。
   * cable_world_object_ids：与 pick 任务中 add 的 world 缆绳段 id 一致，用于 remove 后同步 scene；空则不再额外扫 id */
  bool executePickSolution(const moveit_task_constructor_msgs::msg::Solution& solution_msg,
                          const geometry_msgs::msg::Pose& object_pose_at_grasp,
                          const std::string& object_id,
                          const moveit::core::RobotModelConstPtr& robot_model,
                          HeldObjectContext& held_context_out,
                          WaitForGrippedFn wait_for_gripped,
                          StageReportFn stage_report = nullptr,
                          const std::string& job_id = "",
                          const std::string& task_type = "",
                          const std::vector<std::string>& stage_names = {},
                          const std::vector<std::string>& cable_world_object_ids = {});

private:
  PlanningSceneManager* scene_manager_;
  TrajectoryExecutor* trajectory_executor_;
};

}  // namespace orion_mtc

#endif  // ORION_MTC_EXECUTION_SOLUTION_EXECUTOR_HPP
