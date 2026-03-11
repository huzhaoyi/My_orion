/* 执行层：Solution -> 分段 apply scene、执行轨迹、等待 gripped/unlock；pick 时提取 held context */

#ifndef ORION_MTC_EXECUTION_SOLUTION_EXECUTOR_HPP
#define ORION_MTC_EXECUTION_SOLUTION_EXECUTOR_HPP

#include "orion_mtc/core/held_object.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_task_constructor_msgs/msg/solution.hpp>
#include <functional>
#include <memory>

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

class SolutionExecutor
{
public:
  SolutionExecutor(PlanningSceneManager* scene_manager, TrajectoryExecutor* trajectory_executor);
  ~SolutionExecutor() = default;

  /* 通用 solution 执行：按段 apply scene、发轨迹，手部段后根据闭合/张开调用 wait_for_gripped */
  bool executeSolution(const moveit_task_constructor_msgs::msg::Solution& solution_msg,
                      WaitForGrippedFn wait_for_gripped);

  /* Pick 专用：执行中在 attach 段后根据末端 FK 填充 held_context_out */
  bool executePickSolution(const moveit_task_constructor_msgs::msg::Solution& solution_msg,
                          const geometry_msgs::msg::Pose& object_pose_at_grasp,
                          const std::string& object_id,
                          const moveit::core::RobotModelConstPtr& robot_model,
                          HeldObjectContext& held_context_out,
                          WaitForGrippedFn wait_for_gripped);

private:
  PlanningSceneManager* scene_manager_;
  TrajectoryExecutor* trajectory_executor_;
};

}  // namespace orion_mtc

#endif  // ORION_MTC_EXECUTION_SOLUTION_EXECUTOR_HPP
