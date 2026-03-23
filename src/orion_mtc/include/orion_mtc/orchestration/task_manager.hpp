/* 业务编排层：状态检查、任务构建/规划/执行、成功失败后状态回写、scene 与 held_object 对齐 */

#ifndef ORION_MTC_ORCHESTRATION_TASK_MANAGER_HPP
#define ORION_MTC_ORCHESTRATION_TASK_MANAGER_HPP

#include "orion_mtc/config/mtc_config.hpp"
#include "orion_mtc/config/runtime_policy.hpp"
#include "orion_mtc/core/held_object.hpp"
#include "orion_mtc/core/job_result_code.hpp"
#include "orion_mtc/core/manipulation_job.hpp"
#include "orion_mtc/core/runtime_status.hpp"
#include "orion_mtc/core/task_state.hpp"
#include "orion_mtc/orchestration/job_deduplicator.hpp"
#include "orion_mtc/orchestration/manipulation_state_machine.hpp"
#include "orion_mtc/execution/solution_executor.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <atomic>
#include <cstdint>
#include <functional>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <rclcpp/node.hpp>
#include <string>
#include <thread>
#include <vector>

namespace orion_mtc
{
class PlanningSceneManager;
class TrajectoryExecutor;
class PickTaskBuilder;
class TaskQueue;
class RecoveryActions;
}

namespace orion_mtc
{

class TaskManager
{
public:
  TaskManager(const rclcpp::Node::SharedPtr& node,
              const MTCConfig& config,
             PlanningSceneManager* scene_manager,
             TrajectoryExecutor* trajectory_executor,
             SolutionExecutor* solution_executor,
             WaitForGrippedFn wait_for_gripped_fn);

  ~TaskManager();

  bool handlePick(const geometry_msgs::msg::PoseStamped& object_pose, const std::string& object_id);

  bool handleSyncHeldObject(bool set_holding, bool tracked,
                           const std::string& object_id,
                           const geometry_msgs::msg::Pose& object_pose,
                           const geometry_msgs::msg::Pose& tcp_pose,
                           std::string& out_message);

  bool handleResetHeldObject(std::string& out_message);

  void setGripperLockedCallback(std::function<bool()> fn);

  void setGetLatestObjectPoseCallback(
      std::function<std::optional<geometry_msgs::msg::PoseStamped>()> fn);

  void setGetLatestObjectAxisCallback(
      std::function<std::optional<geometry_msgs::msg::Vector3Stamped>()> fn);

  using TransformToBaseLinkFn =
      std::function<bool(geometry_msgs::msg::PoseStamped&, geometry_msgs::msg::Vector3Stamped*)>;
  void setTransformToBaseLinkCallback(TransformToBaseLinkFn fn);

  using JobEventFn = std::function<void(const std::string& job_id, const std::string& job_type,
                                       const std::string& source, uint32_t priority,
                                       const std::string& event_type, bool success,
                                       const std::string& reason, int64_t created_at_ns,
                                       int64_t started_at_ns, int64_t finished_at_ns)>;
  using HeldObjectStateFn = std::function<void(const HeldObjectContext&)>;
  using RecoveryEventFn = std::function<void(const std::string& recovery_type,
                                             const std::string& trigger_reason,
                                             bool success, const std::string& detail)>;
  using StageReportFn = std::function<void(const std::string& job_id, const std::string& task_type,
                                          std::size_t stage_index, const std::string& stage_name,
                                          const std::string& stage_state, const std::string& detail)>;
  void setJobEventCallback(JobEventFn fn);
  void setHeldObjectStateCallback(HeldObjectStateFn fn);
  void setRecoveryEventCallback(RecoveryEventFn fn);
  void setStageReportCallback(StageReportFn fn);

  std::string getNextJobType() const;

  RobotTaskMode getMode() const;
  std::string getTaskId() const;
  std::string getLastError() const;
  HeldObjectContext getHeldObject() const;

  std::string submitJob(const ManipulationJob& job, std::string* out_reject_reason = nullptr);
  void startWorker();
  void stopWorker();
  bool isWorkerRunning() const;
  WorkerStatus getWorkerStatus() const;
  std::string getCurrentJobId() const;
  std::string getCurrentJobType() const;
  std::shared_ptr<TaskQueue> getQueue();
  void setPolicy(const RuntimePolicy& policy);
  const RuntimePolicy& getPolicy() const;

  bool cancelJob(const std::string& job_id, std::string* out_message = nullptr);

  /** 急停：取消当前 FollowJointTrajectory、清空待执行队列、置位供执行循环中止 */
  void requestEmergencyStop();

  /** 规划并执行回 SRDF ready + 张开手；若正在抓取或 worker 执行 job 则拒绝 */
  bool tryGoToReady(std::string& out_message);

  struct JobExecutionRecordEntry
  {
    std::string job_id;
    std::string job_type;
    std::string source;
    JobResultCode result_code = JobResultCode::UNKNOWN;
    std::string message;
    int64_t created_at_ns = 0;
    int64_t started_at_ns = 0;
    int64_t finished_at_ns = 0;
  };
  std::vector<JobExecutionRecordEntry> getRecentRecords(std::size_t max_count) const;

private:
  void workerLoop();
  bool executeJob(const ManipulationJob& job);
  void setState(RobotTaskMode mode);
  void setStateError(const std::string& err);
  std::string genTaskId(const char* prefix);

  void pushExecutionRecordStart(const ManipulationJob& job, int64_t started_at_ns);
  void updateExecutionRecordFinish(JobResultCode code, const std::string& message, int64_t finished_at_ns);
  void pushExecutionRecordCancelled(const ManipulationJob& job, int64_t finished_at_ns);

  bool retreatToReady();

  std::function<bool()> makeEstopAbortFn() const;

  bool handleOpenGripper();
  bool handleCloseGripper();

  static constexpr std::size_t MAX_RECENT_RECORDS = 50;

  rclcpp::Node::SharedPtr node_;
  MTCConfig config_;
  RuntimePolicy policy_;
  PlanningSceneManager* scene_manager_;
  TrajectoryExecutor* trajectory_executor_;
  SolutionExecutor* solution_executor_;
  WaitForGrippedFn wait_for_gripped_fn_;
  std::function<bool()> is_gripper_locked_fn_;
  std::function<std::optional<geometry_msgs::msg::PoseStamped>()> get_latest_object_pose_fn_;
  std::function<std::optional<geometry_msgs::msg::Vector3Stamped>()> get_latest_object_axis_fn_;
  TransformToBaseLinkFn transform_to_base_link_fn_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_reconstructed_object_pose_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_reconstructed_approach_axis_;
  JobEventFn job_event_fn_;
  HeldObjectStateFn held_object_state_fn_;
  RecoveryEventFn recovery_event_fn_;
  StageReportFn stage_report_fn_;
  std::unique_ptr<PickTaskBuilder> pick_builder_;
  std::shared_ptr<TaskQueue> queue_;
  std::unique_ptr<RecoveryActions> recovery_actions_;
  JobDeduplicator job_deduplicator_;
  ManipulationStateMachine manipulation_fsm_;

  mutable std::mutex state_mutex_;
  RobotTaskMode task_mode_ = RobotTaskMode::IDLE;
  HeldObjectContext held_object_;
  std::string current_task_id_;
  std::string last_error_;

  std::atomic<bool> worker_running_{ false };
  std::atomic<bool> estop_requested_{ false };
  std::thread worker_thread_;
  mutable std::mutex worker_mutex_;
  WorkerStatus worker_status_ = WorkerStatus::STOPPED;
  std::string current_job_id_;
  std::string current_job_type_;
  geometry_msgs::msg::Pose current_job_target_pose_;
  bool current_job_has_pose_ = false;

  JobType last_accepted_type_ = JobType::PICK;
  geometry_msgs::msg::Pose last_accepted_pose_;
  bool last_accepted_has_pose_ = false;
  int64_t last_accepted_time_ns_ = 0;

  mutable std::mutex records_mutex_;
  std::deque<JobExecutionRecordEntry> recent_records_;
};

}  // namespace orion_mtc

#endif  // ORION_MTC_ORCHESTRATION_TASK_MANAGER_HPP
