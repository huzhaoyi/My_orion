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
#include "orion_mtc/execution/solution_executor.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <atomic>
#include <cstdint>
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
class PlaceTaskBuilder;
class PlaceReleaseTaskBuilder;
class TaskQueue;
class RecoveryActions;
class PlaceGenerator;
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

  bool handlePlace(const geometry_msgs::msg::PoseStamped& target_pose);
  void setPlaceGenerator(PlaceGenerator* place_generator);
  bool handlePlaceRelease(const geometry_msgs::msg::PoseStamped& target_tcp_pose);

  /** sync_held_object 逻辑：tracked 时填 object_pose+tcp_pose，untracked 时仅 set_holding */
  bool handleSyncHeldObject(bool set_holding, bool tracked,
                           const std::string& object_id,
                           const geometry_msgs::msg::Pose& object_pose,
                           const geometry_msgs::msg::Pose& tcp_pose,
                           std::string& out_message);

  /** reset_held_object：清空持物状态并清理 scene attach */
  bool handleResetHeldObject(std::string& out_message);

  /** 设置夹爪“有物”查询：返回 true 时 pick 会被拒绝，避免 object 在夹爪上导致规划失败 */
  void setGripperLockedCallback(std::function<bool()> fn);

  /** 话题触发的 PICK 执行时用此回调取最新物体位姿，使物体移动后仍夹当前话题位置；未设置或返回空则用 job 入队时的 object_pose */
  void setGetLatestObjectPoseCallback(
      std::function<std::optional<geometry_msgs::msg::PoseStamped>()> fn);

  /** 分层状态话题回调：由 Node 设置，在对应事件时调用并发布 */
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

  /** 队首 job 类型（用于 runtime_status next_job_type）；空队列返回空串 */
  std::string getNextJobType() const;

  RobotTaskMode getMode() const;
  std::string getTaskId() const;
  std::string getLastError() const;
  HeldObjectContext getHeldObject() const;

  /** 任务队列与 Worker：异步入队、后台消费；返回 job_id（空表示拒绝），拒绝时可选写入 out_reject_reason */
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

  /** 仅取消队列中未执行的 job；正在执行返回 false 且 reason=cannot cancel running job；不存在返回 job not found */
  bool cancelJob(const std::string& job_id, std::string* out_message = nullptr);

  /** 最近 N 条执行记录（含已执行、取消），从新到旧；供 GetRecentJobs 可回看 */
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

  /** 去重：同类型+同目标+短时间窗口内拒绝；与当前执行中同目标也拒绝 */
  bool isDuplicateJob(const ManipulationJob& job, int64_t now_ns, std::string* out_reason) const;
  static bool getJobPoseForDedup(const ManipulationJob& job, geometry_msgs::msg::Pose* out);
  static bool posesNear(const geometry_msgs::msg::Pose& a, const geometry_msgs::msg::Pose& b,
                        double pos_tol_m, double quat_dot_min);

  void pushExecutionRecordStart(const ManipulationJob& job, int64_t started_at_ns);
  void updateExecutionRecordFinish(JobResultCode code, const std::string& message, int64_t finished_at_ns);
  void pushExecutionRecordCancelled(const ManipulationJob& job, int64_t finished_at_ns);

  /** 单次放置尝试（规划+执行）；成功时更新 scene、清 held、设 IDLE；失败仅返回 false，不改 state */
  bool handlePlaceSingle(const geometry_msgs::msg::PoseStamped& target_pose);

  /** 从当前状态规划并执行回到 ready（臂 + 手张开），失败只打日志不抛 */
  bool retreatToReady();

  /** 仅动夹爪：从当前状态（joint_states/joystick）仅对手 group 做 MoveTo open/close，臂关节保持不变 */
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
  JobEventFn job_event_fn_;
  HeldObjectStateFn held_object_state_fn_;
  RecoveryEventFn recovery_event_fn_;
  StageReportFn stage_report_fn_;
  std::unique_ptr<PickTaskBuilder> pick_builder_;
  std::unique_ptr<PlaceTaskBuilder> place_builder_;
  std::unique_ptr<PlaceReleaseTaskBuilder> place_release_builder_;
  std::shared_ptr<TaskQueue> queue_;
  std::unique_ptr<RecoveryActions> recovery_actions_;

  PlaceGenerator* place_generator_ = nullptr;

  mutable std::mutex state_mutex_;
  RobotTaskMode task_mode_ = RobotTaskMode::IDLE;
  HeldObjectContext held_object_;
  std::string current_task_id_;
  std::string last_error_;

  std::atomic<bool> worker_running_{ false };
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
