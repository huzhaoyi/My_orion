/* ManipulatorRosInterface：注册 /manipulator 下订阅、服务、Action 与状态发布定时器 */

#include "orion_mtc/interface/manipulator_ros_interface.hpp"
#include "orion_mtc/interface/robotic_arm_cmd_types.hpp"
#include "orion_mtc/core/constants.hpp"
#include "orion_mtc/perception/perception_snapshot.hpp"
#include "orion_mtc/core/manipulation_job.hpp"
#include "orion_mtc/core/runtime_status.hpp"
#include "orion_mtc/core/task_state.hpp"
#include "orion_mtc/core/job_result_code.hpp"
#include <builtin_interfaces/msg/time.hpp>
#include <orion_mtc_msgs/msg/job_execution_record.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdlib>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <thread>

namespace
{
constexpr char kRoboticArmCmdLogTag[] = "[机械臂任务]";

const char* roboticArmRequestTypeName(uint8_t type)
{
    if (type == orion_mtc::robotic_arm_cmd::REQUEST_TYPE_GRASP)
    {
        return "抓取(0)";
    }
    if (type == orion_mtc::robotic_arm_cmd::REQUEST_TYPE_INSERT)
    {
        return "插孔(1)";
    }
    return "未知类型";
}

const char* roboticArmResultName(uint8_t result)
{
    switch (result)
    {
        case orion_mtc::robotic_arm_cmd::RESULT_SUCCESS:
            return "成功(0)";
        case orion_mtc::robotic_arm_cmd::RESULT_EXEC_FAILED:
            return "执行失败(1)";
        case orion_mtc::robotic_arm_cmd::RESULT_REJECTED_STATE:
            return "状态拒绝(2)";
        case orion_mtc::robotic_arm_cmd::RESULT_REJECTED_NO_HELD:
            return "未持物(3)";
        case orion_mtc::robotic_arm_cmd::RESULT_ESTOP:
            return "急停(4)";
        case orion_mtc::robotic_arm_cmd::RESULT_INVALID:
            return "参数无效(5)";
        default:
            return "未知结果";
    }
}

const char* graspSourceName(orion_mtc::GraspSource gs)
{
    switch (gs)
    {
        case orion_mtc::GraspSource::FUSED:
            return "融合感知链";
        case orion_mtc::GraspSource::TARGET_SENSOR:
            return "目标传感器链";
        case orion_mtc::GraspSource::LEGACY:
        default:
            return "缆绳侧抓链";
    }
}

/* 旧版位姿仍标 base_link（臂根）；与 location 的 ROV base_link 同名，禁止走 /tf。 */
void normalizeLegacyManipulatorPoseFrame(geometry_msgs::msg::PoseStamped& ps,
                                       const rclcpp::Logger& logger,
                                       const char* context)
{
    if (ps.header.frame_id == "base_link")
    {
        RCLCPP_WARN(logger,
                    "%s: legacy frame_id base_link remapped to arm_base_link (no TF)",
                    context);
        ps.header.frame_id = "arm_base_link";
    }
}

const char* roboticArmModeNameChinese(orion_mtc::RobotTaskMode mode)
{
    switch (mode)
    {
        case orion_mtc::RobotTaskMode::PICKING:
            return "抓取中";
        case orion_mtc::RobotTaskMode::HOLDING_TRACKED:
            return "持物跟踪";
        case orion_mtc::RobotTaskMode::HOLDING_UNTRACKED:
            return "持物未跟踪";
        case orion_mtc::RobotTaskMode::ERROR:
            return "错误";
        case orion_mtc::RobotTaskMode::IDLE:
        default:
            return "空闲";
    }
}
}  // namespace

namespace
{
void nsToTime(int64_t ns, builtin_interfaces::msg::Time& t)
{
    if (ns <= 0)
    {
        t.sec = 0;
        t.nanosec = 0u;
        return;
    }
    const int64_t sec = ns / 1000000000;
    const int64_t nsec = ns % 1000000000;
    t.sec = static_cast<int32_t>(sec);
    t.nanosec = static_cast<uint32_t>(nsec);
}

bool tryParseTargetIndexFromObjectId(const std::string& object_id, std::size_t* out_index)
{
    if (out_index == nullptr)
    {
        return false;
    }
    if (object_id.empty())
    {
        return false;
    }
    const std::string prefix = "target_";
    if (object_id.rfind(prefix, 0) != 0)
    {
        return false;
    }
    const std::string suffix = object_id.substr(prefix.size());
    if (suffix.empty())
    {
        return false;
    }
    char* end_ptr = nullptr;
    const long parsed = std::strtol(suffix.c_str(), &end_ptr, 10);
    if (end_ptr == nullptr || *end_ptr != '\0' || parsed < 0)
    {
        return false;
    }
    *out_index = static_cast<std::size_t>(parsed);
    return true;
}
}  // namespace

namespace orion_mtc
{

/* 仅保存 ManipulatorInterfaceContext（缓存、TaskManager、logger 等），注册接口在 register* 中完成。 */
ManipulatorRosInterface::ManipulatorRosInterface(ManipulatorInterfaceContext ctx)
  : ctx_(std::move(ctx))
{
}

/*
 * 在 action_client_node 上注册 /manipulator 下全部订阅、Pick Action 服务端与各业务服务。
 * left_arm_gripped 同时写入原子量并转发 TaskManager::applyGripperFeedbackFromTopic。
 */
void ManipulatorRosInterface::registerSubscriptionsAndServices()
{
    loadRoboticArmCmdParams();
    const std::string ns(MANIPULATOR_NS);
    sub_object_pose_ = ctx_.action_client_node->create_subscription<geometry_msgs::msg::PoseStamped>(
        ns + "/object_pose", 10, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            ctx_.object_pose_cache->update(*msg);
        });
    sub_object_pose_cable_ = ctx_.action_client_node->create_subscription<geometry_msgs::msg::PoseStamped>(
        ns + "/object_pose_cable", 10, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            cable_pose_cache_.update(*msg);
        });
    sub_object_pose_targetsensor_ = ctx_.action_client_node->create_subscription<geometry_msgs::msg::PoseStamped>(
        ns + "/object_pose_targetsensor", 10, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            targetsensor_pose_cache_.update(*msg);
        });
    sub_target_set_ = ctx_.action_client_node->create_subscription<orion_mtc_msgs::msg::TargetSet>(
        ns + "/target_set", 10, [this](const orion_mtc_msgs::msg::TargetSet::SharedPtr msg) {
            if (ctx_.target_cache)
            {
                ctx_.target_cache->update(*msg);
            }
        });
    sub_object_axis_ = ctx_.action_client_node->create_subscription<geometry_msgs::msg::Vector3Stamped>(
        ns + "/object_axis", 10, [this](const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
            ctx_.object_axis_cache->update(*msg);
        });
    if (ctx_.object_pose_fused_cache)
    {
        sub_object_pose_fused_ = ctx_.action_client_node->create_subscription<geometry_msgs::msg::PoseStamped>(
            ns + "/object_pose_fused", 10, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                ctx_.object_pose_fused_cache->update(*msg);
            });
    }
    if (ctx_.object_axis_fused_cache)
    {
        sub_object_axis_fused_ = ctx_.action_client_node->create_subscription<geometry_msgs::msg::Vector3Stamped>(
            ns + "/object_axis_fused", 10, [this](const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
                ctx_.object_axis_fused_cache->update(*msg);
            });
    }
    sub_pick_trigger_ = ctx_.action_client_node->create_subscription<std_msgs::msg::Empty>(
        ns + "/pick_trigger", 10, [this](const std_msgs::msg::Empty::SharedPtr msg) {
            onPickTriggerReceived(msg);
        });
    sub_pick_trigger_cable_ = ctx_.action_client_node->create_subscription<std_msgs::msg::Empty>(
        ns + "/pick_trigger_cable", 10, [this](const std_msgs::msg::Empty::SharedPtr msg) {
            onPickTriggerCableReceived(msg);
        });
    sub_pick_trigger_targetsensor_ = ctx_.action_client_node->create_subscription<std_msgs::msg::Empty>(
        ns + "/pick_trigger_targetsensor", 10, [this](const std_msgs::msg::Empty::SharedPtr msg) {
            onPickTriggerTargetSensorReceived(msg);
        });
    sub_pick_trigger_fused_ = ctx_.action_client_node->create_subscription<std_msgs::msg::Empty>(
        ns + "/pick_trigger_fused", 10, [this](const std_msgs::msg::Empty::SharedPtr msg) {
            onPickTriggerFusedReceived(msg);
        });
    sub_left_arm_gripped_ = ctx_.action_client_node->create_subscription<std_msgs::msg::Float32>(
        ns + "/left_arm_gripped", 10, [this](const std_msgs::msg::Float32::SharedPtr msg) {
            const double v = static_cast<double>(msg->data);
            ctx_.left_arm_gripped->store(v);
            if (ctx_.task_manager)
            {
                ctx_.task_manager->applyGripperFeedbackFromTopic(v);
            }
        });

    pick_action_server_ = rclcpp_action::create_server<orion_mtc_msgs::action::Pick>(
        ctx_.action_client_node, ns + "/pick",
        [this](const rclcpp_action::GoalUUID& uuid,
               std::shared_ptr<const orion_mtc_msgs::action::Pick::Goal> goal) {
            return handlePickGoalRequest(uuid, goal);
        },
        [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<orion_mtc_msgs::action::Pick>>& h) {
            return handlePickGoalCancel(h);
        },
        [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<orion_mtc_msgs::action::Pick>>& h) {
            handlePickGoalAccepted(h);
        });
    if (robotic_arm_cmd_enable_)
    {
        const std::string primary_path = resolveRoboticArmActionPath(robotic_arm_cmd_action_name_, true);
        robotic_arm_cmd_server_ = createRoboticArmCmdActionServer(primary_path);
        std::string alias_note;
        if (robotic_arm_cmd_register_root_alias_ && !robotic_arm_cmd_action_name_.empty() &&
            robotic_arm_cmd_action_name_.front() != '/')
        {
            const std::string root_path = resolveRoboticArmActionPath(robotic_arm_cmd_action_name_, false);
            if (root_path != primary_path)
            {
                robotic_arm_cmd_root_alias_server_ = createRoboticArmCmdActionServer(root_path);
                alias_note = " 根别名=" + root_path;
            }
        }
        RCLCPP_INFO(ctx_.logger,
                    "%s Action 服务已就绪: 主路径=%s（action_name=%s 详细日志=%s 缆绳侧抓=%s 拒绝右臂系=%s 反馈=%.1fHz）%s",
                    kRoboticArmCmdLogTag,
                    primary_path.c_str(),
                    robotic_arm_cmd_action_name_.c_str(),
                    robotic_arm_cmd_verbose_ ? "开" : "关",
                    robotic_arm_cmd_use_cable_side_grasp_ ? "开" : "关",
                    robotic_arm_cmd_reject_right_frames_ ? "开" : "关",
                    robotic_arm_cmd_feedback_hz_,
                    alias_note.c_str());
        RCLCPP_INFO(ctx_.logger,
                    "%s 同事客户端: create_client(..., \"%s\") 时节点命名空间须为 /manipulator，"
                    "或使用绝对名 \"%s\"；无命名空间时可连根别名 /%s",
                    kRoboticArmCmdLogTag,
                    robotic_arm_cmd_action_name_.c_str(),
                    primary_path.c_str(),
                    robotic_arm_cmd_action_name_.c_str());
    }
    get_robot_state_srv_ = ctx_.action_client_node->create_service<orion_mtc_msgs::srv::GetRobotState>(
        ns + "/get_robot_state",
        [this](const std::shared_ptr<orion_mtc_msgs::srv::GetRobotState::Request> req,
               std::shared_ptr<orion_mtc_msgs::srv::GetRobotState::Response> res) {
            handleGetRobotState(req, res);
        });
    get_queue_state_srv_ = ctx_.action_client_node->create_service<orion_mtc_msgs::srv::GetQueueState>(
        ns + "/get_queue_state",
        [this](const std::shared_ptr<orion_mtc_msgs::srv::GetQueueState::Request> req,
               std::shared_ptr<orion_mtc_msgs::srv::GetQueueState::Response> res) {
            handleGetQueueState(req, res);
        });
    get_recent_jobs_srv_ = ctx_.action_client_node->create_service<orion_mtc_msgs::srv::GetRecentJobs>(
        ns + "/get_recent_jobs",
        [this](const std::shared_ptr<orion_mtc_msgs::srv::GetRecentJobs::Request> req,
               std::shared_ptr<orion_mtc_msgs::srv::GetRecentJobs::Response> res) {
            handleGetRecentJobs(req, res);
        });
    submit_job_srv_ = ctx_.action_client_node->create_service<orion_mtc_msgs::srv::SubmitJob>(
        ns + "/submit_job",
        [this](const std::shared_ptr<orion_mtc_msgs::srv::SubmitJob::Request> req,
               std::shared_ptr<orion_mtc_msgs::srv::SubmitJob::Response> res) {
            handleSubmitJob(req, res);
        });
    cancel_job_srv_ = ctx_.action_client_node->create_service<orion_mtc_msgs::srv::CancelJob>(
        ns + "/cancel_job",
        [this](const std::shared_ptr<orion_mtc_msgs::srv::CancelJob::Request> req,
               std::shared_ptr<orion_mtc_msgs::srv::CancelJob::Response> res) {
            handleCancelJob(req, res);
        });
    open_gripper_srv_ = ctx_.action_client_node->create_service<std_srvs::srv::Trigger>(
        ns + "/open_gripper",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
            handleOpenGripper(req, res);
        });
    close_gripper_srv_ = ctx_.action_client_node->create_service<std_srvs::srv::Trigger>(
        ns + "/close_gripper",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
            handleCloseGripper(req, res);
        });
    emergency_stop_srv_ = ctx_.action_client_node->create_service<std_srvs::srv::Trigger>(
        ns + "/emergency_stop",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
            handleEmergencyStopService(req, res);
        });
    clear_estop_srv_ = ctx_.action_client_node->create_service<std_srvs::srv::Trigger>(
        ns + "/clear_estop",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
            handleClearEstopService(req, res);
        });
    go_to_ready_srv_ = ctx_.action_client_node->create_service<std_srvs::srv::Trigger>(
        ns + "/go_to_ready",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
            handleGoToReadyService(req, res);
        });
    reset_held_object_srv_ =
        ctx_.action_client_node->create_service<orion_mtc_msgs::srv::ResetHeldObject>(
            ns + "/reset_held_object",
            [this](const std::shared_ptr<orion_mtc_msgs::srv::ResetHeldObject::Request> req,
                   std::shared_ptr<orion_mtc_msgs::srv::ResetHeldObject::Response> res) {
                handleResetHeldObject(req, res);
            });
    sync_held_object_srv_ = ctx_.action_client_node->create_service<orion_mtc_msgs::srv::SyncHeldObject>(
        ns + "/sync_held_object",
        [this](const std::shared_ptr<orion_mtc_msgs::srv::SyncHeldObject::Request> req,
               std::shared_ptr<orion_mtc_msgs::srv::SyncHeldObject::Response> res) {
            handleSyncHeldObject(req, res);
        });
    check_pick_srv_ = ctx_.action_client_node->create_service<orion_mtc_msgs::srv::CheckPick>(
        ns + "/check_pick",
        [this](const std::shared_ptr<orion_mtc_msgs::srv::CheckPick::Request> req,
               std::shared_ptr<orion_mtc_msgs::srv::CheckPick::Response> res) {
            handleCheckPick(req, res);
        });
}

/*
 * 创建 runtime_status/job_event/task_stage 等 publisher，500ms 定时发 RuntimeStatus；
 * 将 TaskManager 的 job/held/recovery/stage 回调桥接到对应 ROS 消息。
 */
void ManipulatorRosInterface::registerStatusPublishersAndCallbacks()
{
    const std::string ns(MANIPULATOR_NS);
    pub_runtime_status_ =
        ctx_.action_client_node->create_publisher<orion_mtc_msgs::msg::RuntimeStatus>(ns + "/runtime_status", 10);
    pub_job_event_ =
        ctx_.action_client_node->create_publisher<orion_mtc_msgs::msg::JobEvent>(ns + "/job_event", 10);
    pub_task_stage_ =
        ctx_.action_client_node->create_publisher<orion_mtc_msgs::msg::TaskStage>(ns + "/task_stage", 10);
    pub_held_object_state_ =
        ctx_.action_client_node->create_publisher<orion_mtc_msgs::msg::HeldObjectState>(ns + "/held_object_state", 10);
    pub_recovery_event_ =
        ctx_.action_client_node->create_publisher<orion_mtc_msgs::msg::RecoveryEvent>(ns + "/recovery_event", 10);

    runtime_status_timer_ = ctx_.action_client_node->create_wall_timer(
        std::chrono::milliseconds(500), [this]() { publishRuntimeStatus(); });

    ctx_.task_manager->setJobEventCallback(
        [this](const std::string& job_id, const std::string& job_type, const std::string& source,
               uint32_t priority, const std::string& event_type, bool success, const std::string& reason,
               int64_t created_at_ns, int64_t started_at_ns, int64_t finished_at_ns) {
            orion_mtc_msgs::msg::JobEvent msg;
            msg.header.stamp = ctx_.action_client_node->now();
            msg.job_id = job_id;
            msg.job_type = job_type;
            msg.source = source;
            msg.priority = priority;
            msg.event_type = event_type;
            msg.success = success;
            msg.reason = reason;
            nsToTime(created_at_ns, msg.created_at);
            nsToTime(started_at_ns, msg.started_at);
            nsToTime(finished_at_ns, msg.finished_at);
            pub_job_event_->publish(msg);
        });

    ctx_.task_manager->setHeldObjectStateCallback([this](const HeldObjectContext& hctx) {
        orion_mtc_msgs::msg::HeldObjectState msg;
        msg.header.stamp = ctx_.action_client_node->now();
        msg.valid = hctx.valid;
        msg.tracked = isTracked(hctx);
        msg.object_id = hctx.object_id;
        msg.scene_attach_id = hctx.scene_attach_id;
        msg.attach_link = hctx.attach_link;
        msg.object_pose_at_grasp = hctx.object_pose_at_grasp;
        msg.tcp_pose_at_grasp = hctx.tcp_pose_at_grasp;
        msg.weight = static_cast<float>(hctx.weight);
        pub_held_object_state_->publish(msg);
    });

    ctx_.task_manager->setRecoveryEventCallback(
        [this](const std::string& recovery_type, const std::string& trigger_reason, bool success,
               const std::string& detail) {
            orion_mtc_msgs::msg::RecoveryEvent msg;
            msg.header.stamp = ctx_.action_client_node->now();
            msg.recovery_type = recovery_type;
            msg.trigger_reason = trigger_reason;
            msg.success = success;
            msg.detail = detail;
            pub_recovery_event_->publish(msg);
        });

    ctx_.task_manager->setStageReportCallback(
        [this](const std::string& job_id, const std::string& task_type, std::size_t stage_index,
               const std::string& stage_name, const std::string& stage_state, const std::string& detail) {
            (void)stage_index;
            orion_mtc_msgs::msg::TaskStage msg;
            msg.header.stamp = ctx_.action_client_node->now();
            msg.job_id = job_id;
            msg.task_type = task_type;
            msg.stage_name = stage_name;
            msg.stage_state = stage_state;
            msg.detail = detail;
            pub_task_stage_->publish(msg);
        });

    if (ctx_.feasibility_checker)
    {
        ctx_.feasibility_checker->setPickApprovalProgressFn(
            [this](const std::string& stage_name, const std::string& stage_state, const std::string& detail) {
                orion_mtc_msgs::msg::TaskStage msg;
                msg.header.stamp = ctx_.action_client_node->now();
                msg.header.frame_id = "";
                msg.job_id = "";
                msg.task_type = "CHECK_PICK";
                msg.stage_name = stage_name;
                msg.stage_state = stage_state;
                msg.detail = detail;
                pub_task_stage_->publish(msg);
            });
    }
}

/* 聚合 TaskManager 与 TaskQueue 状态填充 RuntimeStatus 并发布（供 Web/监控轮询）。 */
void ManipulatorRosInterface::publishRuntimeStatus()
{
    orion_mtc_msgs::msg::RuntimeStatus msg;
    msg.header.stamp = ctx_.action_client_node->now();
    msg.header.frame_id = "";
    msg.worker_status = toCString(ctx_.task_manager->getWorkerStatus());
    msg.task_mode = toStateString(ctx_.task_manager->getMode());
    msg.current_job_id = ctx_.task_manager->getCurrentJobId();
    msg.current_job_type = ctx_.task_manager->getCurrentJobType();
    msg.next_job_type = ctx_.task_manager->getNextJobType();
    msg.worker_running = ctx_.task_manager->isWorkerRunning();
    std::shared_ptr<TaskQueue> q = ctx_.task_manager->getQueue();
    msg.queue_empty = !q || q->empty();
    msg.queue_size = q ? static_cast<uint32_t>(q->size()) : 0u;
    HeldObjectContext held = ctx_.task_manager->getHeldObject();
    msg.has_held_object = held.valid;
    msg.held_object_id = held.valid ? held.object_id : "";
    msg.held_scene_attach_id = held.valid ? held.scene_attach_id : "";
    msg.insert_latch_locked = ctx_.task_manager->isInsertLatchLocked();
    msg.insert_latch_hole = ctx_.task_manager->getInsertLatchHole();
    msg.last_error = ctx_.task_manager->getLastError();
    pub_runtime_status_->publish(msg);
}

/*
 * pick_trigger：LEGACY 链；后台组 job，优先 TargetSelector，否则 legacy object_pose 缓存，可等待最多 3s。
 */
void ManipulatorRosInterface::onPickTriggerReceived(const std_msgs::msg::Empty::SharedPtr)
{
    std::thread([this]() {
        if (isGripperLocked())
        {
            RCLCPP_WARN(ctx_.logger,
                        "topic_pick_trigger: gripper locked (has object), not enqueued (reset_held_object or "
                        "open_gripper first)");
            return;
        }
        ManipulationJob job;
        job.type = JobType::PICK;
        job.grasp_source = GraspSource::LEGACY;
        job.object_id = "";
        job.source = "topic_pick_trigger";
        std::optional<geometry_msgs::msg::PoseStamped> topic_pose;
        if (ctx_.perception_provider && ctx_.target_selector)
        {
            PerceptionSnapshot snap = ctx_.perception_provider->snapshot();
            topic_pose = ctx_.target_selector->selectPickTarget(snap);
        }
        if (!topic_pose.has_value())
        {
            topic_pose = ctx_.object_pose_cache->latest();
        }
        if (!topic_pose.has_value())
        {
            geometry_msgs::msg::PoseStamped pose;
            if (!ctx_.object_pose_cache->waitForPose(std::chrono::milliseconds(3000), pose))
            {
                RCLCPP_WARN(ctx_.logger, "topic_pick_trigger: no object_pose after wait, not enqueued");
                return;
            }
            topic_pose = pose;
        }
        job.object_pose = *topic_pose;
        std::string reject_reason;
        std::string job_id = ctx_.task_manager->submitJob(job, &reject_reason);
        if (job_id.empty())
        {
            RCLCPP_INFO(ctx_.logger, "topic_pick_trigger: rejected (%s)", reject_reason.c_str());
        }
        else
        {
            RCLCPP_INFO(ctx_.logger, "topic_pick_trigger: accepted job_id=%s", job_id.c_str());
        }
    }).detach();
}

/* pick_trigger_cable：缆绳链；仅用 /object_pose_cable（实时） */
void ManipulatorRosInterface::onPickTriggerCableReceived(const std_msgs::msg::Empty::SharedPtr)
{
    std::thread([this]() {
        if (isGripperLocked())
        {
            RCLCPP_WARN(ctx_.logger,
                        "topic_pick_trigger_cable: gripper locked (has object), not enqueued (reset_held_object or "
                        "open_gripper first)");
            return;
        }
        ManipulationJob job;
        job.type = JobType::PICK;
        job.grasp_source = GraspSource::LEGACY;
        job.object_id = "";
        job.source = "topic_pick_trigger_cable";
        std::optional<geometry_msgs::msg::PoseStamped> topic_pose = cable_pose_cache_.latest();
        if (!topic_pose.has_value())
        {
            geometry_msgs::msg::PoseStamped pose;
            if (!cable_pose_cache_.waitForPose(std::chrono::milliseconds(3000), pose))
            {
                RCLCPP_WARN(ctx_.logger, "topic_pick_trigger_cable: no object_pose_cable after wait, not enqueued");
                return;
            }
            topic_pose = pose;
        }
        job.object_pose = *topic_pose;
        std::string reject_reason;
        std::string job_id = ctx_.task_manager->submitJob(job, &reject_reason);
        if (job_id.empty())
        {
            RCLCPP_INFO(ctx_.logger, "topic_pick_trigger_cable: rejected (%s)", reject_reason.c_str());
        }
        else
        {
            RCLCPP_INFO(ctx_.logger, "topic_pick_trigger_cable: accepted job_id=%s", job_id.c_str());
        }
    }).detach();
}

/* pick_trigger_targetsensor：TargetSensor 链；仅用 /object_pose_targetsensor（实时） */
void ManipulatorRosInterface::onPickTriggerTargetSensorReceived(const std_msgs::msg::Empty::SharedPtr)
{
    std::thread([this]() {
        if (isGripperLocked())
        {
            RCLCPP_WARN(ctx_.logger,
                        "topic_pick_trigger_targetsensor: gripper locked (has object), not enqueued (reset_held_object or "
                        "open_gripper first)");
            return;
        }
        ManipulationJob job;
        job.type = JobType::PICK;
        job.grasp_source = GraspSource::TARGET_SENSOR;
        job.object_id = "";
        job.source = "topic_pick_trigger_targetsensor";
        std::optional<geometry_msgs::msg::PoseStamped> topic_pose = targetsensor_pose_cache_.latest();
        if (!topic_pose.has_value())
        {
            geometry_msgs::msg::PoseStamped pose;
            if (!targetsensor_pose_cache_.waitForPose(std::chrono::milliseconds(3000), pose))
            {
                RCLCPP_WARN(ctx_.logger, "topic_pick_trigger_targetsensor: no object_pose_targetsensor after wait, not enqueued");
                return;
            }
            topic_pose = pose;
        }
        job.object_pose = *topic_pose;
        std::string reject_reason;
        std::string job_id = ctx_.task_manager->submitJob(job, &reject_reason);
        if (job_id.empty())
        {
            RCLCPP_INFO(ctx_.logger, "topic_pick_trigger_targetsensor: rejected (%s)", reject_reason.c_str());
        }
        else
        {
            RCLCPP_INFO(ctx_.logger, "topic_pick_trigger_targetsensor: accepted job_id=%s", job_id.c_str());
        }
    }).detach();
}

/*
 * pick_trigger_fused：FUSED 链；仅用 object_pose_fused 缓存（不用 TargetSelector）。
 */
void ManipulatorRosInterface::onPickTriggerFusedReceived(const std_msgs::msg::Empty::SharedPtr)
{
    std::thread([this]() {
        if (isGripperLocked())
        {
            RCLCPP_WARN(ctx_.logger,
                        "topic_pick_trigger_fused: gripper locked (has object), not enqueued (reset_held_object "
                        "or open_gripper first)");
            return;
        }
        if (!ctx_.object_pose_fused_cache)
        {
            RCLCPP_WARN(ctx_.logger, "topic_pick_trigger_fused: fused pose cache not configured");
            return;
        }
        ManipulationJob job;
        job.type = JobType::PICK;
        job.grasp_source = GraspSource::FUSED;
        job.object_id = "";
        job.source = "topic_pick_trigger_fused";
        std::optional<geometry_msgs::msg::PoseStamped> topic_pose = ctx_.object_pose_fused_cache->latest();
        if (!topic_pose.has_value())
        {
            geometry_msgs::msg::PoseStamped pose;
            if (!ctx_.object_pose_fused_cache->waitForPose(std::chrono::milliseconds(3000), pose))
            {
                RCLCPP_WARN(ctx_.logger, "topic_pick_trigger_fused: no object_pose_fused after wait, not enqueued");
                return;
            }
            topic_pose = pose;
        }
        job.object_pose = *topic_pose;
        std::string reject_reason;
        std::string job_id = ctx_.task_manager->submitJob(job, &reject_reason);
        if (job_id.empty())
        {
            RCLCPP_INFO(ctx_.logger, "topic_pick_trigger_fused: rejected (%s)", reject_reason.c_str());
        }
        else
        {
            RCLCPP_INFO(ctx_.logger, "topic_pick_trigger_fused: accepted job_id=%s", job_id.c_str());
        }
    }).detach();
}

/* 与 applyGripperFeedback 阈值一致：left_arm_gripped >= 0.5 视为夹紧有物。 */
bool ManipulatorRosInterface::isGripperLocked() const
{
    const double threshold = 0.5;
    return ctx_.left_arm_gripped->load() >= threshold;
}

/* Pick Action _goal 校验：已持物、gripper locked、业务 busy 时 REJECT，否则 ACCEPT_AND_EXECUTE。 */
rclcpp_action::GoalResponse ManipulatorRosInterface::handlePickGoalRequest(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const orion_mtc_msgs::action::Pick::Goal>)
{
    RobotTaskMode mode = ctx_.task_manager->getMode();
    if (isHolding(mode))
    {
        RCLCPP_INFO(ctx_.logger, "Pick goal rejected: already holding (reset_held_object first)");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (isGripperLocked())
    {
        RCLCPP_INFO(ctx_.logger,
                    "Pick goal rejected: gripper locked (has object), reset_held_object or open_gripper first");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (!canAcceptPick(mode))
    {
        RCLCPP_INFO(ctx_.logger, "Pick goal rejected: busy");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (ctx_.feasibility_checker && ctx_.feasibility_checker->inJoyModeSwitchIkCooldown())
    {
        RCLCPP_INFO(ctx_.logger, "Pick goal rejected: joy auto/manual switch IK cooldown");
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

/* 当前实现不允许取消已在执行的 Pick Action（避免与 MTC 分段状态难对齐）。 */
rclcpp_action::CancelResponse ManipulatorRosInterface::handlePickGoalCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<orion_mtc_msgs::action::Pick>>&)
{
    return rclcpp_action::CancelResponse::REJECT;
}

/*
 * 接受 goal 后 detach 线程调用 TaskManager::handlePick，成功 succeed、失败 abort，
 * Result 中带 task_id、held_object_id、message/last_error。
 */
void ManipulatorRosInterface::handlePickGoalAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<orion_mtc_msgs::action::Pick>>& goal_handle)
{
    std::thread([this, goal_handle]() {
        const auto goal = goal_handle->get_goal();
        GraspSource gs = GraspSource::LEGACY;
        if (goal->grasp_source == 1U)
        {
            gs = GraspSource::FUSED;
        }
        else if (goal->grasp_source == 2U)
        {
            gs = GraspSource::TARGET_SENSOR;
        }
        bool ok = ctx_.task_manager->handlePick(goal->object_pose,
                                                goal->object_id.empty() ? "object" : goal->object_id, gs);
        auto result = std::make_shared<orion_mtc_msgs::action::Pick::Result>();
        result->success = ok;
        result->task_id = ctx_.task_manager->getTaskId();
        result->held_object_id = ok ? ctx_.task_manager->getHeldObject().object_id : "";
        result->message = ok ? "pick success" : ctx_.task_manager->getLastError();
        if (ok)
        {
            goal_handle->succeed(result);
        }
        else
        {
            goal_handle->abort(result);
        }
    }).detach();
}

/* GetRobotState：返回 mode、task_id、持物 id 标志、last_error（轻量查询）。 */
void ManipulatorRosInterface::handleGetRobotState(
    const std::shared_ptr<orion_mtc_msgs::srv::GetRobotState::Request>,
    std::shared_ptr<orion_mtc_msgs::srv::GetRobotState::Response> res)
{
    res->mode = toStateString(ctx_.task_manager->getMode());
    res->task_id = ctx_.task_manager->getTaskId();
    res->held_object_id =
        ctx_.task_manager->getHeldObject().valid ? ctx_.task_manager->getHeldObject().object_id : "";
    res->has_held_object = ctx_.task_manager->getHeldObject().valid;
    res->last_error = ctx_.task_manager->getLastError();
}

/*
 * GetQueueState：队列长度、当前/下一 job、worker 状态、task_mode、是否空队列等；
 * next_* 来自 peekFront，不弹出。
 */
void ManipulatorRosInterface::handleGetQueueState(
    const std::shared_ptr<orion_mtc_msgs::srv::GetQueueState::Request>,
    std::shared_ptr<orion_mtc_msgs::srv::GetQueueState::Response> res)
{
    std::shared_ptr<TaskQueue> q = ctx_.task_manager->getQueue();
    res->queue_size = q ? static_cast<uint32_t>(q->size()) : 0u;
    res->current_job_id = ctx_.task_manager->getCurrentJobId();
    res->current_job_type = ctx_.task_manager->getCurrentJobType();
    res->next_job_type = "NONE";
    res->next_job_id = "";
    res->next_job_priority = -1;
    if (q)
    {
        ManipulationJob next;
        if (q->peekFront(next))
        {
            res->next_job_type = jobTypeToCString(next.type);
            res->next_job_id = next.job_id;
            res->next_job_priority = static_cast<int32_t>(next.priority);
        }
    }
    WorkerStatus ws = ctx_.task_manager->getWorkerStatus();
    res->worker_status = static_cast<uint8_t>(ws);
    res->task_mode = toStateString(ctx_.task_manager->getMode());
    res->last_error = ctx_.task_manager->getLastError();
    res->worker_running = ctx_.task_manager->isWorkerRunning();
    res->queue_empty = !q || q->empty();
}

/* GetRecentJobs：将 TaskManager 环形执行记录转为消息列表，max_count 默认上限 50。 */
void ManipulatorRosInterface::handleGetRecentJobs(
    const std::shared_ptr<orion_mtc_msgs::srv::GetRecentJobs::Request> req,
    std::shared_ptr<orion_mtc_msgs::srv::GetRecentJobs::Response> res)
{
    const std::uint32_t max_count = req->max_count > 0u ? req->max_count : 50u;
    std::vector<TaskManager::JobExecutionRecordEntry> entries = ctx_.task_manager->getRecentRecords(max_count);
    res->records.clear();
    res->records.reserve(entries.size());
    for (const auto& e : entries)
    {
        orion_mtc_msgs::msg::JobExecutionRecord msg;
        msg.job_id = e.job_id;
        msg.job_type = e.job_type;
        msg.source = e.source;
        msg.result_code = static_cast<uint8_t>(e.result_code);
        msg.message = e.message;
        msg.created_at_ns = e.created_at_ns;
        msg.started_at_ns = e.started_at_ns;
        msg.finished_at_ns = e.finished_at_ns;
        res->records.push_back(msg);
    }
}

/*
 * SubmitJob：校验 job_type；PICK 且 gripper locked 拒绝；组装 ManipulationJob 调 TaskManager::submitJob，
 * 成功返回 job_id，失败写 message。
 */
void ManipulatorRosInterface::handleSubmitJob(
    const std::shared_ptr<orion_mtc_msgs::srv::SubmitJob::Request> req,
    std::shared_ptr<orion_mtc_msgs::srv::SubmitJob::Response> res)
{
    if (req->job_type > 5u)
    {
        res->success = false;
        res->message = "invalid job_type (0=PICK,1=RESET_HELD_OBJECT,2=SYNC_HELD_OBJECT,3=OPEN_GRIPPER,4=CLOSE_GRIPPER,5=TARGET_INSERT)";
        return;
    }
    if (req->job_type == static_cast<uint8_t>(JobType::PICK) && isGripperLocked())
    {
        res->success = false;
        res->message = "gripper locked (has object), reset_held_object or open_gripper first";
        return;
    }
    ManipulationJob job;
    job.job_id = req->job_id;
    job.type = static_cast<JobType>(req->job_type);
    if (req->grasp_source == 1U)
    {
        job.grasp_source = GraspSource::FUSED;
    }
    else if (req->grasp_source == 2U)
    {
        job.grasp_source = GraspSource::TARGET_SENSOR;
    }
    else
    {
        job.grasp_source = GraspSource::LEGACY;
    }
    job.object_id = req->object_id;
    job.tracked = req->tracked;
    job.priority = req->priority;
    job.target_pose = req->target_pose;
    job.object_pose = req->object_pose;
    job.tcp_pose = req->tcp_pose;
    job.source = "submit_job_service";
    if (job.target_pose.has_value())
    {
        normalizeLegacyManipulatorPoseFrame(*job.target_pose, ctx_.logger, "handleSubmitJob target_pose");
    }
    if (job.object_pose.has_value())
    {
        normalizeLegacyManipulatorPoseFrame(*job.object_pose, ctx_.logger, "handleSubmitJob object_pose");
    }
    if (job.type == JobType::PICK && job.grasp_source == GraspSource::TARGET_SENSOR)
    {
        std::size_t target_index = 0;
        const bool has_target_index = tryParseTargetIndexFromObjectId(job.object_id, &target_index);
        if (has_target_index)
        {
            if (!ctx_.target_cache)
            {
                res->success = false;
                res->message = "target_cache unavailable for TARGET_SENSOR pick";
                return;
            }
            std::optional<orion_mtc_msgs::msg::TargetSet> latest_target_set = ctx_.target_cache->latest();
            if (!latest_target_set.has_value())
            {
                res->success = false;
                res->message = "target_set not ready";
                return;
            }
            const std::size_t target_count = latest_target_set->targets.size();
            if (target_index >= target_count)
            {
                res->success = false;
                res->message = "target index out of range";
                return;
            }
            job.object_pose = latest_target_set->targets[target_index];
            if (target_index < latest_target_set->object_ids.size() && !latest_target_set->object_ids[target_index].empty())
            {
                job.object_id = latest_target_set->object_ids[target_index];
            }
            RCLCPP_INFO(
                ctx_.logger,
                "handleSubmitJob: TARGET_SENSOR pick bound to target_set index=%zu object_id=%s",
                target_index,
                job.object_id.c_str());
        }
        else
        {
            RCLCPP_WARN(
                ctx_.logger,
                "handleSubmitJob: TARGET_SENSOR pick object_id(%s) missing target_<index>, keep request pose",
                job.object_id.c_str());
        }
    }
    std::string reject_reason;
    std::string assigned_id = ctx_.task_manager->submitJob(job, &reject_reason);
    if (assigned_id.empty())
    {
        res->success = false;
        res->message = reject_reason.empty() ? "policy rejected" : reject_reason;
        return;
    }
    res->success = true;
    res->message = "queued";
    res->job_id = assigned_id;
}

/* CancelJob：代理 TaskManager::cancelJob；正在执行的 job 不可取消。 */
void ManipulatorRosInterface::handleCancelJob(
    const std::shared_ptr<orion_mtc_msgs::srv::CancelJob::Request> req,
    std::shared_ptr<orion_mtc_msgs::srv::CancelJob::Response> res)
{
    std::string message;
    bool ok = ctx_.task_manager->cancelJob(req->job_id, &message);
    res->success = ok;
    res->message = message;
}

/* ResetHeldObject：调用 handleResetHeldObject，success/message 回填。 */
void ManipulatorRosInterface::handleResetHeldObject(
    const std::shared_ptr<orion_mtc_msgs::srv::ResetHeldObject::Request>,
    std::shared_ptr<orion_mtc_msgs::srv::ResetHeldObject::Response> res)
{
    res->success = ctx_.task_manager->handleResetHeldObject(res->message);
}

/* SyncHeldObject：透传 set_holding、tracked、位姿到 TaskManager::handleSyncHeldObject。 */
void ManipulatorRosInterface::handleSyncHeldObject(
    const std::shared_ptr<orion_mtc_msgs::srv::SyncHeldObject::Request> req,
    std::shared_ptr<orion_mtc_msgs::srv::SyncHeldObject::Response> res)
{
    res->success = ctx_.task_manager->handleSyncHeldObject(
        req->set_holding, req->tracked, req->object_id, req->object_pose, req->tcp_pose, res->message);
}

/* CheckPick：有 FeasibilityChecker 则委托 checkPick；否则拒绝并提示未就绪。 */
void ManipulatorRosInterface::handleCheckPick(
    const std::shared_ptr<orion_mtc_msgs::srv::CheckPick::Request> req,
    std::shared_ptr<orion_mtc_msgs::srv::CheckPick::Response> res)
{
    if (ctx_.feasibility_checker)
    {
        ctx_.feasibility_checker->checkPick(req, res);
    }
    else
    {
        res->approved = false;
        res->severity = 2;
        res->summary = "审批模块未就绪";
    }
}

/* 打开夹爪：异步入队 OPEN_GRIPPER，success 表示已接受队列 job_id。 */
void ManipulatorRosInterface::handleOpenGripper(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                                std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    ManipulationJob job;
    job.type = JobType::OPEN_GRIPPER;
    job.source = "open_gripper_srv";
    std::string reject_reason;
    std::string job_id = ctx_.task_manager->submitJob(job, &reject_reason);
    res->success = !job_id.empty();
    res->message = res->success ? job_id : ("rejected: " + reject_reason);
}

/* 闭合夹爪：异步入队 CLOSE_GRIPPER。 */
void ManipulatorRosInterface::handleCloseGripper(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                                 std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    ManipulationJob job;
    job.type = JobType::CLOSE_GRIPPER;
    job.source = "close_gripper_srv";
    std::string reject_reason;
    std::string job_id = ctx_.task_manager->submitJob(job, &reject_reason);
    res->success = !job_id.empty();
    res->message = res->success ? job_id : ("rejected: " + reject_reason);
}

/* 急停服务：requestEmergencyStop（清队列、取消轨迹），始终 success=true 除非后续扩展。 */
void ManipulatorRosInterface::handleEmergencyStopService(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                                         std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    ctx_.task_manager->requestEmergencyStop();
    res->success = true;
    res->message = "emergency_stop";
    RCLCPP_WARN(ctx_.logger, "service emergency_stop: cancel trajectories + clear queue");
}

/* 解除急停闭锁位，不恢复队列。 */
void ManipulatorRosInterface::handleClearEstopService(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                                      std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    ctx_.task_manager->clearEmergencyStopLatch();
    res->success = true;
    res->message = "clear_estop";
    RCLCPP_INFO(ctx_.logger, "service clear_estop: latch cleared");
}

/* go_to_ready 服务：同步调用 tryGoToReady，busy 时失败 message 说明原因。 */
void ManipulatorRosInterface::handleGoToReadyService(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                                     std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    std::string msg_out;
    bool ok = ctx_.task_manager->tryGoToReady(msg_out);
    res->success = ok;
    res->message = msg_out;
    if (ok)
    {
        RCLCPP_INFO(ctx_.logger, "service go_to_ready: %s", msg_out.c_str());
    }
    else
    {
        RCLCPP_WARN(ctx_.logger, "service go_to_ready: %s", msg_out.c_str());
    }
}

std::string ManipulatorRosInterface::resolveRoboticArmActionPath(const std::string& action_name,
                                                               bool under_manipulator_ns) const
{
    if (action_name.empty())
    {
        return std::string(MANIPULATOR_NS) + "/robotic_arm_cmd";
    }
    if (action_name.front() == '/')
    {
        return action_name;
    }
    if (under_manipulator_ns)
    {
        return std::string(MANIPULATOR_NS) + "/" + action_name;
    }
    return "/" + action_name;
}

rclcpp_action::Server<sealien_ctrlpilot_msgmanagement::action::RoboticArmCmd>::SharedPtr
ManipulatorRosInterface::createRoboticArmCmdActionServer(const std::string& action_path)
{
    return rclcpp_action::create_server<sealien_ctrlpilot_msgmanagement::action::RoboticArmCmd>(
        ctx_.action_client_node,
        action_path,
        [this](const rclcpp_action::GoalUUID& uuid,
               std::shared_ptr<const sealien_ctrlpilot_msgmanagement::action::RoboticArmCmd::Goal> goal) {
            return handleRoboticArmGoalRequest(uuid, goal);
        },
        [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<
                   sealien_ctrlpilot_msgmanagement::action::RoboticArmCmd>>& h) {
            return handleRoboticArmGoalCancel(h);
        },
        [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<
                   sealien_ctrlpilot_msgmanagement::action::RoboticArmCmd>>& h) {
            handleRoboticArmGoalAccepted(h);
        });
}

void ManipulatorRosInterface::loadRoboticArmCmdParams()
{
    if (!ctx_.action_client_node)
    {
        return;
    }
    rclcpp::Node* const node = ctx_.action_client_node.get();
    auto declare_or_get_bool = [node](const char* name, bool default_val) {
        const std::string key(name);
        if (!node->has_parameter(key))
        {
            node->declare_parameter(key, default_val);
        }
        return node->get_parameter(key).get_value<bool>();
    };
    auto declare_or_get_string = [node](const char* name, const std::string& default_val) {
        const std::string key(name);
        if (!node->has_parameter(key))
        {
            node->declare_parameter(key, default_val);
        }
        return node->get_parameter(key).get_value<std::string>();
    };
    auto declare_or_get_double = [node](const char* name, double default_val) {
        const std::string key(name);
        if (!node->has_parameter(key))
        {
            node->declare_parameter(key, default_val);
        }
        return node->get_parameter(key).get_value<double>();
    };

    robotic_arm_cmd_enable_ = declare_or_get_bool("robotic_arm_cmd.enable", true);
    robotic_arm_cmd_action_name_ = declare_or_get_string("robotic_arm_cmd.action_name", "robotic_arm_cmd");
    robotic_arm_cmd_register_root_alias_ =
        declare_or_get_bool("robotic_arm_cmd.register_root_alias", true);
    robotic_arm_cmd_verbose_ = declare_or_get_bool("robotic_arm_cmd.verbose", true);
    robotic_arm_cmd_use_cable_side_grasp_ = declare_or_get_bool("robotic_arm_cmd.use_cable_side_grasp", false);
    robotic_arm_cmd_reject_right_frames_ = declare_or_get_bool("robotic_arm_cmd.reject_right_arm_frames", true);
    robotic_arm_cmd_feedback_hz_ = declare_or_get_double("robotic_arm_cmd.feedback_hz", 10.0);
    robotic_arm_cmd_feedback_tcp_frame_ =
        declare_or_get_string("robotic_arm_cmd.feedback_tcp_frame", "gripper_tcp");
    if (robotic_arm_cmd_feedback_hz_ < 0.5)
    {
        robotic_arm_cmd_feedback_hz_ = 0.5;
    }
}

void ManipulatorRosInterface::logRoboticArmPhase(const char* phase, const std::string& detail) const
{
    if (detail.empty())
    {
        RCLCPP_INFO(ctx_.logger, "%s 阶段=%s", kRoboticArmCmdLogTag, phase);
    }
    else
    {
        RCLCPP_INFO(ctx_.logger, "%s 阶段=%s | %s", kRoboticArmCmdLogTag, phase, detail.c_str());
    }
}

void ManipulatorRosInterface::logRoboticArmPose(const char* phase,
                                                const geometry_msgs::msg::PoseStamped& pose) const
{
    if (!robotic_arm_cmd_verbose_)
    {
        logRoboticArmPhase(phase, "坐标系=" + pose.header.frame_id);
        return;
    }
    RCLCPP_INFO(ctx_.logger,
                "%s 阶段=%s | 坐标系=%s 位置=(%.4f, %.4f, %.4f) 四元数=(%.4f, %.4f, %.4f, %.4f)",
                kRoboticArmCmdLogTag,
                phase,
                pose.header.frame_id.c_str(),
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z,
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w);
}

void ManipulatorRosInterface::logRoboticArmOrderReceived(
    const sealien_ctrlpilot_msgmanagement::msg::RoboticArmRequest& order) const
{
    const char* task_verb =
        (order.type == robotic_arm_cmd::REQUEST_TYPE_INSERT) ? "插孔" : "抓取";
    RCLCPP_INFO(ctx_.logger,
                "%s 阶段=%s收到数据 | 类型=%s 坐标系=%s 位置=(%.4f, %.4f, %.4f) "
                "四元数=(%.4f, %.4f, %.4f, %.4f)",
                kRoboticArmCmdLogTag,
                task_verb,
                roboticArmRequestTypeName(order.type),
                order.header.frame_id.c_str(),
                order.keypoint.position.x,
                order.keypoint.position.y,
                order.keypoint.position.z,
                order.keypoint.orientation.x,
                order.keypoint.orientation.y,
                order.keypoint.orientation.z,
                order.keypoint.orientation.w);
}

void ManipulatorRosInterface::logRoboticArmPlanningPose(
    uint8_t request_type,
    const char* stage,
    const geometry_msgs::msg::PoseStamped& pose) const
{
    const char* task_verb =
        (request_type == robotic_arm_cmd::REQUEST_TYPE_INSERT) ? "插孔" : "抓取";
    RCLCPP_INFO(ctx_.logger,
                "%s 阶段=%s%s | 坐标系=%s 位置=(%.4f, %.4f, %.4f) 四元数=(%.4f, %.4f, %.4f, %.4f)",
                kRoboticArmCmdLogTag,
                task_verb,
                stage,
                pose.header.frame_id.c_str(),
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z,
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w);
}

bool ManipulatorRosInterface::isRoboticArmFrameAllowed(const std::string& frame_id) const
{
    if (frame_id.empty())
    {
        return false;
    }
    if (!robotic_arm_cmd_reject_right_frames_)
    {
        return true;
    }
    std::string lower = frame_id;
    std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return lower.find("right") == std::string::npos;
}

geometry_msgs::msg::PoseStamped ManipulatorRosInterface::roboticArmOrderToPoseStamped(
    const sealien_ctrlpilot_msgmanagement::msg::RoboticArmRequest& order)
{
    geometry_msgs::msg::PoseStamped out;
    out.header = order.header;
    if (out.header.frame_id.empty())
    {
        out.header.frame_id = "arm_base_link";
    }
    else if (out.header.frame_id == "base_link")
    {
        out.header.frame_id = "arm_base_link";
    }
    out.pose = order.keypoint;
    return out;
}

geometry_msgs::msg::Pose ManipulatorRosInterface::lookupTcpPoseInArmBaseLink() const
{
    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1.0;
    const std::shared_ptr<tf2_ros::Buffer> arm_tf =
        ctx_.manipulator_tf_buffer ? ctx_.manipulator_tf_buffer : ctx_.tf_buffer;
    if (!arm_tf)
    {
        return pose;
    }
    try
    {
        const geometry_msgs::msg::TransformStamped T = arm_tf->lookupTransform(
            "arm_base_link", robotic_arm_cmd_feedback_tcp_frame_, tf2::TimePointZero);
        pose.position.x = T.transform.translation.x;
        pose.position.y = T.transform.translation.y;
        pose.position.z = T.transform.translation.z;
        pose.orientation = T.transform.rotation;
    }
    catch (const std::exception& e)
    {
        RCLCPP_DEBUG(ctx_.logger, "lookupTcpPoseInArmBaseLink: %s", e.what());
    }
    return pose;
}

void ManipulatorRosInterface::runRoboticArmFeedbackLoop(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<sealien_ctrlpilot_msgmanagement::action::RoboticArmCmd>>&
        goal_handle,
    std::atomic<bool>& stop_flag) const
{
    const auto period = std::chrono::duration<double>(1.0 / robotic_arm_cmd_feedback_hz_);
    std::size_t tick = 0U;
    while (!stop_flag.load() && rclcpp::ok())
    {
        auto feedback = std::make_shared<sealien_ctrlpilot_msgmanagement::action::RoboticArmCmd::Feedback>();
        feedback->pose = lookupTcpPoseInArmBaseLink();
        goal_handle->publish_feedback(feedback);
        if (robotic_arm_cmd_verbose_ && (tick == 0U || tick % 20U == 0U))
        {
            RCLCPP_INFO(ctx_.logger,
                        "%s 阶段=反馈心跳 | 序号=%zu TCP位置=(%.4f, %.4f, %.4f)",
                        kRoboticArmCmdLogTag,
                        tick,
                        feedback->pose.position.x,
                        feedback->pose.position.y,
                        feedback->pose.position.z);
        }
        ++tick;
        std::this_thread::sleep_for(std::chrono::duration_cast<std::chrono::milliseconds>(period));
    }
    logRoboticArmPhase("反馈线程结束", "心跳次数=" + std::to_string(tick));
}

rclcpp_action::GoalResponse ManipulatorRosInterface::handleRoboticArmGoalRequest(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const sealien_ctrlpilot_msgmanagement::action::RoboticArmCmd::Goal> goal)
{
    logRoboticArmPhase("收到目标");
    if (!goal)
    {
        logRoboticArmPhase("拒绝目标", "目标为空");
        return rclcpp_action::GoalResponse::REJECT;
    }
    const auto& order = goal->order;
    logRoboticArmOrderReceived(order);
    logRoboticArmPhase("校验目标",
                       std::string("类型=") + roboticArmRequestTypeName(order.type) +
                           " 坐标系=" + order.header.frame_id);
    if (order.type != robotic_arm_cmd::REQUEST_TYPE_GRASP && order.type != robotic_arm_cmd::REQUEST_TYPE_INSERT)
    {
        logRoboticArmPhase("拒绝目标", "任务类型无效");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (!isRoboticArmFrameAllowed(order.header.frame_id))
    {
        logRoboticArmPhase("拒绝目标", "右臂坐标系不允许（仅支持左臂）");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (ctx_.task_manager->isEmergencyStopRequested())
    {
        logRoboticArmPhase("拒绝目标", "急停已触发");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (order.type == robotic_arm_cmd::REQUEST_TYPE_GRASP)
    {
        const RobotTaskMode mode = ctx_.task_manager->getMode();
        std::ostringstream oss;
        oss << "模式=" << roboticArmModeNameChinese(mode) << " 夹爪反馈=" << ctx_.left_arm_gripped->load()
            << " 夹爪锁定=" << (isGripperLocked() ? "是" : "否");
        logRoboticArmPhase("抓取前状态", oss.str());
        if (isHolding(mode))
        {
            logRoboticArmPhase("拒绝目标", "已在持物状态");
            return rclcpp_action::GoalResponse::REJECT;
        }
        if (isGripperLocked())
        {
            logRoboticArmPhase("拒绝目标", "夹爪已锁定有物");
            return rclcpp_action::GoalResponse::REJECT;
        }
        if (!canAcceptPick(mode))
        {
            logRoboticArmPhase("拒绝目标", "忙碌，无法接受抓取");
            return rclcpp_action::GoalResponse::REJECT;
        }
        if (ctx_.feasibility_checker && ctx_.feasibility_checker->inJoyModeSwitchIkCooldown())
        {
            logRoboticArmPhase("拒绝目标", "手柄自动/手动切换冷却中");
            return rclcpp_action::GoalResponse::REJECT;
        }
    }
    else
    {
        const RobotTaskMode mode = ctx_.task_manager->getMode();
        const bool held = ctx_.task_manager->getHeldObject().valid || isHolding(mode);
        std::ostringstream oss;
        oss << "模式=" << roboticArmModeNameChinese(mode) << " 已持物=" << (held ? "是" : "否")
            << " 夹爪锁定=" << (isGripperLocked() ? "是" : "否");
        logRoboticArmPhase("插孔前状态", oss.str());
        if (!held && !isGripperLocked())
        {
            logRoboticArmPhase("拒绝目标", "未持物，不能插孔");
            return rclcpp_action::GoalResponse::REJECT;
        }
        if (mode == RobotTaskMode::PICKING)
        {
            logRoboticArmPhase("拒绝目标", "正在抓取中");
            return rclcpp_action::GoalResponse::REJECT;
        }
    }
    logRoboticArmPhase("接受目标", roboticArmRequestTypeName(order.type));
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ManipulatorRosInterface::handleRoboticArmGoalCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<sealien_ctrlpilot_msgmanagement::action::RoboticArmCmd>>&)
{
    logRoboticArmPhase("取消请求", "不支持取消");
    return rclcpp_action::CancelResponse::REJECT;
}

void ManipulatorRosInterface::handleRoboticArmGoalAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<sealien_ctrlpilot_msgmanagement::action::RoboticArmCmd>>&
        goal_handle)
{
    logRoboticArmPhase("执行线程启动");
    std::thread([this, goal_handle]() {
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<sealien_ctrlpilot_msgmanagement::action::RoboticArmCmd::Result>();
        result->result = robotic_arm_cmd::RESULT_INVALID;

        if (!goal)
        {
            logRoboticArmPhase("执行中止", "工作线程内目标为空");
            goal_handle->abort(result);
            return;
        }

        logRoboticArmPhase("开始执行", roboticArmRequestTypeName(goal->order.type));

        if (ctx_.task_manager->isEmergencyStopRequested())
        {
            result->result = robotic_arm_cmd::RESULT_ESTOP;
            logRoboticArmPhase("执行中止", roboticArmResultName(result->result));
            goal_handle->abort(result);
            return;
        }

        if (!isRoboticArmFrameAllowed(goal->order.header.frame_id))
        {
            result->result = robotic_arm_cmd::RESULT_INVALID;
            logRoboticArmPhase("执行中止", "工作线程内坐标系不允许");
            goal_handle->abort(result);
            return;
        }

        const uint8_t request_type = goal->order.type;
        geometry_msgs::msg::PoseStamped stamped = roboticArmOrderToPoseStamped(goal->order);
        logRoboticArmPlanningPose(request_type, "规划输入位姿(变换前)", stamped);
        geometry_msgs::msg::PoseStamped stamped_plan = stamped;
        if (ctx_.task_manager->transformPoseToPlanningFrame(stamped_plan))
        {
            logRoboticArmPlanningPose(request_type, "规划输入位姿(变换后)", stamped_plan);
        }
        else
        {
            logRoboticArmPhase("变换未完成", "将依赖 handlePick/handleTargetInsert 内再次尝试 TF");
        }
        std::atomic<bool> stop_feedback{ false };
        std::thread feedback_thread;
        if (robotic_arm_cmd_feedback_hz_ > 0.0)
        {
            logRoboticArmPhase("反馈线程启动",
                               "频率=" + std::to_string(robotic_arm_cmd_feedback_hz_) + "Hz TCP坐标系=" +
                                   robotic_arm_cmd_feedback_tcp_frame_);
            feedback_thread = std::thread([this, goal_handle, &stop_feedback]() {
                runRoboticArmFeedbackLoop(goal_handle, stop_feedback);
            });
        }
        else
        {
            logRoboticArmPhase("反馈已关闭", "feedback_hz<=0");
        }

        bool ok = false;
        if (goal->order.type == robotic_arm_cmd::REQUEST_TYPE_GRASP)
        {
            GraspSource gs = GraspSource::TARGET_SENSOR;
            if (robotic_arm_cmd_use_cable_side_grasp_ && ctx_.object_axis_cache)
            {
                gs = GraspSource::LEGACY;
                tf2::Quaternion q;
                tf2::fromMsg(stamped.pose.orientation, q);
                const tf2::Matrix3x3 rot(q);
                tf2::Vector3 rod = rot.getColumn(2);
                if (rod.length() < 1e-9)
                {
                    rod = rot.getColumn(1);
                }
                if (rod.length() >= 1e-9)
                {
                    rod.normalize();
                    geometry_msgs::msg::Vector3Stamped axis;
                    axis.header = stamped.header;
                    axis.vector.x = rod.x();
                    axis.vector.y = rod.y();
                    axis.vector.z = rod.z();
                    ctx_.object_axis_cache->update(axis);
                    std::ostringstream oss;
                    oss << std::fixed << std::setprecision(4) << "杆轴=(" << axis.vector.x << ", " << axis.vector.y
                        << ", " << axis.vector.z << ")";
                    logRoboticArmPhase("已注入杆轴", oss.str());
                }
                else
                {
                    logRoboticArmPhase("跳过杆轴注入", "杆轴模长过小");
                }
            }
            logRoboticArmPlanningPose(request_type, "送入抓取规划", stamped_plan);
            logRoboticArmPhase("开始抓取规划", std::string("感知链=") + graspSourceName(gs));
            ok = ctx_.task_manager->handlePick(stamped, "external", gs);
            logRoboticArmPhase("抓取规划结束", ok ? "成功" : "失败");
        }
        else if (goal->order.type == robotic_arm_cmd::REQUEST_TYPE_INSERT)
        {
            logRoboticArmPlanningPose(request_type, "送入插孔规划", stamped_plan);
            logRoboticArmPhase("开始插孔规划");
            ok = ctx_.task_manager->handleTargetInsert(stamped, "external");
            logRoboticArmPhase("插孔规划结束", ok ? "成功" : "失败");
        }
        else
        {
            result->result = robotic_arm_cmd::RESULT_INVALID;
            logRoboticArmPhase("执行中止", "工作线程内任务类型无效");
            stop_feedback.store(true);
            if (feedback_thread.joinable())
            {
                feedback_thread.join();
            }
            goal_handle->abort(result);
            return;
        }

        stop_feedback.store(true);
        if (feedback_thread.joinable())
        {
            logRoboticArmPhase("等待反馈线程结束");
            feedback_thread.join();
        }

        result->result = ok ? robotic_arm_cmd::RESULT_SUCCESS : robotic_arm_cmd::RESULT_EXEC_FAILED;
        if (ctx_.task_manager->isEmergencyStopRequested())
        {
            result->result = robotic_arm_cmd::RESULT_ESTOP;
        }
        const auto fb_final = std::make_shared<sealien_ctrlpilot_msgmanagement::action::RoboticArmCmd::Feedback>();
        fb_final->pose = lookupTcpPoseInArmBaseLink();
        goal_handle->publish_feedback(fb_final);

        std::ostringstream finish_detail;
        finish_detail << "类型=" << roboticArmRequestTypeName(goal->order.type)
                      << " 结果=" << roboticArmResultName(result->result)
                      << " 任务号=" << ctx_.task_manager->getTaskId()
                      << " 模式=" << roboticArmModeNameChinese(ctx_.task_manager->getMode())
                      << " 最近错误=" << ctx_.task_manager->getLastError();
        if (ok)
        {
            logRoboticArmPhase("返回成功", finish_detail.str());
            goal_handle->succeed(result);
        }
        else
        {
            logRoboticArmPhase("返回失败", finish_detail.str());
            goal_handle->abort(result);
        }
        logRoboticArmPhase("执行线程结束");
    }).detach();
}

}  // namespace orion_mtc
