#include "orion_mtc/interface/manipulator_ros_interface.hpp"
#include "orion_mtc/core/constants.hpp"
#include "orion_mtc/perception/perception_snapshot.hpp"
#include "orion_mtc/core/manipulation_job.hpp"
#include "orion_mtc/core/runtime_status.hpp"
#include "orion_mtc/core/task_state.hpp"
#include "orion_mtc/core/job_result_code.hpp"
#include <builtin_interfaces/msg/time.hpp>
#include <orion_mtc_msgs/msg/job_execution_record.hpp>
#include <chrono>
#include <cstdint>
#include <thread>

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
}  // namespace

namespace orion_mtc
{

ManipulatorRosInterface::ManipulatorRosInterface(ManipulatorInterfaceContext ctx)
  : ctx_(std::move(ctx))
{
}

void ManipulatorRosInterface::registerSubscriptionsAndServices()
{
    const std::string ns(MANIPULATOR_NS);
    sub_object_pose_ = ctx_.action_client_node->create_subscription<geometry_msgs::msg::PoseStamped>(
        ns + "/object_pose", 10, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            ctx_.object_pose_cache->update(*msg);
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
    sub_pick_trigger_ = ctx_.action_client_node->create_subscription<std_msgs::msg::Empty>(
        ns + "/pick_trigger", 10, [this](const std_msgs::msg::Empty::SharedPtr msg) {
            onPickTriggerReceived(msg);
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
}

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
    msg.last_error = ctx_.task_manager->getLastError();
    pub_runtime_status_->publish(msg);
}

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

bool ManipulatorRosInterface::isGripperLocked() const
{
    const double threshold = 0.5;
    return ctx_.left_arm_gripped->load() >= threshold;
}

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
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ManipulatorRosInterface::handlePickGoalCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<orion_mtc_msgs::action::Pick>>&)
{
    return rclcpp_action::CancelResponse::REJECT;
}

void ManipulatorRosInterface::handlePickGoalAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<orion_mtc_msgs::action::Pick>>& goal_handle)
{
    std::thread([this, goal_handle]() {
        const auto goal = goal_handle->get_goal();
        bool ok = ctx_.task_manager->handlePick(goal->object_pose,
                                                goal->object_id.empty() ? "object" : goal->object_id);
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

void ManipulatorRosInterface::handleSubmitJob(
    const std::shared_ptr<orion_mtc_msgs::srv::SubmitJob::Request> req,
    std::shared_ptr<orion_mtc_msgs::srv::SubmitJob::Response> res)
{
    if (req->job_type > 4u)
    {
        res->success = false;
        res->message = "invalid job_type (0=PICK,1=RESET_HELD_OBJECT,2=SYNC_HELD_OBJECT,3=OPEN_GRIPPER,4=CLOSE_GRIPPER)";
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
    job.object_id = req->object_id;
    job.tracked = req->tracked;
    job.priority = req->priority;
    job.target_pose = req->target_pose;
    job.object_pose = req->object_pose;
    job.tcp_pose = req->tcp_pose;
    job.source = "submit_job_service";
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

void ManipulatorRosInterface::handleCancelJob(
    const std::shared_ptr<orion_mtc_msgs::srv::CancelJob::Request> req,
    std::shared_ptr<orion_mtc_msgs::srv::CancelJob::Response> res)
{
    std::string message;
    bool ok = ctx_.task_manager->cancelJob(req->job_id, &message);
    res->success = ok;
    res->message = message;
}

void ManipulatorRosInterface::handleResetHeldObject(
    const std::shared_ptr<orion_mtc_msgs::srv::ResetHeldObject::Request>,
    std::shared_ptr<orion_mtc_msgs::srv::ResetHeldObject::Response> res)
{
    res->success = ctx_.task_manager->handleResetHeldObject(res->message);
}

void ManipulatorRosInterface::handleSyncHeldObject(
    const std::shared_ptr<orion_mtc_msgs::srv::SyncHeldObject::Request> req,
    std::shared_ptr<orion_mtc_msgs::srv::SyncHeldObject::Response> res)
{
    res->success = ctx_.task_manager->handleSyncHeldObject(
        req->set_holding, req->tracked, req->object_id, req->object_pose, req->tcp_pose, res->message);
}

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

void ManipulatorRosInterface::handleEmergencyStopService(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                                         std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    ctx_.task_manager->requestEmergencyStop();
    res->success = true;
    res->message = "emergency_stop";
    RCLCPP_WARN(ctx_.logger, "service emergency_stop: cancel trajectories + clear queue");
}

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

}  // namespace orion_mtc
