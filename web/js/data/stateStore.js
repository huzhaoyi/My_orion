/**
 * 全局状态存储：RuntimeStatus、队列、持物、感知、JobEvent、TaskStage、日志
 * 供各 Panel 订阅更新
 */

const initialState = {
  // 连接
  rosConnected: false,
  wsConnected: false,
  backendConnected: false,

  // RuntimeStatus 对应字段
  workerStatus: '',
  taskMode: '',
  currentJobId: '',
  currentJobType: '',
  nextJobType: '',
  nextJobId: '',
  workerRunning: false,
  queueEmpty: true,
  queueSize: 0,
  hasHeldObject: false,
  heldObjectId: '',
  heldSceneAttachId: '',
  lastError: '',

  // 当前任务卡片（来自当前执行 job）
  currentJobSource: '',
  currentJobPriority: 0,
  currentStageName: '',
  currentJobStartTime: null,
  currentJobCreatedAt: null,

  // 队列列表（简要信息，需后端提供或从 GetQueueState + GetRecentJobs 推导）
  queueList: [],

  // HeldObject 状态（HeldObjectState 话题）
  heldValid: false,
  heldTracked: false,
  heldSceneAttachId: '',
  heldAttachLink: '',
  heldObjectId: '',
  heldObjectPoseAtGrasp: null,
  heldTcpPoseAtGrasp: null,

  // 感知状态（来自话题 object_pose）
  objectPoseValid: false,
  perceptionUpdatedAt: null,
  objectPose: null,   // { position: {x,y,z}, orientation: {x,y,z,w} } base_link
  // 单缆绳：object_pose，无多目标集合
  rovPoseInBaseLink: null,  // ROV 在 base_link 下
  rovPoseInWorld: null,     // ROV 在世界系 (map) 下，来自 perception_state

  // 关节状态（来自 joint_states，驱动 3D）
  jointNames: [],
  jointPositions: [],

  // 轨迹点（用于 3D 显示，可选）
  trajectoryPoints: [],

  // Scene 一致性（便于调试）
  worldObjectPresent: false,
  attachedObjectPresent: false,
  heldTrackedPresent: false,
  heldUntrackedPresent: false,

  // 恢复（来自 /manipulator/recovery_event，RecoveryEvent.msg）
  lastRecoveryType: '',
  lastRecoverySuccess: false,
  lastRecoveryDetail: '',
  lastRecoveryTrigger: '',

  // 最近执行记录（来自 get_recent_jobs 服务，JobExecutionRecord[]）
  recentJobs: [],

  // 事件流
  jobEvents: [],
  taskStages: [],
  maxJobEvents: 100,
  maxTaskStages: 100,

  // 系统日志
  systemLogs: [],
  maxSystemLogs: 200,

  // 审批结果（CheckPick 最后一次结果）
  approvalResult: null,  // { type: 'pick', approved, severity, summary, items: [], best_candidate_pose?, at }
  approvalLoading: false,
  approvalTargetIndex: 1,   // 本次审批用的目标索引（0-based），默认第 2 个
  approvalTargetTotal: 0,   // 当前目标总数，用于显示「第 N 个（共 M 个）」
};

let state = { ...initialState };
const listeners = new Set();

function getState() {
  return { ...state };
}

function setState(partial) {
  state = { ...state, ...partial };
  listeners.forEach((fn) => fn(getState()));
}

function subscribe(fn) {
  listeners.add(fn);
  return () => listeners.delete(fn);
}

/* 与 orion_mtc_msgs/msg/RuntimeStatus.msg 字段一一对应（ROS 为 snake_case） */
function applyRuntimeStatus(msg) {
  const prev_job_id = state.currentJobId || '';
  const next_job_id =
    msg.current_job_id !== undefined && msg.current_job_id !== null
      ? String(msg.current_job_id)
      : prev_job_id;
  let next_stage = state.currentStageName;
  if (!next_job_id)
  {
    next_stage = '';
  }
  else if (prev_job_id && next_job_id && prev_job_id !== next_job_id)
  {
    next_stage = '';
  }
  setState({
    workerStatus: msg.worker_status ?? state.workerStatus,
    taskMode: msg.task_mode ?? state.taskMode,
    currentJobId: next_job_id,
    currentJobType: msg.current_job_type ?? state.currentJobType,
    nextJobType: msg.next_job_type ?? state.nextJobType,
    workerRunning: msg.worker_running ?? state.workerRunning,
    queueEmpty: msg.queue_empty ?? state.queueEmpty,
    queueSize: msg.queue_size ?? state.queueSize,
    hasHeldObject: msg.has_held_object ?? state.hasHeldObject,
    heldObjectId: msg.held_object_id ?? state.heldObjectId,
    heldSceneAttachId: msg.held_scene_attach_id ?? state.heldSceneAttachId,
    lastError: msg.last_error ?? state.lastError,
    currentStageName: next_stage,
  });
}

/* 与 orion_mtc_msgs/srv/GetQueueState.srv 响应用于同步 queueList / nextJobId */
function applyQueueStateResponse(res) {
  if (!res)
  {
    return;
  }
  const v = res.values || res;
  const list = [];
  if (v.current_job_id)
  {
    list.push({ job_id: v.current_job_id, job_type: v.current_job_type || '—', is_current: true });
  }
  if (v.next_job_id && v.next_job_id !== v.current_job_id)
  {
    list.push({ job_id: v.next_job_id, job_type: v.next_job_type || '—', is_current: false });
  }
  setState({
    queueList: list,
    nextJobId: v.next_job_id != null ? v.next_job_id : '',
    nextJobType: v.next_job_type != null ? v.next_job_type : '',
    queueSize: v.queue_size != null ? v.queue_size : 0,
    queueEmpty: v.queue_empty != null ? v.queue_empty : true,
  });
}

/* 与 orion_mtc_msgs/msg/HeldObjectState.msg 字段一一对应 */
function applyHeldObjectState(msg) {
  setState({
    heldValid: msg.valid ?? state.heldValid,
    heldTracked: msg.tracked ?? state.heldTracked,
    heldObjectId: msg.object_id ?? state.heldObjectId,
    heldSceneAttachId: msg.scene_attach_id ?? state.heldSceneAttachId,
    heldAttachLink: msg.attach_link ?? state.heldAttachLink,
    heldObjectPoseAtGrasp: msg.object_pose_at_grasp ?? state.heldObjectPoseAtGrasp,
    heldTcpPoseAtGrasp: msg.tcp_pose_at_grasp ?? state.heldTcpPoseAtGrasp,
  });
}

/* 与 orion_mtc_msgs/msg/RecoveryEvent.msg 字段一一对应 */
function applyRecoveryEvent(msg) {
  setState({
    lastRecoveryType: msg.recovery_type ?? state.lastRecoveryType,
    lastRecoverySuccess: msg.success ?? state.lastRecoverySuccess,
    lastRecoveryDetail: msg.detail ?? state.lastRecoveryDetail,
    lastRecoveryTrigger: msg.trigger_reason ?? state.lastRecoveryTrigger,
  });
}

function setRecentJobs(list) {
  setState({ recentJobs: Array.isArray(list) ? list : [] });
}

/* 与 orion_mtc_msgs/srv/GetRobotState.srv 响应字段一一对应 */
function applyGetRobotStateResponse(res) {
  if (!res) return;
  setState({
    taskMode: res.mode != null ? res.mode : state.taskMode,
    currentJobId: res.task_id != null ? res.task_id : state.currentJobId,
    heldObjectId: res.held_object_id != null ? res.held_object_id : state.heldObjectId,
    hasHeldObject: res.has_held_object != null ? res.has_held_object : state.hasHeldObject,
    lastError: res.last_error != null ? res.last_error : state.lastError,
  });
}

function pushJobEvent(event) {
  const list = [...state.jobEvents, event].slice(-state.maxJobEvents);
  setState({ jobEvents: list });
}

function pushTaskStage(stage) {
  const list = [...state.taskStages, stage].slice(-state.maxTaskStages);
  setState({ taskStages: list });
}

function pushSystemLog(level, message, meta = {}) {
  const entry = {
    ts: new Date().toISOString(),
    level,
    message,
    ...meta,
  };
  const list = [...state.systemLogs, entry].slice(-state.maxSystemLogs);
  setState({ systemLogs: list });
}

function setQueueList(list) {
  setState({ queueList: list });
}

function setConnection(which, value) {
  const partial = {};
  if (which === 'ros') partial.rosConnected = value;
  if (which === 'ws') partial.wsConnected = value;
  if (which === 'backend') partial.backendConnected = value;
  setState(partial);
}

function setObjectPose(poseStampedOrNull) {
  if (!poseStampedOrNull) {
    setState({ objectPose: null, objectPoseValid: false });
    return;
  }
  const pose = poseStampedOrNull.pose || poseStampedOrNull;
  setState({
    objectPose: {
      position: pose.position || { x: 0, y: 0, z: 0 },
      orientation: pose.orientation || { x: 0, y: 0, z: 0, w: 1 },
    },
    objectPoseValid: true,
    perceptionUpdatedAt: Date.now(),
  });
}

function setJointState(names, positions) {
  setState({ jointNames: names || [], jointPositions: positions || [] });
}

function setTrajectoryPoints(points) {
  setState({ trajectoryPoints: points || [] });
}

function setRovPoseInBaseLink(poseStampedOrNull) {
  if (!poseStampedOrNull) {
    setState({ rovPoseInBaseLink: null });
    return;
  }
  const pose = poseStampedOrNull.pose || poseStampedOrNull;
  setState({
    rovPoseInBaseLink: {
      position: pose.position || { x: 0, y: 0, z: 0 },
      orientation: pose.orientation || { x: 0, y: 0, z: 0, w: 1 },
    },
  });
}

function _isMeaningfulPosition(pos) {
  if (!pos) return false;
  const x = pos.x != null ? Number(pos.x) : 0;
  const y = pos.y != null ? Number(pos.y) : 0;
  const z = pos.z != null ? Number(pos.z) : 0;
  const eps = 1e-6;
  return Math.abs(x) > eps || Math.abs(y) > eps || Math.abs(z) > eps;
}

/* 从 /manipulator/perception_state 一次更新：物体位姿、ROV 在 base_link 下位姿、多目标集合 */
function setPerceptionState(msg) {
  if (!msg) return;
  const patch = { perceptionUpdatedAt: Date.now() };
  /* 仅在有意义的物体位姿时更新，避免 (0,0,0) 覆盖导致 0↔有数据 闪烁 */
  if (msg.object_pose && msg.object_pose.pose) {
    const p = msg.object_pose.pose;
    const pos = p.position || { x: 0, y: 0, z: 0 };
    if (_isMeaningfulPosition(pos)) {
      patch.objectPose = {
        position: pos,
        orientation: p.orientation || { x: 0, y: 0, z: 0, w: 1 },
      };
      patch.objectPoseValid = true;
    }
  }
  if (msg.rov_pose_in_base_link && msg.rov_pose_in_base_link.pose) {
    const p = msg.rov_pose_in_base_link.pose;
    patch.rovPoseInBaseLink = {
      position: p.position || { x: 0, y: 0, z: 0 },
      orientation: p.orientation || { x: 0, y: 0, z: 0, w: 1 },
    };
  } else {
    patch.rovPoseInBaseLink = null;
  }
  if (msg.rov_pose_in_world && msg.rov_pose_in_world.pose) {
    const p = msg.rov_pose_in_world.pose;
    patch.rovPoseInWorld = {
      position: p.position || { x: 0, y: 0, z: 0 },
      orientation: p.orientation || { x: 0, y: 0, z: 0, w: 1 },
    };
  } else {
    patch.rovPoseInWorld = null;
  }
  setState(patch);
}

function setApprovalResult(payload) {
  setState({
    approvalResult: payload == null ? null : {
      type: payload.type || 'pick',
      approved: payload.approved,
      severity: payload.severity != null ? payload.severity : 0,
      summary: payload.summary || '',
      items: Array.isArray(payload.items) ? payload.items : [],
      best_candidate_pose: payload.best_candidate_pose || null,
      at: Date.now(),
    },
  });
}

export default {
  getState,
  setState,
  subscribe,
  applyRuntimeStatus,
  applyQueueStateResponse,
  applyHeldObjectState,
  pushJobEvent,
  pushTaskStage,
  pushSystemLog,
  setQueueList,
  setConnection,
  setObjectPose,
  setJointState,
  setTrajectoryPoints,
  setRovPoseInBaseLink,
  setPerceptionState,
  applyRecoveryEvent,
  setRecentJobs,
  applyGetRobotStateResponse,
  setApprovalResult,
};
