/**
 * 全局状态存储：RuntimeStatus、队列、持物、感知、JobEvent、TaskStage、日志
 * 供各 Panel 订阅更新
 */

const initialState = {
  wsConnected: false,

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
  objectPose: null,   // { position: {x,y,z}, orientation: {x,y,z,w} } base_link，与 /object_pose 一致
  cableObjectPoseValid: false,
  cableObjectPose: null,   // perception_state.cable_object_pose（缆绳）
  targetSensorObjectPoseValid: false,
  targetSensorObjectPose: null,   // perception_state.target_sensor_object_pose（TargetSensor）
  targetSensorSelectedIndex: -1, // perception_state.target_sensor_selected_index；-1 无
  targetSetTargets: [],          // /target_set 解析：{ index, objectId, position, orientation }[]
  targetSetValid: false,
  targetSetUpdatedAt: null,
  targetInsertHolePoses: [],     // /target_insert_holes: base_link 孔位 PoseArray
  targetInsertHolesValid: false,
  targetInsertHolesUpdatedAt: null,
  // 视觉+声呐中心线融合链：/manipulator/object_pose_fused
  fusedObjectPoseValid: false,
  fusedObjectPose: null,
  fusedPerceptionUpdatedAt: null,
  // keypoint_to_arm_tf 发布的 PoseArray（与 object_pose_fused 同 output_grasp_frame，默认 base_link）
  keypointsTraceValid: false,
  keypointsTrace: null,  // { frameId: string, points: [{x,y,z}] }
  keypointsTraceUpdatedAt: null,
  // 单缆绳：object_pose，无多目标集合
  rovPoseInBaseLink: null,  // ROV 在 base_link 下
  rovPoseInWorld: null,     // ROV 在世界系 (map) 下，来自 perception_state

  /* 感知相关话题最后一次收到 rosbridge 消息的时间戳（ms，与 data.topic 全名一致） */
  rosTopicLastRxAt: {},

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

  // joy_manipulator_node 上位机状态（/joy_manipulator/manual_mode、throttle_percent）
  joyManualMode: null,      // null 未收到；true 手动；false 自动
  joyThrottlePercent: null, // null 未收到；0～100 臂油门比例
  joyThrottlePulseSeq: 0,   // 油门数值变化时递增，供顶栏短脉冲动画
};

let state = { ...initialState };
const listeners = new Set();
/* 仅刷新「ROS 话题收包时间」列表面板，避免 touchRosTopicRx 触发全量 subscribe（拖慢 3D / 关节表）。 */
const rosTopicRxListeners = new Set();

/** 浅拷贝当前 state（避免外部直接改引用时可再封装）。 */
function getState() {
  return { ...state };
}

/** 合并 partial 并通知全部 subscribe 监听者。 */
function setState(partial) {
  state = { ...state, ...partial };
  listeners.forEach((fn) => fn(getState()));
}

/** 注册渲染回调；返回 unsubscribe 函数。 */
function subscribe(fn) {
  listeners.add(fn);
  return () => listeners.delete(fn);
}

/** 仅订阅话题最后收包时间变化（不经过全量 setState）。 */
function subscribeRosTopicRx(fn) {
  rosTopicRxListeners.add(fn);
  return () => rosTopicRxListeners.delete(fn);
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
  if (which !== 'ws') {
    return;
  }
  const partial = { wsConnected: value };
  if (!value) {
    partial.rosTopicLastRxAt = {};
    partial.targetSetTargets = [];
    partial.targetSetValid = false;
    partial.targetSetUpdatedAt = null;
    partial.targetInsertHolePoses = [];
    partial.targetInsertHolesValid = false;
    partial.targetInsertHolesUpdatedAt = null;
  }
  setState(partial);
}

/** 与 wsClient 列表一致：统一带前导 /，避免 rosbridge 与订阅名不一致导致对不上。 */
function normalizeRosTopicKey(topic) {
  if (!topic || typeof topic !== 'string')
  {
    return '';
  }
  const t = topic.trim();
  if (!t)
  {
    return '';
  }
  return t.startsWith('/') ? t : `/${t}`;
}

/** 记录某 ROS 话题收到一帧（键名为 normalize 后的全路径）；只通知 rosTopicRxListeners，不触发全量面板刷新。 */
function touchRosTopicRx(topic) {
  const key = normalizeRosTopicKey(topic);
  if (!key)
  {
    return;
  }
  if (!state.rosTopicLastRxAt)
  {
    state.rosTopicLastRxAt = {};
  }
  state.rosTopicLastRxAt[key] = Date.now();
  rosTopicRxListeners.forEach((fn) => {
    try
    {
      fn();
    }
    catch (err)
    {
      /* 单面板回调异常不影响其它订阅者 */
    }
  });
}

/*
 * rosbridge 将 geometry_msgs 序列化为 JSON 时常把 float 编成 string；
 * PerceptionCard 用 typeof === 'number' 会整段显示为「—」。此处统一成 number。
 */
function coercePoseFromMsg(poseStampedOrNull) {
  if (!poseStampedOrNull || typeof poseStampedOrNull !== 'object') {
    return null;
  }
  const pose = poseStampedOrNull.pose || poseStampedOrNull;
  const pos = pose.position;
  if (!pos || pos.x === undefined || pos.y === undefined || pos.z === undefined) {
    return null;
  }
  const x = Number(pos.x);
  const y = Number(pos.y);
  const z = Number(pos.z);
  if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) {
    return null;
  }
  const orient = pose.orientation || {};
  const ox = Number(orient.x);
  const oy = Number(orient.y);
  const oz = Number(orient.z);
  const owRaw = Number(orient.w);
  const quatOk =
    Number.isFinite(ox) &&
    Number.isFinite(oy) &&
    Number.isFinite(oz) &&
    Number.isFinite(owRaw);
  const q = quatOk
    ? { x: ox, y: oy, z: oz, w: owRaw }
    : { x: 0.0, y: 0.0, z: 0.0, w: 1.0 };
  return {
    position: { x, y, z },
    orientation: q,
  };
}

function setObjectPose(poseStampedOrNull) {
  if (!poseStampedOrNull) {
    setState({ objectPose: null, objectPoseValid: false });
    return;
  }
  const coerced = coercePoseFromMsg(poseStampedOrNull);
  if (!coerced) {
    setState({ objectPose: null, objectPoseValid: false });
    return;
  }
  setState({
    objectPose: coerced,
    objectPoseValid: true,
    perceptionUpdatedAt: Date.now(),
  });
}

let _keypointsTraceWarnAt = 0;

function _warnKeypointsTraceParseOnce(message) {
  const now = Date.now();
  if (now - _keypointsTraceWarnAt < 10000) {
    return;
  }
  _keypointsTraceWarnAt = now;
  pushSystemLog('warn', message);
}

/** rosbridge 常见：float 编成 string、poses 偶发为非数组对象。 */
function setKeypointsTrace(poseArrayMsg) {
  const raw = poseArrayMsg;
  if (!raw || typeof raw !== 'object') {
    setState({
      keypointsTrace: null,
      keypointsTraceValid: false,
    });
    _warnKeypointsTraceParseOnce('keypoints_base_link: 消息为空或非对象，无法显示点列');
    return;
  }
  let poses = raw.poses;
  if (poses != null && typeof poses === 'object' && !Array.isArray(poses)) {
    poses = Object.values(poses);
  }
  if (!Array.isArray(poses)) {
    setState({
      keypointsTrace: null,
      keypointsTraceValid: false,
    });
    _warnKeypointsTraceParseOnce(
      'keypoints_base_link: 无 poses 数组（请确认 rosbridge 类型 geometry_msgs/PoseArray 与 topic 一致）'
    );
    return;
  }
  const frameId =
    raw.header && raw.header.frame_id != null ? String(raw.header.frame_id) : 'base_link';

  const numOrNull = (v) => {
    const n = v === undefined || v === null ? NaN : Number(v);
    return Number.isFinite(n) ? n : null;
  };

  const points = poses
    .map((p) => {
      if (!p || typeof p !== 'object') {
        return null;
      }
      const pos = p.position;
      if (!pos || typeof pos !== 'object') {
        return null;
      }
      const x = numOrNull(pos.x);
      const y = numOrNull(pos.y);
      const z = numOrNull(pos.z);
      if (x === null || y === null || z === null) {
        return null;
      }
      return { x, y, z };
    })
    .filter((pt) => pt != null);

  if (points.length === 0) {
    setState({
      keypointsTrace: null,
      keypointsTraceValid: false,
    });
    if (poses.length > 0) {
      _warnKeypointsTraceParseOnce(
        'keypoints_base_link: poses 有元素但 position 无法解析为数字（检查 rosbridge JSON）'
      );
    }
    return;
  }

  setState({
    keypointsTrace: { frameId, points },
    keypointsTraceValid: true,
    keypointsTraceUpdatedAt: Date.now(),
  });
}

function setFusedObjectPose(poseStampedOrNull) {
  if (!poseStampedOrNull) {
    setState({
      fusedObjectPose: null,
      fusedObjectPoseValid: false,
    });
    return;
  }
  const coerced = coercePoseFromMsg(poseStampedOrNull);
  if (!coerced) {
    setState({
      fusedObjectPose: null,
      fusedObjectPoseValid: false,
    });
    return;
  }
  setState({
    fusedObjectPose: coerced,
    fusedObjectPoseValid: true,
    fusedPerceptionUpdatedAt: Date.now(),
  });
}

const JOINT_POS_EPS = 1e-5;
let _jointRafId = null;
let _pendingJointFlush = null;

function _effectiveJointNamesPositions() {
  if (_pendingJointFlush)
  {
    return {
      names: _pendingJointFlush.names,
      pos: _pendingJointFlush.positions,
    };
  }
  return {
    names: state.jointNames || [],
    pos: state.jointPositions || [],
  };
}

function _flushPendingJointState() {
  _jointRafId = null;
  const p = _pendingJointFlush;
  _pendingJointFlush = null;
  if (!p)
  {
    return;
  }
  const partial = { jointNames: p.names, jointPositions: p.positions };
  if (p.topicForRx)
  {
    const key = normalizeRosTopicKey(p.topicForRx);
    if (key)
    {
      partial.rosTopicLastRxAt = { ...(state.rosTopicLastRxAt || {}), [key]: Date.now() };
    }
  }
  setState(partial);
}

function _jointsUnchanged(prevNames, prevPos, nextNames, nextPos) {
  const pn = prevNames || [];
  const pp = prevPos || [];
  const nn = nextNames || [];
  const np = nextPos || [];
  if (pn.length !== nn.length || pp.length !== np.length || nn.length !== np.length) {
    return false;
  }
  for (let i = 0; i < nn.length; i += 1) {
    if (pn[i] !== nn[i]) {
      return false;
    }
    const a = Number(pp[i]);
    const b = Number(np[i]);
    if (!Number.isFinite(a) || !Number.isFinite(b)) {
      return false;
    }
    if (Math.abs(a - b) > JOINT_POS_EPS) {
      return false;
    }
  }
  return true;
}

/**
 * @param {string[]} names
 * @param {number[]} positions
 * @param {string} [topicForRx] 若为 joint_states 全路径，则与关节在同一拍合并刷新 rosTopicLastRxAt
 */
function setJointState(names, positions, topicForRx) {
  const n = names || [];
  const p = (positions || []).map((v) => {
    const x = Number(v);
    return Number.isFinite(x) ? x : 0.0;
  });
  const eff = _effectiveJointNamesPositions();
  if (_jointsUnchanged(eff.names, eff.pos, n, p))
  {
    if (topicForRx)
    {
      touchRosTopicRx(topicForRx);
    }
    return;
  }
  _pendingJointFlush = {
    names: n,
    positions: p,
    topicForRx: topicForRx || null,
  };
  if (_jointRafId == null)
  {
    _jointRafId = requestAnimationFrame(_flushPendingJointState);
  }
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

/* 从 /manipulator/perception_state 一次更新：分源位姿、ROV；未带某字段时不覆盖该源（避免双节点交替清空） */
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
  if (msg.cable_object_pose && msg.cable_object_pose.pose) {
    const cp = msg.cable_object_pose.pose;
    const cpos = cp.position || { x: 0, y: 0, z: 0 };
    if (_isMeaningfulPosition(cpos)) {
      patch.cableObjectPose = {
        position: cpos,
        orientation: cp.orientation || { x: 0, y: 0, z: 0, w: 1 },
      };
      patch.cableObjectPoseValid = true;
    }
  }
  if (msg.target_sensor_object_pose && msg.target_sensor_object_pose.pose) {
    const tp = msg.target_sensor_object_pose.pose;
    const tpos = tp.position || { x: 0, y: 0, z: 0 };
    if (_isMeaningfulPosition(tpos)) {
      patch.targetSensorObjectPose = {
        position: tpos,
        orientation: tp.orientation || { x: 0, y: 0, z: 0, w: 1 },
      };
      patch.targetSensorObjectPoseValid = true;
    }
  }
  if (msg.target_sensor_selected_index !== undefined && msg.target_sensor_selected_index !== null) {
    const si = Number(msg.target_sensor_selected_index);
    patch.targetSensorSelectedIndex = Number.isFinite(si) ? si : -1;
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

/** /manipulator/target_set：多目标 base_link 位姿表（与 MTC TargetSelector 同源）。 */
function setTargetSet(msg) {
  if (!msg) {
    setState({
      targetSetTargets: [],
      targetSetValid: false,
      targetSetUpdatedAt: null,
    });
    return;
  }
  const raw = msg.targets;
  if (!Array.isArray(raw) || raw.length === 0) {
    setState({
      targetSetTargets: [],
      targetSetValid: false,
      targetSetUpdatedAt: Date.now(),
    });
    return;
  }
  const ids = Array.isArray(msg.object_ids) ? msg.object_ids : [];
  const rows = raw.map((pst, i) => {
    const pose = pst && pst.pose ? pst.pose : pst;
    const pos = pose && pose.position ? pose.position : { x: 0, y: 0, z: 0 };
    const orient = pose && pose.orientation ? pose.orientation : { x: 0, y: 0, z: 0, w: 1 };
    return {
      index: i,
      objectId: ids[i] != null && ids[i] !== '' ? String(ids[i]) : '',
      position: { x: Number(pos.x), y: Number(pos.y), z: Number(pos.z) },
      orientation: {
        x: Number(orient.x),
        y: Number(orient.y),
        z: Number(orient.z),
        w: Number(orient.w),
      },
    };
  });
  setState({
    targetSetTargets: rows,
    targetSetValid: true,
    targetSetUpdatedAt: Date.now(),
  });
}

function setTargetInsertHoles(poseArrayMsg) {
  const raw = poseArrayMsg;
  if (!raw || !Array.isArray(raw.poses) || raw.poses.length === 0) {
    setState({
      targetInsertHolePoses: [],
      targetInsertHolesValid: false,
      targetInsertHolesUpdatedAt: Date.now(),
    });
    return;
  }
  const rows = raw.poses
    .map((p) => {
      const pos = p && p.position ? p.position : null;
      if (!pos) {
        return null;
      }
      const x = Number(pos.x);
      const y = Number(pos.y);
      const z = Number(pos.z);
      if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) {
        return null;
      }
      const q = p.orientation || {};
      const qx = Number(q.x);
      const qy = Number(q.y);
      const qz = Number(q.z);
      const qw = Number(q.w);
      return {
        position: { x, y, z },
        orientation: {
          x: Number.isFinite(qx) ? qx : 0.0,
          y: Number.isFinite(qy) ? qy : 0.0,
          z: Number.isFinite(qz) ? qz : 0.0,
          w: Number.isFinite(qw) ? qw : 1.0,
        },
      };
    })
    .filter((v) => v != null);
  setState({
    targetInsertHolePoses: rows,
    targetInsertHolesValid: rows.length > 0,
    targetInsertHolesUpdatedAt: Date.now(),
  });
}

/** 写入最后一次 CheckPick 结构化结果（含 items、best_candidate_pose）。 */
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

/** /joy_manipulator/manual_mode：null 表示尚未收到。 */
function setJoyBridgeManual(isManual) {
  if (isManual === null || isManual === undefined)
  {
    if (state.joyManualMode === null)
    {
      return;
    }
    setState({ joyManualMode: null });
    return;
  }
  const next = isManual === true;
  if (state.joyManualMode === next)
  {
    return;
  }
  setState({ joyManualMode: next });
}

/** 臂油门 0～100；变化超阈值时递增 joyThrottlePulseSeq 供顶栏动画。 */
function setJoyBridgeThrottle(percent) {
  if (percent === null || percent === undefined)
  {
    if (state.joyThrottlePercent === null)
    {
      return;
    }
    setState({ joyThrottlePercent: null });
    return;
  }
  const p = Number(percent);
  if (!Number.isFinite(p))
  {
    return;
  }
  const clamped = Math.max(0.0, Math.min(100.0, p));
  const prev = state.joyThrottlePercent;
  let seq = state.joyThrottlePulseSeq;
  if (prev !== null && Number.isFinite(prev) && Math.abs(clamped - prev) >= 0.25)
  {
    seq = seq + 1;
  }
  if (
    prev !== null
    && Number.isFinite(prev)
    && Math.abs(clamped - prev) < 0.02
    && seq === state.joyThrottlePulseSeq
  )
  {
    return;
  }
  setState({ joyThrottlePercent: clamped, joyThrottlePulseSeq: seq });
}

export default {
  getState,
  setState,
  subscribe,
  subscribeRosTopicRx,
  applyRuntimeStatus,
  applyQueueStateResponse,
  applyHeldObjectState,
  pushJobEvent,
  pushTaskStage,
  pushSystemLog,
  setQueueList,
  setConnection,
  setObjectPose,
  setFusedObjectPose,
  setKeypointsTrace,
  setJointState,
  setTrajectoryPoints,
  setRovPoseInBaseLink,
  setPerceptionState,
  setTargetSet,
  setTargetInsertHoles,
  setRecentJobs,
  applyGetRobotStateResponse,
  setApprovalResult,
  setJoyBridgeManual,
  setJoyBridgeThrottle,
  touchRosTopicRx,
  normalizeRosTopicKey,
};
