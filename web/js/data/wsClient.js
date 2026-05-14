/**
 * WebSocket 客户端：连接 rosbridge，与 ROS2 orion_mtc 一一对应
 * 话题（与 orion_mtc_node 发布一致）：
 *   /manipulator/runtime_status   (RuntimeStatus)
 *   /manipulator/job_event        (JobEvent)   — 注意单数
 *   /manipulator/task_stage       (TaskStage)  — 注意单数
 *   /manipulator/held_object_state (HeldObjectState)
 *   /manipulator/object_pose（原抓取方式：各类感知桥接）
 *   /manipulator/object_pose_fused（视觉+声呐 Keypoints 中心线）
 *   /manipulator/keypoints_base_link（geometry_msgs/PoseArray：各关键点已变换到 base_link 等，供 UI）
 *   /manipulator/perception_state (PerceptionState：物体+ROV+多目标，供感知卡片与 3D 显示)
 *   /manipulator/target_set (TargetSet：TargetSensor 多目标 base_link，与 MTC 一致)
 *   /manipulator/target_insert_holes (PoseArray：插孔位姿，供前端/RViz 调试显示)
 *   /manipulator/web/joint_states (JointState；Web 降频通道，默认 15Hz，可用 ?joint_states_topic= 覆盖)
 *   /joy_manipulator/manual_mode   (std_msgs/Bool 手柄手动=true)
 *   /joy_manipulator/throttle_percent (std_msgs/Float32 臂油门 0～100，可选 ?joy_ui= 改前缀)
 * 服务（与 orion_mtc_node 一致）：
 *   /manipulator/get_robot_state, get_queue_state, get_recent_jobs, submit_job, cancel_job,
 *   open_gripper, close_gripper, emergency_stop, clear_estop, go_to_ready（std_srvs/Trigger）,
 *   reset_held_object, sync_held_object, check_pick
 */

import stateStore from './stateStore.js';

/** 页面为 localhost / file / 空 host 时用回环；否则 WS 与页面同主机（同机开网页 + rosbridge 时局域网浏览器可连）。其它机器上的 rosbridge 用 ?ws= 覆盖。 */
function isLocalPageHost(hostname) {
  if (hostname == null || hostname === '') {
    return true;
  }
  const h = String(hostname).toLowerCase();
  return h === 'localhost' || h === '127.0.0.1' || h === '[::1]' || h === '::1';
}

function getDefaultWsUrl() {
  if (typeof window === 'undefined' || !window.location) {
    return 'ws://127.0.0.1:9091';
  }
  const hostname = window.location.hostname;
  if (isLocalPageHost(hostname)) {
    return 'ws://127.0.0.1:9091';
  }
  const scheme = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
  return `${scheme}//${hostname}:9091`;
}

let ws = null;
let reconnectTimer = null;
let reconnectAttempts = 0;
const MAX_RECONNECT_ATTEMPTS = 10;
/** true：整页卸载/刷新，不再 scheduleReconnect，并主动 close，减轻 rosbridge 对已死连接写 1Hz 时的 WARN */
let pageUnloading = false;
/** 当前连接已发送 subscribe 的话题列表；断开前尽量先 unsubscribe，降低 rosbridge 告警。 */
let activeSubscriptions = [];
/** get_queue_state 单飞：并发请求只保留一次“补拉”机会，避免服务堆积与乱序覆盖。 */
let queueStateInFlight = false;
let queueStateNeedRefresh = false;
let queueStateCallbacks = [];

function flushQueueStateCallbacks(result) {
  const callbacks = queueStateCallbacks.slice();
  queueStateCallbacks = [];
  callbacks.forEach((cb) => {
    try {
      cb(result);
    } catch (_) {
      /* ignore callback errors */
    }
  });
}

function runQueueStateRequest() {
  if (queueStateInFlight) {
    return;
  }
  if (!ws || ws.readyState !== WebSocket.OPEN) {
    return;
  }
  queueStateInFlight = true;
  callService(
    getTopicPrefix() + '/get_queue_state',
    {},
    (res) => {
      queueStateInFlight = false;
      flushQueueStateCallbacks(res);
      if (queueStateNeedRefresh) {
        queueStateNeedRefresh = false;
        runQueueStateRequest();
      }
    },
    {
      service_missing_retries: 10,
      on_timeout: () => {
        queueStateInFlight = false;
        flushQueueStateCallbacks(null);
        if (queueStateNeedRefresh) {
          queueStateNeedRefresh = false;
          runQueueStateRequest();
        }
      },
    }
  );
}

function handlePageLeave() {
  pageUnloading = true;
  if (reconnectTimer) {
    clearTimeout(reconnectTimer);
    reconnectTimer = null;
  }
  if (ws) {
    unsubscribeTopics();
    try {
      ws.close(1000, 'page leave');
    } catch (_) {
      /* ignore */
    }
    ws = null;
  }
  activeSubscriptions = [];
  queueStateInFlight = false;
  queueStateNeedRefresh = false;
  queueStateCallbacks = [];
  stateStore.setConnection('ws', false);
}

if (typeof window !== 'undefined') {
  window.addEventListener('pagehide', handlePageLeave);
  window.addEventListener('beforeunload', handlePageLeave);
}

/** `?ws=` 可覆写完整 WebSocket URL，否则用 getDefaultWsUrl。 */
function getWsUrl() {
  const query = typeof window !== 'undefined' && window.location ? window.location.search : '';
  const params = new URLSearchParams(query);
  const fromQuery = params.get('ws');
  if (fromQuery) return fromQuery;
  return getDefaultWsUrl();
}

/** ROS 话题/服务根前缀，默认 `/manipulator`；`?ns=` 或 `?topic_prefix=` 覆盖。 */
function getTopicPrefix() {
  const params = new URLSearchParams(window.location.search);
  return params.get('ns') || params.get('topic_prefix') || '/manipulator';
}

/**
 * Keypoints PoseArray 订阅全名；默认 `{prefix}/keypoints_base_link`。
 * `?keypoints_topic=/a/b` 或 `?keypoints=/a/b` 覆盖（须与 keypoint_to_arm_tf 的 keypoints_posearray_topic 一致）。
 */
function getKeypointsSubscribeTopic() {
  const params = new URLSearchParams(typeof window !== 'undefined' && window.location ? window.location.search : '');
  const custom = params.get('keypoints_topic') || params.get('keypoints');
  if (custom) {
    const t = String(custom).trim();
    if (t.length === 0) {
      /* fall through */
    } else if (t.startsWith('/')) {
      return t.replace(/\/+$/, '');
    } else {
      return '/' + t.replace(/\/+$/, '');
    }
  }
  const p = String(getTopicPrefix()).replace(/\/+$/, '');
  return `${p}/keypoints_base_link`;
}

/** 最近一次订阅的 keypoints 话题全名（供 inferType / handleMessage 对齐）。 */
let subscribedKeypointsTopic = '';

/** 手柄桥接 UI 状态话题前缀，默认 /joy_manipulator（`?joy_ui=` 覆盖）。 */
function getJoyUiPrefix() {
  const params = new URLSearchParams(typeof window !== 'undefined' && window.location ? window.location.search : '');
  const p = params.get('joy_ui');
  if (p) return p.replace(/\/$/, '');
  return '/joy_manipulator';
}

/** Web 侧 JointState 订阅话题；默认 /manipulator/web/joint_states，可用 ?joint_states_topic= 覆盖。 */
function getWebJointStatesTopic() {
  const params = new URLSearchParams(typeof window !== 'undefined' && window.location ? window.location.search : '');
  const custom = params.get('joint_states_topic');
  if (custom) {
    const t = String(custom).trim();
    if (t.length > 0) {
      return t.startsWith('/') ? t.replace(/\/+$/, '') : '/' + t.replace(/\/+$/, '');
    }
  }
  return '/manipulator/web/joint_states';
}

/** 建立 WebSocket、订阅业务话题、断线指数退避重连。 */
function connect() {
  pageUnloading = false;
  const url = getWsUrl();
  if (ws && (ws.readyState === WebSocket.OPEN || ws.readyState === WebSocket.CONNECTING)) {
    return;
  }
  try {
    ws = new WebSocket(url);
  } catch (e) {
    stateStore.pushSystemLog('error', 'WebSocket 创建失败: ' + e.message);
    stateStore.setConnection('ws', false);
    scheduleReconnect();
    return;
  }

  ws.onopen = () => {
    reconnectAttempts = 0;
    stateStore.setConnection('ws', true);
    stateStore.pushSystemLog('info', 'WebSocket 已连接: ' + url);
    subscribeTopics();
    getQueueState((res) => {
      stateStore.applyQueueStateResponse(res);
    });
    stateStore.pushSystemLog('info', `订阅 Keypoints 轨迹: ${subscribedKeypointsTopic}（geometry_msgs/PoseArray）`);
  };

  ws.onclose = (ev) => {
    stateStore.setConnection('ws', false);
    activeSubscriptions = [];
    queueStateInFlight = false;
    queueStateNeedRefresh = false;
    queueStateCallbacks = [];
    stateStore.setJoyBridgeManual(null);
    stateStore.setJoyBridgeThrottle(null);
    stateStore.setState({
      keypointsTrace: null,
      keypointsTraceValid: false,
      keypointsTraceUpdatedAt: null,
    });
    if (!pageUnloading) {
      const code = (ev && Number.isFinite(ev.code)) ? ev.code : 0;
      const reason = (ev && typeof ev.reason === 'string' && ev.reason.length > 0) ? ev.reason : 'n/a';
      stateStore.pushSystemLog('warn', `WebSocket 已断开 (code=${code}, reason=${reason})`);
      scheduleReconnect();
    }
  };

  ws.onerror = (ev) => {
    stateStore.pushSystemLog('error', 'WebSocket 错误');
  };

  ws.onmessage = (ev) => {
    try {
      const data = JSON.parse(ev.data);
      handleMessage(data);
    } catch (e) {
      stateStore.pushSystemLog('error', '消息解析失败: ' + e.message);
    }
  };
}

/** 指数退避重连 WebSocket，上限 MAX_RECONNECT_ATTEMPTS 次。 */
function scheduleReconnect() {
  if (reconnectTimer) clearTimeout(reconnectTimer);
  if (reconnectAttempts >= MAX_RECONNECT_ATTEMPTS) return;
  reconnectAttempts += 1;
  const delay = Math.min(500 * Math.pow(2, Math.max(0, reconnectAttempts - 1)), 30000);
  stateStore.pushSystemLog('info', `${delay / 1000}s 后尝试重连 (${reconnectAttempts}/${MAX_RECONNECT_ATTEMPTS})`);
  reconnectTimer = setTimeout(connect, delay);
}

/** 与 subscribedKeypointsTopic 匹配（rosbridge 发来的 topic 有时带/不带尾缀，以全名或后缀为准）。 */
function isKeypointsTraceTopic(topic) {
  if (!topic || typeof topic !== 'string') {
    return false;
  }
  if (subscribedKeypointsTopic && topic === subscribedKeypointsTopic) {
    return true;
  }
  const base = '/keypoints_base_link';
  return topic === base || topic.endsWith(base);
}

/** 与 subscribeTopics 相同顺序、已去重的话题全名（供 UI 列出订阅 coverage）。 */
function getSubscribedTopicsFlat() {
  const prefix = getTopicPrefix();
  const joyUi = getJoyUiPrefix();
  const kp = getKeypointsSubscribeTopic();
  const webJointStates = getWebJointStatesTopic();
  const topics = [
    prefix + '/runtime_status',
    prefix + '/job_event',
    prefix + '/task_stage',
    prefix + '/held_object_state',
    prefix + '/object_pose_fused',
    kp,
    prefix + '/object_pose',
    prefix + '/perception_state',
    prefix + '/target_set',
    prefix + '/target_insert_holes',
    webJointStates,
    joyUi + '/manual_mode',
    joyUi + '/throttle_percent',
  ];
  const seen = new Set();
  const out = [];
  topics.forEach((topic) => {
    if (seen.has(topic)) {
      return;
    }
    seen.add(topic);
    out.push(topic);
  });
  return out;
}

/** 向 rosbridge 发送 subscribe：manipulator 状态/感知/全局 joint_states/手柄 UI（无 recovery、无 /manipulator/joint_states）。 */
function subscribeTopics() {
  subscribedKeypointsTopic = getKeypointsSubscribeTopic();
  const topics = getSubscribedTopicsFlat();
  activeSubscriptions = topics.slice();
  topics.forEach((topic) => {
    send({
      op: 'subscribe',
      topic,
      type: inferType(topic),
    });
  });
}

/** 在连接关闭前主动取消订阅，缩短 rosbridge 清理关闭连接的窗口。 */
function unsubscribeTopics() {
  if (!ws || ws.readyState !== WebSocket.OPEN) {
    return;
  }
  const topics = activeSubscriptions.length > 0 ? activeSubscriptions : getSubscribedTopicsFlat();
  topics.forEach((topic) => {
    send({
      op: 'unsubscribe',
      topic,
    });
  });
}

/** 按话题名片段推断 ROS 消息类型字符串（供 rosbridge subscribe）。 */
function inferType(topic) {
  if (topic.includes('runtime_status')) return 'orion_mtc_msgs/msg/RuntimeStatus';
  if (topic.includes('job_event')) return 'orion_mtc_msgs/msg/JobEvent';
  if (topic.includes('task_stage')) return 'orion_mtc_msgs/msg/TaskStage';
  if (topic.includes('held_object_state')) return 'orion_mtc_msgs/msg/HeldObjectState';
  if (topic.includes('perception_state')) return 'orion_mtc_msgs/msg/PerceptionState';
  if (topic.endsWith('/target_set')) return 'orion_mtc_msgs/msg/TargetSet';
  if (topic.endsWith('/target_insert_holes')) return 'geometry_msgs/msg/PoseArray';
  if (topic.endsWith('/object_pose_fused')) return 'geometry_msgs/msg/PoseStamped';
  if (subscribedKeypointsTopic && topic === subscribedKeypointsTopic) {
    return 'geometry_msgs/msg/PoseArray';
  }
  if (topic.endsWith('/keypoints_base_link')) return 'geometry_msgs/msg/PoseArray';
  if (topic.includes('object_pose')) return 'geometry_msgs/msg/PoseStamped';
  if (topic.includes('joint_states')) return 'sensor_msgs/msg/JointState';
  if (topic.endsWith('/manual_mode')) return 'std_msgs/msg/Bool';
  if (topic.endsWith('/throttle_percent')) return 'std_msgs/msg/Float32';
  return 'std_msgs/msg/String';
}

/** 单条 rosbridge 入站：分发到 stateStore（runtime_status、job_event、task_stage、持物、位姿、关节等）。 */
function handleMessage(data) {
  if (!data.topic || !data.msg) return;
  /* joint_states 高频：收包时间戳与关节值在 setJointState 内合并或节流，避免每帧两次 setState。 */
  if (!data.topic.endsWith('/joint_states')) {
    stateStore.touchRosTopicRx(data.topic);
  }
  if (data.topic.endsWith('/manual_mode') && Object.prototype.hasOwnProperty.call(data.msg, 'data')) {
    stateStore.setJoyBridgeManual(data.msg.data === true);
    return;
  }
  if (data.topic.endsWith('/throttle_percent') && data.msg.data != null) {
    const v = Number(data.msg.data);
    if (Number.isFinite(v)) stateStore.setJoyBridgeThrottle(v);
    return;
  }
  if (data.topic.endsWith('/runtime_status')) {
    const prev_job = stateStore.getState().currentJobId || '';
    stateStore.applyRuntimeStatus(data.msg);
    const next_job = stateStore.getState().currentJobId || '';
    if (prev_job !== next_job) {
      getQueueState((res) => {
        stateStore.applyQueueStateResponse(res);
      });
    }
    return;
  }
  if (data.topic.endsWith('/job_event')) {
    stateStore.pushJobEvent({ ...data.msg, _ts: Date.now() });
    const et = ((data.msg && data.msg.event_type) || '').toUpperCase();
    if (['SUCCEEDED', 'FAILED', 'CANCELLED', 'REJECTED'].includes(et)) {
      stateStore.setState({ currentStageName: '', currentStageDetail: '' });
      getQueueState((res) => {
        stateStore.applyQueueStateResponse(res);
      });
    }
    return;
  }
  if (data.topic && data.topic.endsWith('/task_stage') && data.msg) {
    const stage = { ...data.msg, _ts: Date.now() };
    stateStore.pushTaskStage(stage);
    const tt = (stage.task_type || '').toUpperCase();
    const st = (stage.stage_state || '').toUpperCase();
    const name = stage.stage_name || '';
    const detail = stage.detail || '';
    const patch = {};
    if (tt === 'CHECK_PICK') {
      if (st === 'DONE' || name === 'check_pick_idle') {
        patch.checkPickProgress = null;
      } else {
        patch.checkPickProgress = { stageName: name, detail };
      }
    }
    if (st === 'ENTER' || st === 'RUNNING') {
      if (tt !== 'CHECK_PICK') {
        patch.currentStageName = name;
        patch.currentStageDetail = detail;
      }
    }
    if (Object.keys(patch).length) {
      stateStore.setState(patch);
    }
    return;
  }
  if (data.topic && data.topic.endsWith('/held_object_state') && data.msg) {
    stateStore.applyHeldObjectState(data.msg);
    return;
  }
  if (data.topic && data.topic.endsWith('/object_pose_fused') && data.msg) {
    stateStore.setFusedObjectPose(data.msg);
    return;
  }
  if (data.msg && isKeypointsTraceTopic(data.topic)) {
    stateStore.setKeypointsTrace(data.msg);
    return;
  }
  if (data.topic && data.topic.endsWith('/object_pose') && data.msg) {
    stateStore.setObjectPose(data.msg);
    return;
  }
  if (data.topic && data.topic.endsWith('/joint_states') && data.msg) {
    const names = data.msg.name || [];
    const pos = data.msg.position || [];
    stateStore.setJointState(names, pos, data.topic);
    return;
  }
  if (data.topic && data.topic.endsWith('/perception_state') && data.msg) {
    stateStore.setPerceptionState(data.msg);
    return;
  }
  if (data.topic && data.topic.endsWith('/target_set') && data.msg) {
    stateStore.setTargetSet(data.msg);
    return;
  }
  if (data.topic && data.topic.endsWith('/target_insert_holes') && data.msg) {
    stateStore.setTargetInsertHoles(data.msg);
    return;
  }
}

/** OPEN 状态下 JSON.stringify 写出；否则静默丢弃。 */
function send(obj) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify(obj));
  }
}

/** 发布任意 ROS 话题（rosbridge op=publish）。 */
function publishTopic(topic, msg = {}) {
  send({
    op: 'publish',
    topic,
    msg,
  });
}

/** 发布 std_msgs/Empty（msg 为 {}）。 */
function publishEmpty(topic) {
  publishTopic(topic, {});
}

/** 与 open_gripper 相同：std_srvs/Trigger，args 为空对象 */
function callEmergencyStop(callback) {
  callService(getTopicPrefix() + '/emergency_stop', {}, callback);
}

/** 解除急停闭锁：对应后端 /clear_estop（不恢复队列）。 */
function callClearEstop(callback) {
  callService(getTopicPrefix() + '/clear_estop', {}, callback);
}

/** 回 ready 可能较久（规划+执行），延长超时 */
function callGoToReady(callback) {
  callService(getTopicPrefix() + '/go_to_ready', {}, callback, { timeout_ms: 120000 });
}

/**
 * rosbridge call_service：生成 id、监听匹配回包或超时；callback 收到扁平化 result.values。
 * options.timeout_ms、options.on_timeout 可选。
 * options.service_missing_retries：>0 时若回包含「服务不存在」类错误则退避重试（减轻 launch 竞态）。
 */
function callService(service, request = {}, callback, options = {}) {
  const missing_max = Number(options.service_missing_retries);
  let missing_retries_left = Number.isFinite(missing_max) && missing_max > 0 ? Math.floor(missing_max) : 0;

  const timeoutMs = Number.isFinite(options.timeout_ms) ? options.timeout_ms : 3000;

  const run = () => {
    if (!ws || ws.readyState !== WebSocket.OPEN) {
      return;
    }
    const id = 'srv_' + Date.now() + '_' + Math.random().toString(36).slice(2);
    let timeoutTimer = null;
    const handler = (ev) => {
      try {
        const data = JSON.parse(ev.data);
        if (data.id !== id) {
          return;
        }
        ws.removeEventListener('message', handler);
        if (timeoutTimer) {
          clearTimeout(timeoutTimer);
        }
        const envelope = JSON.stringify(data);
        const service_missing = /does not exist|InvalidServiceException|Service .* does not exist/i.test(envelope);
        if (service_missing && missing_retries_left > 0) {
          missing_retries_left -= 1;
          const attempt_n = missing_max - missing_retries_left;
          const delay = Math.min(500 + attempt_n * 400, 4000);
          setTimeout(run, delay);
          return;
        }
        if (service_missing) {
          stateStore.pushSystemLog('warn', `服务仍不可用: ${service}（已达到重试上限或无法解析回包）`);
        }
        const raw = data.result || data;
        const result = raw && raw.values ? raw.values : raw;
        if (callback) {
          callback(result);
        }
      } catch (_) {
        /* ignore malformed chunks */
      }
    };
    ws.addEventListener('message', handler);
    timeoutTimer = setTimeout(() => {
      try {
        ws.removeEventListener('message', handler);
      } catch (_) {
        /* ignore */
      }
      stateStore.pushSystemLog('warn', `服务调用超时: ${service} (${timeoutMs}ms)`);
      if (options.on_timeout) {
        options.on_timeout();
      }
    }, timeoutMs);
    send({
      op: 'call_service',
      id,
      service,
      args: request,
    });
  };

  run();
}

const JOB_TYPE = {
  PICK: 0,
  RESET_HELD_OBJECT: 1,
  SYNC_HELD_OBJECT: 2,
  OPEN_GRIPPER: 3,
  CLOSE_GRIPPER: 4,
  TARGET_INSERT: 5,
};

/** 与 ManipulationJob.grasp_source 一致：0=LEGACY，1=FUSED，2=TARGET_SENSOR。 */
const GRASP_SOURCE = {
  LEGACY: 0,
  FUSED: 1,
  TARGET_SENSOR: 2,
};

/** 构造简化 geometry_msgs/PoseStamped 字面量（供 submit_job 嵌套）。 */
function buildPoseStamped(position, orientation, frameId = 'base_link') {
  return {
    header: { frame_id: frameId, stamp: { sec: 0, nanosec: 0 } },
    pose: {
      position: position || { x: 0, y: 0, z: 0 },
      orientation: orientation || { x: 0, y: 0, z: 0, w: 1 },
    },
  };
}

/** 调用 /submit_job：从 options 组装 job_type、位姿与优先级。 */
function submitJob(options, callback) {
  const {
    job_type,
    job_id = '',
    target_pose = null,
    object_pose = null,
    tcp_pose = null,
    object_id = '',
    tracked = false,
    priority = 0,
    grasp_source = GRASP_SOURCE.LEGACY,
  } = options;
  const args = {
    job_type,
    job_id,
    object_id,
    tracked,
    priority,
    grasp_source,
  };
  if (target_pose) args.target_pose = target_pose.header ? target_pose : buildPoseStamped(target_pose.pose?.position || target_pose.position, target_pose.pose?.orientation || target_pose.orientation);
  if (object_pose) args.object_pose = object_pose.header ? object_pose : buildPoseStamped(object_pose.pose?.position || object_pose.position, object_pose.pose?.orientation || object_pose.orientation);
  if (tcp_pose) args.tcp_pose = tcp_pose.pose || tcp_pose;
  callService(getTopicPrefix() + '/submit_job', args, callback);
}

/** get_queue_state 服务，回调队列与 next job 摘要（带服务未就绪重试，与 pick_holoocean 启动时序配合）。 */
function getQueueState(callback) {
  if (!ws || ws.readyState !== WebSocket.OPEN) {
    return;
  }
  if (callback) {
    queueStateCallbacks.push(callback);
  }
  if (queueStateInFlight) {
    queueStateNeedRefresh = true;
    return;
  }
  runQueueStateRequest();
}

/** get_recent_jobs：max_count 钳位 1～200，超时 5s。 */
function getRecentJobs(maxCount, callback) {
  const n = Math.max(1, Math.min(200, Number(maxCount) || 50));
  callService(getTopicPrefix() + '/get_recent_jobs', { max_count: n }, callback, { timeout_ms: 5000 });
}

/** check_pick：object_pose 可为 PoseStamped 或裸 pose，服务超时 5s。 */
function checkPick(objectPose, callback) {
  const pose = objectPose && objectPose.pose
    ? objectPose
    : { header: { frame_id: 'base_link' }, pose: objectPose || { position: { x: 0, y: 0, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } } };
  if (!pose.header) pose.header = { frame_id: 'base_link' };
  callService(getTopicPrefix() + '/check_pick', { object_pose: pose }, callback, { timeout_ms: 5000 });
}

/** 停止重连、关闭 WebSocket、标记未连接（不 rclpy.shutdown）。 */
function disconnect() {
  if (reconnectTimer) {
    clearTimeout(reconnectTimer);
    reconnectTimer = null;
  }
  reconnectAttempts = MAX_RECONNECT_ATTEMPTS;
  if (ws) {
    unsubscribeTopics();
    ws.close();
    ws = null;
  }
  activeSubscriptions = [];
  queueStateInFlight = false;
  queueStateNeedRefresh = false;
  queueStateCallbacks = [];
  stateStore.setConnection('ws', false);
}

export default {
  connect,
  disconnect,
  send,
  callEmergencyStop,
  callClearEstop,
  callGoToReady,
  callService,
  publishTopic,
  publishEmpty,
  getTopicPrefix,
  getKeypointsSubscribeTopic,
  getJoyUiPrefix,
  getSubscribedTopicsFlat,
  isConnected: () => ws && ws.readyState === WebSocket.OPEN,
  JOB_TYPE,
  GRASP_SOURCE,
  buildPoseStamped,
  submitJob,
  getQueueState,
  getRecentJobs,
  checkPick,
};
