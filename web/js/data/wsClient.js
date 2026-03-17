/**
 * WebSocket 客户端：连接 rosbridge，与 ROS2 orion_mtc 一一对应
 * 话题（与 orion_mtc_node 发布一致）：
 *   /manipulator/runtime_status   (RuntimeStatus)
 *   /manipulator/job_event        (JobEvent)   — 注意单数
 *   /manipulator/task_stage       (TaskStage)  — 注意单数
 *   /manipulator/held_object_state (HeldObjectState)
 *   /manipulator/object_pose, /manipulator/place_pose (PoseStamped，来自 CableSensor 缆绳单目标)
 *   /manipulator/perception_state (PerceptionState：物体+ROV+多目标，供感知卡片与 3D 显示)
 *   /joint_states                 (JointState，通常由 robot_state_publisher 发布)
 * 服务（与 orion_mtc_node 一致）：
 *   /manipulator/get_robot_state, get_queue_state, submit_job, cancel_job,
 *   open_gripper, close_gripper, reset_held_object, sync_held_object
 */

import stateStore from './stateStore.js';

// 默认 WS 使用当前页面主机（外机访问时自动连到该主机上的 rosbridge）
function getDefaultWsUrl() {
  const host = typeof window !== 'undefined' && window.location && window.location.hostname
    ? window.location.hostname
    : 'localhost';
  return 'ws://' + host + ':9090';
}

let ws = null;
let reconnectTimer = null;
let reconnectAttempts = 0;
const MAX_RECONNECT_ATTEMPTS = 10;

function getWsUrl() {
  const query = typeof window !== 'undefined' && window.location ? window.location.search : '';
  const params = new URLSearchParams(query);
  const fromQuery = params.get('ws');
  if (fromQuery) return fromQuery;
  return getDefaultWsUrl();
}

function getTopicPrefix() {
  const params = new URLSearchParams(window.location.search);
  return params.get('ns') || params.get('topic_prefix') || '/manipulator';
}

function connect() {
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
  };

  ws.onclose = () => {
    stateStore.setConnection('ws', false);
    stateStore.pushSystemLog('warn', 'WebSocket 已断开');
    scheduleReconnect();
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

function scheduleReconnect() {
  if (reconnectTimer) clearTimeout(reconnectTimer);
  if (reconnectAttempts >= MAX_RECONNECT_ATTEMPTS) return;
  reconnectAttempts += 1;
  const delay = Math.min(1000 * Math.pow(2, reconnectAttempts), 30000);
  stateStore.pushSystemLog('info', `${delay / 1000}s 后尝试重连 (${reconnectAttempts}/${MAX_RECONNECT_ATTEMPTS})`);
  reconnectTimer = setTimeout(connect, delay);
}

function subscribeTopics() {
  const prefix = getTopicPrefix();
  const topics = [
    prefix + '/runtime_status',
    prefix + '/job_event',
    prefix + '/task_stage',
    prefix + '/held_object_state',
    prefix + '/object_pose',
    prefix + '/place_pose',
    prefix + '/perception_state',
    prefix + '/joint_states',
    '/joint_states',
  ];
  const seen = new Set();
  topics.forEach((topic) => {
    if (seen.has(topic)) return;
    seen.add(topic);
    send({
      op: 'subscribe',
      topic,
      type: inferType(topic),
    });
  });
}

function inferType(topic) {
  if (topic.includes('runtime_status')) return 'orion_mtc_msgs/msg/RuntimeStatus';
  if (topic.includes('job_event')) return 'orion_mtc_msgs/msg/JobEvent';
  if (topic.includes('task_stage')) return 'orion_mtc_msgs/msg/TaskStage';
  if (topic.includes('held_object_state')) return 'orion_mtc_msgs/msg/HeldObjectState';
  if (topic.includes('perception_state')) return 'orion_mtc_msgs/msg/PerceptionState';
  if (topic.includes('object_pose') || topic.includes('place_pose')) return 'geometry_msgs/msg/PoseStamped';
  if (topic.includes('joint_states')) return 'sensor_msgs/msg/JointState';
  return 'std_msgs/msg/String';
}

function handleMessage(data) {
  if (!data.topic || !data.msg) return;
  if (data.topic.endsWith('/runtime_status')) {
    stateStore.applyRuntimeStatus(data.msg);
    return;
  }
  if (data.topic.endsWith('/job_event')) {
    stateStore.pushJobEvent({ ...data.msg, _ts: Date.now() });
    return;
  }
  if (data.topic && data.topic.endsWith('/task_stage') && data.msg) {
    const stage = { ...data.msg, _ts: Date.now() };
    stateStore.pushTaskStage(stage);
    const st = (stage.stage_state || '').toUpperCase();
    if (st === 'ENTER' || st === 'RUNNING') {
      stateStore.setState({ currentStageName: stage.stage_name || '' });
    }
    return;
  }
  if (data.topic && data.topic.endsWith('/held_object_state') && data.msg) {
    stateStore.applyHeldObjectState(data.msg);
    return;
  }
  if (data.topic && data.topic.endsWith('/object_pose') && data.msg) {
    stateStore.setObjectPose(data.msg);
    return;
  }
  if (data.topic && data.topic.endsWith('/place_pose') && data.msg) {
    stateStore.setPlacePose(data.msg);
    return;
  }
  if (data.topic && data.topic.endsWith('/joint_states') && data.msg) {
    const names = data.msg.name || [];
    const pos = data.msg.position || [];
    stateStore.setJointState(names, pos);
    return;
  }
  if (data.topic && data.topic.endsWith('/perception_state') && data.msg) {
    stateStore.setPerceptionState(data.msg);
    return;
  }
}

function send(obj) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify(obj));
  }
}

function callService(service, request = {}, callback, options = {}) {
  const id = 'srv_' + Date.now() + '_' + Math.random().toString(36).slice(2);
  const timeoutMs = Number.isFinite(options.timeout_ms) ? options.timeout_ms : 3000;
  let timeoutTimer = null;
  const handler = (ev) => {
    try {
      const data = JSON.parse(ev.data);
      if (data.id === id) {
        ws.removeEventListener('message', handler);
        if (timeoutTimer) clearTimeout(timeoutTimer);
        const raw = data.result || data;
        const result = raw && raw.values ? raw.values : raw;
        if (callback) callback(result);
      }
    } catch (_) {}
  };
  ws.addEventListener('message', handler);
  timeoutTimer = setTimeout(() => {
    try {
      ws.removeEventListener('message', handler);
    } catch (_) {}
    stateStore.pushSystemLog('warn', `服务调用超时: ${service} (${timeoutMs}ms)`);
    if (options.on_timeout) options.on_timeout();
  }, timeoutMs);
  send({
    op: 'call_service',
    id,
    service,
    args: request,
  });
}

const JOB_TYPE = { PICK: 0, PLACE: 1, PLACE_RELEASE: 2, RESET_HELD_OBJECT: 3, SYNC_HELD_OBJECT: 4 };

function buildPoseStamped(position, orientation, frameId = 'base_link') {
  return {
    header: { frame_id: frameId, stamp: { sec: 0, nanosec: 0 } },
    pose: {
      position: position || { x: 0, y: 0, z: 0 },
      orientation: orientation || { x: 0, y: 0, z: 0, w: 1 },
    },
  };
}

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
  } = options;
  const args = {
    job_type,
    job_id,
    object_id,
    tracked,
    priority,
  };
  if (target_pose) args.target_pose = target_pose.header ? target_pose : buildPoseStamped(target_pose.pose?.position || target_pose.position, target_pose.pose?.orientation || target_pose.orientation);
  if (object_pose) args.object_pose = object_pose.header ? object_pose : buildPoseStamped(object_pose.pose?.position || object_pose.position, object_pose.pose?.orientation || object_pose.orientation);
  if (tcp_pose) args.tcp_pose = tcp_pose.pose || tcp_pose;
  callService(getTopicPrefix() + '/submit_job', args, callback);
}

function getQueueState(callback) {
  callService(getTopicPrefix() + '/get_queue_state', {}, callback);
}

function getRobotState(callback) {
  callService(getTopicPrefix() + '/get_robot_state', {}, callback);
}

function checkPick(objectPose, callback) {
  const pose = objectPose && objectPose.pose
    ? objectPose
    : { header: { frame_id: 'base_link' }, pose: objectPose || { position: { x: 0, y: 0, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } } };
  if (!pose.header) pose.header = { frame_id: 'base_link' };
  callService(getTopicPrefix() + '/check_pick', { object_pose: pose }, callback, { timeout_ms: 5000 });
}

function checkPlace(placePose, hasHeldObject, callback) {
  const pose = placePose && placePose.pose
    ? placePose
    : { header: { frame_id: 'base_link' }, pose: placePose || { position: { x: 0, y: 0, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } } };
  if (!pose.header) pose.header = { frame_id: 'base_link' };
  callService(getTopicPrefix() + '/check_place', { place_pose: pose, has_held_object: !!hasHeldObject }, callback, { timeout_ms: 5000 });
}

function disconnect() {
  if (reconnectTimer) {
    clearTimeout(reconnectTimer);
    reconnectTimer = null;
  }
  reconnectAttempts = MAX_RECONNECT_ATTEMPTS;
  if (ws) {
    ws.close();
    ws = null;
  }
  stateStore.setConnection('ws', false);
}

export default {
  connect,
  disconnect,
  send,
  callService,
  getTopicPrefix,
  isConnected: () => ws && ws.readyState === WebSocket.OPEN,
  JOB_TYPE,
  buildPoseStamped,
  submitJob,
  getQueueState,
  getRobotState,
  checkPick,
  checkPlace,
};
