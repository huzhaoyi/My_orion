/**
 * 中英 UI 文案：localStorage + window 事件同步各面板
 * 键名使用点分层，与业务模块松耦合
 */

const STORAGE_KEY = 'orion_ui_locale';

const MESSAGES = {
  zh: {
    'app.title': 'Orion 机械臂上位机',
    'lang.zh': '中文',
    'lang.en': 'English',
    'lang.switch': '语言',

    'top.brand': 'Orion',
    'top.connected': '已连接',
    'top.disconnected': '未连接',
    'top.ws_title_disconnected': '需先启动 rosbridge（默认 ws://127.0.0.1:9091，可用 ?ws= 覆盖）',
    'top.worker': '工作线程',
    'top.task': '任务',
    'top.queue': '队列',
    'top.joy_unknown': '手柄 —',
    'top.manual': '手动',
    'top.auto': '自动',
    'top.joy_title': 'joy_manipulator：/joy_manipulator/manual_mode',
    'top.throttle_unknown': '油门 —',
    'top.throttle_title': '臂油门 0～100%（/joy_manipulator/throttle_percent）；颜色随档位变化，变化时短暂高亮',
    'top.clear_queue': '清空队列',
    'top.clear_queue_title': 'cancel_job 清空待执行任务',
    'top.reset_held': '重置持物',
    'top.reset_held_title': 'reset_held_object',

    'status.idle': '空闲',
    'status.running': '运行中',
    'status.holding': '持物中',
    'status.error': '异常',
    'status.recovering': '恢复中',

    'right.tab_task': '任务',
    'right.tab_debug': '调试',
    'right.emergency_ready_title': '急停与回位',
    'right.emergency_ready_hint': 'emergency_stop / go_to_ready 服务',
    'right.emergency_stop': '急停',
    'right.go_ready': '回 ready',
    'right.approval_title': '审批结果',
    'right.approval_hint': 'check_pick 服务；无物体位姿时不可用',
    'right.approval_btn': '审批抓取',
    'right.pick_title': '抓取（二选一）',
    'right.pick_legacy': '原抓取方式',
    'right.pick_legacy_title': '原先一路：感知/缆绳/重建等桥接到 /manipulator/object_pose（与 Keypoints 无关）',
    'right.pick_fused': '视觉+声呐中心线',
    'right.pick_fused_title': '视觉 + 声呐 Keypoints → 中心线拟合 → /manipulator/object_pose_fused',
    'right.pick_hint': '左：原链路 object_pose；右：keypoint_to_arm_tf 发布的融合位姿（需话题有数据）。均 submit_job 异步入队。',
    'right.gripper_title': '夹爪',
    'right.gripper_hint': '仅动夹爪，臂关节保持当前姿态',
    'right.open_gripper': '打开夹爪',
    'right.close_gripper': '闭合夹爪',
    'right.ws_label': '工作空间',
    'right.ws_title': 'base_link：gripper_tcp 粗采样∩feasibility 球/带 硬限后的示意 AABB（3D 线框同源），角点未必可达',
    'right.ws_note': '勿超出',
    'right.target_cable': '目标（缆绳）',
    'right.target_cable_title': 'object_pose 缆绳中心在 base_link；与 check_pick / 抓取前硬限一致（orion_mtc_params feasibility）',
    'right.debug_title': '调试工具',
    'right.sync_tracked': '同步场景（已跟踪）',
    'right.sync_untracked': '同步场景（未跟踪）',
    'right.show_collision': '显示碰撞体',

    'viewport.layers': '显示层',
    'viewport.layer.axes': '坐标轴',
    'viewport.layer.frames': 'ROV/目标系',
    'viewport.layer.world': '场景物体',
    'viewport.layer.attached': '附着物体',
    'viewport.layer.collision': '碰撞体',
    'viewport.layer.trajectory': '轨迹',
    'viewport.layer.targets': '目标点',
    'viewport.layer.workspace': '工作空间',
    'viewport.joints': '关节角度',
    'viewport.joint_col': '关节',
    'viewport.no_data': '暂无数据',
    'viewport.gripper': '夹爪',
    'viewport.gripper_title': '夹爪 (Link7/Link8)',
    'viewport.view_top': '顶视',
    'viewport.view_front': '前视',
    'viewport.view_side': '侧视',
    'viewport.view_default': '默认',
    'viewport.follow_tcp': '跟随末端',

    'bottom.tab_events': '事件流',
    'bottom.tab_recent': '最近执行',
    'bottom.tab_system': '系统日志',
    'bottom.level_all': '全部',
    'bottom.level_error': '错误',
    'bottom.level_warn': '警告',
    'bottom.level_info': '信息',
    'bottom.level_success': '成功',
    'bottom.search_ph': '搜索',
    'bottom.autoscroll': '自动滚动',
    'bottom.clear': '清空',
    'bottom.no_events': '暂无事件',
    'bottom.refresh_recent': '刷新（获取最近执行）',
    'bottom.no_recent': '暂无记录，点击刷新从后端拉取',
    'bottom.result_code': '结果码',
    'bottom.no_logs': '暂无日志',

    'card.runtime.title': '当前执行',
    'card.runtime.type': '类型',
    'card.runtime.job_id': '任务ID',
    'card.runtime.stage': '阶段',
    'card.runtime.worker': '工作线程',
    'card.runtime.mode': '模式',

    'card.queue.title': '队列',
    'card.queue.empty': '队列为空',
    'card.queue.running': '执行中',
    'card.queue.cancel': '取消',

    'card.perception.title': '感知状态',
    'card.perception.legacy_label': '原抓取方式 · 目标位姿',
    'card.perception.legacy_hint': '/object_pose（感知桥接）· base_link',
    'card.perception.pos': '位置 (m)',
    'card.perception.quat': '姿态 (四元数)',
    'card.perception.updated': '更新时间',
    'card.perception.fused_label': '视觉+声呐 · 中心线抓取点',
    'card.perception.fused_hint': '/object_pose_fused · Keypoints · base_link · ',
    'card.perception.valid': '有效',
    'card.perception.invalid': '无效',
    'card.perception.pose_fuse_invalid': '无效（未收到或未解析 /manipulator/object_pose_fused）',
    'card.perception.fused_time_invalid': '—（无效，无有效更新时间）',
    'card.perception.kp_title': '关键点点列',
    'card.perception.kp_count_suffix': ' 点',
    'card.perception.kp_ok': ' · 3D 琥珀球+折线',
    'card.perception.kp_hint_suffix': 'PoseArray · 与融合同坐标系',
    'card.perception.kp_missing': '未收到 /manipulator/keypoints_base_link（需 keypoint_to_arm_tf）',
    'card.perception.legend': '图例',
    'card.perception.legend_text': '青=桥接抓取点 · 品红=拟合抓取点 · 琥珀=各关键点',
    'card.perception.rov_map': 'ROV位姿 (map)',
    'card.perception.rov_base': 'ROV位姿 (base_link)',
    'card.perception.pose_orient': '姿态',

    'card.held.title': '持物状态',
    'card.held.holding': '持物',
    'card.held.tracked': '已跟踪',
    'card.held.object_id': '物体ID',
    'card.held.scene_attach': '场景附着ID',
    'card.held.scene_title': '场景一致性',
    'card.held.world_obj': '场景物体',
    'card.held.attached_obj': '附着物体',
    'card.held.tracked_held': '已跟踪持物',
    'card.held.untracked_held': '未跟踪持物',
    'card.held.yes': '是',
    'card.held.no': '否',
    'card.held.present': '有',
    'card.held.absent': '无',

    'card.error.title': '最近错误',
    'card.error.none': '无',

    'card.topics.title': '订阅话题（收包）',
    'card.topics.hint': '与当前 rosbridge 订阅列表一致；状态按最后收到消息的时间（久无新帧标黄）。',
    'card.topics.ws_off': '桥接未连接',
    'card.topics.waiting': '等待首帧',
    'card.topics.live': '有数据',
    'card.topics.stale': '无新数据',

    'approval.toast_no_ws': '未连接 ROS，无法审批抓取',
    'approval.toast_no_pose': '无物体位姿，无法审批抓取',
    'approval.log_no_response': '审批抓取无响应',
    'approval.toast_sent': '已发送审批请求…',
    'approval.log_sent': '发起审批抓取…',
    'approval.log_line_prefix': '审批抓取: ',
    'approval.toast_done_ok': '审批完成：',
    'approval.toast_done_reject': '审批未通过：',
    'approval.toast_done_info': '审批完成：',
    'approval.target_pre': '第 ',
    'approval.target_mid': ' 个（共 ',
    'approval.target_post': ' 个）',
    'approval.step1': '1. 几何范围检查',
    'approval.step2': '2. IK / 关节余量',
    'approval.step3': '3. 碰撞 / 安全性',
    'approval.loading': '审批中…',
    'approval.target': '目标',
    'approval.pick_pass': '通过',
    'approval.pick_warn': '可执行（有风险）',
    'approval.pick_reject': '禁止执行',
    'approval.badge_pick_prefix': '抓取',
    'approval.summary_title': '抓取审批',
    'approval.summary_ok': '所有检查通过',
    'approval.summary_empty': '完成（无摘要）',
    'approval.fallback_reject': '未通过',
    'approval.suggest_prefix': '建议: ',
    'approval.suggest_pose': '推荐抓取位姿:',
    'approval.empty_hint': '点击「审批抓取」获取结果',
    'approval.th_code': '代码',
    'approval.th_level': '级别',
    'approval.th_msg': '说明',
    'approval.th_sug': '建议',
    'approval.level.info': '信息',
    'approval.level.warn': '警告',
    'approval.level.err': '错误',

    'stage.current': '当前状态',
    'stage.segment': '执行段',
    'stage.state.enter': '进入',
    'stage.state.running': '运行中',
    'stage.state.done': '完成',
    'stage.state.failed': '失败',
    'stage.state.skipped': '跳过',

    'job.pick': '抓取',
    'job.open_gripper': '打开夹爪',
    'job.close_gripper': '闭合夹爪',
    'job.reset_held': '重置持物',
    'job.sync_held': '同步持物',

    'startup.log': 'Orion 上位机已启动',

    'toast.not_connected_clear': '未连接，无法清空队列',
    'toast.clearing': '正在清空队列…',
    'toast.not_connected_reset': '未连接，无法重置持物',
    'toast.not_connected_cancel': '未连接，无法取消任务',
    'toast.not_connected_pick': '未连接，无法提交抓取',
    'toast.no_pose_legacy': '无原链路位姿，无法提交「原抓取方式」',
    'toast.no_pose_fused': '无中心线融合位姿，无法提交「视觉+声呐中心线」',
    'toast.not_connected_sync': '未连接，无法同步持物',
    'toast.not_connected_gripper_o': '未连接，无法打开夹爪',
    'toast.not_connected_gripper_c': '未连接，无法关闭夹爪',
    'toast.not_connected_estop': '未连接，无法急停',
    'toast.not_connected_ready': '未连接，无法回 ready',
    'toast.not_connected_recent': '未连接，无法获取最近执行',

    'log.queue_cleared': '队列清空完成',
    'log.cancel_cap_a': '已尝试取消',
    'log.cancel_cap_b': ' 次，停止（仍有 next_job_id=',
    'log.cancel_cap_c': '）',
    'log.cancel_next': '取消 next_job_id:',
    'log.recent_jobs_loaded_a': '已拉取 ',
    'log.recent_jobs_loaded_b': ' 条最近执行',
    'log.sync_need_pose': 'SyncHeldObject(tracked) 需要 object_pose + tcp_pose（当前缺失），将改为 untracked 同步',
  },
  en: {
    'app.title': 'Orion Manipulator Console',
    'lang.zh': '中文',
    'lang.en': 'English',
    'lang.switch': 'Language',

    'top.brand': 'Orion',
    'top.connected': 'Connected',
    'top.disconnected': 'Disconnected',
    'top.ws_title_disconnected': 'Start rosbridge first (default ws://127.0.0.1:9091; override with ?ws=)',
    'top.worker': 'Worker',
    'top.task': 'Task',
    'top.queue': 'Queue',
    'top.joy_unknown': 'Gamepad —',
    'top.manual': 'Manual',
    'top.auto': 'Auto',
    'top.joy_title': 'joy_manipulator: /joy_manipulator/manual_mode',
    'top.throttle_unknown': 'Throttle —',
    'top.throttle_title': 'Arm throttle 0–100% (/joy_manipulator/throttle_percent); color by level, pulse on change',
    'top.clear_queue': 'Clear queue',
    'top.clear_queue_title': 'cancel_job until queue empty',
    'top.reset_held': 'Reset held object',
    'top.reset_held_title': 'reset_held_object',

    'status.idle': 'Idle',
    'status.running': 'Running',
    'status.holding': 'Holding',
    'status.error': 'Error',
    'status.recovering': 'Recovering',

    'right.tab_task': 'Task',
    'right.tab_debug': 'Debug',
    'right.emergency_ready_title': 'E-stop & ready',
    'right.emergency_ready_hint': 'emergency_stop / go_to_ready services',
    'right.emergency_stop': 'E-stop',
    'right.go_ready': 'Go to ready',
    'right.approval_title': 'Approval',
    'right.approval_hint': 'check_pick; needs object pose',
    'right.approval_btn': 'Approve pick',
    'right.pick_title': 'Pick (choose one)',
    'right.pick_legacy': 'Legacy perception',
    'right.pick_legacy_title': 'Bridge to /manipulator/object_pose (no Keypoints)',
    'right.pick_fused': 'Vision + sonar centerline',
    'right.pick_fused_title': 'Keypoints → centerline → /manipulator/object_pose_fused',
    'right.pick_hint': 'Left: object_pose; Right: fused pose from keypoint_to_arm_tf. Both use submit_job (async).',
    'right.gripper_title': 'Gripper',
    'right.gripper_hint': 'Gripper only; arm joints unchanged',
    'right.open_gripper': 'Open gripper',
    'right.close_gripper': 'Close gripper',
    'right.ws_label': 'Workspace',
    'right.ws_title': 'base_link: indicative AABB after feasibility clamp (wireframe in 3D); corners may be unreachable',
    'right.ws_note': 'Stay inside',
    'right.target_cable': 'Target (cable)',
    'right.target_cable_title': 'Cable center in base_link; same limits as check_pick / grasp (feasibility)',
    'right.debug_title': 'Debug',
    'right.sync_tracked': 'Sync scene (tracked)',
    'right.sync_untracked': 'Sync scene (untracked)',
    'right.show_collision': 'Show collision shapes',

    'viewport.layers': 'Layers',
    'viewport.layer.axes': 'Axes',
    'viewport.layer.frames': 'ROV / targets',
    'viewport.layer.world': 'World object',
    'viewport.layer.attached': 'Attached',
    'viewport.layer.collision': 'Collision',
    'viewport.layer.trajectory': 'Trajectory',
    'viewport.layer.targets': 'Targets',
    'viewport.layer.workspace': 'Workspace',
    'viewport.joints': 'Joint angles',
    'viewport.joint_col': 'Joint',
    'viewport.no_data': 'No data',
    'viewport.gripper': 'Gripper',
    'viewport.gripper_title': 'Gripper (Link7/Link8)',
    'viewport.view_top': 'Top',
    'viewport.view_front': 'Front',
    'viewport.view_side': 'Side',
    'viewport.view_default': 'Default',
    'viewport.follow_tcp': 'Follow TCP',

    'bottom.tab_events': 'Events',
    'bottom.tab_recent': 'Recent jobs',
    'bottom.tab_system': 'System log',
    'bottom.level_all': 'All',
    'bottom.level_error': 'Error',
    'bottom.level_warn': 'Warn',
    'bottom.level_info': 'Info',
    'bottom.level_success': 'OK',
    'bottom.search_ph': 'Search',
    'bottom.autoscroll': 'Auto-scroll',
    'bottom.clear': 'Clear',
    'bottom.no_events': 'No events',
    'bottom.refresh_recent': 'Refresh (load recent)',
    'bottom.no_recent': 'No records; click refresh',
    'bottom.result_code': 'code',
    'bottom.no_logs': 'No logs',

    'card.runtime.title': 'Current job',
    'card.runtime.type': 'Type',
    'card.runtime.job_id': 'Job ID',
    'card.runtime.stage': 'Stage',
    'card.runtime.worker': 'Worker',
    'card.runtime.mode': 'Mode',

    'card.queue.title': 'Queue',
    'card.queue.empty': 'Queue empty',
    'card.queue.running': 'Running',
    'card.queue.cancel': 'Cancel',

    'card.perception.title': 'Perception',
    'card.perception.legacy_label': 'Legacy · target pose',
    'card.perception.legacy_hint': '/object_pose (bridge) · base_link',
    'card.perception.pos': 'Position (m)',
    'card.perception.quat': 'Orientation (quat)',
    'card.perception.updated': 'Updated',
    'card.perception.fused_label': 'Vision+sonar · centerline grasp',
    'card.perception.fused_hint': '/object_pose_fused · Keypoints · base_link · ',
    'card.perception.valid': 'valid',
    'card.perception.invalid': 'invalid',
    'card.perception.pose_fuse_invalid': 'Invalid (no /manipulator/object_pose_fused)',
    'card.perception.fused_time_invalid': '— (invalid, no time)',
    'card.perception.kp_title': 'Keypoints',
    'card.perception.kp_count_suffix': ' pts',
    'card.perception.kp_ok': ' · amber spheres + polyline',
    'card.perception.kp_hint_suffix': 'PoseArray · same frame as fused',
    'card.perception.kp_missing': 'No /manipulator/keypoints_base_link (needs keypoint_to_arm_tf)',
    'card.perception.legend': 'Legend',
    'card.perception.legend_text': 'Cyan=bridge · Magenta=fused · Amber=keypoints',
    'card.perception.rov_map': 'ROV (map)',
    'card.perception.rov_base': 'ROV (base_link)',
    'card.perception.pose_orient': 'Orient',

    'card.held.title': 'Held object',
    'card.held.holding': 'Holding',
    'card.held.tracked': 'Tracked',
    'card.held.object_id': 'Object ID',
    'card.held.scene_attach': 'Scene attach ID',
    'card.held.scene_title': 'Scene consistency',
    'card.held.world_obj': 'World object',
    'card.held.attached_obj': 'Attached',
    'card.held.tracked_held': 'Tracked hold',
    'card.held.untracked_held': 'Untracked hold',
    'card.held.yes': 'Yes',
    'card.held.no': 'No',
    'card.held.present': 'Yes',
    'card.held.absent': 'No',

    'card.error.title': 'Last error',
    'card.error.none': 'None',

    'card.topics.title': 'Subscribed topics (RX)',
    'card.topics.hint': 'Matches rosbridge subscribe list; status from last message time (stale if idle).',
    'card.topics.ws_off': 'WS offline',
    'card.topics.waiting': 'No frame yet',
    'card.topics.live': 'Receiving',
    'card.topics.stale': 'Stale',

    'approval.toast_no_ws': 'Not connected; cannot approve pick',
    'approval.toast_no_pose': 'No object pose; cannot approve pick',
    'approval.log_no_response': 'Approve pick: no response',
    'approval.toast_sent': 'Approval request sent…',
    'approval.log_sent': 'Approve pick requested…',
    'approval.log_line_prefix': 'Approve pick: ',
    'approval.toast_done_ok': 'Approval: ',
    'approval.toast_done_reject': 'Rejected: ',
    'approval.toast_done_info': 'Approval: ',
    'approval.target_pre': '#',
    'approval.target_mid': ' of ',
    'approval.target_post': '',
    'approval.step1': '1. Reach workspace',
    'approval.step2': '2. IK / joint margin',
    'approval.step3': '3. Collision / safety',
    'approval.loading': 'Approving…',
    'approval.target': 'Target',
    'approval.pick_pass': 'approved',
    'approval.pick_warn': 'allowed (risk)',
    'approval.pick_reject': 'rejected',
    'approval.badge_pick_prefix': 'Pick ',
    'approval.summary_title': 'Pick approval',
    'approval.summary_ok': 'All checks passed',
    'approval.summary_empty': 'Done (no summary)',
    'approval.fallback_reject': 'Not approved',
    'approval.suggest_prefix': 'Suggest: ',
    'approval.suggest_pose': 'Suggested grasp pose:',
    'approval.empty_hint': 'Click Approve pick for results',
    'approval.th_code': 'Code',
    'approval.th_level': 'Level',
    'approval.th_msg': 'Message',
    'approval.th_sug': 'Suggestion',
    'approval.level.info': 'Info',
    'approval.level.warn': 'Warning',
    'approval.level.err': 'Error',

    'stage.current': 'Current state',
    'stage.segment': 'Segment',
    'stage.state.enter': 'Enter',
    'stage.state.running': 'Running',
    'stage.state.done': 'Done',
    'stage.state.failed': 'Failed',
    'stage.state.skipped': 'Skipped',

    'job.pick': 'Pick',
    'job.open_gripper': 'Open gripper',
    'job.close_gripper': 'Close gripper',
    'job.reset_held': 'Reset held',
    'job.sync_held': 'Sync held',

    'startup.log': 'Orion console started',

    'toast.not_connected_clear': 'Not connected; cannot clear queue',
    'toast.clearing': 'Clearing queue…',
    'toast.not_connected_reset': 'Not connected; cannot reset held',
    'toast.not_connected_cancel': 'Not connected; cannot cancel',
    'toast.not_connected_pick': 'Not connected; cannot submit pick',
    'toast.no_pose_legacy': 'No legacy object_pose for this pick mode',
    'toast.no_pose_fused': 'No fused pose for centerline pick',
    'toast.not_connected_sync': 'Not connected; cannot sync held',
    'toast.not_connected_gripper_o': 'Not connected; cannot open gripper',
    'toast.not_connected_gripper_c': 'Not connected; cannot close gripper',
    'toast.not_connected_estop': 'Not connected; cannot E-stop',
    'toast.not_connected_ready': 'Not connected; cannot go to ready',
    'toast.not_connected_recent': 'Not connected; cannot load recent jobs',

    'log.queue_cleared': 'Queue cleared',
    'log.cancel_cap_a': 'Cancel capped after',
    'log.cancel_cap_b': ' attempts; stopping (next_job_id=',
    'log.cancel_cap_c': ')',
    'log.cancel_next': 'Cancel next_job_id:',
    'log.recent_jobs_loaded_a': 'Loaded ',
    'log.recent_jobs_loaded_b': ' recent job(s)',
    'log.sync_need_pose': 'SyncHeldObject(tracked) needs object_pose + tcp_pose; falling back to untracked',
  },
};

const listeners = new Set();

function normalizeLocale(raw) {
  if (raw === 'en' || raw === 'zh') {
    return raw;
  }
  return 'zh';
}

/** 从 URL ?lang=zh|en 读取（仅首次应用时 main 可调用） */
export function localeFromSearchParams() {
  try {
    const q = new URLSearchParams(window.location.search || '').get('lang');
    if (q) {
      const l = String(q).toLowerCase();
      if (l === 'en' || l === 'zh') {
        return l;
      }
    }
  } catch (e) {
    /* ignore */
  }
  return null;
}

export function getLocale() {
  return normalizeLocale(localStorage.getItem(STORAGE_KEY) || 'zh');
}

export function setLocale(loc) {
  const next = normalizeLocale(loc);
  localStorage.setItem(STORAGE_KEY, next);
  document.documentElement.lang = next === 'zh' ? 'zh-CN' : 'en';
  const titleEl = document.querySelector('title');
  if (titleEl) {
    titleEl.textContent = t('app.title');
  }
  listeners.forEach((fn) => {
    try {
      fn(next);
    } catch (e) {
      /* ignore subscriber errors */
    }
  });
  window.dispatchEvent(new CustomEvent('orion:locale-changed', { detail: { locale: next } }));
}

export function subscribeLocale(fn) {
  listeners.add(fn);
  try {
    fn(getLocale());
  } catch (e) {
    /* ignore */
  }
  return () => listeners.delete(fn);
}

export function t(key, fallback) {
  const loc = getLocale();
  const table = MESSAGES[loc] || MESSAGES.zh;
  const s = table[key];
  if (s != null && s !== '') {
    return s;
  }
  if (fallback != null) {
    return fallback;
  }
  return key;
}

/** 安装：文档语言、默认标题；若 URL 带 lang= 则覆盖写入 */
export function initI18n() {
  const fromUrl = localeFromSearchParams();
  if (fromUrl) {
    localStorage.setItem(STORAGE_KEY, fromUrl);
  }
  const loc = getLocale();
  document.documentElement.lang = loc === 'zh' ? 'zh-CN' : 'en';
  const titleEl = document.querySelector('title');
  if (titleEl) {
    titleEl.textContent = t('app.title');
  }
}
