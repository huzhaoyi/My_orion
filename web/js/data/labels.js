/**
 * 前端统一中文标签：任务类型、阶段名、阶段状态
 * 阶段配色：stagePillClass() 供「当前执行 / 事件流」彩色区分
 */

/** ROS job_type / 任务类型英文枚举 → 中文短标签。 */
export function jobTypeLabel(t) {
  if (!t) return '—';
  const u = String(t).toUpperCase();
  if (u === 'PICK') return '抓取';
  if (u === 'OPEN_GRIPPER') return '打开夹爪';
  if (u === 'CLOSE_GRIPPER') return '闭合夹爪';
  if (u === 'RESET_HELD_OBJECT') return '重置持物';
  if (u === 'SYNC_HELD_OBJECT') return '同步持物';
  return t;
}

/**
 * MTC / TaskManager 上报的 stage_name（与 task_manager.cpp、各 Task setName 一致）
 */
const STAGE_NAME_ZH = {
  current: '当前状态',
  'move to ready': '回到就绪',
  'add_cable_segments': '添加缆绳分段',
  'open hand': '张开手爪',
  'allow self-collision (pregrasp)': '允许自碰(预抓)',
  'allow collision (cable local) for pregrasp': '允许缆绳碰撞(预抓)',
  'move to pregrasp': '移至预抓',
  'allow collision (cable local) for approach': '允许缆绳碰撞(接近)',
  'approach to grasp (LIN)': '直线接近抓取',
  'close hand': '闭合手爪',
  'remove_cable_segments': '移除缆绳分段',
  'retreat short': '短距离退离',
  'open gripper': '打开夹爪',
  'close gripper': '闭合夹爪',
  'retreat to ready': '回到就绪(退离)',
  'orion pick (cable side, segmented)': '侧向抓取(分段)',
  'close hand (ready)': '闭合手爪(就绪)',
  move_to_ready: '回到就绪',
  add_object: '添加物体',
  open_hand: '张开手爪',
  allow_self_collision_pregrasp: '允许自碰(预抓)',
  allow_collision_arm_object_pregrasp: '允许臂物碰撞(预抓)',
  move_to_pregrasp: '移至预抓',
  allow_collision_hand_object: '允许手物碰撞',
  approach_object: '接近物体',
  allow_collision_before_close: '允许碰撞(闭合前)',
  close_hand: '闭合手爪',
  attach_object: '附着物体',
  allow_collision_object_support: '允许物面碰撞',
  forbid_collision_object_support: '禁止物面碰撞',
  detach_object: '脱离物体',
  retreat: '退离',
  forbid_collision_hand_object: '禁止手物碰撞',
  allow_collision_object_arm: '允许物臂碰撞',
  close_hand_ready: '闭合手爪(就绪)',
};

/** MTC stage_name（空格或下划线）→ 中文；segment_k 显示为「执行段 k」。 */
export function stageNameLabel(name) {
  if (!name || !String(name).trim()) return '—';
  const key = String(name).trim();
  if (STAGE_NAME_ZH[key]) return STAGE_NAME_ZH[key];
  const under = key.replace(/\s+/g, '_');
  if (STAGE_NAME_ZH[under]) return STAGE_NAME_ZH[under];
  if (/^segment_\d+$/i.test(key)) return '执行段 ' + key.replace(/^segment_/i, '');
  return key;
}

/**
 * 阶段 -> 样式类（与 theme.css .stage-pill--* 对应）
 */
export function stagePillClass(name) {
  if (!name || !String(name).trim()) return 'stage-pill--idle';
  const k = String(name).trim().toLowerCase();
  if (k === 'current' || /^segment_\d+$/.test(k)) return 'stage-pill--meta';
  if (k.includes('ready') || k === 'move to ready' || k === 'retreat to ready') return 'stage-pill--ready';
  if (k.includes('open hand') || k.includes('open_gripper') || k === 'open gripper') return 'stage-pill--open';
  if (k.includes('close hand') || k.includes('close_gripper') || k === 'close gripper') return 'stage-pill--close';
  if (k.includes('approach') || k.includes('pregrasp')) return 'stage-pill--approach';
  if (k.includes('retreat')) return 'stage-pill--retreat';
  if (k.includes('collision') || k.includes('allow ') || k.includes('forbid')) return 'stage-pill--scene';
  if (k.includes('cable') || k.includes('segment')) return 'stage-pill--cable';
  if (k.includes('orion pick') || k.includes('pick')) return 'stage-pill--task';
  return 'stage-pill--default';
}

/** 与 stage-pill 组合：按 ENTER/RUNNING/DONE/FAILED 微调描边（可选） */
export function stageStateModifier(state) {
  if (!state) return '';
  const u = String(state).toUpperCase();
  if (u === 'FAILED') return 'stage-pill--state-fail';
  if (u === 'DONE') return 'stage-pill--state-done';
  if (u === 'RUNNING' || u === 'ENTER') return 'stage-pill--state-run';
  return '';
}

/** TaskStage.stage_state → 事件流中文（进入/运行中/完成/失败）。 */
export function stageStateLabel(state) {
  if (!state) return '';
  const u = String(state).toUpperCase();
  if (u === 'ENTER') return '进入';
  if (u === 'RUNNING') return '运行中';
  if (u === 'DONE') return '完成';
  if (u === 'FAILED') return '失败';
  if (u === 'SKIPPED') return '跳过';
  return state;
}
