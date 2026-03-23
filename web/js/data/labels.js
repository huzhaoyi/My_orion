/**
 * 前端统一中文标签：任务类型、阶段名、阶段状态
 */

export function jobTypeLabel(t) {
  if (!t) return '—';
  const u = (String(t)).toUpperCase();
  if (u === 'PICK') return '抓取';
  if (u === 'OPEN_GRIPPER') return '打开夹爪';
  if (u === 'CLOSE_GRIPPER') return '闭合夹爪';
  if (u === 'RESET_HELD_OBJECT') return '重置持物';
  if (u === 'SYNC_HELD_OBJECT') return '同步持物';
  return t;
}

/** MTC 阶段名 -> 中文（与后端 PICK 流程阶段名对应） */
const STAGE_NAME_ZH = {
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
  lift_object: '抬升物体',
  forbid_collision_object_support: '禁止物面碰撞',
  detach_object: '脱离物体',
  retreat: '退离',
  forbid_collision_hand_object: '禁止手物碰撞',
  allow_collision_object_arm: '允许物臂碰撞',
  open_hand_ready: '张开手爪(就绪)',
};

export function stageNameLabel(name) {
  if (!name || !String(name).trim()) return '—';
  const key = String(name).trim();
  if (STAGE_NAME_ZH[key]) return STAGE_NAME_ZH[key];
  if (/^segment_\d+$/i.test(key)) return '段' + key.replace(/^segment_/i, '');
  return name;
}

/** stage_state -> 中文 */
export function stageStateLabel(state) {
  if (!state) return '';
  const u = (String(state)).toUpperCase();
  if (u === 'ENTER') return '进入';
  if (u === 'RUNNING') return '运行中';
  if (u === 'DONE') return '完成';
  if (u === 'FAILED') return '失败';
  if (u === 'SKIPPED') return '跳过';
  return state;
}
