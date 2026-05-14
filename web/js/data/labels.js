/**
 * 任务类型、阶段名、阶段状态（随 i18n 中英切换）
 * 阶段配色：stagePillClass() 供「当前执行 / 事件流」彩色区分
 */

import { getLocale, t } from './i18n.js';

/** ROS job_type / 任务类型 → 当前语言短标签。 */
export function jobTypeLabel(jobT) {
  if (!jobT) return '—';
  const u = String(jobT).toUpperCase();
  if (getLocale() === 'en') {
    if (u === 'PICK') return t('job.pick');
    if (u === 'OPEN_GRIPPER') return t('job.open_gripper');
    if (u === 'CLOSE_GRIPPER') return t('job.close_gripper');
    if (u === 'RESET_HELD_OBJECT') return t('job.reset_held');
    if (u === 'SYNC_HELD_OBJECT') return t('job.sync_held');
    if (u === 'TARGET_INSERT') return t('job.target_insert');
    return jobT;
  }
  if (u === 'PICK') return t('job.pick');
  if (u === 'OPEN_GRIPPER') return t('job.open_gripper');
  if (u === 'CLOSE_GRIPPER') return t('job.close_gripper');
  if (u === 'RESET_HELD_OBJECT') return t('job.reset_held');
  if (u === 'SYNC_HELD_OBJECT') return t('job.sync_held');
  if (u === 'TARGET_INSERT') return t('job.target_insert');
  return jobT;
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
  'add targetsensor peg mesh': '添加目标插销碰撞体',
  'allow collision (targetsensor peg) for pregrasp': '允许目标插销碰撞(预抓)',
  'allow collision (targetsensor peg) for approach': '允许目标插销碰撞(接近)',
  'remove targetsensor peg mesh': '移除目标插销碰撞体',
  'move to pregrasp (holding)': '持物后回到预抓',
  'close hand (at pregrasp)': '预抓位闭合手爪',
  'target insert': '目标插孔流程',
  'move to ready (before insert)': '插孔前回到就绪',
  'move to front-waypoint': '移动到前置航点',
  'front-waypoint to pre-insert (align)': '前置航点到预插入(对齐)',
  'front-waypoint to pre-insert': '前置航点到预插入',
  'move to pre-insert': '移动到预插入',
  'insert approach': '插孔接近',
  'insert chamfer +u': '倒角搜索 +u',
  'insert chamfer -u': '倒角搜索 -u',
  'insert chamfer +v': '倒角搜索 +v',
  'insert chamfer -v': '倒角搜索 -v',
  'insert descend': '插入下压',
  'lift clear': '抬升脱离',
  'move to ready (after release)': '释放后回到就绪',
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
  pick_prepare_candidates: '候选准备',
  pick_precheck_ik: '预检（IK / 关节 / 碰撞）',
  pick_mtc_plan: 'MTC 运动规划',
  check_pick_geometry: '审批：几何范围',
  check_pick_ik: '审批：逆解与关节余量',
  check_pick_collision: '审批：场景碰撞',
  check_pick_suggest_search: '审批：搜索建议位姿',
  insert_prepare: '插孔：准备',
  insert_pre_extract_prepare: '插孔：预拔准备',
  insert_pre_extract_plan: '插孔：预拔规划',
  insert_mtc_plan: '插孔：主流程规划',
  close_hand_ready: '闭合手爪(就绪)',
};

/** MTC stage_name：中文表或英文原样；segment_k → 执行段 k / Segment k。 */
export function stageNameLabel(name) {
  if (!name || !String(name).trim()) return '—';
  const key = String(name).trim();
  if (/^segment_\d+$/i.test(key)) {
    return `${t('stage.segment')} ${key.replace(/^segment_/i, '')}`;
  }
  if (/^insert descend segment \d+$/i.test(key)) {
    return `插入下压段 ${key.replace(/^insert descend segment\s+/i, '')}`;
  }
  if (/^lift clear segment \d+$/i.test(key)) {
    return `抬升脱离段 ${key.replace(/^lift clear segment\s+/i, '')}`;
  }
  if (getLocale() === 'en') {
    if (key.toLowerCase() === 'current') {
      return t('stage.current');
    }
    return key;
  }
  if (STAGE_NAME_ZH[key]) return STAGE_NAME_ZH[key];
  const under = key.replace(/\s+/g, '_');
  if (STAGE_NAME_ZH[under]) return STAGE_NAME_ZH[under];
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
  if (k.includes('orion pick') || k.includes('pick') || k.startsWith('insert_')) return 'stage-pill--task';
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

/** TaskStage.stage_state → 事件流标签。 */
export function stageStateLabel(state) {
  if (!state) return '';
  const u = String(state).toUpperCase();
  if (u === 'ENTER') return t('stage.state.enter');
  if (u === 'RUNNING') return t('stage.state.running');
  if (u === 'DONE') return t('stage.state.done');
  if (u === 'FAILED') return t('stage.state.failed');
  if (u === 'SKIPPED') return t('stage.state.skipped');
  return state;
}
