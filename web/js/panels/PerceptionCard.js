/**
 * 左侧 - 感知状态卡片（来自 perception_state：单缆绳位姿、ROV 位姿）
 */

import stateStore from '../data/stateStore.js';

/** 位置对象格式化为 "x, y, z" 三位小数（rosbridge 可能给 string，用 Number）。 */
function fmtPos(pos) {
  if (!pos) {
    return '—';
  }
  const x = Number(pos.x);
  const y = Number(pos.y);
  const z = Number(pos.z);
  if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) {
    return '—';
  }
  return `${x.toFixed(3)}, ${y.toFixed(3)}, ${z.toFixed(3)}`;
}

/** 四元数格式化为简短 qx..qw 串。 */
function fmtQuat(q) {
  if (!q) {
    return '—';
  }
  const x = Number(q.x);
  const y = Number(q.y);
  const z = Number(q.z);
  const w = Number(q.w);
  if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z) || !Number.isFinite(w)) {
    return '—';
  }
  return `qx=${x.toFixed(3)} qy=${y.toFixed(3)} qz=${z.toFixed(3)} qw=${w.toFixed(3)}`;
}

/** 感知卡片：object_pose、ROV base/world、更新时间（来自 perception_state / 缓存）。 */
function render(parentEl) {
  if (!parentEl) return;
  const wrap = document.createElement('div');
  wrap.className = 'card';
  wrap.id = 'perception-card';
  parentEl.appendChild(wrap);

  function update(s) {
    if (!wrap.isConnected) return;
    const state = s != null ? s : stateStore.getState();
    const objPose = state.objectPose || null;
    const objPos = objPose ? objPose.position : null;
    const objQuat = objPose ? objPose.orientation : null;
    const fusedPose = state.fusedObjectPose || null;
    const fusedPos = fusedPose ? fusedPose.position : null;
    const fusedQuat = fusedPose ? fusedPose.orientation : null;
    const fusedValid = !!state.fusedObjectPoseValid;
    const fusedInvalidHint = '无效（未收到或未解析 /manipulator/object_pose_fused）';
    const fusedPosDisp = fusedValid ? fmtPos(fusedPos) : fusedInvalidHint;
    const fusedQuatDisp = fusedValid ? (fusedQuat ? fmtQuat(fusedQuat) : '—') : fusedInvalidHint;
    const tfu = state.fusedPerceptionUpdatedAt
      ? new Date(state.fusedPerceptionUpdatedAt).toLocaleTimeString()
      : '—';
    const fusedTimeDisp = fusedValid ? tfu : '—（无效，无有效更新时间）';
    const rovBase = state.rovPoseInBaseLink || null;
    const rovWorld = state.rovPoseInWorld || null;
    const rovPosBase = rovBase ? rovBase.position : null;
    const rovPosWorld = rovWorld ? rovWorld.position : null;
    const rovQuatBase = rovBase ? rovBase.orientation : null;
    const rovQuatWorld = rovWorld ? rovWorld.orientation : null;
    const t = state.perceptionUpdatedAt
      ? new Date(state.perceptionUpdatedAt).toLocaleTimeString()
      : '—';

    wrap.innerHTML = `
      <div class="card-title">感知状态</div>
      <div class="perception-card__pose-block">
        <div class="card-row">
          <span class="card-label">原抓取方式 · 目标位姿</span>
          <span class="card-value" style="font-size:10px;color:var(--text-secondary);">/object_pose（感知桥接）· base_link</span>
        </div>
        <div class="card-row card-row--indent">
          <span class="card-label">位置 (m)</span>
          <span class="card-value">${fmtPos(objPos)}</span>
        </div>
        <div class="card-row card-row--indent">
          <span class="card-label">姿态 (四元数)</span>
          <span class="card-value perception-card__quat">${objQuat ? fmtQuat(objQuat) : '—'}</span>
        </div>
        <div class="card-row card-row--indent"><span class="card-label">更新时间</span><span class="card-value">${t}</span></div>
      </div>
      <div class="perception-card__pose-block perception-card__pose-block--fused">
        <div class="card-row">
          <span class="card-label">视觉+声呐 · 中心线抓取点</span>
          <span class="card-value" style="font-size:10px;color:var(--text-secondary);">/object_pose_fused · Keypoints · base_link · <span style="color:${fusedValid ? '#22c55e' : '#f97316'}">${fusedValid ? '有效' : '无效'}</span></span>
        </div>
        <div class="card-row card-row--indent">
          <span class="card-label">位置 (m)</span>
          <span class="card-value perception-card__fused-pos">${fusedPosDisp}</span>
        </div>
        <div class="card-row card-row--indent">
          <span class="card-label">姿态</span>
          <span class="card-value perception-card__quat">${fusedQuatDisp}</span>
        </div>
        <div class="card-row card-row--indent"><span class="card-label">更新时间</span><span class="card-value">${fusedTimeDisp}</span></div>
      </div>
      <div class="perception-card__pose-block">
        <div class="card-row"><span class="card-label">ROV位姿 (map)</span></div>
        <div class="card-row card-row--indent"><span class="card-label">位置</span><span class="card-value">${rovPosWorld ? fmtPos(rovPosWorld) : '—'}</span></div>
        <div class="card-row card-row--indent"><span class="card-label">姿态</span><span class="card-value perception-card__quat">${rovQuatWorld ? fmtQuat(rovQuatWorld) : '—'}</span></div>
      </div>
      <div class="perception-card__pose-block">
        <div class="card-row"><span class="card-label">ROV位姿 (base_link)</span></div>
        <div class="card-row card-row--indent"><span class="card-label">位置</span><span class="card-value">${rovPosBase ? fmtPos(rovPosBase) : '—'}</span></div>
        <div class="card-row card-row--indent"><span class="card-label">姿态</span><span class="card-value perception-card__quat">${rovQuatBase ? fmtQuat(rovQuatBase) : '—'}</span></div>
      </div>
    `;
  }

  update();
  stateStore.subscribe((newState) => update(newState));
}

export default { render };
