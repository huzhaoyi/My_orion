/**
 * 左侧 - 感知状态卡片（来自 perception_state：单缆绳位姿、放置、ROV 位姿）
 */

import stateStore from '../data/stateStore.js';

function fmtPos(pos) {
  if (!pos || typeof pos.x !== 'number' || typeof pos.y !== 'number' || typeof pos.z !== 'number') {
    return '—';
  }
  return `${pos.x.toFixed(3)}, ${pos.y.toFixed(3)}, ${pos.z.toFixed(3)}`;
}

function fmtQuat(q) {
  if (!q || typeof q.w !== 'number') return '—';
  const x = (q.x != null ? q.x : 0);
  const y = (q.y != null ? q.y : 0);
  const z = (q.z != null ? q.z : 0);
  const w = (q.w != null ? q.w : 1);
  return `qx=${x.toFixed(3)} qy=${y.toFixed(3)} qz=${z.toFixed(3)} qw=${w.toFixed(3)}`;
}

function render(parentEl) {
  if (!parentEl) return;
  const wrap = document.createElement('div');
  wrap.className = 'card';
  wrap.id = 'perception-card';
  parentEl.appendChild(wrap);

  function update(s) {
    if (!wrap.isConnected) return;
    const state = s != null ? s : stateStore.getState();
    const objPose = state.objectPoseValid && state.objectPose ? state.objectPose : null;
    const objPos = objPose ? objPose.position : null;
    const objQuat = objPose ? objPose.orientation : null;
    const placePos = state.placePoseValid && state.placePose ? state.placePose.position : null;
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
          <span class="card-label">目标：缆绳 (base_link)</span>
          <span class="card-value" style="font-size:10px;color:var(--text-secondary);">长度 3m，直径 5cm</span>
        </div>
        <div class="card-row card-row--indent">
          <span class="card-label">位置 (m)</span>
          <span class="card-value">${state.objectPoseValid ? fmtPos(objPos) : '无效'}</span>
        </div>
        <div class="card-row card-row--indent">
          <span class="card-label">姿态 (四元数)</span>
          <span class="card-value perception-card__quat">${objQuat ? fmtQuat(objQuat) : '—'}</span>
        </div>
        <div class="card-row card-row--indent">
          <span class="card-label">说明</span>
          <span class="card-value" style="font-size:10px;color:var(--text-secondary);">base_link=机械臂基座坐标系</span>
        </div>
      </div>
      <div class="card-row"><span class="card-label">放置 (同缆绳目标 base_link)</span><span class="card-value">${state.placePoseValid ? fmtPos(placePos) : '无效'}</span></div>
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
      <div class="card-row"><span class="card-label">更新时间</span><span class="card-value">${t}</span></div>
    `;
  }

  update();
  stateStore.subscribe((newState) => update(newState));
}

export default { render };
