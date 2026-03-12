/**
 * 左侧 - 感知状态卡片（object_pose / place_pose 是否有效等）
 */

import stateStore from '../data/stateStore.js';

function render(parentEl) {
  if (!parentEl) return;
  const wrap = document.createElement('div');
  wrap.className = 'card';
  wrap.id = 'perception-card';
  parentEl.appendChild(wrap);

  function update() {
    const s = stateStore.getState();
    const t = s.perceptionUpdatedAt ? new Date(s.perceptionUpdatedAt).toLocaleTimeString() : '—';
    wrap.innerHTML = `
      <div class="card-title">感知状态</div>
      <div class="card-row"><span class="card-label">object_pose</span><span class="card-value">${s.objectPoseValid ? '有效' : '无效'}</span></div>
      <div class="card-row"><span class="card-label">place_pose</span><span class="card-value">${s.placePoseValid ? '有效' : '无效'}</span></div>
      <div class="card-row"><span class="card-label">更新时间</span><span class="card-value">${t}</span></div>
      <div class="card-row"><span class="card-label">目标数</span><span class="card-value">${s.targetCount}</span></div>
    `;
  }

  update();
  stateStore.subscribe(update);
}

export default { render };
