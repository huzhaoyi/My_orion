/**
 * 左侧 - 持物状态卡片 + Scene 一致性提示
 */

import stateStore from '../data/stateStore.js';

function render(parentEl) {
  if (!parentEl) return;
  const wrap = document.createElement('div');
  wrap.className = 'card';
  wrap.id = 'held-object-card';
  parentEl.appendChild(wrap);

  function update() {
    const s = stateStore.getState();
    wrap.innerHTML = `
      <div class="card-title">持物状态</div>
      <div class="card-row"><span class="card-label">has_held_object</span><span class="card-value">${s.hasHeldObject ? '是' : '否'}</span></div>
      <div class="card-row"><span class="card-label">tracked</span><span class="card-value">${s.heldTracked ? '是' : '否'}</span></div>
      <div class="card-row"><span class="card-label">object_id</span><span class="card-value">${s.heldObjectId || '—'}</span></div>
      <div class="card-row"><span class="card-label">scene_attach_id</span><span class="card-value">${(s.heldSceneAttachId || '—').slice(0, 20)}</span></div>
      <div class="card-title" style="margin-top: 10px;">Scene 一致性</div>
      <div class="card-row"><span class="card-label">world.object</span><span class="card-value">${s.worldObjectPresent ? '有' : '无'}</span></div>
      <div class="card-row"><span class="card-label">attached.object</span><span class="card-value">${s.attachedObjectPresent ? '有' : '无'}</span></div>
      <div class="card-row"><span class="card-label">held_tracked</span><span class="card-value">${s.heldTrackedPresent ? '有' : '无'}</span></div>
      <div class="card-row"><span class="card-label">held_untracked</span><span class="card-value">${s.heldUntrackedPresent ? '有' : '无'}</span></div>
    `;
  }

  update();
  stateStore.subscribe(update);
}

export default { render };
