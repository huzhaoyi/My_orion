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
      <div class="card-row"><span class="card-label">持物</span><span class="card-value">${s.hasHeldObject ? '是' : '否'}</span></div>
      <div class="card-row"><span class="card-label">已跟踪</span><span class="card-value">${s.heldTracked ? '是' : '否'}</span></div>
      <div class="card-row"><span class="card-label">物体ID</span><span class="card-value">${s.heldObjectId || '—'}</span></div>
      <div class="card-row"><span class="card-label">场景附着ID</span><span class="card-value">${(s.heldSceneAttachId || '—').slice(0, 20)}</span></div>
      <div class="card-title" style="margin-top: 10px;">场景一致性</div>
      <div class="card-row"><span class="card-label">场景物体</span><span class="card-value">${s.worldObjectPresent ? '有' : '无'}</span></div>
      <div class="card-row"><span class="card-label">附着物体</span><span class="card-value">${s.attachedObjectPresent ? '有' : '无'}</span></div>
      <div class="card-row"><span class="card-label">已跟踪持物</span><span class="card-value">${s.heldTrackedPresent ? '有' : '无'}</span></div>
      <div class="card-row"><span class="card-label">未跟踪持物</span><span class="card-value">${s.heldUntrackedPresent ? '有' : '无'}</span></div>
    `;
  }

  update();
  stateStore.subscribe(update);
}

export default { render };
