/**
 * 左侧 - 当前执行 / 运行时状态卡片（可合并到 LeftPanel 或单独挂载）
 */

import stateStore from '../data/stateStore.js';

function render(parentEl) {
  if (!parentEl) return;
  const wrap = document.createElement('div');
  wrap.className = 'card';
  wrap.id = 'runtime-status-card';
  parentEl.appendChild(wrap);

  function update() {
    const s = stateStore.getState();
    wrap.innerHTML = `
      <div class="card-title">当前执行</div>
      <div class="card-row"><span class="card-label">类型</span><span class="card-value">${s.currentJobType || '—'}</span></div>
      <div class="card-row"><span class="card-label">Job ID</span><span class="card-value">${(s.currentJobId || '—').slice(0, 16)}</span></div>
      <div class="card-row"><span class="card-label">阶段</span><span class="card-value">${s.currentStageName || '—'}</span></div>
      <div class="card-row"><span class="card-label">Worker</span><span class="card-value">${s.workerStatus || '—'}</span></div>
      <div class="card-row"><span class="card-label">模式</span><span class="card-value">${s.taskMode || '—'}</span></div>
    `;
  }

  update();
  stateStore.subscribe(update);
}

export default { render };
