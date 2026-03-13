/**
 * 左侧 - 当前执行 / 运行时状态卡片（可合并到 LeftPanel 或单独挂载）
 */

import stateStore from '../data/stateStore.js';
import { jobTypeLabel, stageNameLabel } from '../data/labels.js';

function render(parentEl) {
  if (!parentEl) return;
  const wrap = document.createElement('div');
  wrap.className = 'card';
  wrap.id = 'runtime-status-card';
  parentEl.appendChild(wrap);

  function statusLabel(s) {
    if (!s) return '—';
    const u = (s + '').toUpperCase();
    if (u.includes('RUNNING') || u.includes('PICKING') || u.includes('PLACING')) return '运行中';
    if (u.includes('HOLDING')) return '持物中';
    if (u.includes('ERROR') || u.includes('DISCONNECTED')) return '异常';
    if (u.includes('RECOVERING') || u.includes('WARNING')) return '恢复中';
    if (u.includes('IDLE')) return '空闲';
    return s;
  }
  function update() {
    const s = stateStore.getState();
    wrap.innerHTML = `
      <div class="card-title">当前执行</div>
      <div class="card-row"><span class="card-label">类型</span><span class="card-value">${jobTypeLabel(s.currentJobType)}</span></div>
      <div class="card-row"><span class="card-label">任务ID</span><span class="card-value">${(s.currentJobId || '—').slice(0, 16)}</span></div>
      <div class="card-row"><span class="card-label">阶段</span><span class="card-value">${stageNameLabel(s.currentStageName) || '—'}</span></div>
      <div class="card-row"><span class="card-label">工作线程</span><span class="card-value">${statusLabel(s.workerStatus)}</span></div>
      <div class="card-row"><span class="card-label">模式</span><span class="card-value">${statusLabel(s.taskMode)}</span></div>
    `;
  }

  update();
  stateStore.subscribe(update);
}

export default { render };
