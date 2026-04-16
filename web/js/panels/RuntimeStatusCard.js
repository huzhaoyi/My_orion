/**
 * 左侧 - 当前执行 / 运行时状态卡片（可合并到 LeftPanel 或单独挂载）
 */

import stateStore from '../data/stateStore.js';
import { jobTypeLabel, stageNameLabel, stagePillClass } from '../data/labels.js';
import { t, subscribeLocale } from '../data/i18n.js';

/** 在 parentEl 下追加「当前执行」卡片并订阅刷新。 */
function render(parentEl) {
  if (!parentEl) return;
  const wrap = document.createElement('div');
  wrap.className = 'card';
  wrap.id = 'runtime-status-card';
  parentEl.appendChild(wrap);

  function statusLabel(s) {
    if (!s) return '—';
    const u = (s + '').toUpperCase();
    if (u.includes('RUNNING') || u.includes('PICKING')) return t('status.running');
    if (u.includes('HOLDING')) return t('status.holding');
    if (u.includes('ERROR') || u.includes('DISCONNECTED')) return t('status.error');
    if (u.includes('RECOVERING') || u.includes('WARNING')) return t('status.recovering');
    if (u.includes('IDLE')) return t('status.idle');
    return s;
  }
  function update() {
    const s = stateStore.getState();
    const latchLabel = s.insertLatchLocked ? t('card.held.yes') : t('card.held.no');
    const latchHole = Number.isFinite(Number(s.insertLatchHole)) && Number(s.insertLatchHole) > 0
      ? String(s.insertLatchHole)
      : '—';
    wrap.innerHTML = `
      <div class="card-title">${t('card.runtime.title')}</div>
      <div class="card-row"><span class="card-label">${t('card.runtime.type')}</span><span class="card-value">${jobTypeLabel(s.currentJobType)}</span></div>
      <div class="card-row"><span class="card-label">${t('card.runtime.job_id')}</span><span class="card-value">${(s.currentJobId || '—').slice(0, 16)}</span></div>
      <div class="card-row"><span class="card-label">${t('card.runtime.stage')}</span><span class="card-value"><span class="stage-pill ${stagePillClass(s.currentStageName)}">${stageNameLabel(s.currentStageName) || '—'}</span></span></div>
      <div class="card-row"><span class="card-label">${t('card.runtime.worker')}</span><span class="card-value">${statusLabel(s.workerStatus)}</span></div>
      <div class="card-row"><span class="card-label">${t('card.runtime.mode')}</span><span class="card-value">${statusLabel(s.taskMode)}</span></div>
      <div class="card-row"><span class="card-label">${t('card.runtime.insert_latch')}</span><span class="card-value">${latchLabel}</span></div>
      <div class="card-row"><span class="card-label">${t('card.runtime.insert_hole')}</span><span class="card-value">${latchHole}</span></div>
    `;
  }

  update();
  stateStore.subscribe(update);
  subscribeLocale(update);
}

export default { render };
