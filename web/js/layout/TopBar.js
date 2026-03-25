/**
 * 顶部状态栏：连接状态、机器人状态、队列长度、快捷操作（与后端服务一致）
 */

import stateStore from '../data/stateStore.js';

function workerBadgeClass(workerStatus) {
  const s = (workerStatus || '').toUpperCase();
  if (s.includes('ERROR') || s.includes('DISCONNECTED')) return 'badge-error';
  if (s.includes('RECOVERING') || s.includes('WARNING')) return 'badge-warning';
  if (s.includes('RUNNING') || s.includes('PICKING') || s.includes('HOLDING')) return 'badge-running';
  return 'badge-idle';
}

function taskModeBadgeClass(taskMode) {
  const s = (taskMode || '').toUpperCase();
  if (s.includes('ERROR')) return 'badge-error';
  if (s.includes('RECOVERING') || s.includes('WARNING')) return 'badge-warning';
  if (s.includes('RUNNING') || s.includes('PICKING') || s.includes('HOLDING')) return 'badge-taskmode';
  return 'badge-taskmode';
}

function statusToLabel(s) {
  if (!s) return '空闲';
  const u = (s + '').toUpperCase();
  if (u.includes('RUNNING') || u.includes('PICKING')) return '运行中';
  if (u.includes('HOLDING')) return '持物中';
  if (u.includes('ERROR') || u.includes('DISCONNECTED')) return '异常';
  if (u.includes('RECOVERING') || u.includes('WARNING')) return '恢复中';
  if (u.includes('IDLE')) return '空闲';
  return s;
}

function joyModeBadgeClass(manual) {
  if (manual === null || manual === undefined) return 'badge-idle';
  return manual ? 'badge-warning' : 'badge-running';
}

function joyModeLabel(manual) {
  if (manual === null || manual === undefined) return '手柄 —';
  return manual ? '手动' : '自动';
}

/** 手柄油门徽章：按 0～100% 分档着色；无数据时用默认 info 色 */
function throttleBadgeClass(percent) {
  if (percent === null || percent === undefined || !Number.isFinite(percent)) return 'badge-throttle';
  if (percent < 34) return 'badge-throttle-low';
  if (percent < 67) return 'badge-throttle-mid';
  return 'badge-throttle-high';
}

let lastThrottlePulseSeq = 0;

function render(el) {
  if (!el) return;

  const tag = (cls, icon, text, title) =>
    `<span class="badge ${cls}"${title ? ` title="${title}"` : ''}><span class="badge-icon">${icon}</span> ${text}</span>`;
  const section = (content) => `<div class="top-bar__section">${content}</div>`;

  const conn = stateStore.getState();
  const wsBadge = conn.wsConnected ? 'badge-connected' : 'badge-disconnected';
  const workerBadge = workerBadgeClass(conn.workerStatus);
  const taskBadge = taskModeBadgeClass(conn.taskMode);
  const queueCount = conn.queueSize ?? 0;

  const thPct = conn.joyThrottlePercent;
  const thBase = throttleBadgeClass(thPct);
  const thPulse = conn.joyThrottlePulseSeq !== lastThrottlePulseSeq;
  if (thPulse) lastThrottlePulseSeq = conn.joyThrottlePulseSeq;
  const thCls = thBase + (thPulse ? ' badge-throttle--pulse' : '');
  const thText = thPct != null && Number.isFinite(thPct) ? `${Math.round(thPct)}%` : '油门 —';

  el.innerHTML = `
    <div class="top-bar__brand">
      <img src="SEALIEN-LOGO.png" alt="Orion" class="top-bar__logo" />
      <span class="top-bar__brand-title">Orion</span>
    </div>
    ${section(tag(wsBadge, '●', conn.wsConnected ? '已连接' : '未连接', conn.wsConnected ? '' : '需先启动 rosbridge (默认 ws://当前主机:9090，可用 ?ws= 覆盖)'))}
    ${section(tag(workerBadge, '⚙', '工作线程 ' + statusToLabel(conn.workerStatus)))}
    ${section(tag(taskBadge, '◇', '任务 ' + statusToLabel(conn.taskMode)))}
    ${section(tag('badge-queue', '☰', '队列 ' + queueCount))}
    ${section(tag(joyModeBadgeClass(conn.joyManualMode), '🎮', joyModeLabel(conn.joyManualMode), 'joy_manipulator：/joy_manipulator/manual_mode'))}
    ${section(tag(thCls, '⏱', thText, '臂油门 0～100%（/joy_manipulator/throttle_percent）；颜色随档位变化，变化时短暂高亮'))}
    <div class="top-bar__section top-bar__section--emergency" style="margin-left: auto;">
      <button type="button" id="btn-clear-queue" class="btn-secondary" title="cancel_job 清空待执行任务">清空队列</button>
      <button type="button" id="btn-reset-held" class="btn-secondary" title="reset_held_object">重置持物</button>
    </div>
  `;

  el.querySelector('#btn-clear-queue')?.addEventListener('click', () => window.dispatchEvent(new CustomEvent('orion:clear-queue')));
  el.querySelector('#btn-reset-held')?.addEventListener('click', () => window.dispatchEvent(new CustomEvent('orion:reset-held')));
}

function mount(containerId) {
  const el = document.getElementById(containerId);
  if (!el) return;

  render(el);
  stateStore.subscribe(() => render(el));
}

export default { mount, render };
