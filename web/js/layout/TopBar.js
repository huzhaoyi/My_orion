/**
 * 顶部状态栏：连接状态、机器人状态、当前任务、队列长度、急停/停止、当前模式
 */

import stateStore from '../data/stateStore.js';

function workerBadgeClass(workerStatus) {
  const s = (workerStatus || '').toUpperCase();
  if (s.includes('ERROR') || s.includes('DISCONNECTED')) return 'badge-error';
  if (s.includes('RECOVERING') || s.includes('WARNING')) return 'badge-warning';
  if (s.includes('RUNNING') || s.includes('PICKING') || s.includes('PLACING') || s.includes('HOLDING')) return 'badge-running';
  return 'badge-idle';
}

function taskModeBadgeClass(taskMode) {
  const s = (taskMode || '').toUpperCase();
  if (s.includes('ERROR')) return 'badge-error';
  if (s.includes('RECOVERING') || s.includes('WARNING')) return 'badge-warning';
  if (s.includes('RUNNING') || s.includes('PICKING') || s.includes('PLACING') || s.includes('HOLDING')) return 'badge-taskmode';
  return 'badge-taskmode';
}

function render(el) {
  if (!el) return;

  const tag = (cls, icon, text, title) =>
    `<span class="badge ${cls}"${title ? ` title="${title}"` : ''}><span class="badge-icon">${icon}</span> ${text}</span>`;
  const section = (content) => `<div class="top-bar__section">${content}</div>`;

  const conn = stateStore.getState();
  const wsBadge = conn.wsConnected ? 'badge-connected' : 'badge-disconnected';
  const workerBadge = workerBadgeClass(conn.workerStatus);
  const taskBadge = taskModeBadgeClass(conn.taskMode);
  const jobLabel = (conn.currentJobType || 'NONE').toUpperCase().slice(0, 12);
  const acceptNewJobs = conn.acceptNewJobs !== false;

  el.innerHTML = `
    <div class="top-bar__brand">
      <img src="SEALIEN-LOGO.png" alt="SEALIEN" class="top-bar__logo" />
      <span class="top-bar__brand-title">Orion 上位机</span>
    </div>
    ${section(tag(wsBadge, '●', conn.wsConnected ? 'WS CONNECTED' : 'WS DISCONNECTED', conn.wsConnected ? '' : '需先启动 rosbridge (默认 ws://localhost:9090)，或访问 ?ws=ws://host:port'))}
    ${section(tag(workerBadge, '⚙', 'WORKER ' + (conn.workerStatus || 'IDLE').toUpperCase()))}
    ${section(tag(taskBadge, '◇', 'TASK MODE ' + (conn.taskMode || 'IDLE').toUpperCase()))}
    ${section(tag('badge-job', '▣', 'JOB ' + jobLabel, conn.currentJobId || ''))}
    ${section(tag('badge-queue', '☰', 'QUEUE ' + (conn.queueSize ?? 0)))}
    <div class="top-bar__section" style="margin-left: auto;">
      <button type="button" id="btn-stop-queue" title="仅前端拦截 submit_job，不影响后端已有队列">${acceptNewJobs ? '停止入队' : '恢复入队'}</button>
      <button type="button" id="btn-clear-queue" title="清空队列">清空队列</button>
      <button type="button" id="btn-reset-held" title="Reset 持物">ResetHeld</button>
      <button type="button" id="btn-recover" title="恢复">Recover</button>
      <button type="button" id="btn-go-home" title="回 Home">Home</button>
    </div>
  `;

  el.querySelector('#btn-stop-queue')?.addEventListener('click', () => window.dispatchEvent(new CustomEvent('orion:stop-queue')));
  el.querySelector('#btn-clear-queue')?.addEventListener('click', () => window.dispatchEvent(new CustomEvent('orion:clear-queue')));
  el.querySelector('#btn-reset-held')?.addEventListener('click', () => window.dispatchEvent(new CustomEvent('orion:reset-held')));
  el.querySelector('#btn-recover')?.addEventListener('click', () => window.dispatchEvent(new CustomEvent('orion:recover')));
  el.querySelector('#btn-go-home')?.addEventListener('click', () => window.dispatchEvent(new CustomEvent('orion:go-home')));
}

function mount(containerId) {
  const el = document.getElementById(containerId);
  if (!el) return;

  render(el);
  stateStore.subscribe(() => render(el));
}

export default { mount, render };
