/**
 * 顶部状态栏：连接状态、机器人状态、队列长度、快捷操作（与后端服务一致）
 */

import stateStore from '../data/stateStore.js';
import { t, getLocale, setLocale, subscribeLocale } from '../data/i18n.js';

/** worker_status 字符串 → CSS 徽章类（错误/恢复/运行/空闲）。 */
function workerBadgeClass(workerStatus) {
  const s = (workerStatus || '').toUpperCase();
  if (s.includes('ERROR') || s.includes('DISCONNECTED')) return 'badge-error';
  if (s.includes('RECOVERING') || s.includes('WARNING')) return 'badge-warning';
  if (s.includes('RUNNING') || s.includes('PICKING') || s.includes('HOLDING')) return 'badge-running';
  return 'badge-idle';
}

/** task_mode → 任务模式徽章配色。 */
function taskModeBadgeClass(taskMode) {
  const s = (taskMode || '').toUpperCase();
  if (s.includes('ERROR')) return 'badge-error';
  if (s.includes('RECOVERING') || s.includes('WARNING')) return 'badge-warning';
  if (s.includes('RUNNING') || s.includes('PICKING') || s.includes('HOLDING')) return 'badge-taskmode';
  return 'badge-taskmode';
}

/** 后端英文状态 → 顶栏短标签。 */
function statusToLabel(s) {
  if (!s) return t('status.idle');
  const u = (s + '').toUpperCase();
  if (u.includes('RUNNING') || u.includes('PICKING')) return t('status.running');
  if (u.includes('HOLDING')) return t('status.holding');
  if (u.includes('ERROR') || u.includes('DISCONNECTED')) return t('status.error');
  if (u.includes('RECOVERING') || u.includes('WARNING')) return t('status.recovering');
  if (u.includes('IDLE')) return t('status.idle');
  return s;
}

/** 手柄手动/自动/null 未知 → 徽章类。 */
function joyModeBadgeClass(manual) {
  if (manual === null || manual === undefined) return 'badge-idle';
  return manual ? 'badge-warning' : 'badge-running';
}

/** 手柄模式文案（含「—」无数据）。 */
function joyModeLabel(manual) {
  if (manual === null || manual === undefined) return t('top.joy_unknown');
  return manual ? t('top.manual') : t('top.auto');
}

/** 手柄油门徽章：按 0～100% 分档着色；无数据时用默认 info 色 */
function throttleBadgeClass(percent) {
  if (percent === null || percent === undefined || !Number.isFinite(percent)) return 'badge-throttle';
  if (percent < 34) return 'badge-throttle-low';
  if (percent < 67) return 'badge-throttle-mid';
  return 'badge-throttle-high';
}

let lastThrottlePulseSeq = 0;

/** 渲染顶栏 HTML 并绑定「清空队列」「重置持物」为全局 CustomEvent。 */
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
  const thText = thPct != null && Number.isFinite(thPct) ? `${Math.round(thPct)}%` : t('top.throttle_unknown');

  const loc = getLocale();
  const langZhActive = loc === 'zh' ? ' top-bar__lang-btn--active' : '';
  const langEnActive = loc === 'en' ? ' top-bar__lang-btn--active' : '';

  el.innerHTML = `
    <div class="top-bar__brand">
      <img src="SEALIEN-LOGO.png" alt="Orion" class="top-bar__logo" />
      <span class="top-bar__brand-title">${t('top.brand')}</span>
    </div>
    <div class="top-bar__section top-bar__lang" title="${t('lang.switch')}">
      <button type="button" class="top-bar__lang-btn${langZhActive}" data-locale="zh" aria-pressed="${loc === 'zh'}">${t('lang.zh')}</button>
      <span class="top-bar__lang-sep">/</span>
      <button type="button" class="top-bar__lang-btn${langEnActive}" data-locale="en" aria-pressed="${loc === 'en'}">${t('lang.en')}</button>
    </div>
    ${section(tag(wsBadge, '●', conn.wsConnected ? t('top.connected') : t('top.disconnected'), conn.wsConnected ? '' : t('top.ws_title_disconnected')))}
    ${section(tag(workerBadge, '⚙', t('top.worker') + ' ' + statusToLabel(conn.workerStatus)))}
    ${section(tag(taskBadge, '◇', t('top.task') + ' ' + statusToLabel(conn.taskMode)))}
    ${section(tag('badge-queue', '☰', t('top.queue') + ' ' + queueCount))}
    ${section(tag(joyModeBadgeClass(conn.joyManualMode), '🎮', joyModeLabel(conn.joyManualMode), t('top.joy_title')))}
    ${section(tag(thCls, '⏱', thText, t('top.throttle_title')))}
    <div class="top-bar__section top-bar__section--emergency" style="margin-left: auto;">
      <button type="button" id="btn-clear-queue" class="btn-secondary" title="${t('top.clear_queue_title')}">${t('top.clear_queue')}</button>
      <button type="button" id="btn-reset-held" class="btn-secondary" title="${t('top.reset_held_title')}">${t('top.reset_held')}</button>
    </div>
  `;

  el.querySelector('#btn-clear-queue')?.addEventListener('click', () => window.dispatchEvent(new CustomEvent('orion:clear-queue')));
  el.querySelector('#btn-reset-held')?.addEventListener('click', () => window.dispatchEvent(new CustomEvent('orion:reset-held')));
  el.querySelectorAll('.top-bar__lang-btn').forEach((btn) => {
    btn.addEventListener('click', () => {
      const next = btn.getAttribute('data-locale');
      if (next === 'zh' || next === 'en') {
        setLocale(next);
      }
    });
  });
}

/** 挂载到 #containerId，订阅 stateStore 与语言变更以刷新。 */
function mount(containerId) {
  const el = document.getElementById(containerId);
  if (!el) return;

  const rerender = () => render(el);
  rerender();
  stateStore.subscribe(rerender);
  subscribeLocale(rerender);
}

export default { mount, render };
