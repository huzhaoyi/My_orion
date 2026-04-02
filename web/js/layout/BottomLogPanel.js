/**
 * 底部：事件流 (JobEvent / TaskStage) + 系统日志，带筛选与清空
 */

import stateStore from '../data/stateStore.js';
import wsClient from '../data/wsClient.js';
import {
  jobTypeLabel,
  stageNameLabel,
  stagePillClass,
  stageStateLabel,
  stageStateModifier,
} from '../data/labels.js';
import { t, subscribeLocale } from '../data/i18n.js';

const TAB_EVENTS = 'events';
const TAB_RECENT_JOBS = 'recent';
const TAB_SYSTEM = 'system';

/**
 * 底部面板：事件流（JobEvent+TaskStage）、最近任务、系统日志三 Tab；筛选/搜索/自动滚动/清空。
 */
function mount(containerId) {
  const el = document.getElementById(containerId);
  if (!el) return;

  const tabBar = document.createElement('div');
  tabBar.className = 'bottom-panel__tabs';
  const btnEvents = document.createElement('button');
  btnEvents.type = 'button';
  btnEvents.className = 'bottom-panel__tab active';
  btnEvents.textContent = t('bottom.tab_events');
  btnEvents.dataset.tab = TAB_EVENTS;
  const btnRecent = document.createElement('button');
  btnRecent.type = 'button';
  btnRecent.className = 'bottom-panel__tab';
  btnRecent.textContent = t('bottom.tab_recent');
  btnRecent.dataset.tab = TAB_RECENT_JOBS;
  const btnSystem = document.createElement('button');
  btnSystem.type = 'button';
  btnSystem.className = 'bottom-panel__tab';
  btnSystem.textContent = t('bottom.tab_system');
  btnSystem.dataset.tab = TAB_SYSTEM;
  tabBar.appendChild(btnEvents);
  tabBar.appendChild(btnRecent);
  tabBar.appendChild(btnSystem);

  const toolbar = document.createElement('div');
  toolbar.className = 'bottom-panel__toolbar';
  function toolbarHtml() {
    return `
    <select id="log-level">
      <option value="all">${t('bottom.level_all')}</option>
      <option value="error">${t('bottom.level_error')}</option>
      <option value="warn">${t('bottom.level_warn')}</option>
      <option value="info">${t('bottom.level_info')}</option>
      <option value="success">${t('bottom.level_success')}</option>
    </select>
    <input type="text" id="log-search" placeholder="${t('bottom.search_ph')}" style="width:120px;">
    <label><input type="checkbox" id="log-autoscroll" checked> ${t('bottom.autoscroll')}</label>
    <button type="button" id="log-clear">${t('bottom.clear')}</button>
  `;
  }
  toolbar.innerHTML = toolbarHtml();

  function bindToolbarHandlers() {
    toolbar.querySelector('#log-level')?.addEventListener('change', (e) => {
      levelFilter = e.target.value;
      refresh();
    });
    toolbar.querySelector('#log-search')?.addEventListener('input', (e) => {
      searchText = e.target.value.trim();
      refresh();
    });
    toolbar.querySelector('#log-autoscroll')?.addEventListener('change', (e) => {
      autoScroll = e.target.checked;
    });
    toolbar.querySelector('#log-clear')?.addEventListener('click', () => {
      if (currentTab === TAB_EVENTS) {
        stateStore.setState({ jobEvents: [], taskStages: [] });
      } else if (currentTab === TAB_RECENT_JOBS) {
        stateStore.setRecentJobs([]);
      } else {
        stateStore.setState({ systemLogs: [] });
      }
      refresh();
    });
  }

  const logContainer = document.createElement('div');
  logContainer.className = 'bottom-panel__log';
  logContainer.id = 'bottom-log-content';

  el.appendChild(tabBar);
  el.appendChild(toolbar);
  el.appendChild(logContainer);

  let currentTab = TAB_EVENTS;
  let levelFilter = 'all';
  let searchText = '';
  let autoScroll = true;

  function renderEvents() {
    const s = stateStore.getState();
    const events = [...(s.jobEvents || []), ...(s.taskStages || [])].sort((a, b) => (a._ts || 0) - (b._ts || 0));
    const list = events.slice(-80).reverse();
    logContainer.innerHTML = list.map((e) => {
      const ts = e.header?.stamp ? `${e.header.stamp.sec}.${String(e.header.stamp.nanosec || 0).slice(0, 3)}` : (e._ts ? new Date(e._ts).toLocaleTimeString() : '');
      const typeZh =
        (e.stage_name ? stageNameLabel(e.stage_name) : null) ||
        (e.task_type ? jobTypeLabel(e.task_type) : null) ||
        e.event_type ||
        '—';
      const id = e.job_id || '—';
      const detailPart = e.stage_state ? stageStateLabel(e.stage_state) : e.reason || e.detail || '';
      const pill =
        e.stage_name != null && String(e.stage_name).trim() !== ''
          ? `stage-pill ${stagePillClass(e.stage_name)} ${stageStateModifier(e.stage_state)}`.trim()
          : '';
      const typeHtml = pill ? `<span class="${pill}">${typeZh}</span>` : typeZh;
      return `<div class="log-line log-line--event">${ts} | ${typeHtml} | ${id} ${detailPart ? '| ' + detailPart : ''}</div>`;
    }).join('') || `<div class="log-line" style="color:var(--text-muted);">${t('bottom.no_events')}</div>`;
    if (autoScroll) logContainer.scrollTop = 0;
  }

  function renderRecentJobs() {
    const s = stateStore.getState();
    const records = s.recentJobs || [];
    const nsToStr = (ns) => {
      if (ns == null || ns === undefined) return '—';
      return new Date(Number(ns) / 1e6).toLocaleTimeString();
    };
    logContainer.innerHTML = `
      <div class="bottom-panel__recent-toolbar">
        <button type="button" id="log-refresh-recent" class="primary">${t('bottom.refresh_recent')}</button>
      </div>
      <div class="bottom-panel__recent-list">
        ${records.length === 0
          ? `<div class="log-line" style="color:var(--text-muted);">${t('bottom.no_recent')}</div>`
          : records.map((r) => {
              const created = nsToStr(r.created_at_ns);
              const result = r.result_code != null ? `${t('bottom.result_code')}=${r.result_code}` : '';
              const msg = (r.message || '').slice(0, 80);
              return `<div class="log-line">${created} | ${jobTypeLabel(r.job_type)} | ${(r.job_id || '—').slice(0, 12)} | ${r.source || '—'} ${result} ${msg ? '| ' + msg : ''}</div>`;
            }).join('')}
      </div>
    `;
    logContainer.querySelector('#log-refresh-recent')?.addEventListener('click', () => {
      if (!wsClient.isConnected()) {
        stateStore.pushSystemLog('warn', t('toast.not_connected_recent'));
        return;
      }
      wsClient.getRecentJobs(50, (res) => {
        const v = res && res.values ? res.values : res;
        const list = (v && v.records) ? v.records : (Array.isArray(v) ? v : []);
        stateStore.setRecentJobs(list);
        stateStore.pushSystemLog(
          'info',
          `${t('log.recent_jobs_loaded_a')}${list.length}${t('log.recent_jobs_loaded_b')}`
        );
        renderRecentJobs();
      });
    });
  }

  function renderSystem() {
    const s = stateStore.getState();
    let list = s.systemLogs || [];
    if (levelFilter !== 'all') list = list.filter((l) => l.level === levelFilter);
    if (searchText) list = list.filter((l) => (l.message || '').toLowerCase().includes(searchText.toLowerCase()));
    list = list.slice(-100).reverse();
    const levelLabel = {
      error: t('bottom.level_error'),
      warn: t('bottom.level_warn'),
      info: t('bottom.level_info'),
      success: t('bottom.level_success'),
    };
    logContainer.innerHTML = list.map((l) => {
      const cls = l.level === 'error' ? 'log-error' : l.level === 'warn' ? 'log-warn' : l.level === 'success' ? 'log-success' : 'log-info';
      const label = levelLabel[l.level] || l.level;
      return `<div class="log-line ${cls}">${l.ts} [${label}] ${l.message}</div>`;
    }).join('') || `<div class="log-line" style="color:var(--text-muted);">${t('bottom.no_logs')}</div>`;
    if (autoScroll) logContainer.scrollTop = 0;
  }

  function refresh() {
    if (currentTab === TAB_EVENTS) renderEvents();
    else if (currentTab === TAB_RECENT_JOBS) renderRecentJobs();
    else renderSystem();
  }

  tabBar.querySelectorAll('.bottom-panel__tab').forEach((btn) => {
    btn.addEventListener('click', () => {
      currentTab = btn.dataset.tab;
      tabBar.querySelectorAll('.bottom-panel__tab').forEach((b) => b.classList.remove('active'));
      btn.classList.add('active');
      refresh();
    });
  });

  subscribeLocale(() => {
    btnEvents.textContent = t('bottom.tab_events');
    btnRecent.textContent = t('bottom.tab_recent');
    btnSystem.textContent = t('bottom.tab_system');
    toolbar.innerHTML = toolbarHtml();
    const lvlEl = toolbar.querySelector('#log-level');
    if (lvlEl) {
      lvlEl.value = levelFilter;
    }
    const searchEl = toolbar.querySelector('#log-search');
    if (searchEl) {
      searchEl.value = searchText;
    }
    const scrollEl = toolbar.querySelector('#log-autoscroll');
    if (scrollEl) {
      scrollEl.checked = autoScroll;
    }
    bindToolbarHandlers();
    refresh();
  });

  stateStore.subscribe(refresh);
  refresh();
}

export default { mount };
