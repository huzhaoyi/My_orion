/**
 * 底部：事件流 (JobEvent / TaskStage) + 系统日志，带筛选与清空
 */

import stateStore from '../data/stateStore.js';

const TAB_EVENTS = 'events';
const TAB_SYSTEM = 'system';

function mount(containerId) {
  const el = document.getElementById(containerId);
  if (!el) return;

  const tabBar = document.createElement('div');
  tabBar.className = 'bottom-panel__tabs';
  const btnEvents = document.createElement('button');
  btnEvents.type = 'button';
  btnEvents.className = 'bottom-panel__tab active';
  btnEvents.textContent = '事件流';
  btnEvents.dataset.tab = TAB_EVENTS;
  const btnRecent = document.createElement('button');
  btnRecent.type = 'button';
  btnRecent.className = 'bottom-panel__tab';
  btnRecent.textContent = '最近执行';
  btnRecent.dataset.tab = TAB_RECENT_JOBS;
  const btnSystem = document.createElement('button');
  btnSystem.type = 'button';
  btnSystem.className = 'bottom-panel__tab';
  btnSystem.textContent = '系统日志';
  btnSystem.dataset.tab = TAB_SYSTEM;
  tabBar.appendChild(btnEvents);
  tabBar.appendChild(btnRecent);
  tabBar.appendChild(btnSystem);

  const toolbar = document.createElement('div');
  toolbar.className = 'bottom-panel__toolbar';
  toolbar.innerHTML = `
    <select id="log-level">
      <option value="all">全部</option>
      <option value="error">Error</option>
      <option value="warn">Warn</option>
      <option value="info">Info</option>
      <option value="success">Success</option>
    </select>
    <input type="text" id="log-search" placeholder="搜索" style="width:120px;">
    <label><input type="checkbox" id="log-autoscroll" checked> 自动滚动</label>
    <button type="button" id="log-clear">清空</button>
  `;

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
      const type = e.event_type || e.task_type || e.stage_name || '—';
      const id = e.job_id || '—';
      const detail = e.reason || e.detail || e.stage_state || '';
      return `<div class="log-line">${ts} | ${type} | ${id} ${detail ? '| ' + detail : ''}</div>`;
    }).join('') || '<div class="log-line" style="color:var(--text-muted);">暂无事件</div>';
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
        <button type="button" id="log-refresh-recent" class="primary">刷新 (get_recent_jobs)</button>
      </div>
      <div class="bottom-panel__recent-list">
        ${records.length === 0
          ? '<div class="log-line" style="color:var(--text-muted);">暂无记录，点击刷新从后端拉取</div>'
          : records.map((r) => {
              const created = nsToStr(r.created_at_ns);
              const result = r.result_code != null ? `code=${r.result_code}` : '';
              const msg = (r.message || '').slice(0, 80);
              return `<div class="log-line">${created} | ${r.job_type || '—'} | ${(r.job_id || '—').slice(0, 12)} | ${r.source || '—'} ${result} ${msg ? '| ' + msg : ''}</div>`;
            }).join('')}
      </div>
    `;
    logContainer.querySelector('#log-refresh-recent')?.addEventListener('click', () => {
      if (!wsClient.isConnected()) {
        stateStore.pushSystemLog('warn', '未连接，无法获取最近执行');
        return;
      }
      wsClient.getRecentJobs(50, (res) => {
        const v = res && res.values ? res.values : res;
        const list = (v && v.records) ? v.records : (Array.isArray(v) ? v : []);
        stateStore.setRecentJobs(list);
        stateStore.pushSystemLog('info', `已拉取 ${list.length} 条最近执行`);
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
    logContainer.innerHTML = list.map((l) => {
      const cls = l.level === 'error' ? 'log-error' : l.level === 'warn' ? 'log-warn' : l.level === 'success' ? 'log-success' : 'log-info';
      return `<div class="log-line ${cls}">${l.ts} [${l.level}] ${l.message}</div>`;
    }).join('') || '<div class="log-line" style="color:var(--text-muted);">暂无日志</div>';
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

  stateStore.subscribe(refresh);
  refresh();
}

export default { mount };
