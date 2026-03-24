/**
 * 右侧栏任务页顺序：急停与回位 → 审批结果 → 抓取 → 夹爪 → 工作空间提示；另含调试 Tab
 * Tab = 任务 | 调试
 */

import { getWorkspaceBoundsForDoc } from '../robot/RobotModelLoader.js';
import ApprovalCard from '../panels/ApprovalCard.js';

function mount(containerId) {
  const el = document.getElementById(containerId);
  if (!el) return;

  const tabs = [
    { id: 'task', label: '任务', content: renderTaskTab },
    { id: 'debug', label: '调试', content: renderDebugTab },
  ];

  const tabBar = document.createElement('div');
  tabBar.className = 'tabs right-panel__tabs';
  tabs.forEach((t, i) => {
    const btn = document.createElement('button');
    btn.type = 'button';
    btn.className = 'tab' + (i === 0 ? ' active' : '');
    btn.textContent = t.label;
    btn.dataset.tab = t.id;
    tabBar.appendChild(btn);
  });

  const content = document.createElement('div');
  content.className = 'right-panel__content';
  content.id = 'right-panel-content';

  el.appendChild(tabBar);
  el.appendChild(content);

  function showTab(id) {
    tabs.forEach((t) => {
      const btn = tabBar.querySelector(`[data-tab="${t.id}"]`);
      if (btn) btn.classList.toggle('active', t.id === id);
    });
    content.innerHTML = '';
    const tab = tabs.find((t) => t.id === id);
    if (tab && tab.content) tab.content(content);
  }

  tabBar.querySelectorAll('.tab').forEach((btn) => {
    btn.addEventListener('click', () => showTab(btn.dataset.tab));
  });

  showTab('task');
}

function renderTaskTab(container) {
  const ws = getWorkspaceBoundsForDoc();
  const u = ws.urdf_frame;
  container.innerHTML = `
    <div class="card">
      <div class="card-title">急停与回位</div>
      <p style="font-size:11px; color:var(--text-muted); margin:0 0 8px 0;">emergency_stop / go_to_ready 服务</p>
      <div class="form-actions form-actions--row">
        <button type="button" id="btn-task-emergency-stop" class="btn-secondary" style="background:#b91c1c;border-color:#991b1b;color:#fff;font-weight:600;">急停</button>
        <button type="button" id="btn-task-go-ready" class="btn-secondary">回 ready</button>
      </div>
    </div>
    <div class="card approval-card">
      <div class="card-title">审批结果</div>
      <p style="font-size:11px; color:var(--text-muted); margin:0 0 8px 0;">check_pick 服务；无物体位姿时不可用</p>
      <div class="form-actions form-actions--row">
        <button type="button" id="btn-approval-pick" class="primary btn-action">审批抓取</button>
      </div>
      <div id="approval-result-container"></div>
    </div>
    <div class="card">
      <div class="card-title">抓取</div>
      <div class="form-actions form-actions--row">
        <button type="button" id="btn-pick-send" class="primary btn-action">提交抓取</button>
      </div>
      <p style="font-size:11px; color:var(--text-muted); margin:4px 0 0 0;">submit_job（异步入队），目标为当前话题物体位姿</p>
    </div>
    <div class="card">
      <div class="card-title">夹爪</div>
      <p style="font-size:11px; color:var(--text-muted); margin:0 0 8px 0;">仅动夹爪，臂关节保持当前姿态</p>
      <div class="form-actions form-actions--row">
        <button type="button" id="btn-open-gripper" class="primary btn-action">打开夹爪</button>
        <button type="button" id="btn-close-gripper" class="btn-secondary">闭合夹爪</button>
      </div>
    </div>
    <div class="workspace-hint">
      <span class="workspace-hint__label">工作空间</span>
      <span class="workspace-hint__axes">X <var>${u.x_m.min.toFixed(2)}</var>～<var>${u.x_m.max.toFixed(2)}</var></span>
      <span class="workspace-hint__axes">Y <var>${u.y_m.min.toFixed(2)}</var>～<var>${u.y_m.max.toFixed(2)}</var></span>
      <span class="workspace-hint__axes">Z <var>${u.z_m.min.toFixed(2)}</var>～<var>${u.z_m.max.toFixed(2)}</var></span>
      <span class="workspace-hint__unit">m</span>
      <span class="workspace-hint__note">勿超出</span>
    </div>
  `;
  container.querySelector('#btn-pick-send')?.addEventListener('click', () => window.dispatchEvent(new CustomEvent('orion:pick')));
  container.querySelector('#btn-task-emergency-stop')?.addEventListener('click', () => window.dispatchEvent(new CustomEvent('orion:emergency-stop')));
  container.querySelector('#btn-task-go-ready')?.addEventListener('click', () => window.dispatchEvent(new CustomEvent('orion:go-to-ready')));
  container.querySelector('#btn-open-gripper')?.addEventListener('click', () => window.dispatchEvent(new CustomEvent('orion:open-gripper')));
  container.querySelector('#btn-close-gripper')?.addEventListener('click', () => window.dispatchEvent(new CustomEvent('orion:close-gripper')));
  container.querySelector('#btn-approval-pick')?.addEventListener('click', (e) => ApprovalCard.handlePickClick(e));

  ApprovalCard.renderResult(container.querySelector('#approval-result-container'));
}

function renderDebugTab(container) {
  container.innerHTML = `
    <div class="card">
      <div class="card-title">调试工具</div>
      <div class="form-actions form-actions--row" style="flex-direction: column; align-items: stretch;">
        <button type="button" id="btn-reset-held" class="btn-action">重置持物</button>
        <button type="button" id="btn-sync-tracked" class="btn-secondary">同步场景（已跟踪）</button>
        <button type="button" id="btn-sync-untracked" class="btn-secondary">同步场景（未跟踪）</button>
        <label style="display:flex; align-items:center; gap:8px; margin-top:8px; font-size:12px; color:var(--text-secondary);">
          <input type="checkbox" id="debug-show-collision"> 显示碰撞体
        </label>
      </div>
    </div>
  `;
  container.querySelector('#btn-reset-held')?.addEventListener('click', () => window.dispatchEvent(new CustomEvent('orion:reset-held')));
  container.querySelector('#btn-sync-tracked')?.addEventListener('click', () => window.dispatchEvent(new CustomEvent('orion:sync-held', { detail: { tracked: true } })));
  container.querySelector('#btn-sync-untracked')?.addEventListener('click', () => window.dispatchEvent(new CustomEvent('orion:sync-held', { detail: { tracked: false } })));
  const collisionCb = container.querySelector('#debug-show-collision');
  if (collisionCb) {
    collisionCb.addEventListener('change', () => {
      window.dispatchEvent(new CustomEvent('orion:toggle-show-collision', { detail: { visible: collisionCb.checked } }));
    });
  }
}

export default { mount };
