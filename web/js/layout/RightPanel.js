/**
 * 右侧栏任务页顺序：急停与回位 → 审批结果 → 抓取 → 夹爪 → 工作空间提示；另含调试 Tab
 * Tab = 任务 | 调试
 */

import { FEASIBILITY_WORKSPACE } from '../data/feasibilityWorkspace.js';
import { getWorkspaceBoundsForDoc } from '../robot/RobotModelLoader.js';
import ApprovalCard from '../panels/ApprovalCard.js';
import { t, subscribeLocale } from '../data/i18n.js';

/** 右侧栏：任务/调试 Tab，任务页绑定 CustomEvent 与 ApprovalCard。 */
function mount(containerId) {
  const el = document.getElementById(containerId);
  if (!el) return;

  let activeTabId = 'task';

  const tabBar = document.createElement('div');
  tabBar.className = 'tabs right-panel__tabs';

  const content = document.createElement('div');
  content.className = 'right-panel__content';
  content.id = 'right-panel-content';

  el.appendChild(tabBar);
  el.appendChild(content);

  function tabSpecs() {
    return [
      { id: 'task', label: t('right.tab_task'), content: renderTaskTab },
      { id: 'debug', label: t('right.tab_debug'), content: renderDebugTab },
    ];
  }

  function rebuildTabButtons() {
    tabBar.innerHTML = '';
    tabSpecs().forEach((spec, i) => {
      const btn = document.createElement('button');
      btn.type = 'button';
      btn.className = 'tab' + (spec.id === activeTabId ? ' active' : '');
      btn.textContent = spec.label;
      btn.dataset.tab = spec.id;
      btn.addEventListener('click', () => showTab(spec.id));
      tabBar.appendChild(btn);
    });
  }

  function showTab(id) {
    activeTabId = id;
    tabBar.querySelectorAll('.tab').forEach((btn) => {
      btn.classList.toggle('active', btn.dataset.tab === id);
    });
    content.innerHTML = '';
    const spec = tabSpecs().find((x) => x.id === id);
    if (spec && spec.content) spec.content(content);
  }

  rebuildTabButtons();
  showTab(activeTabId);

  subscribeLocale(() => {
    rebuildTabButtons();
    showTab(activeTabId);
  });
}

/** 任务 Tab：急停、审批、抓取、夹爪、工作空间提示与工作空间 AABB 文案。 */
function renderTaskTab(container) {
  const F = FEASIBILITY_WORKSPACE;
  const ws = getWorkspaceBoundsForDoc();
  const u = ws.urdf_frame;
  const f2 = (v) => Number(v).toFixed(2);
  container.innerHTML = `
    <div class="card">
      <div class="card-title">${t('right.emergency_ready_title')}</div>
      <p style="font-size:11px; color:var(--text-muted); margin:0 0 8px 0;">${t('right.emergency_ready_hint')}</p>
      <div class="form-actions form-actions--row">
        <button type="button" id="btn-task-emergency-stop" class="btn-secondary" style="background:#b91c1c;border-color:#991b1b;color:#fff;font-weight:600;">${t('right.emergency_stop')}</button>
        <button type="button" id="btn-task-go-ready" class="btn-secondary">${t('right.go_ready')}</button>
      </div>
    </div>
    <div class="card approval-card">
      <div class="card-title">${t('right.approval_title')}</div>
      <p style="font-size:11px; color:var(--text-muted); margin:0 0 8px 0;">${t('right.approval_hint')}</p>
      <div class="form-actions form-actions--row">
        <button type="button" id="btn-approval-pick" class="primary btn-action">${t('right.approval_btn')}</button>
      </div>
      <div id="approval-result-container"></div>
    </div>
    <div class="card">
      <div class="card-title">${t('right.pick_title')}</div>
      <div class="form-actions form-actions--row" style="margin-bottom:8px;">
        <button type="button" id="btn-pick-cable" class="primary btn-action" style="flex:1;min-width:0;" title="${t('right.pick_cable_title')}">${t('right.pick_cable')}</button>
      </div>
      <div class="form-actions form-actions--row" style="margin-bottom:8px;">
        <button type="button" id="btn-pick-target-sensor" class="primary btn-action" style="flex:1;min-width:0;background:#c2410c;border-color:#9a3412;" title="${t('right.pick_target_sensor_title')}">${t('right.pick_target_sensor')}</button>
      </div>
      <div class="form-actions form-actions--row">
        <button type="button" id="btn-pick-fused" class="primary btn-action" style="flex:1;min-width:0;background:#a21caf;border-color:#86198f;" title="${t('right.pick_fused_title')}">${t('right.pick_fused')}</button>
      </div>
      <p style="font-size:11px; color:var(--text-muted); margin:8px 0 0 0;">${t('right.pick_hint')}</p>
    </div>
    <div class="card">
      <div class="card-title">${t('right.gripper_title')}</div>
      <p style="font-size:11px; color:var(--text-muted); margin:0 0 8px 0;">${t('right.gripper_hint')}</p>
      <div class="form-actions form-actions--row">
        <button type="button" id="btn-open-gripper" class="primary btn-action">${t('right.open_gripper')}</button>
        <button type="button" id="btn-close-gripper" class="btn-secondary">${t('right.close_gripper')}</button>
      </div>
    </div>
    <div class="workspace-hint-stack">
      <div class="workspace-hint workspace-hint--compact" title="${t('right.ws_title')}">
        <span class="workspace-hint__label">${t('right.ws_label')}</span>
        <span class="workspace-hint__axes">X <var>${f2(u.x_m.min)}</var>～<var>${f2(u.x_m.max)}</var> Y <var>${f2(u.y_m.min)}</var>～<var>${f2(u.y_m.max)}</var> Z <var>${f2(u.z_m.min)}</var>～<var>${f2(u.z_m.max)}</var> m</span>
        <span class="workspace-hint__note">${t('right.ws_note')}</span>
      </div>
      <div class="workspace-hint workspace-hint--compact" title="${t('right.target_cable_title')}">
        <span class="workspace-hint__label">${t('right.target_cable')}</span>
        <span class="workspace-hint__axes">‖p‖ <var>${f2(F.min_reach_safe_m)}</var>～<var>${f2(F.max_reach_hard_m)}</var> m Z <var>${f2(F.z_min_m)}</var>～<var>${f2(F.z_max_m)}</var> m 软‖p‖ <var>${f2(F.max_reach_soft_m)}</var> m</span>
        <span class="workspace-hint__note">${t('right.ws_note')}</span>
      </div>
    </div>
  `;
  container.querySelector('#btn-pick-cable')?.addEventListener('click', () => window.dispatchEvent(new CustomEvent('orion:pick:cable')));
  container.querySelector('#btn-pick-target-sensor')?.addEventListener('click', () => window.dispatchEvent(new CustomEvent('orion:pick:target_sensor')));
  container.querySelector('#btn-pick-fused')?.addEventListener('click', () => window.dispatchEvent(new CustomEvent('orion:pick:fused')));
  container.querySelector('#btn-task-emergency-stop')?.addEventListener('click', () => window.dispatchEvent(new CustomEvent('orion:emergency-stop')));
  container.querySelector('#btn-task-go-ready')?.addEventListener('click', () => window.dispatchEvent(new CustomEvent('orion:go-to-ready')));
  container.querySelector('#btn-open-gripper')?.addEventListener('click', () => window.dispatchEvent(new CustomEvent('orion:open-gripper')));
  container.querySelector('#btn-close-gripper')?.addEventListener('click', () => window.dispatchEvent(new CustomEvent('orion:close-gripper')));
  container.querySelector('#btn-approval-pick')?.addEventListener('click', (e) => ApprovalCard.handlePickClick(e));

  ApprovalCard.renderResult(container.querySelector('#approval-result-container'));
}

/** 调试 Tab：持物重置、同步场景、碰撞体显示开关（发 orion:toggle-show-collision）。 */
function renderDebugTab(container) {
  container.innerHTML = `
    <div class="card">
      <div class="card-title">${t('right.debug_title')}</div>
      <div class="form-actions form-actions--row" style="flex-direction: column; align-items: stretch;">
        <button type="button" id="btn-reset-held" class="btn-action">${t('top.reset_held')}</button>
        <button type="button" id="btn-sync-tracked" class="btn-secondary">${t('right.sync_tracked')}</button>
        <button type="button" id="btn-sync-untracked" class="btn-secondary">${t('right.sync_untracked')}</button>
        <label style="display:flex; align-items:center; gap:8px; margin-top:8px; font-size:12px; color:var(--text-secondary);">
          <input type="checkbox" id="debug-show-collision"> ${t('right.show_collision')}
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
