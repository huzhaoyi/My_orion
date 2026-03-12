/**
 * 右侧栏：工业机械臂控制区
 * Tab = 任务(Task) | 机器人(Robot) | 调试(Debug) | 配置(Config)
 * 主操作 = primary，次操作 = btn-secondary
 */

import { getWorkspaceBoundsForDoc } from '../robot/RobotModelLoader.js';

function mount(containerId) {
  const el = document.getElementById(containerId);
  if (!el) return;

  const tabs = [
    { id: 'task', label: '任务', content: renderTaskTab },
    { id: 'robot', label: '机器人', content: renderRobotTab },
    { id: 'debug', label: '调试', content: renderDebugTab },
    { id: 'config', label: '配置', content: renderConfigTab },
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
    <div class="workspace-hint">
      <span class="workspace-hint__label">工作空间</span>
      <span class="workspace-hint__axes">X <var>${u.x_m.min.toFixed(2)}</var>～<var>${u.x_m.max.toFixed(2)}</var></span>
      <span class="workspace-hint__axes">Y <var>${u.y_m.min.toFixed(2)}</var>～<var>${u.y_m.max.toFixed(2)}</var></span>
      <span class="workspace-hint__axes">Z <var>${u.z_m.min.toFixed(2)}</var>～<var>${u.z_m.max.toFixed(2)}</var></span>
      <span class="workspace-hint__unit">m</span>
      <span class="workspace-hint__note">勿超出</span>
    </div>
    <div class="card">
      <div class="card-title">抓取</div>
      <div class="form-actions form-actions--row">
        <button type="button" id="btn-pick-send" class="primary btn-action">发送抓取</button>
        <button type="button" id="btn-pick-queue" class="btn-secondary">加入队列</button>
      </div>
      <div class="form-row"><label>物体ID</label><input type="text" id="input-object-id" placeholder="可选" style="flex:1; max-width:140px;"></div>
      <p style="font-size:11px; color:var(--text-muted); margin:4px 0 0 0;">使用当前物体位姿</p>
    </div>
    <div class="card">
      <div class="card-title">放置</div>
      <div class="form-actions form-actions--row">
        <button type="button" id="btn-place-send" class="primary btn-action">发送放置</button>
        <button type="button" id="btn-place-queue" class="btn-secondary">加入队列</button>
      </div>
      <div class="form-row form-row--xyz">
        <label>X</label><input type="number" id="place-x" value="0.45" step="0.01">
        <label>Y</label><input type="number" id="place-y" value="0" step="0.01">
        <label>Z</label><input type="number" id="place-z" value="0.4" step="0.01">
      </div>
      <div class="form-row form-row--quat">
        <label>qx</label><input type="number" id="place-qx" value="0" step="0.01">
        <label>qy</label><input type="number" id="place-qy" value="0" step="0.01">
        <label>qz</label><input type="number" id="place-qz" value="0" step="0.01">
        <label>qw</label><input type="number" id="place-qw" value="1" step="0.01">
      </div>
    </div>
    <div class="card">
      <div class="card-title">放置释放</div>
      <div class="form-actions form-actions--row">
        <button type="button" id="btn-place-release-send" class="primary btn-action">发送放置释放</button>
        <button type="button" id="btn-place-release-queue" class="btn-secondary">加入队列</button>
      </div>
      <p style="font-size:11px; color:var(--text-muted); margin:4px 0 0 0;">TCP 目标位姿（同放置输入框）</p>
    </div>
  `;
  const getObjectId = () => (container.querySelector('#input-object-id')?.value || '').trim();
  const getPlacePose = () => ({
    x: parseFloat(container.querySelector('#place-x')?.value) || 0.45,
    y: parseFloat(container.querySelector('#place-y')?.value) || 0,
    z: parseFloat(container.querySelector('#place-z')?.value) || 0.4,
    qx: parseFloat(container.querySelector('#place-qx')?.value) || 0,
    qy: parseFloat(container.querySelector('#place-qy')?.value) || 0,
    qz: parseFloat(container.querySelector('#place-qz')?.value) || 0,
    qw: parseFloat(container.querySelector('#place-qw')?.value) || 1,
  });
  container.querySelector('#btn-pick-send')?.addEventListener('click', () => window.dispatchEvent(new CustomEvent('orion:pick', { detail: { immediate: true, object_id: getObjectId() } })));
  container.querySelector('#btn-pick-queue')?.addEventListener('click', () => window.dispatchEvent(new CustomEvent('orion:pick', { detail: { immediate: false, object_id: getObjectId() } })));
  container.querySelector('#btn-place-send')?.addEventListener('click', () => {
    const p = getPlacePose();
    window.dispatchEvent(new CustomEvent('orion:place', { detail: { ...p, immediate: true } }));
  });
  container.querySelector('#btn-place-queue')?.addEventListener('click', () => {
    const p = getPlacePose();
    window.dispatchEvent(new CustomEvent('orion:place', { detail: { ...p, immediate: false } }));
  });
  container.querySelector('#btn-place-release-send')?.addEventListener('click', () => {
    const p = getPlacePose();
    window.dispatchEvent(new CustomEvent('orion:place-release', { detail: { ...p, immediate: true } }));
  });
  container.querySelector('#btn-place-release-queue')?.addEventListener('click', () => {
    const p = getPlacePose();
    window.dispatchEvent(new CustomEvent('orion:place-release', { detail: { ...p, immediate: false } }));
  });
}

function renderRobotTab(container) {
  container.innerHTML = `
    <div class="card">
      <div class="card-title">夹爪</div>
      <p style="font-size:11px; color:var(--text-muted); margin:0 0 8px 0;">仅动夹爪，臂关节保持当前姿态</p>
      <div class="form-actions form-actions--row">
        <button type="button" id="btn-open-gripper" class="primary btn-action">打开夹爪</button>
        <button type="button" id="btn-close-gripper" class="btn-secondary">闭合夹爪</button>
      </div>
    </div>
    <div class="card">
      <div class="card-title">运动</div>
      <div class="form-actions form-actions--row">
        <button type="button" id="btn-stop-queue" class="btn-secondary">停止入队</button>
      </div>
      <p style="font-size:11px; color:var(--text-muted); margin:4px 0 0 0;">停止：仅前端拦截入队；清空队列见顶部栏</p>
    </div>
  `;
  container.querySelector('#btn-open-gripper')?.addEventListener('click', () => window.dispatchEvent(new CustomEvent('orion:open-gripper')));
  container.querySelector('#btn-close-gripper')?.addEventListener('click', () => window.dispatchEvent(new CustomEvent('orion:close-gripper')));
  container.querySelector('#btn-stop-queue')?.addEventListener('click', () => window.dispatchEvent(new CustomEvent('orion:stop-queue')));
}

function renderDebugTab(container) {
  container.innerHTML = `
    <div class="card">
      <div class="card-title">调试工具</div>
      <div class="form-actions form-actions--row" style="flex-direction: column; align-items: stretch;">
        <button type="button" id="btn-reset-held" class="btn-action">重置持物</button>
        <button type="button" id="btn-recover" class="btn-action">恢复</button>
        <button type="button" id="btn-sync-tracked" class="btn-secondary">同步场景（已跟踪）</button>
        <button type="button" id="btn-sync-untracked" class="btn-secondary">同步场景（未跟踪）</button>
        <label style="display:flex; align-items:center; gap:8px; margin-top:8px; font-size:12px; color:var(--text-secondary);">
          <input type="checkbox" id="debug-show-collision"> 显示碰撞体
        </label>
      </div>
    </div>
    <div class="card">
      <div class="card-title">场景 / 模型</div>
      <div class="form-actions form-actions--row" style="flex-direction: column; align-items: stretch;">
        <button type="button" id="btn-clear-attached" class="btn-secondary">清除附着残留</button>
        <button type="button" id="btn-reload-model" class="btn-secondary">重新加载模型</button>
      </div>
    </div>
  `;
  container.querySelector('#btn-reset-held')?.addEventListener('click', () => window.dispatchEvent(new CustomEvent('orion:reset-held')));
  container.querySelector('#btn-recover')?.addEventListener('click', () => window.dispatchEvent(new CustomEvent('orion:recover')));
  container.querySelector('#btn-sync-tracked')?.addEventListener('click', () => window.dispatchEvent(new CustomEvent('orion:sync-held', { detail: { tracked: true } })));
  container.querySelector('#btn-sync-untracked')?.addEventListener('click', () => window.dispatchEvent(new CustomEvent('orion:sync-held', { detail: { tracked: false } })));
  container.querySelector('#btn-clear-attached')?.addEventListener('click', () => window.dispatchEvent(new CustomEvent('orion:clear-attached')));
  container.querySelector('#btn-reload-model')?.addEventListener('click', () => window.dispatchEvent(new CustomEvent('orion:reload-model')));
  const collisionCb = container.querySelector('#debug-show-collision');
  if (collisionCb) {
    collisionCb.addEventListener('change', () => {
      window.dispatchEvent(new CustomEvent('orion:toggle-show-collision', { detail: { visible: collisionCb.checked } }));
    });
  }
}

function renderConfigTab(container) {
  container.innerHTML = `
    <div class="card">
      <div class="card-title">规划参数与运行策略</div>
      <p style="font-size:11px; color:var(--text-muted); margin:0 0 10px 0;">由后端 launch 与配置文件加载，当前无运行时修改接口。</p>
      <div class="form-row"><label>接近距离</label><input type="number" id="param-approach" value="0.05" step="0.01" readonly disabled></div>
      <div class="form-row"><label>抬升距离</label><input type="number" id="param-lift" value="0.05" step="0.01" readonly disabled></div>
      <div class="form-row"><label>放置下压</label><input type="number" id="param-lower" value="0.02" step="0.01" readonly disabled></div>
      <div class="form-row"><label>回退距离</label><input type="number" id="param-retreat" value="0.05" step="0.01" readonly disabled></div>
      <div class="form-row"><label>速度/加速度比例</label><input type="text" value="由 config 加载" readonly disabled style="flex:1;"></div>
    </div>
  `;
}

export default { mount };
