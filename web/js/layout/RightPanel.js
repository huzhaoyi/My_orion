/**
 * 右侧栏：操作与参数，Tab = 任务操作 | 运行策略 | 调试工具 | 参数配置
 */

function mount(containerId) {
  const el = document.getElementById(containerId);
  if (!el) return;

  const tabs = [
    { id: 'task', label: '任务操作', content: renderTaskTab },
    { id: 'policy', label: '运行策略', content: renderPolicyTab },
    { id: 'debug', label: '调试工具', content: renderDebugTab },
    { id: 'params', label: '参数配置', content: renderParamsTab },
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
    tabs.forEach((t, i) => {
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
  container.innerHTML = `
    <div class="card">
      <div class="card-title">Pick</div>
      <div class="form-actions form-actions--row">
        <button type="button" id="btn-pick-send" class="primary btn-action">发送 Pick</button>
        <button type="button" id="btn-pick-queue" class="btn-secondary">加入队列</button>
      </div>
      <div class="form-row"><label>object_id</label><input type="text" id="input-object-id" placeholder="可选" style="flex:1; max-width:140px;"></div>
      <p style="font-size:11px; color:var(--text-muted); margin:4px 0 0 0;">使用当前 object_pose</p>
    </div>
    <div class="card">
      <div class="card-title">Place</div>
      <div class="form-actions form-actions--row">
        <button type="button" id="btn-place-send" class="primary btn-action">发送 Place</button>
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
      <div class="card-title">PlaceRelease</div>
      <div class="form-actions form-actions--row">
        <button type="button" id="btn-place-release-send" class="primary btn-action">发送 PlaceRelease</button>
        <button type="button" id="btn-place-release-queue" class="btn-secondary">加入队列</button>
      </div>
      <p style="font-size:11px; color:var(--text-muted); margin:4px 0 0 0;">TCP 目标位姿（同 Place 输入框）</p>
    </div>
    <div class="card">
      <div class="card-title">夹爪</div>
      <p style="font-size:11px; color:var(--text-muted); margin:0 0 8px 0;">仅动夹爪，臂关节保持当前 joint_states</p>
      <div class="form-actions form-actions--row">
        <button type="button" id="btn-open-gripper" class="primary btn-action">打开夹爪</button>
        <button type="button" id="btn-close-gripper" class="btn-action">关闭夹爪</button>
      </div>
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
  container.querySelector('#btn-open-gripper')?.addEventListener('click', () => {
    window.dispatchEvent(new CustomEvent('orion:open-gripper'));
  });
  container.querySelector('#btn-close-gripper')?.addEventListener('click', () => {
    window.dispatchEvent(new CustomEvent('orion:close-gripper'));
  });
}

function renderPolicyTab(container) {
  container.innerHTML = `
    <div class="card">
      <div class="card-title">运行策略 (RuntimePolicy)</div>
      <div class="form-row"><label>auto_start_worker</label><input type="checkbox" id="policy-auto-start" checked></div>
      <div class="form-row"><label>retry_on_plan_failure</label><input type="checkbox" id="policy-retry"></div>
      <div class="form-row"><label>max_retries</label><input type="number" id="policy-max-retries" value="2" min="0"></div>
      <div class="form-row"><label>auto_clear_scene_before_sync</label><input type="checkbox" id="policy-clear-scene" checked></div>
      <div class="form-row"><label>auto_reset_after_execution_failure</label><input type="checkbox" id="policy-auto-reset"></div>
      <div class="form-row"><label>auto_go_home_after_failure</label><input type="checkbox" id="policy-go-home"></div>
      <div class="form-row"><label>allow_place_fallback_to_release</label><input type="checkbox" id="policy-place-fallback"></div>
      <div class="form-row"><label>reject_new_jobs_while_busy</label><input type="checkbox" id="policy-reject-busy"></div>
      <button type="button" id="btn-apply-policy" class="primary">应用策略</button>
    </div>
  `;
  container.querySelector('#btn-apply-policy')?.addEventListener('click', () => window.dispatchEvent(new CustomEvent('orion:apply-policy', {
    detail: {
      auto_start_worker: container.querySelector('#policy-auto-start').checked,
      retry_on_plan_failure: container.querySelector('#policy-retry').checked,
      max_retries: parseInt(container.querySelector('#policy-max-retries').value, 10) || 2,
      auto_clear_scene_before_sync: container.querySelector('#policy-clear-scene').checked,
      auto_reset_after_execution_failure: container.querySelector('#policy-auto-reset').checked,
      auto_go_home_after_failure: container.querySelector('#policy-go-home').checked,
      allow_place_fallback_to_release: container.querySelector('#policy-place-fallback').checked,
      reject_new_jobs_while_busy: container.querySelector('#policy-reject-busy').checked,
    },
  })));
}

function renderDebugTab(container) {
  container.innerHTML = `
    <div class="card">
      <div class="card-title">调试工具</div>
      <button type="button" id="btn-open-gripper">打开夹爪</button>
      <button type="button" id="btn-close-gripper">关闭夹爪</button>
      <button type="button" id="btn-sync-tracked">Sync tracked held object</button>
      <button type="button" id="btn-sync-untracked">Sync untracked held object</button>
      <button type="button" id="btn-reset-held">ResetHeldObject</button>
      <button type="button" id="btn-clear-attached">Clear attached residual</button>
      <button type="button" id="btn-remove-world">Remove world object</button>
      <button type="button" id="btn-go-home">手动 Go Home</button>
      <button type="button" id="btn-reload-model">重新加载模型</button>
    </div>
  `;
  container.querySelector('#btn-open-gripper')?.addEventListener('click', () => window.dispatchEvent(new CustomEvent('orion:open-gripper')));
  container.querySelector('#btn-close-gripper')?.addEventListener('click', () => window.dispatchEvent(new CustomEvent('orion:close-gripper')));
  container.querySelector('#btn-sync-tracked')?.addEventListener('click', () => window.dispatchEvent(new CustomEvent('orion:sync-held', { detail: { tracked: true } })));
  container.querySelector('#btn-sync-untracked')?.addEventListener('click', () => window.dispatchEvent(new CustomEvent('orion:sync-held', { detail: { tracked: false } })));
  container.querySelector('#btn-reset-held')?.addEventListener('click', () => window.dispatchEvent(new CustomEvent('orion:reset-held')));
  container.querySelector('#btn-clear-attached')?.addEventListener('click', () => window.dispatchEvent(new CustomEvent('orion:clear-attached')));
  container.querySelector('#btn-remove-world')?.addEventListener('click', () => window.dispatchEvent(new CustomEvent('orion:remove-world-object')));
  container.querySelector('#btn-go-home')?.addEventListener('click', () => window.dispatchEvent(new CustomEvent('orion:go-home')));
  container.querySelector('#btn-reload-model')?.addEventListener('click', () => window.dispatchEvent(new CustomEvent('orion:reload-model')));
}

function renderParamsTab(container) {
  container.innerHTML = `
    <div class="card">
      <div class="card-title">参数配置</div>
      <div class="form-row"><label>approach_distance</label><input type="number" id="param-approach" value="0.05" step="0.01"></div>
      <div class="form-row"><label>lift_distance</label><input type="number" id="param-lift" value="0.05" step="0.01"></div>
      <div class="form-row"><label>lower_to_place</label><input type="number" id="param-lower" value="0.02" step="0.01"></div>
      <div class="form-row"><label>retreat_distance</label><input type="number" id="param-retreat" value="0.05" step="0.01"></div>
      <div class="form-row"><label>velocity_scaling</label><input type="number" id="param-velocity" value="1" step="0.1" min="0.1" max="1"></div>
      <div class="form-row"><label>acceleration_scaling</label><input type="number" id="param-acceleration" value="1" step="0.1" min="0.1" max="1"></div>
      <button type="button" id="btn-apply-params" class="primary">应用参数</button>
    </div>
  `;
  container.querySelector('#btn-apply-params')?.addEventListener('click', () => window.dispatchEvent(new CustomEvent('orion:apply-params', {
    detail: {
      approach_distance: parseFloat(container.querySelector('#param-approach').value) || 0.05,
      lift_distance: parseFloat(container.querySelector('#param-lift').value) || 0.05,
      lower_to_place: parseFloat(container.querySelector('#param-lower').value) || 0.02,
      retreat_distance: parseFloat(container.querySelector('#param-retreat').value) || 0.05,
      velocity_scaling: parseFloat(container.querySelector('#param-velocity').value) || 1,
      acceleration_scaling: parseFloat(container.querySelector('#param-acceleration').value) || 1,
    },
  })));
}

export default { mount };
