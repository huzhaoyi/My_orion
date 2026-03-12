/**
 * 左侧 - 最近一次错误卡片（固定显示，便于排查）
 */

import stateStore from '../data/stateStore.js';

function render(parentEl) {
  if (!parentEl) return;
  const wrap = document.createElement('div');
  wrap.className = 'card';
  wrap.style.borderLeft = '3px solid var(--color-error)';
  wrap.id = 'last-error-card';
  parentEl.appendChild(wrap);

  function update() {
    const s = stateStore.getState();
    const err = s.lastError && s.lastError.trim();
    wrap.innerHTML = `
      <div class="card-title">最近错误</div>
      <div class="card-row"><span class="card-value" style="word-break: break-all;">${err ? err : '无'}</span></div>
    `;
  }

  update();
  stateStore.subscribe(update);
}

export default { render };
