/**
 * 左侧 - 队列列表卡片
 */

import stateStore from '../data/stateStore.js';

function render(parentEl) {
  if (!parentEl) return;
  const wrap = document.createElement('div');
  wrap.className = 'card';
  wrap.innerHTML = '<div class="card-title">队列</div><ul class="queue-list" id="queue-list-root"></ul>';
  parentEl.appendChild(wrap);
  const listEl = wrap.querySelector('#queue-list-root');

  function jobTypeLabel(t) {
    if (!t) return '—';
    const u = (t + '').toUpperCase();
    if (u === 'PICK') return '抓取';
    if (u === 'PLACE') return '放置';
    if (u === 'PLACE_RELEASE') return '放置释放';
    if (u === 'OPEN_GRIPPER') return '打开夹爪';
    if (u === 'CLOSE_GRIPPER') return '闭合夹爪';
    if (u === 'RESET_HELD_OBJECT') return '重置持物';
    if (u === 'SYNC_HELD_OBJECT') return '同步持物';
    return t;
  }

  function update() {
    const s = stateStore.getState();
    listEl.innerHTML = '';
    if (s.queueEmpty && (!s.queueList || s.queueList.length === 0)) {
      listEl.innerHTML = '<li style="color: var(--text-muted);">队列为空</li>';
      return;
    }
    const items = s.queueList && s.queueList.length > 0 ? s.queueList : [
      ...(s.currentJobId ? [{ job_id: s.currentJobId, job_type: s.currentJobType, is_current: true }] : []),
      ...(s.nextJobType ? [{ job_id: s.nextJobId, job_type: s.nextJobType }] : []),
    ];
    items.forEach((item) => {
      const li = document.createElement('li');
      li.innerHTML = `
        <span>${jobTypeLabel(item.job_type)} ${item.job_id ? `(${String(item.job_id).slice(0, 8)})` : ''}</span>
        ${item.is_current ? '<span class="badge badge-running">执行中</span>' : `<button type="button" class="queue-item-cancel" data-job-id="${item.job_id || ''}">取消</button>`}
      `;
      const btn = li.querySelector('.queue-item-cancel');
      if (btn && btn.dataset.jobId) {
        btn.addEventListener('click', () => window.dispatchEvent(new CustomEvent('orion:cancel-job', { detail: { job_id: btn.dataset.jobId } })));
      }
      listEl.appendChild(li);
    });
  }

  update();
  stateStore.subscribe(update);
}

export default { render };
