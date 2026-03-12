/**
 * Toast 弹框：前后端交互结果提示（成功/失败/警告/信息）
 * 类型: success | error | warn | info
 */

const TOAST_DURATION_MS = 3200;
const TOAST_MAX = 3;

let container = null;

function getContainer() {
  if (container && container.isConnected) {
    return container;
  }
  container = document.createElement('div');
  container.id = 'toast-container';
  container.setAttribute('aria-live', 'polite');
  document.body.appendChild(container);
  return container;
}

function showToast(message, type = 'info') {
  if (!message || typeof message !== 'string') {
    return;
  }
  const el = document.createElement('div');
  el.className = `toast toast--${type}`;
  el.setAttribute('role', 'alert');

  const iconMap = {
    success: '✓',
    error: '✕',
    warn: '!',
    info: 'i',
  };
  const icon = iconMap[type] || iconMap.info;
  el.innerHTML = `<span class="toast__icon" aria-hidden="true">${icon}</span><span class="toast__text">${escapeHtml(message)}</span>`;

  const parent = getContainer();
  const existing = parent.querySelectorAll('.toast');
  if (existing.length >= TOAST_MAX) {
    existing[0].remove();
  }
  parent.appendChild(el);

  requestAnimationFrame(() => {
    el.classList.add('toast--visible');
  });

  const t = setTimeout(() => {
    el.classList.remove('toast--visible');
    setTimeout(() => el.remove(), 280);
  }, TOAST_DURATION_MS);

  el._toastTimer = t;
}

function escapeHtml(str) {
  const div = document.createElement('div');
  div.textContent = str;
  return div.innerHTML;
}

export default {
  showToast,
  success: (msg) => showToast(msg, 'success'),
  error: (msg) => showToast(msg, 'error'),
  warn: (msg) => showToast(msg, 'warn'),
  info: (msg) => showToast(msg, 'info'),
};
