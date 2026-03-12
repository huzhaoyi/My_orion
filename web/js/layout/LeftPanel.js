/**
 * 左侧栏：任务与状态看板（当前任务、队列、持物、感知、最近错误）
 */

import RuntimeStatusCard from '../panels/RuntimeStatusCard.js';
import QueueCard from '../panels/QueueCard.js';
import HeldObjectCard from '../panels/HeldObjectCard.js';
import PerceptionCard from '../panels/PerceptionCard.js';
import LastErrorCard from '../panels/LastErrorCard.js';

function mount(containerId) {
  const el = document.getElementById(containerId);
  if (!el) return;

  const inner = document.createElement('div');
  inner.className = 'left-panel-inner';
  el.appendChild(inner);

  RuntimeStatusCard.render(inner);
  QueueCard.render(inner);
  HeldObjectCard.render(inner);
  PerceptionCard.render(inner);
  LastErrorCard.render(inner);
}

export default { mount };
