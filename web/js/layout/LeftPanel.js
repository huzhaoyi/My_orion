/**
 * 左侧栏：任务与状态看板（当前任务、队列、感知、持物、最近错误）
 */

import RuntimeStatusCard from '../panels/RuntimeStatusCard.js';
import QueueCard from '../panels/QueueCard.js';
import PerceptionCard from '../panels/PerceptionCard.js';
import HeldObjectCard from '../panels/HeldObjectCard.js';
import LastErrorCard from '../panels/LastErrorCard.js';

/**
 * 左侧栏：依次挂载 RuntimeStatus / Queue / Perception / HeldObject / LastError 卡片。
 * 各卡片自行 subscribe stateStore。
 */
function mount(containerId) {
  const el = document.getElementById(containerId);
  if (!el) return;

  const inner = document.createElement('div');
  inner.className = 'left-panel-inner';
  el.appendChild(inner);

  RuntimeStatusCard.render(inner);
  QueueCard.render(inner);
  PerceptionCard.render(inner);
  HeldObjectCard.render(inner);
  LastErrorCard.render(inner);
}

export default { mount };
