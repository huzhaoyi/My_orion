/**
 * 左侧 - 当前 wsClient 订阅的全部 ROS 话题收包情况（最后消息时间）
 */

import stateStore from '../data/stateStore.js';
import wsClient from '../data/wsClient.js';
import { t, subscribeLocale } from '../data/i18n.js';

const STALE_MS = 5000.0;

function formatAgeSec(ms) {
  if (ms == null || !Number.isFinite(ms)) {
    return '';
  }
  const s = ms / 1000.0;
  if (s < 100.0) {
    return s.toFixed(1);
  }
  return String(Math.round(s));
}

/** 单列话题行：桥接状态 + 距上一帧时长。 */
function render(parentEl) {
  if (!parentEl) return;
  const wrap = document.createElement('div');
  wrap.className = 'card ros-topics-card';
  wrap.id = 'ros-topics-card';
  parentEl.appendChild(wrap);

  let tickTimer = null;

  function rowStatus(wsOk, lastAt, nowMs) {
    if (!wsOk) {
      return { cls: 'ros-topics__status--off', text: t('card.topics.ws_off'), age: '' };
    }
    if (lastAt == null || !Number.isFinite(lastAt)) {
      return { cls: 'ros-topics__status--wait', text: t('card.topics.waiting'), age: '' };
    }
    const age = nowMs - lastAt;
    const ageStr = formatAgeSec(age);
    if (age > STALE_MS) {
      return { cls: 'ros-topics__status--stale', text: t('card.topics.stale'), age: ageStr };
    }
    return { cls: 'ros-topics__status--live', text: t('card.topics.live'), age: ageStr };
  }

  let topicRxRaf = null;
  function scheduleUpdateFromTopicRx() {
    if (topicRxRaf != null) {
      return;
    }
    topicRxRaf = requestAnimationFrame(() => {
      topicRxRaf = null;
      update();
    });
  }

  function update() {
    if (!wrap.isConnected) {
      return;
    }
    const st = stateStore.getState();
    const wsOk = !!st.wsConnected;
    const map = st.rosTopicLastRxAt || {};
    const nowMs = Date.now();
    const topics = wsClient.getSubscribedTopicsFlat();
    const rows = topics
      .map((topic) => {
        const key = stateStore.normalizeRosTopicKey(topic);
        const lastAt = map[key];
        const rs = rowStatus(wsOk, lastAt, nowMs);
        const suffix = rs.age ? ` · ${rs.age}s` : '';
        return `
      <li class="ros-topics__row">
        <span class="ros-topics__name" title="${topic}">${topic}</span>
        <span class="ros-topics__status ${rs.cls}">${rs.text}${suffix}</span>
      </li>`;
      })
      .join('');

    wrap.innerHTML = `
      <div class="card-title">${t('card.topics.title')}</div>
      <p class="ros-topics__hint">${t('card.topics.hint')}</p>
      <ul class="ros-topics__list">${rows}</ul>
    `;
  }

  let prevWs = stateStore.getState().wsConnected;
  update();
  stateStore.subscribe((s) => {
    if (s.wsConnected !== prevWs) {
      prevWs = s.wsConnected;
      update();
    }
  });
  stateStore.subscribeRosTopicRx(() => scheduleUpdateFromTopicRx());
  subscribeLocale(() => update());

  tickTimer = setInterval(update, 200);

  window.addEventListener('pagehide', () => {
    if (tickTimer) {
      clearInterval(tickTimer);
      tickTimer = null;
    }
    if (topicRxRaf != null) {
      cancelAnimationFrame(topicRxRaf);
      topicRxRaf = null;
    }
  });
}

export default { render };
