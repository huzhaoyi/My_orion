/**
 * 左侧 - 持物状态卡片 + Scene 一致性提示
 */

import stateStore from '../data/stateStore.js';
import { t, subscribeLocale } from '../data/i18n.js';

/** 持物与 planning scene 一致性摘要卡片。 */
function render(parentEl) {
  if (!parentEl) return;
  const wrap = document.createElement('div');
  wrap.className = 'card';
  wrap.id = 'held-object-card';
  parentEl.appendChild(wrap);

  function yn(b) {
    return b ? t('card.held.yes') : t('card.held.no');
  }

  function pr(b) {
    return b ? t('card.held.present') : t('card.held.absent');
  }

  function update() {
    const s = stateStore.getState();
    wrap.innerHTML = `
      <div class="card-title">${t('card.held.title')}</div>
      <div class="card-row"><span class="card-label">${t('card.held.holding')}</span><span class="card-value">${yn(s.hasHeldObject)}</span></div>
      <div class="card-row"><span class="card-label">${t('card.held.tracked')}</span><span class="card-value">${yn(s.heldTracked)}</span></div>
      <div class="card-row"><span class="card-label">${t('card.held.object_id')}</span><span class="card-value">${s.heldObjectId || '—'}</span></div>
      <div class="card-row"><span class="card-label">${t('card.held.scene_attach')}</span><span class="card-value">${(s.heldSceneAttachId || '—').slice(0, 20)}</span></div>
      <div class="card-title" style="margin-top: 10px;">${t('card.held.scene_title')}</div>
      <div class="card-row"><span class="card-label">${t('card.held.world_obj')}</span><span class="card-value">${pr(s.worldObjectPresent)}</span></div>
      <div class="card-row"><span class="card-label">${t('card.held.attached_obj')}</span><span class="card-value">${pr(s.attachedObjectPresent)}</span></div>
      <div class="card-row"><span class="card-label">${t('card.held.tracked_held')}</span><span class="card-value">${pr(s.heldTrackedPresent)}</span></div>
      <div class="card-row"><span class="card-label">${t('card.held.untracked_held')}</span><span class="card-value">${pr(s.heldUntrackedPresent)}</span></div>
    `;
  }

  update();
  stateStore.subscribe(update);
  subscribeLocale(update);
}

export default { render };
