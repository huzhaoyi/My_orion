/**
 * 左侧 - 感知状态卡片（来自 perception_state：物体/放置/ROV 位姿，多目标用表格）
 */

import stateStore from '../data/stateStore.js';

function fmtNum(n) {
  return typeof n === 'number' && !Number.isNaN(n) ? n.toFixed(3) : '—';
}

function fmtPos(pos) {
  if (!pos || typeof pos.x !== 'number' || typeof pos.y !== 'number' || typeof pos.z !== 'number') {
    return '—';
  }
  return `${pos.x.toFixed(3)}, ${pos.y.toFixed(3)}, ${pos.z.toFixed(3)}`;
}

function buildTargetRows(targetSet) {
  if (!targetSet || !targetSet.positions || targetSet.num_targets <= 0) {
    return '';
  }
  const n = targetSet.num_targets;
  const pos = targetSet.positions;
  const rows = [];
  for (let i = 0; i < n; i++) {
    const j = i * 3;
    const x = pos[j];
    const y = pos[j + 1];
    const z = pos[j + 2];
    rows.push(
      `<tr><td>${i}</td><td>${fmtNum(x)}</td><td>${fmtNum(y)}</td><td>${fmtNum(z)}</td></tr>`
    );
  }
  return rows.join('');
}

function render(parentEl) {
  if (!parentEl) return;
  const wrap = document.createElement('div');
  wrap.className = 'card';
  wrap.id = 'perception-card';
  parentEl.appendChild(wrap);

  function update(s) {
    if (!wrap.isConnected) return;
    const state = s != null ? s : stateStore.getState();
    const objPos = state.objectPoseValid && state.objectPose ? state.objectPose.position : null;
    const placePos = state.placePoseValid && state.placePose ? state.placePose.position : null;
    const rovPosBase = state.rovPoseInBaseLink ? state.rovPoseInBaseLink.position : null;
    const rovPosWorld = state.rovPoseInWorld ? state.rovPoseInWorld.position : null;
    const targetCount = state.targetCount ?? 0;
    const t = state.perceptionUpdatedAt
      ? new Date(state.perceptionUpdatedAt).toLocaleTimeString()
      : '—';
    const targetRowsWorld = buildTargetRows(state.targetSetWorld);
    const targetRows = buildTargetRows(state.targetSet);
    const tableTpl = (rows) =>
      rows
        ? `<table class="perception-card__table"><thead><tr><th>#</th><th>x</th><th>y</th><th>z</th></tr></thead><tbody>${rows}</tbody></table>`
        : '<div class="card-row"><span class="card-value">—</span></div>';
    const tableHtmlWorld = tableTpl(targetRowsWorld);
    const tableHtml = tableTpl(targetRows);
    wrap.innerHTML = `
      <div class="card-title">感知状态</div>
      <div class="card-row"><span class="card-label">物体 (base_link)</span><span class="card-value">${state.objectPoseValid ? fmtPos(objPos) : '无效'}</span></div>
      <div class="card-row"><span class="card-label">放置 (base_link)</span><span class="card-value">${state.placePoseValid ? fmtPos(placePos) : '无效'}</span></div>
      <div class="card-row"><span class="card-label">ROV (map)</span><span class="card-value">${rovPosWorld ? fmtPos(rovPosWorld) : '—'}</span></div>
      <div class="card-row"><span class="card-label">ROV (base_link)</span><span class="card-value">${rovPosBase ? fmtPos(rovPosBase) : '—'}</span></div>
      <div class="card-row"><span class="card-label">目标数</span><span class="card-value">${targetCount}</span></div>
      <div class="card-row"><span class="card-label">目标→世界坐标 (map)</span><span class="card-value" style="font-size:10px;color:var(--text-secondary);">单位 m</span></div>
      <div class="perception-card__targets">${tableHtmlWorld}</div>
      <div class="card-row"><span class="card-label">目标→机械臂基坐标 (base_link)</span><span class="card-value" style="font-size:10px;color:var(--text-secondary);">单位 m</span></div>
      <div class="perception-card__targets">${tableHtml}</div>
      <div class="card-row"><span class="card-label">更新时间</span><span class="card-value">${t}</span></div>
    `;
  }

  update();
  stateStore.subscribe((newState) => update(newState));
}

export default { render };
