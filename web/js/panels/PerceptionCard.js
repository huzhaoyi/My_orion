/**
 * 左侧 - 感知状态：缆绳 / TargetSensor 分栏 + 表格；稳定 DOM + 节流，避免 joint_states 等高频整卡 innerHTML 导致标签无法点击。
 */

import stateStore from '../data/stateStore.js';
import { t, subscribeLocale } from '../data/i18n.js';

const THROTTLE_MS = 120;

function fmtPos(pos) {
  if (!pos) {
    return '—';
  }
  const x = Number(pos.x);
  const y = Number(pos.y);
  const z = Number(pos.z);
  if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) {
    return '—';
  }
  return `${x.toFixed(3)}, ${y.toFixed(3)}, ${z.toFixed(3)}`;
}

function fmtQuat(q) {
  if (!q) {
    return '—';
  }
  const x = Number(q.x);
  const y = Number(q.y);
  const z = Number(q.z);
  const w = Number(q.w);
  if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z) || !Number.isFinite(w)) {
    return '—';
  }
  return `qx=${x.toFixed(3)} qy=${y.toFixed(3)} qz=${z.toFixed(3)} qw=${w.toFixed(3)}`;
}

function fmtQuatShort(q) {
  const s = fmtQuat(q);
  if (s === '—') {
    return '—';
  }
  return s.length > 42 ? `${s.slice(0, 39)}…` : s;
}

function defaultPerceptionSourceTab(cableValid, tsValid) {
  if (cableValid && !tsValid) {
    return 'cable';
  }
  if (!cableValid && tsValid) {
    return 'target_sensor';
  }
  return 'cable';
}

function render(parentEl) {
  if (!parentEl) return;
  const wrap = document.createElement('div');
  wrap.className = 'card';
  wrap.id = 'perception-card';
  parentEl.appendChild(wrap);

  let activePerceptionSource = 'cable';
  let perceptionSourceUserPicked = false;
  let shellBuilt = false;
  let throttleTimer = null;
  let flushImmediate = false;
  let pendingState = null;

  function ensureShell() {
    if (shellBuilt) {
      return;
    }
    shellBuilt = true;
    wrap.innerHTML = `
      <div class="card-title" id="pc-title"></div>
      <div class="perception-card__source-tabs" id="pc-tabs" role="tablist" aria-label="">
        <button type="button" id="pc-tab-cable" class="perception-card__source-tab" data-perception-source-tab="cable"></button>
        <button type="button" id="pc-tab-ts" class="perception-card__source-tab" data-perception-source-tab="target_sensor"></button>
      </div>
      <div id="pc-panel-cable" class="perception-card__pose-block">
        <div class="perception-card__table-caption" id="pc-cable-cap"></div>
        <div class="card-row">
          <span class="card-label" id="pc-cable-hlabel"></span>
          <span class="card-value" id="pc-cable-hval" style="font-size:10px;color:var(--text-secondary);"></span>
        </div>
        <div class="perception-card__targets">
          <table class="perception-card__table" aria-label="cable">
            <thead><tr>
              <th id="pc-cth0"></th><th id="pc-cth1"></th><th id="pc-cth2"></th>
            </tr></thead>
            <tbody id="pc-cable-tbody"></tbody>
          </table>
        </div>
        <div class="card-row card-row--indent"><span class="card-label" id="pc-cable-up-l"></span><span class="card-value" id="pc-cable-up-v"></span></div>
      </div>
      <div id="pc-panel-ts" class="perception-card__pose-block" hidden>
        <div class="perception-card__table-caption" id="pc-ts-cap"></div>
        <div class="card-row">
          <span class="card-label" id="pc-ts-hlabel"></span>
          <span class="card-value" id="pc-ts-hval" style="font-size:10px;color:var(--text-secondary);"></span>
        </div>
        <div class="card-row card-row--indent"><span class="card-label" id="pc-ts-idx-l"></span><span class="card-value" id="pc-ts-idx-v"></span></div>
        <div class="perception-card__targets">
          <table class="perception-card__table" aria-label="targets">
            <thead><tr>
              <th id="pc-tth0"></th><th id="pc-tth1"></th><th id="pc-tth2"></th><th id="pc-tth3"></th>
            </tr></thead>
            <tbody id="pc-ts-tbody"></tbody>
          </table>
        </div>
        <div class="perception-card__table-caption" id="pc-th-cap"></div>
        <div class="perception-card__targets">
          <table class="perception-card__table" aria-label="target_insert_holes">
            <thead><tr>
              <th id="pc-hth0"></th><th id="pc-hth1"></th><th id="pc-hth2"></th>
            </tr></thead>
            <tbody id="pc-th-tbody"></tbody>
          </table>
        </div>
        <div class="card-row card-row--indent"><span class="card-label" id="pc-th-up-l"></span><span class="card-value" id="pc-th-up-v"></span></div>
        <div class="card-row card-row--indent"><span class="card-label" id="pc-ts-up-l"></span><span class="card-value" id="pc-ts-up-v"></span></div>
      </div>
      <div class="perception-card__pose-block perception-card__pose-block--fused">
        <div class="card-row">
          <span class="card-label" id="pc-flabel"></span>
          <span class="card-value" id="pc-fhint" style="font-size:10px;color:var(--text-secondary);"></span>
        </div>
        <div class="card-row card-row--indent"><span class="card-label" id="pc-fpl"></span><span class="card-value perception-card__fused-pos" id="pc-fpv"></span></div>
        <div class="card-row card-row--indent"><span class="card-label" id="pc-fol"></span><span class="card-value perception-card__quat" id="pc-fov"></span></div>
        <div class="card-row card-row--indent"><span class="card-label" id="pc-ful"></span><span class="card-value" id="pc-fuv"></span></div>
      </div>
      <div class="perception-card__pose-block perception-card__pose-block--keypoints">
        <div class="card-row">
          <span class="card-label" id="pc-kptl"></span>
          <span class="card-value" id="pc-kph" style="font-size:10px;color:var(--text-secondary);"></span>
        </div>
        <div id="pc-kp-rows"></div>
        <div class="card-row card-row--indent"><span class="card-label" id="pc-kpul"></span><span class="card-value" id="pc-kpuv"></span></div>
        <div class="card-row card-row--indent"><span class="card-label" id="pc-legl"></span><span class="card-value" id="pc-legv" style="font-size:10px;"></span></div>
      </div>
      <div class="perception-card__pose-block">
        <div class="card-row"><span class="card-label" id="pc-rwtl"></span></div>
        <div class="card-row card-row--indent"><span class="card-label" id="pc-rwpl"></span><span class="card-value" id="pc-rwpv"></span></div>
        <div class="card-row card-row--indent"><span class="card-label" id="pc-rwol"></span><span class="card-value perception-card__quat" id="pc-rwov"></span></div>
      </div>
      <div class="perception-card__pose-block">
        <div class="card-row"><span class="card-label" id="pc-rbtl"></span></div>
        <div class="card-row card-row--indent"><span class="card-label" id="pc-rbpl"></span><span class="card-value" id="pc-rbpv"></span></div>
        <div class="card-row card-row--indent"><span class="card-label" id="pc-rbol"></span><span class="card-value perception-card__quat" id="pc-rbov"></span></div>
      </div>
    `;
    wrap.querySelector('#pc-tabs').setAttribute('aria-label', t('card.perception.source_tab_aria'));
  }

  function applyI18nStatic() {
    if (!shellBuilt) {
      return;
    }
    const g = (id) => wrap.querySelector(`#${id}`);
    g('pc-title').textContent = t('card.perception.title');
    g('pc-cable-cap').textContent = t('card.perception.cable_table_caption');
    g('pc-cable-hlabel').textContent = t('card.perception.cable_label');
    g('pc-cable-up-l').textContent = t('card.perception.updated');
    g('pc-ts-cap').textContent = t('card.perception.target_table_caption');
    g('pc-ts-hlabel').textContent = t('card.perception.target_sensor_label');
    g('pc-ts-idx-l').textContent = t('card.perception.target_grasp_index');
    g('pc-ts-up-l').textContent = t('card.perception.updated');
    g('pc-th-cap').textContent = t('card.perception.target_insert_holes_caption');
    g('pc-th-up-l').textContent = t('card.perception.updated');
    g('pc-cth0').textContent = t('card.perception.table_idx');
    g('pc-cth1').textContent = t('card.perception.table_pos');
    g('pc-cth2').textContent = t('card.perception.table_quat');
    g('pc-tth0').textContent = t('card.perception.table_idx');
    g('pc-tth1').textContent = t('card.perception.table_pos');
    g('pc-tth2').textContent = t('card.perception.table_quat');
    g('pc-tth3').textContent = t('card.perception.table_id');
    g('pc-hth0').textContent = t('card.perception.table_idx');
    g('pc-hth1').textContent = t('card.perception.table_pos');
    g('pc-hth2').textContent = t('card.perception.table_quat');
    g('pc-flabel').textContent = t('card.perception.fused_label');
    g('pc-fpl').textContent = t('card.perception.pos');
    g('pc-fol').textContent = t('card.perception.pose_orient');
    g('pc-ful').textContent = t('card.perception.updated');
    g('pc-kptl').textContent = t('card.perception.kp_title');
    g('pc-kpul').textContent = t('card.perception.updated');
    g('pc-legl').textContent = t('card.perception.legend');
    g('pc-rwtl').textContent = t('card.perception.rov_map');
    g('pc-rwpl').textContent = t('card.perception.pos');
    g('pc-rwol').textContent = t('card.perception.pose_orient');
    g('pc-rbtl').textContent = t('card.perception.rov_base');
    g('pc-rbpl').textContent = t('card.perception.pos');
    g('pc-rbol').textContent = t('card.perception.pose_orient');
  }

  function flush(s) {
    if (!wrap.isConnected) return;
    const state = s != null ? s : stateStore.getState();
    ensureShell();
    applyI18nStatic();

    const fusedPose = state.fusedObjectPose || null;
    const fusedPos = fusedPose ? fusedPose.position : null;
    const fusedQuat = fusedPose ? fusedPose.orientation : null;
    const fusedValid = !!state.fusedObjectPoseValid;
    const fusedInvalidHint = t('card.perception.pose_fuse_invalid');
    const fusedPosDisp = fusedValid ? fmtPos(fusedPos) : fusedInvalidHint;
    const fusedQuatDisp = fusedValid ? (fusedQuat ? fmtQuat(fusedQuat) : '—') : fusedInvalidHint;
    const tfu = state.fusedPerceptionUpdatedAt
      ? new Date(state.fusedPerceptionUpdatedAt).toLocaleTimeString()
      : '—';
    const fusedTimeDisp = fusedValid ? tfu : t('card.perception.fused_time_invalid');

    const kpTrace = state.keypointsTrace;
    const kpValid = !!state.keypointsTraceValid && kpTrace && Array.isArray(kpTrace.points);
    const kpPts = kpValid ? kpTrace.points : [];
    const kpTf =
      state.keypointsTraceUpdatedAt != null
        ? new Date(state.keypointsTraceUpdatedAt).toLocaleTimeString()
        : '—';
    let kpRowsHtml = '';
    if (kpValid && kpPts.length > 0) {
      kpRowsHtml = kpPts
        .map(
          (p, idx) => `
        <div class="card-row card-row--indent perception-card__kp-row">
          <span class="card-label">#${idx}</span>
          <span class="card-value" style="font-size:10px;">${fmtPos(p)}</span>
        </div>`
        )
        .join('');
    } else {
      kpRowsHtml = `<div class="card-row card-row--indent"><span class="card-value" style="font-size:10px;color:var(--text-secondary);">${t('card.perception.kp_missing')}</span></div>`;
    }
    wrap.querySelector('#pc-kp-rows').innerHTML = kpRowsHtml;
    wrap.querySelector('#pc-kph').innerHTML = kpValid
      ? `${kpTrace.frameId} · ${kpPts.length}${t('card.perception.kp_count_suffix')}${t('card.perception.kp_ok')}`
      : t('card.perception.kp_hint_suffix');
    wrap.querySelector('#pc-kpuv').textContent = kpValid ? kpTf : '—';
    wrap.querySelector('#pc-legv').textContent = t('card.perception.legend_text');

    const rovBase = state.rovPoseInBaseLink || null;
    const rovWorld = state.rovPoseInWorld || null;
    const rovPosBase = rovBase ? rovBase.position : null;
    const rovPosWorld = rovWorld ? rovWorld.position : null;
    const rovQuatBase = rovBase ? rovBase.orientation : null;
    const rovQuatWorld = rovWorld ? rovWorld.orientation : null;
    wrap.querySelector('#pc-rwpv').textContent = rovPosWorld ? fmtPos(rovPosWorld) : '—';
    wrap.querySelector('#pc-rwov').textContent = rovQuatWorld ? fmtQuat(rovQuatWorld) : '—';
    wrap.querySelector('#pc-rbpv').textContent = rovPosBase ? fmtPos(rovPosBase) : '—';
    wrap.querySelector('#pc-rbov').textContent = rovQuatBase ? fmtQuat(rovQuatBase) : '—';

    const perceptionTimeStr = state.perceptionUpdatedAt
      ? new Date(state.perceptionUpdatedAt).toLocaleTimeString()
      : '—';
    const cableValid = !!state.cableObjectPoseValid && state.cableObjectPose;
    const cablePos = cableValid ? state.cableObjectPose.position : null;
    const cableQuat = cableValid ? state.cableObjectPose.orientation : null;
    const cableTimeDisp = cableValid ? perceptionTimeStr : '—';
    const tsValid = !!state.targetSensorObjectPoseValid && state.targetSensorObjectPose;
    const tsHintExtra = tsValid ? '' : ` · ${t('card.perception.source_missing')}`;
    const cableHintExtra = cableValid ? '' : ` · ${t('card.perception.source_missing')}`;
    const tsUpStr =
      state.targetSetValid && state.targetSetUpdatedAt != null
        ? new Date(state.targetSetUpdatedAt).toLocaleTimeString()
        : tsValid
          ? perceptionTimeStr
          : '—';

    if (!perceptionSourceUserPicked) {
      activePerceptionSource = defaultPerceptionSourceTab(cableValid, tsValid);
    }

    const tabCableActive = activePerceptionSource === 'cable';
    const tabTsActive = activePerceptionSource === 'target_sensor';
    const dotTitle = t('card.perception.tab_has_data');
    const tabC = wrap.querySelector('#pc-tab-cable');
    const tabT = wrap.querySelector('#pc-tab-ts');
    tabC.classList.toggle('active', tabCableActive);
    tabT.classList.toggle('active', tabTsActive);
    tabC.setAttribute('aria-selected', tabCableActive ? 'true' : 'false');
    tabT.setAttribute('aria-selected', tabTsActive ? 'true' : 'false');
    tabC.innerHTML = `${t('card.perception.source_tab_cable')}${
      cableValid ? `<span class="perception-card__source-tab-dot" title="${dotTitle}">●</span>` : ''
    }`;
    tabT.innerHTML = `${t('card.perception.source_tab_target')}${
      tsValid || state.targetSetValid
        ? `<span class="perception-card__source-tab-dot" title="${dotTitle}">●</span>`
        : ''
    }`;

    wrap.querySelector('#pc-panel-cable').hidden = !tabCableActive;
    wrap.querySelector('#pc-panel-ts').hidden = !tabTsActive;

    wrap.querySelector('#pc-cable-hval').textContent = `${t('card.perception.cable_hint')}${cableHintExtra}`;
    wrap.querySelector('#pc-cable-up-v').textContent = cableTimeDisp;
    const cableTbody = wrap.querySelector('#pc-cable-tbody');
    if (cableValid) {
      cableTbody.innerHTML = `<tr><td>0</td><td style="font-size:10px;">${fmtPos(cablePos)}</td><td style="font-size:10px;">${fmtQuatShort(cableQuat)}</td></tr>`;
    } else {
      cableTbody.innerHTML = `<tr><td colspan="3" style="text-align:left;color:var(--text-secondary);font-size:10px;">${t('card.perception.table_empty')}</td></tr>`;
    }

    wrap.querySelector('#pc-ts-hval').textContent = `${t('card.perception.target_sensor_hint')}${tsHintExtra}`;
    const si = state.targetSensorSelectedIndex;
    wrap.querySelector('#pc-ts-idx-v').textContent =
      si != null && si >= 0 ? String(si) : '—';
    wrap.querySelector('#pc-ts-up-v').textContent = tsUpStr;

    const tsTbody = wrap.querySelector('#pc-ts-tbody');
    const rows = state.targetSetTargets || [];
    const selIdx = state.targetSensorSelectedIndex;
    if (rows.length === 0) {
      tsTbody.innerHTML = `<tr><td colspan="4" style="text-align:left;color:var(--text-secondary);font-size:10px;">${t('card.perception.table_empty')}</td></tr>`;
    } else {
      tsTbody.innerHTML = rows
        .map((row) => {
          const isSel = selIdx != null && selIdx >= 0 && row.index === selIdx;
          const trCls = isSel ? ' class="perception-card__row--selected"' : '';
          const idCell = row.objectId ? escapeHtml(row.objectId) : '—';
          return `<tr${trCls}><td>${row.index}</td><td style="font-size:10px;">${fmtPos(row.position)}</td><td style="font-size:10px;">${fmtQuatShort(row.orientation)}</td><td style="font-size:10px;">${idCell}</td></tr>`;
        })
        .join('');
    }

    const holeRows = state.targetInsertHolePoses || [];
    const holeTbody = wrap.querySelector('#pc-th-tbody');
    if (holeRows.length === 0) {
      holeTbody.innerHTML = `<tr><td colspan="3" style="text-align:left;color:var(--text-secondary);font-size:10px;">${t('card.perception.table_empty')}</td></tr>`;
    } else {
      holeTbody.innerHTML = holeRows
        .map((row, rowIndex) => {
          return `<tr><td>${rowIndex + 1}</td><td style="font-size:10px;">${fmtPos(row.position)}</td><td style="font-size:10px;">${fmtQuatShort(row.orientation)}</td></tr>`;
        })
        .join('');
    }
    const holesUpStr =
      state.targetInsertHolesValid && state.targetInsertHolesUpdatedAt != null
        ? new Date(state.targetInsertHolesUpdatedAt).toLocaleTimeString()
        : '—';
    wrap.querySelector('#pc-th-up-v').textContent = holesUpStr;

    const fh = wrap.querySelector('#pc-fhint');
    fh.innerHTML = `${t('card.perception.fused_hint')}<span style="color:${fusedValid ? '#22c55e' : '#f97316'}">${fusedValid ? t('card.perception.valid') : t('card.perception.invalid')}</span>`;
    wrap.querySelector('#pc-fpv').textContent = fusedPosDisp;
    wrap.querySelector('#pc-fov').textContent = fusedQuatDisp;
    wrap.querySelector('#pc-fuv').textContent = fusedTimeDisp;
  }

  function escapeHtml(s) {
    return String(s)
      .replace(/&/g, '&amp;')
      .replace(/</g, '&lt;')
      .replace(/>/g, '&gt;')
      .replace(/"/g, '&quot;');
  }

  function scheduleFlush(s) {
    if (flushImmediate) {
      flushImmediate = false;
      flush(s);
      return;
    }
    pendingState = s;
    if (throttleTimer != null) {
      return;
    }
    throttleTimer = setTimeout(() => {
      throttleTimer = null;
      flush(pendingState);
    }, THROTTLE_MS);
  }

  wrap.addEventListener('click', (ev) => {
    const el = ev.target.closest('[data-perception-source-tab]');
    if (!el || !wrap.contains(el)) {
      return;
    }
    ev.preventDefault();
    const tab = el.getAttribute('data-perception-source-tab');
    if (tab !== 'cable' && tab !== 'target_sensor') {
      return;
    }
    if (tab === activePerceptionSource) {
      return;
    }
    perceptionSourceUserPicked = true;
    activePerceptionSource = tab;
    if (throttleTimer != null) {
      clearTimeout(throttleTimer);
      throttleTimer = null;
    }
    flushImmediate = true;
    flush(stateStore.getState());
  });

  flush(stateStore.getState());
  stateStore.subscribe((newState) => scheduleFlush(newState));
  subscribeLocale(() => {
    flushImmediate = true;
    if (throttleTimer != null) {
      clearTimeout(throttleTimer);
      throttleTimer = null;
    }
    flush(stateStore.getState());
  });
}

export default { render };
