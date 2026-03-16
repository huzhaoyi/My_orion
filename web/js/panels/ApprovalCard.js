/**
 * 审批结果卡片：审批抓取/审批放置，展示 approved、severity、summary、items、建议位姿
 */

import stateStore from '../data/stateStore.js';
import wsClient from '../data/wsClient.js';

const SEV_PASS = 0;
const SEV_WARNING = 1;
const SEV_REJECT = 2;

/** 审批抓取默认使用第几个目标点（0-based），与桥接 target_index=1 一致 */
const DEFAULT_PICK_TARGET_INDEX = 1;

/**
 * 从 targetSet 取第 index 个目标的位姿（仅位置，姿态用默认竖直向下）
 * 若有 object_pose 且来自同一目标，可优先用 object_pose 以保留姿态
 */
function getPoseForTargetIndex(state, index) {
  const ts = state.targetSet;
  if (!ts || !ts.positions || (ts.num_targets || 0) <= index) return null;
  const i = index * 3;
  const x = ts.positions[i];
  const y = ts.positions[i + 1];
  const z = ts.positions[i + 2];
  if (typeof x !== 'number' || typeof y !== 'number' || typeof z !== 'number') return null;
  return {
    header: { frame_id: 'base_link' },
    pose: {
      position: { x, y, z },
      orientation: { x: 0, y: 0, z: 0, w: 1 },
    },
  };
}

function levelLabel(level) {
  if (level === 0) return '信息';
  if (level === 1) return '警告';
  return '错误';
}

function levelClass(level) {
  if (level === 0) return 'approval-item--info';
  if (level === 1) return 'approval-item--warn';
  return 'approval-item--error';
}

function severityLabel(severity) {
  if (severity === SEV_PASS) return '通过';
  if (severity === SEV_WARNING) return '可执行（有风险）';
  return '禁止执行';
}

function severityClass(severity) {
  if (severity === SEV_PASS) return 'approval-badge--pass';
  if (severity === SEV_WARNING) return 'approval-badge--warn';
  return 'approval-badge--reject';
}

function handlePickClick(e) {
  e.preventDefault();
  const btn = e.currentTarget;
  if (!btn) {
    return;
  }
  if (btn.disabled) {
    return;
  }
  if (!wsClient.isConnected()) {
    stateStore.pushSystemLog('warn', '未连接 ROS，无法审批抓取');
    return;
  }
  const s = stateStore.getState();
  const targetIndex = DEFAULT_PICK_TARGET_INDEX;
  const totalTargets = (s.targetSet && s.targetSet.num_targets) || 0;
  let objectPose = getPoseForTargetIndex(s, targetIndex);
  if (!objectPose && s.objectPoseValid && s.objectPose) {
    objectPose = { header: { frame_id: 'base_link' }, pose: s.objectPose };
  }
  if (!objectPose) {
    stateStore.pushSystemLog('warn', '无感知数据，无法审批抓取');
    return;
  }
  stateStore.setState({
    approvalLoading: true,
    approvalTargetIndex: targetIndex,
    approvalTargetTotal: totalTargets,
  });
  wsClient.checkPick(objectPose, (raw) => {
    stateStore.setState({ approvalLoading: false });
    const r = raw && raw.values ? raw.values : raw;
    if (r != null) {
      stateStore.setApprovalResult({
        type: 'pick',
        approved: r.approved,
        severity: r.severity,
        summary: r.summary,
        items: r.items || [],
        best_candidate_pose: r.best_candidate_pose || null,
        adjusted_place_pose: null,
      });
      stateStore.pushSystemLog(
        'info',
        `审批抓取: ${r.summary || (r.approved ? '通过' : '未通过')}`,
      );
    } else {
      stateStore.pushSystemLog('warn', '审批抓取无响应');
    }
  });
  stateStore.pushSystemLog('info', '发起审批抓取…');
}

function handlePlaceClick(e) {
  e.preventDefault();
  const btn = e.currentTarget;
  if (!btn) {
    return;
  }
  if (btn.disabled) {
    return;
  }
  if (!wsClient.isConnected()) {
    stateStore.pushSystemLog('warn', '未连接 ROS，无法审批放置');
    return;
  }
  stateStore.setState({ approvalLoading: true });
  const s = stateStore.getState();
  const placePose = s.placePoseValid && s.placePose
    ? { header: { frame_id: 'base_link' }, pose: s.placePose }
    : {
        header: { frame_id: 'base_link' },
        pose: {
          position: { x: 0.35, y: 0.15, z: 0.4 },
          orientation: { x: 0, y: 0, z: 0, w: 1 },
        },
      };
  const hasHeld = s.hasHeldObject || s.heldValid;
  wsClient.checkPlace(placePose, hasHeld, (raw) => {
    stateStore.setState({ approvalLoading: false });
    const r = raw && raw.values ? raw.values : raw;
    if (r != null) {
      stateStore.setApprovalResult({
        type: 'place',
        approved: r.approved,
        severity: r.severity,
        summary: r.summary,
        items: r.items || [],
        best_candidate_pose: null,
        adjusted_place_pose: r.adjusted_place_pose || null,
      });
      stateStore.pushSystemLog(
        'info',
        `审批放置: ${r.summary || (r.approved ? '通过' : '未通过')}`,
      );
    } else {
      stateStore.pushSystemLog('warn', '审批放置无响应');
    }
  });
  stateStore.pushSystemLog('info', '发起审批放置…');
}

/**
 * 仅更新“审批结果”展示区域（由 RightPanel 提供容器，按钮在 RightPanel 内绑定）
 */
function renderResult(resultContainerEl) {
  if (!resultContainerEl) return;
  function update() {
    if (!resultContainerEl.isConnected) return;
    const state = stateStore.getState();
    const res = state.approvalResult;
    const loading = state.approvalLoading || false;
    const targetIndex = (state.approvalTargetIndex != null ? state.approvalTargetIndex : DEFAULT_PICK_TARGET_INDEX) + 1;
    const targetTotal = state.approvalTargetTotal != null ? state.approvalTargetTotal : (state.targetSet && state.targetSet.num_targets) || 0;
    const targetLabel = targetTotal > 0 ? `第 ${targetIndex} 个（共 ${targetTotal} 个）` : '';

    const stepsHtml = `
      <div class="approval-steps">
        <div class="approval-step">1. 几何范围检查</div>
        <div class="approval-step">2. IK / 关节余量</div>
        <div class="approval-step">3. 碰撞 / 安全性</div>
      </div>
    `;

    let resultHtml = '';
    if (loading && !res) {
      // 仅有加载状态时，显示“审批中 + 步骤”
      resultHtml = `
        <div class="approval-loading">
          <div>审批中… ${targetLabel ? `(目标 ${targetLabel})` : ''}</div>
          ${stepsHtml}
        </div>
      `;
    } else if (res) {
      const badgeClass = severityClass(res.severity);
      const badgeTextBase = severityLabel(res.severity);
      const actionLabel = res.type === 'place' ? '放置' : '抓取';
      const badgeText = `${actionLabel}${badgeTextBase}`;
      const summaryPrefix = res.type === 'place' ? '放置审批' : '抓取审批';
      const summaryTextRaw = (res.summary || '').trim();
      const summaryText = summaryTextRaw || (res.approved ? `${summaryPrefix}：所有检查通过` : `${summaryPrefix}：完成（无摘要）`);
      const items = (res.items || []).map((it) => {
        const msg = (it.message || '').trim();
        const sug = (it.suggestion || '').trim();
        const row = `<tr class="approval-item ${levelClass(it.level || 0)}">
          <td class="approval-item__code">${(it.code || '').trim() || '—'}</td>
          <td class="approval-item__level">${levelLabel(it.level || 0)}</td>
          <td class="approval-item__msg">${msg || '—'}</td>
          <td class="approval-item__sug">${sug ? `建议: ${sug}` : '—'}</td>
        </tr>`;
        return row;
      }).join('');
      resultHtml = `
        <div class="approval-result">
          <div class="approval-badge ${badgeClass}">${badgeText}</div>
          <div class="approval-summary">${summaryText}</div>
          ${items.length ? `
            <div class="approval-items-wrap">
              <table class="approval-items">
                <thead><tr><th>代码</th><th>级别</th><th>说明</th><th>建议</th></tr></thead>
                <tbody>${items}</tbody>
              </table>
            </div>
          ` : ''}
          ${res.best_candidate_pose && res.type === 'pick' ? `
            <div class="approval-suggestion">
              <span class="approval-suggestion__label">推荐抓取位姿:</span>
              <span class="approval-suggestion__pos">x=${(res.best_candidate_pose.pose?.position?.x ?? 0).toFixed(3)} y=${(res.best_candidate_pose.pose?.position?.y ?? 0).toFixed(3)} z=${(res.best_candidate_pose.pose?.position?.z ?? 0).toFixed(3)}</span>
            </div>
          ` : ''}
          ${stepsHtml}
        </div>
      `;
    } else {
      resultHtml = '<div class="approval-result approval-result--empty">点击「审批抓取」或「审批放置」获取结果</div>';
    }
    resultContainerEl.innerHTML = resultHtml;
  }
  update();
  stateStore.subscribe(() => update());
}

export default {
  render: renderResult,
  renderResult,
  handlePickClick,
  handlePlaceClick,
};
