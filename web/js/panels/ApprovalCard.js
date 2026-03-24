/**
 * 审批结果卡片：审批抓取，展示 approved、severity、summary、items、建议位姿
 */

import stateStore from '../data/stateStore.js';
import wsClient from '../data/wsClient.js';
import toast from '../ui/toast.js';

const SEV_PASS = 0;
const SEV_WARNING = 1;
const SEV_REJECT = 2;
const DEFAULT_PICK_TARGET_INDEX = 0;

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
    toast.warn('未连接，无法审批抓取');
    return;
  }
  const s = stateStore.getState();
  const objectPose = s.objectPoseValid && s.objectPose
    ? { header: { frame_id: 'base_link' }, pose: s.objectPose }
    : null;
  if (!objectPose) {
    stateStore.pushSystemLog('warn', '无缆绳位姿 (object_pose)，无法审批抓取');
    toast.warn('无物体位姿，无法审批抓取');
    return;
  }
  stateStore.setState({
    approvalLoading: true,
    approvalTargetIndex: 0,
    approvalTargetTotal: 1,
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
      });
      const line = r.summary || (r.approved ? '通过' : '未通过');
      stateStore.pushSystemLog('info', `审批抓取: ${line}`);
      if (r.approved) {
        toast.success(`审批完成：${line}`);
      } else if (r.severity === SEV_REJECT) {
        toast.warn(`审批未通过：${line}`);
      } else {
        toast.info(`审批完成：${line}`);
      }
    } else {
      stateStore.pushSystemLog('warn', '审批抓取无响应');
      toast.warn('审批抓取无响应');
    }
  });
  stateStore.pushSystemLog('info', '发起审批抓取…');
  toast.info('已发送审批请求…');
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
    const targetTotal = state.approvalTargetTotal != null ? state.approvalTargetTotal : 1;
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
      resultHtml = `
        <div class="approval-loading">
          <div>审批中… ${targetLabel ? `(目标 ${targetLabel})` : ''}</div>
          ${stepsHtml}
        </div>
      `;
    } else if (res) {
      const badgeClass = severityClass(res.severity);
      const badgeTextBase = severityLabel(res.severity);
      const badgeText = `抓取${badgeTextBase}`;
      const summaryPrefix = '抓取审批';
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
          ${res.best_candidate_pose ? `
            <div class="approval-suggestion">
              <span class="approval-suggestion__label">推荐抓取位姿:</span>
              <span class="approval-suggestion__pos">x=${(res.best_candidate_pose.pose?.position?.x ?? 0).toFixed(3)} y=${(res.best_candidate_pose.pose?.position?.y ?? 0).toFixed(3)} z=${(res.best_candidate_pose.pose?.position?.z ?? 0).toFixed(3)}</span>
            </div>
          ` : ''}
          ${stepsHtml}
        </div>
      `;
    } else {
      resultHtml = '<div class="approval-result approval-result--empty">点击「审批抓取」获取结果</div>';
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
};
