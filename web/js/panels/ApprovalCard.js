/**
 * 审批结果卡片：审批抓取，展示 approved、severity、summary、items、建议位姿
 */

import stateStore from '../data/stateStore.js';
import wsClient from '../data/wsClient.js';
import toast from '../ui/toast.js';
import { t, subscribeLocale } from '../data/i18n.js';
import { stageNameLabel } from '../data/labels.js';

const SEV_PASS = 0;
const SEV_WARNING = 1;
const SEV_REJECT = 2;
const DEFAULT_PICK_TARGET_INDEX = 0;

/** 右栏任务 Tab 切换/语言切换会重复调用 renderResult，须释放上轮订阅避免监听器泄漏。 */
let approvalUnsubscribeState = null;
let approvalUnsubscribeLocale = null;

/** CheckPick item.level → 等级标签 */
function levelLabel(level) {
  if (level === 0) return t('approval.level.info');
  if (level === 1) return t('approval.level.warn');
  return t('approval.level.err');
}

/** item.level → 列表项 CSS 修饰类。 */
function levelClass(level) {
  if (level === 0) return 'approval-item--info';
  if (level === 1) return 'approval-item--warn';
  return 'approval-item--error';
}

/** severity 枚举 → 标签 */
function severityLabel(severity) {
  if (severity === SEV_PASS) return t('approval.pick_pass');
  if (severity === SEV_WARNING) return t('approval.pick_warn');
  return t('approval.pick_reject');
}

/** severity → 顶栏徽章样式类。 */
function severityClass(severity) {
  if (severity === SEV_PASS) return 'approval-badge--pass';
  if (severity === SEV_WARNING) return 'approval-badge--warn';
  return 'approval-badge--reject';
}

/** 右栏「审批抓取」：校验连接与 object_pose 后调用 wsClient.checkPick，结果写入 stateStore + toast。 */
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
    stateStore.pushSystemLog('warn', t('approval.toast_no_ws'));
    toast.warn(t('approval.toast_no_ws'));
    return;
  }
  const s = stateStore.getState();
  const objectPose = s.objectPoseValid && s.objectPose
    ? { header: { frame_id: 'arm_base_link' }, pose: s.objectPose }
    : null;
  if (!objectPose) {
    stateStore.pushSystemLog('warn', t('approval.toast_no_pose'));
    toast.warn(t('approval.toast_no_pose'));
    return;
  }
  stateStore.setState({
    approvalLoading: true,
    approvalTargetIndex: 0,
    approvalTargetTotal: 1,
    checkPickProgress: null,
  });
  wsClient.checkPick(objectPose, (raw) => {
    stateStore.setState({ approvalLoading: false, checkPickProgress: null });
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
      const line = r.summary || (r.approved ? t('approval.pick_pass') : t('approval.fallback_reject'));
      stateStore.pushSystemLog('info', `${t('approval.log_line_prefix')}${line}`);
      if (r.approved) {
        toast.success(`${t('approval.toast_done_ok')}${line}`);
      } else if (r.severity === SEV_REJECT) {
        toast.warn(`${t('approval.toast_done_reject')}${line}`);
      } else {
        toast.info(`${t('approval.toast_done_info')}${line}`);
      }
    } else {
      stateStore.pushSystemLog('warn', t('approval.log_no_response'));
      toast.warn(t('approval.log_no_response'));
    }
  });
  stateStore.pushSystemLog('info', t('approval.log_sent'));
  toast.info(t('approval.toast_sent'));
}

/**
 * 仅更新“审批结果”展示区域（由 RightPanel 提供容器，按钮在 RightPanel 内绑定）
 */
function renderResult(resultContainerEl) {
  if (!resultContainerEl) return;
  if (approvalUnsubscribeState) {
    approvalUnsubscribeState();
    approvalUnsubscribeState = null;
  }
  if (approvalUnsubscribeLocale) {
    approvalUnsubscribeLocale();
    approvalUnsubscribeLocale = null;
  }
  function update() {
    if (!resultContainerEl.isConnected) return;
    const state = stateStore.getState();
    const res = state.approvalResult;
    const loading = state.approvalLoading || false;
    const cp = state.checkPickProgress;
    const esc = (x) => String(x)
      .replace(/&/g, '&amp;')
      .replace(/</g, '&lt;')
      .replace(/"/g, '&quot;');
    const phaseLine =
      loading && cp && cp.stageName
        ? `${stageNameLabel(cp.stageName)}${cp.detail ? ` · ${esc(cp.detail)}` : ''}`
        : '';
    const targetIndex = (state.approvalTargetIndex != null ? state.approvalTargetIndex : DEFAULT_PICK_TARGET_INDEX) + 1;
    const targetTotal = state.approvalTargetTotal != null ? state.approvalTargetTotal : 1;
    const targetLabel =
      targetTotal > 0
        ? `${t('approval.target_pre')}${targetIndex}${t('approval.target_mid')}${targetTotal}${t('approval.target_post')}`
        : '';

    const stepsHtml = `
      <div class="approval-steps">
        <div class="approval-step">${t('approval.step1')}</div>
        <div class="approval-step">${t('approval.step2')}</div>
        <div class="approval-step">${t('approval.step3')}</div>
      </div>
    `;

    let resultHtml = '';
    if (loading) {
      resultHtml = `
        <div class="approval-loading">
          <div>${t('approval.loading')}${targetLabel ? ` (${t('approval.target')} ${targetLabel})` : ''}</div>
          ${phaseLine ? `<div class="approval-phase-line">${phaseLine}</div>` : ''}
          ${stepsHtml}
        </div>
      `;
    } else if (res) {
      const badgeClass = severityClass(res.severity);
      const badgeTextBase = severityLabel(res.severity);
      const badgeText = `${t('approval.badge_pick_prefix')}${badgeTextBase}`;
      const summaryPrefix = t('approval.summary_title');
      const summaryTextRaw = (res.summary || '').trim();
      const summaryText =
        summaryTextRaw || (res.approved ? `${summaryPrefix}：${t('approval.summary_ok')}` : `${summaryPrefix}：${t('approval.summary_empty')}`);
      const items = (res.items || []).map((it) => {
        const msg = (it.message || '').trim();
        const sug = (it.suggestion || '').trim();
        const row = `<tr class="approval-item ${levelClass(it.level || 0)}">
          <td class="approval-item__code">${(it.code || '').trim() || '—'}</td>
          <td class="approval-item__level">${levelLabel(it.level || 0)}</td>
          <td class="approval-item__msg">${msg || '—'}</td>
          <td class="approval-item__sug">${sug ? `${t('approval.suggest_prefix')}${sug}` : '—'}</td>
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
                <thead><tr><th>${t('approval.th_code')}</th><th>${t('approval.th_level')}</th><th>${t('approval.th_msg')}</th><th>${t('approval.th_sug')}</th></tr></thead>
                <tbody>${items}</tbody>
              </table>
            </div>
          ` : ''}
          ${res.best_candidate_pose ? `
            <div class="approval-suggestion">
              <span class="approval-suggestion__label">${t('approval.suggest_pose')}</span>
              <span class="approval-suggestion__pos">x=${(res.best_candidate_pose.pose?.position?.x ?? 0).toFixed(3)} y=${(res.best_candidate_pose.pose?.position?.y ?? 0).toFixed(3)} z=${(res.best_candidate_pose.pose?.position?.z ?? 0).toFixed(3)}</span>
            </div>
          ` : ''}
          ${stepsHtml}
        </div>
      `;
    } else {
      resultHtml = `<div class="approval-result approval-result--empty">${t('approval.empty_hint')}</div>`;
    }
    resultContainerEl.innerHTML = resultHtml;
  }
  update();
  approvalUnsubscribeState = stateStore.subscribe(() => update());
  approvalUnsubscribeLocale = subscribeLocale(() => update());
}

export default {
  render: renderResult,
  renderResult,
  handlePickClick,
};
