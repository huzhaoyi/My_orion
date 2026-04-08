/**
 * Orion 上位机入口：挂载布局、连接数据层、注册全局事件。
 *
 * - init：挂载五块布局、wsClient.connect、系统日志、全局 CustomEvent（清队列、reset held、急停等）。
 * - registerGlobalHandlers：orion:* 事件与键盘快捷键由这里统一派发。
 * WebSocket URL：默认 ws://127.0.0.1:9091（与 pick_holoocean rosbridge 默认端口一致）；?ws= 覆盖；ROS 话题：?ns= / ?topic_prefix=；Keypoints：?keypoints_topic= / ?keypoints=。
 */

import TopBar from './layout/TopBar.js';
import LeftPanel from './layout/LeftPanel.js';
import Viewport3D from './layout/Viewport3D.js';
import RightPanel from './layout/RightPanel.js';
import BottomLogPanel from './layout/BottomLogPanel.js';
import wsClient from './data/wsClient.js';
import stateStore from './data/stateStore.js';
import toast from './ui/toast.js';
import { initI18n, t } from './data/i18n.js';

/** 创建各面板 DOM 挂载点、建立 rosbridge 连接并注册全局快捷键与服务封装事件。 */
function init() {
  initI18n();
  TopBar.mount('top-bar');
  LeftPanel.mount('left-panel');
  Viewport3D.mount('viewport-3d');
  RightPanel.mount('right-panel');
  BottomLogPanel.mount('bottom-panel');

  wsClient.connect();
  stateStore.pushSystemLog('info', t('startup.log'));

  registerGlobalHandlers();
}

/** get_queue_state 回调：把响应写入 stateStore（兼容 res.values 封装）。 */
function applyQueueStateToStore(res) {
  stateStore.applyQueueStateResponse(res);
}

/**
 * 注册 document 级 CustomEvent（orion:clear-queue、orion:reset-held 等）及键盘监听；
 * 与 TopBar/卡片按钮发出的事件名保持一致。
 */
function registerGlobalHandlers() {
  const handlers = {
    'orion:clear-queue': () => {
      if (!wsClient.isConnected()) {
        stateStore.pushSystemLog('warn', t('toast.not_connected_clear'));
        toast.warn(t('toast.not_connected_clear'));
        return;
      }
      toast.info(t('toast.clearing'));

      const MAX_CANCEL = 20;
      let cancelCount = 0;

      const cancelNext = () => {
        wsClient.getQueueState((res) => {
          const v = res && res.values ? res.values : res;
          const nextId = v && v.next_job_id ? String(v.next_job_id) : '';
          const empty = v && (v.queue_empty === true || (v.queue_size === 0));
          if (empty || !nextId) {
            stateStore.pushSystemLog('info', t('log.queue_cleared'));
            wsClient.getQueueState(applyQueueStateToStore);
            return;
          }
          if (cancelCount >= MAX_CANCEL) {
            stateStore.pushSystemLog(
              'warn',
              `${t('log.cancel_cap_a')} ${MAX_CANCEL}${t('log.cancel_cap_b')}${nextId.slice(0, 12)}${t('log.cancel_cap_c')}`
            );
            wsClient.getQueueState(applyQueueStateToStore);
            return;
          }
          cancelCount += 1;
          stateStore.pushSystemLog(
            'info',
            `${t('log.cancel_next')} ${nextId.slice(0, 12)}... (${cancelCount}/${MAX_CANCEL})`
          );
          wsClient.callService(wsClient.getTopicPrefix() + '/cancel_job', { job_id: nextId }, (r2) => {
            const v2 = r2 && r2.values ? r2.values : r2;
            const ok = v2 && (v2.success === true || v2.success === undefined);
            const msg2 = (v2 && v2.message) || (ok ? '已取消' : '取消失败');
            stateStore.pushSystemLog(ok ? 'info' : 'warn', msg2);
            if (ok) toast.success(msg2); else toast.warn(msg2);
            setTimeout(cancelNext, 200);
          });
        });
      };

      cancelNext();
    },
    'orion:reset-held': () => {
      if (!wsClient.isConnected()) {
        stateStore.pushSystemLog('warn', t('toast.not_connected_reset'));
        toast.warn(t('toast.not_connected_reset'));
        return;
      }
      wsClient.callService(wsClient.getTopicPrefix() + '/reset_held_object', {}, (res) => {
        const v = res && res.values ? res.values : res;
        const ok = v && (v.success === true || v.success === undefined);
        const msg = (v && v.message) || (ok ? 'ResetHeldObject 成功' : 'ResetHeldObject 失败');
        stateStore.pushSystemLog(ok ? 'info' : 'error', msg);
        if (ok) toast.success(msg); else toast.error(msg);
      });
    },
    'orion:cancel-job': (e) => {
      const jobId = e.detail?.job_id;
      if (!jobId) return;
      if (!wsClient.isConnected()) {
        stateStore.pushSystemLog('warn', t('toast.not_connected_cancel'));
        toast.warn(t('toast.not_connected_cancel'));
        return;
      }
      wsClient.callService(wsClient.getTopicPrefix() + '/cancel_job', { job_id: jobId }, (res) => {
        const v = res && res.values ? res.values : res;
        const ok = v && (v.success === true || v.success === undefined);
        const msg = (v && v.message) || (ok ? '已取消' : '取消失败');
        stateStore.pushSystemLog(ok ? 'info' : 'warn', msg);
        if (ok) toast.success(msg); else toast.warn(msg);
        if (ok) wsClient.getQueueState(applyQueueStateToStore);
      });
    },
    'orion:pick:cable': () => {
      if (!wsClient.isConnected()) {
        stateStore.pushSystemLog('warn', t('toast.not_connected_pick'));
        toast.warn(t('toast.not_connected_pick'));
        return;
      }
      const s = stateStore.getState();
      const objPose = s.cableObjectPoseValid && s.cableObjectPose ? s.cableObjectPose : null;
      if (!objPose) {
        stateStore.pushSystemLog('warn', t('toast.no_pose_cable'));
        toast.warn(t('toast.no_pose_cable'));
        return;
      }
      const object_pose = wsClient.buildPoseStamped(objPose.position, objPose.orientation);
      wsClient.submitJob({
        job_type: wsClient.JOB_TYPE.PICK,
        grasp_source: wsClient.GRASP_SOURCE.LEGACY,
        object_pose,
        object_id: '',
      }, (res) => {
        const v = res && res.values ? res.values : res;
        const ok = v && (v.success === true || v.success === undefined);
        const jid = (v && v.job_id) || '';
        const msg =
          (v && v.message) ||
          (ok ? `${t('toast.pick_cable_ok')} ${jid}`.trim() : t('toast.pick_submit_fail'));
        stateStore.pushSystemLog(ok ? 'info' : 'error', msg);
        if (ok) toast.success(msg); else toast.error(msg);
        if (ok) wsClient.getQueueState(applyQueueStateToStore);
      });
    },
    'orion:pick:target_sensor': () => {
      if (!wsClient.isConnected()) {
        stateStore.pushSystemLog('warn', t('toast.not_connected_pick'));
        toast.warn(t('toast.not_connected_pick'));
        return;
      }
      const s = stateStore.getState();
      const objPose =
        s.targetSensorObjectPoseValid && s.targetSensorObjectPose ? s.targetSensorObjectPose : null;
      if (!objPose) {
        stateStore.pushSystemLog('warn', t('toast.no_pose_target_sensor'));
        toast.warn(t('toast.no_pose_target_sensor'));
        return;
      }
      const object_pose = wsClient.buildPoseStamped(objPose.position, objPose.orientation);
      wsClient.submitJob({
        job_type: wsClient.JOB_TYPE.PICK,
        grasp_source: wsClient.GRASP_SOURCE.LEGACY,
        object_pose,
        object_id: '',
      }, (res) => {
        const v = res && res.values ? res.values : res;
        const ok = v && (v.success === true || v.success === undefined);
        const jid = (v && v.job_id) || '';
        const msg =
          (v && v.message) ||
          (ok ? `${t('toast.pick_target_sensor_ok')} ${jid}`.trim() : t('toast.pick_submit_fail'));
        stateStore.pushSystemLog(ok ? 'info' : 'error', msg);
        if (ok) toast.success(msg); else toast.error(msg);
        if (ok) wsClient.getQueueState(applyQueueStateToStore);
      });
    },
    'orion:pick:fused': () => {
      if (!wsClient.isConnected()) {
        stateStore.pushSystemLog('warn', t('toast.not_connected_pick'));
        toast.warn(t('toast.not_connected_pick'));
        return;
      }
      const s = stateStore.getState();
      const objPose = s.fusedObjectPose;
      if (!objPose || !s.fusedObjectPoseValid) {
        stateStore.pushSystemLog(
          'warn',
          'Fused pick: no object_pose_fused; check keypoint_to_arm_tf / Keypoints'
        );
        toast.warn(t('toast.no_pose_fused'));
        return;
      }
      const object_pose = wsClient.buildPoseStamped(objPose.position, objPose.orientation);
      wsClient.submitJob({
        job_type: wsClient.JOB_TYPE.PICK,
        grasp_source: wsClient.GRASP_SOURCE.FUSED,
        object_pose,
        object_id: '',
      }, (res) => {
        const v = res && res.values ? res.values : res;
        const ok = v && (v.success === true || v.success === undefined);
        const jid = (v && v.job_id) || '';
        const msg =
          (v && v.message) ||
          (ok ? `${t('toast.pick_fused_ok')} ${jid}`.trim() : t('toast.pick_submit_fail'));
        stateStore.pushSystemLog(ok ? 'info' : 'error', msg);
        if (ok) toast.success(msg); else toast.error(msg);
        if (ok) wsClient.getQueueState(applyQueueStateToStore);
      });
    },
    'orion:sync-held': (e) => {
      const tracked = e.detail?.tracked ?? true;
      if (!wsClient.isConnected()) {
        stateStore.pushSystemLog('warn', t('toast.not_connected_sync'));
        toast.warn(t('toast.not_connected_sync'));
        return;
      }
      const s = stateStore.getState();
      const object_id = s.heldObjectId || 'unknown';
      const req = {
        set_holding: true,
        object_id,
        tracked: !!tracked,
        object_pose: s.heldObjectPoseAtGrasp || (s.objectPose ? (s.objectPose.pose || s.objectPose) : null),
        tcp_pose: s.heldTcpPoseAtGrasp || null,
      };
      if (req.tracked && (!req.object_pose || !req.tcp_pose)) {
        stateStore.pushSystemLog('warn', t('log.sync_need_pose'));
        req.tracked = false;
        req.object_pose = { position: { x: 0, y: 0, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } };
        req.tcp_pose = { position: { x: 0, y: 0, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } };
      }
      if (!req.object_pose) {
        req.object_pose = { position: { x: 0, y: 0, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } };
      }
      if (!req.tcp_pose) {
        req.tcp_pose = { position: { x: 0, y: 0, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } };
      }

      wsClient.callService(wsClient.getTopicPrefix() + '/sync_held_object', req, (res) => {
        const v = res && res.values ? res.values : res;
        const ok = v && v.success;
        const msg = (v && v.message) || (ok ? '持物同步成功' : '持物同步失败');
        stateStore.pushSystemLog(ok ? 'info' : 'error', msg);
        if (ok) toast.success(msg); else toast.error(msg);
      });
    },
    'orion:open-gripper': () => {
      if (!wsClient.isConnected()) {
        stateStore.pushSystemLog('warn', t('toast.not_connected_gripper_o'));
        toast.warn(t('toast.not_connected_gripper_o'));
        return;
      }
      wsClient.callService(wsClient.getTopicPrefix() + '/open_gripper', {}, (res) => {
        const v = res && res.values ? res.values : res;
        const ok = v && (v.success === true || v.success === undefined);
        const msg = (v && v.message) || (ok ? '打开夹爪已入队' : '打开夹爪失败');
        stateStore.pushSystemLog(ok ? 'info' : 'error', msg);
        if (ok) toast.success(msg); else toast.error(msg);
        if (ok) wsClient.getQueueState(applyQueueStateToStore);
      });
    },
    'orion:close-gripper': () => {
      if (!wsClient.isConnected()) {
        stateStore.pushSystemLog('warn', t('toast.not_connected_gripper_c'));
        toast.warn(t('toast.not_connected_gripper_c'));
        return;
      }
      wsClient.callService(wsClient.getTopicPrefix() + '/close_gripper', {}, (res) => {
        const v = res && res.values ? res.values : res;
        const ok = v && (v.success === true || v.success === undefined);
        const msg = (v && v.message) || (ok ? '关闭夹爪已入队' : '关闭夹爪失败');
        stateStore.pushSystemLog(ok ? 'info' : 'error', msg);
        if (ok) toast.success(msg); else toast.error(msg);
        if (ok) wsClient.getQueueState(applyQueueStateToStore);
      });
    },
    'orion:emergency-stop': () => {
      if (!wsClient.isConnected()) {
        stateStore.pushSystemLog('warn', t('toast.not_connected_estop'));
        toast.warn(t('toast.not_connected_estop'));
        return;
      }
      wsClient.callEmergencyStop((res) => {
        const v = res && res.values ? res.values : res;
        const ok = v && (v.success === true || v.success === undefined);
        const msg = (v && v.message) || (ok ? '急停已执行' : '急停失败');
        stateStore.pushSystemLog(ok ? 'warn' : 'error', msg);
        if (ok) toast.warn(msg);
        else toast.error(msg);
        if (ok) wsClient.getQueueState(applyQueueStateToStore);
      });
    },
    'orion:go-to-ready': () => {
      if (!wsClient.isConnected()) {
        stateStore.pushSystemLog('warn', t('toast.not_connected_ready'));
        toast.warn(t('toast.not_connected_ready'));
        return;
      }
      wsClient.callGoToReady((res) => {
        const v = res && res.values ? res.values : res;
        const ok = v && (v.success === true || v.success === undefined);
        const msg = (v && v.message) || (ok ? '回 ready 成功' : '回 ready 失败');
        stateStore.pushSystemLog(ok ? 'info' : 'error', msg);
        if (ok) toast.success(msg); else toast.error(msg);
        if (ok) wsClient.getQueueState(applyQueueStateToStore);
      });
    },
  };

  Object.entries(handlers).forEach(([event, fn]) => {
    window.addEventListener(event, fn);
  });

  setInterval(() => {
    if (wsClient.isConnected()) wsClient.getQueueState(applyQueueStateToStore);
  }, 1500);
}

init();
