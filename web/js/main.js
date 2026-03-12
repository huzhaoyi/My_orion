/**
 * Orion 上位机入口：挂载布局、连接数据层、注册全局事件
 */

import TopBar from './layout/TopBar.js';
import LeftPanel from './layout/LeftPanel.js';
import Viewport3D from './layout/Viewport3D.js';
import RightPanel from './layout/RightPanel.js';
import BottomLogPanel from './layout/BottomLogPanel.js';
import wsClient from './data/wsClient.js';
import stateStore from './data/stateStore.js';
import toast from './ui/toast.js';

function init() {
  TopBar.mount('top-bar');
  LeftPanel.mount('left-panel');
  Viewport3D.mount('viewport-3d');
  RightPanel.mount('right-panel');
  BottomLogPanel.mount('bottom-panel');

  wsClient.connect();
  stateStore.pushSystemLog('info', 'Orion 上位机已启动');

  registerGlobalHandlers();
}

/* 与 orion_mtc_msgs/srv/GetQueueState.srv 响应字段一一对应 */
function applyQueueStateToStore(res) {
  if (!res) return;
  const v = res.values || res;
  const list = [];
  if (v.current_job_id) {
    list.push({ job_id: v.current_job_id, job_type: v.current_job_type || '—', is_current: true });
  }
  if (v.next_job_id && v.next_job_id !== v.current_job_id) {
    list.push({ job_id: v.next_job_id, job_type: v.next_job_type || '—', is_current: false });
  }
  stateStore.setQueueList(list);
  stateStore.setState({
    nextJobId: v.next_job_id != null ? v.next_job_id : '',
    nextJobType: v.next_job_type != null ? v.next_job_type : '',
    queueSize: v.queue_size != null ? v.queue_size : 0,
    queueEmpty: v.queue_empty != null ? v.queue_empty : true,
  });
}

function registerGlobalHandlers() {
  const handlers = {
    'orion:stop-queue': () => {
      const s = stateStore.getState();
      stateStore.setAcceptNewJobs(!s.acceptNewJobs);
      stateStore.pushSystemLog('info', `停止入队(前端拦截): ${!s.acceptNewJobs ? '已开启' : '已关闭'}`);
    },
    'orion:clear-queue': () => {
      if (!wsClient.isConnected()) {
        stateStore.pushSystemLog('warn', '未连接，无法清空队列');
        return;
      }

      const MAX_CANCEL = 20;
      let cancelCount = 0;

      const cancelNext = () => {
        wsClient.getQueueState((res) => {
          const v = res && res.values ? res.values : res;
          const nextId = v && v.next_job_id ? String(v.next_job_id) : '';
          const empty = v && (v.queue_empty === true || (v.queue_size === 0));
          if (empty || !nextId) {
            stateStore.pushSystemLog('info', '队列清空完成');
            wsClient.getQueueState(applyQueueStateToStore);
            return;
          }
          if (cancelCount >= MAX_CANCEL) {
            stateStore.pushSystemLog('warn', `已尝试取消 ${MAX_CANCEL} 次，停止（仍有 next_job_id=${nextId.slice(0, 12)}）`);
            wsClient.getQueueState(applyQueueStateToStore);
            return;
          }
          cancelCount += 1;
          stateStore.pushSystemLog('info', `取消 next_job_id: ${nextId.slice(0, 12)}... (${cancelCount}/${MAX_CANCEL})`);
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
        stateStore.pushSystemLog('warn', '未连接，无法调用 ResetHeldObject');
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
    'orion:recover': () => {
      if (!wsClient.isConnected()) {
        stateStore.pushSystemLog('warn', '未连接，无法 Recover');
        return;
      }
      stateStore.pushSystemLog('info', 'Recover: 执行 ResetHeldObject（后端 goHomeIfSafe 目前未实现）');
      wsClient.callService(wsClient.getTopicPrefix() + '/reset_held_object', {}, (res) => {
        const v = res && res.values ? res.values : res;
        const ok = v && (v.success === true || v.success === undefined);
        const msg = (v && v.message) || (ok ? 'Recover 成功' : 'Recover 失败');
        stateStore.pushSystemLog(ok ? 'info' : 'error', msg);
        if (ok) toast.success(msg); else toast.error(msg);
        if (ok) wsClient.getQueueState(applyQueueStateToStore);
      });
    },
    'orion:cancel-job': (e) => {
      const jobId = e.detail?.job_id;
      if (!jobId) return;
      if (!wsClient.isConnected()) {
        stateStore.pushSystemLog('warn', '未连接，无法取消');
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
    'orion:pick': (e) => {
      const immediate = e.detail?.immediate ?? true;
      const objectId = (e.detail?.object_id || '').trim();
      if (!wsClient.isConnected()) {
        stateStore.pushSystemLog('warn', '未连接，无法提交 Pick');
        return;
      }
      if (!stateStore.getState().acceptNewJobs && immediate === false) {
        stateStore.pushSystemLog('warn', '已停止入队（前端拦截），不会提交新任务');
        return;
      }
      const s = stateStore.getState();
      const objPose = s.objectPose;
      if (!objPose) {
        stateStore.pushSystemLog('warn', '无 object_pose，请确保 ' + wsClient.getTopicPrefix() + '/object_pose 有数据');
        return;
      }
      const object_pose = wsClient.buildPoseStamped(objPose.position, objPose.orientation);
      wsClient.submitJob({
        job_type: wsClient.JOB_TYPE.PICK,
        object_pose,
        object_id: objectId,
      }, (res) => {
        const v = res && res.values ? res.values : res;
        const ok = v && (v.success === true || v.success === undefined);
        const msg = (v && v.message) || (ok ? `抓取已提交 ${(v && v.job_id) || ''}` : '抓取提交失败');
        stateStore.pushSystemLog(ok ? 'info' : 'error', msg);
        if (ok) toast.success(msg); else toast.error(msg);
        if (ok) wsClient.getQueueState(applyQueueStateToStore);
      });
    },
    'orion:place': (e) => {
      if (!wsClient.isConnected()) {
        stateStore.pushSystemLog('warn', '未连接，无法提交 Place');
        return;
      }
      if (!stateStore.getState().acceptNewJobs && e.detail?.immediate === false) {
        stateStore.pushSystemLog('warn', '已停止入队（前端拦截），不会提交新任务');
        return;
      }
      const x = parseFloat(e.detail?.x) || 0.45;
      const y = parseFloat(e.detail?.y) || 0;
      const z = parseFloat(e.detail?.z) || 0.4;
      const qx = parseFloat(e.detail?.qx) || 0;
      const qy = parseFloat(e.detail?.qy) || 0;
      const qz = parseFloat(e.detail?.qz) || 0;
      const qw = parseFloat(e.detail?.qw) || 1.0;
      const target_pose = wsClient.buildPoseStamped({ x, y, z }, { x: qx, y: qy, z: qz, w: qw });
      wsClient.submitJob({ job_type: wsClient.JOB_TYPE.PLACE, target_pose }, (res) => {
        const v = res && res.values ? res.values : res;
        const ok = v && (v.success === true || v.success === undefined);
        const msg = (v && v.message) || (ok ? `放置已提交 ${(v && v.job_id) || ''}` : '放置提交失败');
        stateStore.pushSystemLog(ok ? 'info' : 'error', msg);
        if (ok) toast.success(msg); else toast.error(msg);
        if (ok) wsClient.getQueueState(applyQueueStateToStore);
      });
    },
    'orion:place-release': (e) => {
      if (!wsClient.isConnected()) {
        stateStore.pushSystemLog('warn', '未连接，无法提交 PlaceRelease');
        return;
      }
      if (!stateStore.getState().acceptNewJobs && e.detail?.immediate === false) {
        stateStore.pushSystemLog('warn', '已停止入队（前端拦截），不会提交新任务');
        return;
      }
      const x = parseFloat(e.detail?.x) ?? parseFloat(e.detail?.place_x) ?? 0.45;
      const y = parseFloat(e.detail?.y) ?? parseFloat(e.detail?.place_y) ?? 0;
      const z = parseFloat(e.detail?.z) ?? parseFloat(e.detail?.place_z) ?? 0.4;
      const qx = parseFloat(e.detail?.qx) ?? 0;
      const qy = parseFloat(e.detail?.qy) ?? 0;
      const qz = parseFloat(e.detail?.qz) ?? 0;
      const qw = parseFloat(e.detail?.qw) ?? parseFloat(e.detail?.place_qw) ?? 1.0;
      const target_pose = wsClient.buildPoseStamped({ x, y, z }, { x: qx, y: qy, z: qz, w: qw });
      wsClient.submitJob({ job_type: wsClient.JOB_TYPE.PLACE_RELEASE, target_pose }, (res) => {
        const v = res && res.values ? res.values : res;
        const ok = v && (v.success === true || v.success === undefined);
        const msg = (v && v.message) || (ok ? `放置释放已提交 ${(v && v.job_id) || ''}` : '放置释放提交失败');
        stateStore.pushSystemLog(ok ? 'info' : 'error', msg);
        if (ok) toast.success(msg); else toast.error(msg);
        if (ok) wsClient.getQueueState(applyQueueStateToStore);
      });
    },
    'orion:sync-held': (e) => {
      const tracked = e.detail?.tracked ?? true;
      if (!wsClient.isConnected()) {
        stateStore.pushSystemLog('warn', '未连接，无法调用 SyncHeldObject');
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
        stateStore.pushSystemLog('warn', 'SyncHeldObject(tracked) 需要 object_pose + tcp_pose（当前缺失），将改为 untracked 同步');
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
    'orion:clear-attached': () => {
      if (!wsClient.isConnected()) {
        stateStore.pushSystemLog('warn', '未连接，无法 Clear attached');
        return;
      }
      stateStore.pushSystemLog('info', 'Clear attached: 调用 ResetHeldObject');
      wsClient.callService(wsClient.getTopicPrefix() + '/reset_held_object', {}, (res) => {
        const v = res && res.values ? res.values : res;
        const ok = v && (v.success === true || v.success === undefined);
        const msg = (v && v.message) || (ok ? '清除附着成功' : '清除附着失败');
        stateStore.pushSystemLog(ok ? 'info' : 'error', msg);
        if (ok) toast.success(msg); else toast.error(msg);
        if (ok) wsClient.getQueueState(applyQueueStateToStore);
      });
    },
    'orion:reload-model': () => {
      window.dispatchEvent(new CustomEvent('orion:viewport-reload-model'));
    },
    'orion:open-gripper': () => {
      if (!wsClient.isConnected()) {
        stateStore.pushSystemLog('warn', '未连接，无法调用打开夹爪');
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
        stateStore.pushSystemLog('warn', '未连接，无法调用关闭夹爪');
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
  };

  Object.entries(handlers).forEach(([event, fn]) => {
    window.addEventListener(event, fn);
  });

  setInterval(() => {
    if (wsClient.isConnected()) wsClient.getQueueState(applyQueueStateToStore);
  }, 3000);
}

init();
