/**
 * 中间主区域：Three.js 3D 视图 + 图层开关浮层 + 视角快捷按钮
 * 根据 stateStore 的 objectPose / jointPositions 更新 marker 与机械臂
 */

import * as THREE from 'three';
import RobotScene from '../robot/RobotScene.js';
import stateStore from '../data/stateStore.js';
import { t, subscribeLocale } from '../data/i18n.js';

let sceneApi = null;
let unsubscribeState = null;

/* base_link 为 Z-up（ROS），场景为 Y-up（Three.js），坐标变换：rotateX(-π/2) */
const Z_UP_TO_Y_UP = new THREE.Quaternion().setFromEuler(new THREE.Euler(-Math.PI / 2, 0, 0));
const layerToggles = {
  showTargets: true,
  showTrajectory: true,
  showAxes: true,
  showWorldObject: true,
  showAttachedObject: true,
  showCollision: false,
  showWorkspace: false,
  showCoordFrames: true,
};

/** ROS geometry_msgs/Point → Three 用的 {x,y,z}（暂不翻轴，后续 applyBaseLinkToScene）。 */
function rosToThreePosition(pos) {
  if (!pos) return { x: 0, y: 0, z: 0 };
  const x = pos.x || 0;
  const y = pos.y || 0;
  const z = pos.z || 0;
  return { x, y, z };
}

/** ROS 四元数 → THREE.Quaternion（w 默认 1）。 */
function rosToThreeQuaternion(q) {
  if (!q) return null;
  return new THREE.Quaternion(
    q.x ?? 0,
    q.y ?? 0,
    q.z ?? 0,
    (q.w ?? 1)
  );
}

/** 浮层勾选：同步 layerToggles 与 RobotScene 内 mesh 可见性。 */
function createLayerToggles(containerEl, sceneApiRef) {
  const panel = document.createElement('div');
  panel.className = 'viewport-3d__layer-panel';
  const title = document.createElement('div');
  title.className = 'viewport-3d__layer-panel-title';
  title.textContent = t('viewport.layers');
  panel.appendChild(title);
  const div = document.createElement('div');
  div.className = 'viewport-3d__toolbar';
  const toggles = [
    { id: 'show-axes', i18n: 'viewport.layer.axes', key: 'showAxes', default: true },
    { id: 'show-coord-frames', i18n: 'viewport.layer.frames', key: 'showCoordFrames', default: true },
    { id: 'show-world', i18n: 'viewport.layer.world', key: 'showWorldObject', default: true },
    { id: 'show-attached', i18n: 'viewport.layer.attached', key: 'showAttachedObject', default: true },
    { id: 'show-collision', i18n: 'viewport.layer.collision', key: 'showCollision', default: false },
    { id: 'show-trajectory', i18n: 'viewport.layer.trajectory', key: 'showTrajectory', default: true },
    { id: 'show-targets', i18n: 'viewport.layer.targets', key: 'showTargets', default: true },
    { id: 'show-workspace', i18n: 'viewport.layer.workspace', key: 'showWorkspace', default: false },
  ];
  const state = {};
  const layerI18nRefs = [];
  toggles.forEach((tInfo) => {
    state[tInfo.key] = tInfo.default;
    const label = document.createElement('label');
    const input = document.createElement('input');
    input.type = 'checkbox';
    input.id = tInfo.id;
    input.checked = tInfo.default;
    const span = document.createElement('span');
    span.className = 'viewport-3d__layer-label-text';
    span.textContent = ` ${t(tInfo.i18n)}`;
    layerI18nRefs.push({ span, key: tInfo.i18n });
    label.appendChild(input);
    label.appendChild(span);
    const cb = input;
    cb.addEventListener('change', () => {
      state[tInfo.key] = cb.checked;
      if (layerToggles[tInfo.key] !== undefined) layerToggles[tInfo.key] = cb.checked;
      if (sceneApiRef) {
        if (tInfo.key === 'showAxes') {
          const baseAxes = sceneApiRef.world.getObjectByName('base_axes');
          if (baseAxes) baseAxes.visible = cb.checked;
        }
        if (tInfo.key === 'showWorldObject') {
          const wo = sceneApiRef.targets.getObjectByName('world_object');
          if (wo) wo.visible = cb.checked && wo.userData.valid;
          const stW = stateStore.getState();
          if (sceneApiRef.targetObjectComposed) {
            sceneApiRef.targetObjectComposed.visible =
              cb.checked && !!stW.cableObjectPoseValid;
          }
        }
        if (tInfo.key === 'showAttachedObject') {
          const ao = sceneApiRef.targets.getObjectByName('attached_object');
          if (ao) ao.visible = cb.checked && ao.userData.valid;
        }
        if (tInfo.key === 'showCollision') {
          if (sceneApiRef.setCollisionDebugVisible) sceneApiRef.setCollisionDebugVisible(cb.checked);
        }
        if (tInfo.key === 'showTrajectory') sceneApiRef.trajectoryLine.visible = cb.checked;
        if (tInfo.key === 'showTargets') {
          const st = stateStore.getState();
          sceneApiRef.pickMarker.visible = cb.checked && !!st.objectPoseValid;
          if (sceneApiRef.pickMarkerTargetSensor) {
            sceneApiRef.pickMarkerTargetSensor.visible =
              cb.checked && !!st.targetSensorObjectPoseValid;
          }
          if (sceneApiRef.targetSensorObjectComposed) {
            sceneApiRef.targetSensorObjectComposed.visible =
              cb.checked && !!st.targetSensorObjectPoseValid;
          }
          if (sceneApiRef.pickMarkerFused) {
            sceneApiRef.pickMarkerFused.visible = cb.checked;
            const fv = !!st.fusedObjectPoseValid;
            const mat = sceneApiRef.pickMarkerFused.material;
            mat.transparent = !fv;
            mat.opacity = fv ? 1.0 : 0.38;
          }
          const showKp = cb.checked && !!st.keypointsTraceValid && (st.keypointsTrace?.points?.length || 0) > 0;
          if (Array.isArray(sceneApiRef.kpTraceSpheres)) {
            sceneApiRef.kpTraceSpheres.forEach((m, idx) => {
              if (!m) {
                return;
              }
              m.visible = showKp && idx < (st.keypointsTrace?.points?.length || 0);
            });
          }
          if (sceneApiRef.keypointsPolyline) {
            sceneApiRef.keypointsPolyline.visible =
              showKp && (st.keypointsTrace?.points?.length || 0) > 1;
          }
        }
        if (tInfo.key === 'showWorkspace') {
          const ws = sceneApiRef.world.getObjectByName('workspace_box');
          if (ws) ws.visible = cb.checked;
        }
        if (tInfo.key === 'showCoordFrames') {
          if (sceneApiRef.rovAxesGroup) sceneApiRef.rovAxesGroup.visible = cb.checked;
        }
      }
    });
    div.appendChild(label);
  });
  panel.appendChild(div);
  panel._viewportI18n = { title, layerI18nRefs };
  return panel;
}

const RAD_TO_DEG = 180 / Math.PI;

/** 左下角关节角表格壳子，返回 panel 与 tbody 引用。 */
function createJoystickTable(containerEl) {
  const panel = document.createElement('div');
  panel.className = 'viewport-3d__joystick-panel';
  const title = document.createElement('div');
  title.className = 'viewport-3d__joystick-panel-title';
  title.textContent = t('viewport.joints');
  panel.appendChild(title);
  const tableWrap = document.createElement('div');
  tableWrap.className = 'viewport-3d__joystick-table-wrap';
  const table = document.createElement('table');
  table.className = 'viewport-3d__joystick-table';
  table.innerHTML = `<thead><tr><th>${t('viewport.joint_col')}</th><th>rad</th><th>°</th></tr></thead><tbody id="joystick-table-body"></tbody>`;
  tableWrap.appendChild(table);
  panel.appendChild(tableWrap);
  return { panel, tbody: table.querySelector('#joystick-table-body') };
}

/* Orion：6 臂关节 + 2 夹爪关节(Link7/Link8)，表格显示为 6+1=7 行 */
const HAND_JOINT_NAMES = ['joint_Link6_Link7', 'joint_Link6_Link8'];

/** @param {{ html: string }} [cacheRef] 与上一帧 HTML 相同时跳过 innerHTML，减轻高频 joint_states 下的 DOM 压力。 */
function updateJoystickTable(tbody, jointNames, jointPositions, cacheRef) {
  if (!tbody) return;
  const names = jointNames || [];
  const positions = jointPositions || [];
  let html;
  if (names.length === 0 && positions.length === 0) {
    html = `<tr><td colspan="3" class="viewport-3d__joystick-empty">${t('viewport.no_data')}</td></tr>`;
  } else {
    const rows = [];
    let i = 0;
    while (i < names.length) {
      const name = names[i];
      const rad = positions[i] != null ? Number(positions[i]) : 0;
      const deg = (rad * RAD_TO_DEG).toFixed(2);
      const radStr = rad.toFixed(4);
      const isHand = HAND_JOINT_NAMES.includes(name);
      if (isHand && i + 1 < names.length && HAND_JOINT_NAMES.includes(names[i + 1])) {
        /* 两个夹爪关节合并为一行「夹爪」，用 Link7 的值表示 */
        rows.push(`<tr><td class="viewport-3d__joystick-name" title="${t('viewport.gripper_title')}">${t('viewport.gripper')}</td><td>${radStr}</td><td>${deg}</td></tr>`);
        i += 2;
      } else if (isHand) {
        rows.push(`<tr><td class="viewport-3d__joystick-name" title="${name}">${t('viewport.gripper')}</td><td>${radStr}</td><td>${deg}</td></tr>`);
        i += 1;
      } else {
        const shortName = name.replace(/^joint_/, '').replace(/Link/g, 'L');
        rows.push(`<tr><td class="viewport-3d__joystick-name" title="${name}">${shortName}</td><td>${radStr}</td><td>${deg}</td></tr>`);
        i += 1;
      }
    }
    html = rows.join('');
  }
  if (cacheRef && cacheRef.html === html) {
    return;
  }
  if (cacheRef) {
    cacheRef.html = html;
  }
  tbody.innerHTML = html;
}

/** 预设机位按钮 +「跟随末端」勾选。 */
function createViewButtons(containerEl, controls, camera, sceneApiRef) {
  const div = document.createElement('div');
  div.className = 'viewport-3d__view-buttons';
  div.style.cssText = 'position:absolute; top:10px; right:10px; display:flex; flex-direction:column; gap:4px; z-index:10;';
  const views = [
    { i18n: 'viewport.view_top', pos: [0, 1.5, 0.01], target: [0, 0, 0] },
    { i18n: 'viewport.view_front', pos: [1.2, 0, 0], target: [0, 0, 0] },
    { i18n: 'viewport.view_side', pos: [0, 0.5, 1.2], target: [0, 0, 0] },
    { i18n: 'viewport.view_default', pos: [1.5, 1.2, 1.5], target: [0, 0, 0] },
  ];
  views.forEach((v) => {
    const btn = document.createElement('button');
    btn.textContent = t(v.i18n);
    btn.setAttribute('data-i18n', v.i18n);
    btn.type = 'button';
    btn.addEventListener('click', () => {
      if (sceneApiRef && sceneApiRef.setFollowTcp) sceneApiRef.setFollowTcp(false);
      camera.position.set(v.pos[0], v.pos[1], v.pos[2]);
      controls.target.set(v.target[0], v.target[1], v.target[2]);
    });
    div.appendChild(btn);
  });
  const followLabel = document.createElement('label');
  followLabel.style.cssText = 'margin-top:6px; font-size:11px; color:var(--text-secondary); cursor:pointer;';
  const followInput = document.createElement('input');
  followInput.type = 'checkbox';
  followInput.id = 'view-follow-tcp';
  const followSpan = document.createElement('span');
  followSpan.className = 'viewport-3d__follow-label-text';
  followSpan.textContent = ` ${t('viewport.follow_tcp')}`;
  followLabel.appendChild(followInput);
  followLabel.appendChild(followSpan);
  followInput.addEventListener('change', () => {
    if (sceneApiRef && sceneApiRef.setFollowTcp) sceneApiRef.setFollowTcp(followInput.checked);
  });
  div.appendChild(followLabel);
  return div;
}

/**
 * 创建画布、RobotScene、图层开关、视角按钮、关节表；订阅 stateStore 驱动目标点/ROV/轨迹/关节。
 * @returns {object|null} RobotScene API（供外部 getScene）
 */
function mount(containerId) {
  const el = document.getElementById(containerId);
  if (!el) return null;

  const canvasWrap = document.createElement('div');
  canvasWrap.className = 'viewport-3d__canvas-wrap';
  el.appendChild(canvasWrap);

  sceneApi = RobotScene.createScene(canvasWrap);
  const toolbar = createLayerToggles(el, sceneApi);
  el.appendChild(toolbar);
  const viewBtns = createViewButtons(el, sceneApi.controls, sceneApi.camera, sceneApi);
  el.appendChild(viewBtns);

  const { panel: joystickPanel, tbody: joystickTbody } = createJoystickTable(el);
  el.appendChild(joystickPanel);

  const joystickTableHtmlCache = { html: '' };

  function applyViewportI18n() {
    const layerPanel = el.querySelector('.viewport-3d__layer-panel');
    if (layerPanel && layerPanel._viewportI18n) {
      const { title: layerTitle, layerI18nRefs } = layerPanel._viewportI18n;
      layerTitle.textContent = t('viewport.layers');
      layerI18nRefs.forEach(({ span, key }) => {
        span.textContent = ` ${t(key)}`;
      });
    }
    const joyTitle = el.querySelector('.viewport-3d__joystick-panel-title');
    if (joyTitle) {
      joyTitle.textContent = t('viewport.joints');
    }
    const joyThead = el.querySelector('.viewport-3d__joystick-table thead tr');
    if (joyThead) {
      joyThead.innerHTML = `<th>${t('viewport.joint_col')}</th><th>rad</th><th>°</th>`;
    }
    el.querySelectorAll('.viewport-3d__view-buttons button[data-i18n]').forEach((btn) => {
      const k = btn.getAttribute('data-i18n');
      if (k) {
        btn.textContent = t(k);
      }
    });
    const followSpan = el.querySelector('.viewport-3d__follow-label-text');
    if (followSpan) {
      followSpan.textContent = ` ${t('viewport.follow_tcp')}`;
    }
    joystickTableHtmlCache.html = '';
    updateJoystickTable(
      joystickTbody,
      stateStore.getState().jointNames,
      stateStore.getState().jointPositions,
      null
    );
  }

  subscribeLocale(applyViewportI18n);

  let viewportRaf = null;
  function scheduleUpdateFromState() {
    if (viewportRaf != null) {
      return;
    }
    viewportRaf = requestAnimationFrame(() => {
      viewportRaf = null;
      updateFromState(stateStore.getState());
    });
  }

  function updateFromState(s) {
    if (!sceneApi) return;
    updateJoystickTable(joystickTbody, s.jointNames, s.jointPositions, joystickTableHtmlCache);
    function applyBaseLinkToScene(pos) {
      const v = new THREE.Vector3(pos.x, pos.y, pos.z);
      v.applyQuaternion(Z_UP_TO_Y_UP);
      return v;
    }

    const pickPos = rosToThreePosition(s.objectPose?.position);
    const tScene = applyBaseLinkToScene(pickPos);
    sceneApi.pickMarker.position.copy(tScene);
    sceneApi.pickMarker.visible = layerToggles.showTargets && !!s.objectPoseValid;
    const tf = rosToThreePosition(s.fusedObjectPose?.position);
    const tfScene = applyBaseLinkToScene(tf);
    if (sceneApi.pickMarkerFused) {
      sceneApi.pickMarkerFused.position.copy(tfScene);
      sceneApi.pickMarkerFused.visible = layerToggles.showTargets;
      const fmat = sceneApi.pickMarkerFused.material;
      fmat.transparent = !s.fusedObjectPoseValid;
      fmat.opacity = s.fusedObjectPoseValid ? 1.0 : 0.38;
    }

    const tsPickPos = rosToThreePosition(s.targetSensorObjectPose?.position);
    const tsScene = applyBaseLinkToScene(tsPickPos);
    if (sceneApi.pickMarkerTargetSensor) {
      sceneApi.pickMarkerTargetSensor.position.copy(tsScene);
      const tsOrient = s.targetSensorObjectPose?.orientation;
      if (tsOrient) {
        const tq = rosToThreeQuaternion(tsOrient);
        if (tq) {
          sceneApi.pickMarkerTargetSensor.quaternion.copy(
            new THREE.Quaternion().copy(Z_UP_TO_Y_UP).multiply(tq)
          );
        }
      } else {
        sceneApi.pickMarkerTargetSensor.quaternion.identity();
      }
      sceneApi.pickMarkerTargetSensor.visible =
        layerToggles.showTargets && !!s.targetSensorObjectPoseValid;
    }
    if (sceneApi.targetSensorObjectComposed) {
      sceneApi.targetSensorObjectComposed.position.copy(tsScene);
      const compQ = sceneApi.targetSensorObjectComposed.userData.model_compensation_quaternion;
      let modelCompensationQuat = new THREE.Quaternion(0, 0, 0, 1);
      if (Array.isArray(compQ) && compQ.length === 4) {
        modelCompensationQuat = new THREE.Quaternion(
          Number(compQ[0]) || 0,
          Number(compQ[1]) || 0,
          Number(compQ[2]) || 0,
          Number(compQ[3]) || 1
        );
      }
      const tsObjOrient = s.targetSensorObjectPose?.orientation;
      if (tsObjOrient) {
        const tqo = rosToThreeQuaternion(tsObjOrient);
        if (tqo) {
          sceneApi.targetSensorObjectComposed.quaternion.copy(
            new THREE.Quaternion().copy(Z_UP_TO_Y_UP).multiply(tqo).multiply(modelCompensationQuat)
          );
        }
      } else {
        sceneApi.targetSensorObjectComposed.quaternion.copy(modelCompensationQuat);
      }
      sceneApi.targetSensorObjectComposed.userData.valid = !!s.targetSensorObjectPoseValid;
      sceneApi.targetSensorObjectComposed.visible =
        layerToggles.showTargets && !!s.targetSensorObjectPoseValid;
    }

    const kpPts = s.keypointsTraceValid && s.keypointsTrace?.points ? s.keypointsTrace.points : [];
    const showKpTrace = layerToggles.showTargets && kpPts.length > 0;
    if (Array.isArray(sceneApi.kpTraceSpheres)) {
      for (let i = 0; i < sceneApi.kpTraceSpheres.length; i += 1) {
        const m = sceneApi.kpTraceSpheres[i];
        if (!m) {
          continue;
        }
        if (i < kpPts.length) {
          const v = applyBaseLinkToScene(kpPts[i]);
          m.position.copy(v);
          m.visible = showKpTrace;
        } else {
          m.visible = false;
        }
      }
    }
    if (sceneApi.keypointsPolyline) {
      if (showKpTrace && kpPts.length > 1) {
        const pts3 = kpPts.map((p) => applyBaseLinkToScene(p));
        sceneApi.keypointsPolyline.geometry.setFromPoints(pts3);
        sceneApi.keypointsPolyline.visible = true;
      } else {
        sceneApi.keypointsPolyline.visible = false;
      }
    }
    const rovPos = rosToThreePosition(s.rovPoseInBaseLink?.position);
    if (sceneApi.rovAxesGroup) {
      sceneApi.rovAxesGroup.position.copy(applyBaseLinkToScene(rovPos));
      /* 与机械臂/base_axes 一致：ROS base_link(Z-up) → 场景，再乘 ROV 在 base_link 下的姿态 */
      const qScene = new THREE.Quaternion().copy(Z_UP_TO_Y_UP);
      const qRov = s.rovPoseInBaseLink?.orientation
        ? rosToThreeQuaternion(s.rovPoseInBaseLink.orientation)
        : null;
      if (qRov) {
        qScene.multiply(qRov);
      }
      sceneApi.rovAxesGroup.quaternion.copy(qScene);
      sceneApi.rovAxesGroup.visible = layerToggles.showCoordFrames && !!s.rovPoseInBaseLink;
    }
    const cablePosRos = rosToThreePosition(
      s.cableObjectPoseValid ? s.cableObjectPose?.position : null
    );
    const cableScenePos = applyBaseLinkToScene(cablePosRos);
    const wo = sceneApi.targets.getObjectByName('world_object');
    if (wo) {
      wo.position.copy(cableScenePos);
      wo.userData.valid = !!s.cableObjectPoseValid;
      wo.visible = false;
    }
    const composed = sceneApi.targetObjectComposed;
    if (composed) {
      composed.position.copy(cableScenePos);
      const cOrient = s.cableObjectPoseValid ? s.cableObjectPose?.orientation : null;
      if (cOrient) {
        const quat = rosToThreeQuaternion(cOrient);
        if (quat) {
          const qScene = new THREE.Quaternion().copy(Z_UP_TO_Y_UP).multiply(quat);
          composed.quaternion.copy(qScene);
        }
      } else {
        composed.quaternion.identity();
      }
      composed.userData.valid = !!s.cableObjectPoseValid;
      composed.visible = layerToggles.showWorldObject && !!s.cableObjectPoseValid;
    }
    const heldPos = rosToThreePosition(s.heldObjectPoseAtGrasp?.position || s.heldObjectPoseAtGrasp);
    const ao = sceneApi.targets.getObjectByName('attached_object');
    if (ao) {
      ao.position.copy(applyBaseLinkToScene(heldPos));
      ao.userData.valid = !!s.heldValid;
      ao.visible = layerToggles.showAttachedObject && !!s.heldValid;
    }
    const names = s.jointNames || [];
    const positions = s.jointPositions || [];
    if (sceneApi.setRobotJointValues && names.length && positions.length) {
      sceneApi.setRobotJointValues(names, positions);
    }
    if (s.trajectoryPoints && s.trajectoryPoints.length > 1) {
      const points = s.trajectoryPoints.map((pt) => {
        const q = rosToThreePosition(pt.position || pt);
        return applyBaseLinkToScene(q);
      });
      sceneApi.trajectoryLine.geometry.setFromPoints(points);
      sceneApi.trajectoryLine.visible = layerToggles.showTrajectory;
    } else {
      sceneApi.trajectoryLine.visible = false;
    }
  }

  unsubscribeState = stateStore.subscribe(scheduleUpdateFromState);
  updateFromState(stateStore.getState());

  window.addEventListener('resize', () => sceneApi.resize());
  window.addEventListener('orion:viewport-reload-model', () => {
    updateFromState(stateStore.getState());
  });
  window.addEventListener('orion:toggle-show-collision', (e) => {
    const cb = el.querySelector('#show-collision');
    if (cb) {
      cb.checked = !!e.detail.visible;
      cb.dispatchEvent(new Event('change'));
    }
  });

  return sceneApi;
}

/** 返回 mount 创建的 sceneApi 引用（未 mount 时为 null）。 */
function getScene() {
  return sceneApi;
}

export default { mount, getScene };
