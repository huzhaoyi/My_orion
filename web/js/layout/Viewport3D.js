/**
 * 中间主区域：Three.js 3D 视图 + 图层开关浮层 + 视角快捷按钮
 * 根据 stateStore 的 objectPose / placePose / jointPositions 更新 marker 与机械臂
 */

import * as THREE from 'three';
import RobotScene from '../robot/RobotScene.js';
import stateStore from '../data/stateStore.js';

let sceneApi = null;
let unsubscribeState = null;
const layerToggles = { showTargets: true, showTrajectory: true };

function rosToThreePosition(pos) {
  if (!pos) return { x: 0, y: 0, z: 0 };
  const x = pos.x || 0;
  const y = pos.y || 0;
  const z = pos.z || 0;
  return { x, y, z };
}

function createLayerToggles(containerEl, sceneApiRef) {
  const panel = document.createElement('div');
  panel.className = 'viewport-3d__layer-panel';
  const title = document.createElement('div');
  title.className = 'viewport-3d__layer-panel-title';
  title.textContent = '显示层';
  panel.appendChild(title);
  const div = document.createElement('div');
  div.className = 'viewport-3d__toolbar';
  const toggles = [
    { id: 'show-axes', label: '坐标轴', key: 'showAxes', default: true },
    { id: 'show-world', label: 'World 物体', key: 'showWorldObject', default: true },
    { id: 'show-attached', label: 'Attached 物体', key: 'showAttachedObject', default: true },
    { id: 'show-collision', label: '碰撞体', key: 'showCollision', default: false },
    { id: 'show-trajectory', label: '轨迹', key: 'showTrajectory', default: true },
    { id: 'show-targets', label: '目标点', key: 'showTargets', default: true },
    { id: 'show-workspace', label: '工作空间', key: 'showWorkspace', default: false },
  ];
  const state = {};
  toggles.forEach((t) => {
    state[t.key] = t.default;
    const label = document.createElement('label');
    label.innerHTML = `<input type="checkbox" id="${t.id}" ${t.default ? 'checked' : ''}> ${t.label}`;
    const cb = label.querySelector('input');
    cb.addEventListener('change', () => {
      state[t.key] = cb.checked;
      if (t.key === 'showTargets') layerToggles.showTargets = cb.checked;
      if (t.key === 'showTrajectory') layerToggles.showTrajectory = cb.checked;
        if (sceneApiRef) {
          if (t.key === 'showAxes') {
            sceneApiRef.world.getObjectByName('base_axes').visible = cb.checked;
            const tcp = sceneApiRef.robot.getObjectByName('tcp_axes');
            if (tcp) tcp.visible = cb.checked;
          }
          if (t.key === 'showTrajectory') sceneApiRef.trajectoryLine.visible = cb.checked;
          if (t.key === 'showTargets') {
            sceneApiRef.pickMarker.visible = cb.checked;
            sceneApiRef.placeMarker.visible = cb.checked;
          }
        }
    });
    div.appendChild(label);
  });
  panel.appendChild(div);
  return panel;
}

const RAD_TO_DEG = 180 / Math.PI;

function createJoystickTable(containerEl) {
  const panel = document.createElement('div');
  panel.className = 'viewport-3d__joystick-panel';
  const title = document.createElement('div');
  title.className = 'viewport-3d__joystick-panel-title';
  title.textContent = '关节角度 (Joystick / joint_states)';
  panel.appendChild(title);
  const tableWrap = document.createElement('div');
  tableWrap.className = 'viewport-3d__joystick-table-wrap';
  const table = document.createElement('table');
  table.className = 'viewport-3d__joystick-table';
  table.innerHTML = '<thead><tr><th>关节</th><th>rad</th><th>°</th></tr></thead><tbody id="joystick-table-body"></tbody>';
  tableWrap.appendChild(table);
  panel.appendChild(tableWrap);
  return { panel, tbody: table.querySelector('#joystick-table-body') };
}

/* Orion：6 臂关节 + 2 夹爪关节(Link7/Link8)，表格显示为 6+1=7 行 */
const HAND_JOINT_NAMES = ['joint_Link6_Link7', 'joint_Link6_Link8'];

function updateJoystickTable(tbody, jointNames, jointPositions) {
  if (!tbody) return;
  const names = jointNames || [];
  const positions = jointPositions || [];
  if (names.length === 0 && positions.length === 0) {
    tbody.innerHTML = '<tr><td colspan="3" class="viewport-3d__joystick-empty">暂无数据</td></tr>';
    return;
  }
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
      rows.push(`<tr><td class="viewport-3d__joystick-name" title="夹爪 (Link7/Link8)">夹爪</td><td>${radStr}</td><td>${deg}</td></tr>`);
      i += 2;
    } else if (isHand) {
      rows.push(`<tr><td class="viewport-3d__joystick-name" title="${name}">夹爪</td><td>${radStr}</td><td>${deg}</td></tr>`);
      i += 1;
    } else {
      const shortName = name.replace(/^joint_/, '').replace(/Link/g, 'L');
      rows.push(`<tr><td class="viewport-3d__joystick-name" title="${name}">${shortName}</td><td>${radStr}</td><td>${deg}</td></tr>`);
      i += 1;
    }
  }
  tbody.innerHTML = rows.join('');
}

function createViewButtons(containerEl, controls, camera, sceneApiRef) {
  const div = document.createElement('div');
  div.className = 'viewport-3d__view-buttons';
  div.style.cssText = 'position:absolute; top:10px; right:10px; display:flex; flex-direction:column; gap:4px; z-index:10;';
  const views = [
    { label: 'Top', pos: [0, 1.5, 0.01], target: [0, 0, 0] },
    { label: 'Front', pos: [1.2, 0, 0], target: [0, 0, 0] },
    { label: 'Side', pos: [0, 0.5, 1.2], target: [0, 0, 0] },
    { label: 'Home', pos: [1.5, 1.2, 1.5], target: [0, 0, 0] },
  ];
  views.forEach((v) => {
    const btn = document.createElement('button');
    btn.textContent = v.label;
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
  followLabel.innerHTML = '<input type="checkbox" id="view-follow-tcp"> Follow TCP';
  const followCb = followLabel.querySelector('input');
  followCb.addEventListener('change', () => {
    if (sceneApiRef && sceneApiRef.setFollowTcp) sceneApiRef.setFollowTcp(followCb.checked);
  });
  div.appendChild(followLabel);
  return div;
}

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

  function updateFromState(s) {
    if (!sceneApi) return;
    updateJoystickTable(joystickTbody, s.jointNames, s.jointPositions);
    const t = rosToThreePosition(s.objectPose?.position);
    sceneApi.pickMarker.position.set(t.x, t.y, t.z);
    sceneApi.pickMarker.visible = layerToggles.showTargets && !!s.objectPoseValid;
    const p = rosToThreePosition(s.placePose?.position);
    sceneApi.placeMarker.position.set(p.x, p.y, p.z);
    sceneApi.placeMarker.visible = layerToggles.showTargets && !!s.placePoseValid;
    const names = s.jointNames || [];
    const positions = s.jointPositions || [];
    if (sceneApi.setRobotJointValues && names.length && positions.length) {
      sceneApi.setRobotJointValues(names, positions);
    }
    if (s.trajectoryPoints && s.trajectoryPoints.length > 1) {
      const points = s.trajectoryPoints.map((pt) => {
        const q = rosToThreePosition(pt.position || pt);
        return new THREE.Vector3(q.x, q.y, q.z);
      });
      sceneApi.trajectoryLine.geometry.setFromPoints(points);
      sceneApi.trajectoryLine.visible = layerToggles.showTrajectory;
    } else {
      sceneApi.trajectoryLine.visible = false;
    }
  }

  unsubscribeState = stateStore.subscribe(updateFromState);
  updateFromState(stateStore.getState());

  window.addEventListener('resize', () => sceneApi.resize());
  window.addEventListener('orion:viewport-reload-model', () => {
    updateFromState(stateStore.getState());
  });

  return sceneApi;
}

function getScene() {
  return sceneApi;
}

export default { mount, getScene };
