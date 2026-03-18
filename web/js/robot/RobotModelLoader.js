/**
 * 从 web/robot/ 加载 Orion STL，按 URDF 层级构建（保持 URDF 坐标），整机做 Z-up -> Y-up
 */

import * as THREE from 'three';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';

const MESH_DIR = '/robot/meshes/stl/';

const JOINT_DEFS = [
  { name: 'joint_base_link_Link1', xyz: [0, 0, 0], rpy: [0, 0, 0], axis: [1, 0, 0] },
  { name: 'joint_Link1_Link2', xyz: [-0.0105, 0, 0.0699], rpy: [0, 0, 0], axis: [0, 1, 0] },
  { name: 'joint_Link2_Link3', xyz: [-0.2247, 0, 0.8662], rpy: [0, 0, 0], axis: [0, 1, 0] },
  { name: 'joint_Link3_LinkVirtual', xyz: [0.1835, 0, 0.0157], rpy: [0, -3.97248, 0], axis: null },
  { name: 'joint_LinkVirtual_Link4', xyz: [0, 0, 0], rpy: [0, 0, 0], axis: [0, 0, 1] },
  { name: 'joint_Link4_Link5', xyz: [0, -0.0091, 0.3784], rpy: [0, 0, 0], axis: [0, 1, 0] },
  { name: 'joint_Link5_Link6', xyz: [-0.0113, 0.0091, 0.2516], rpy: [0, 0, 0], axis: [0, 0, 1] },
  { name: 'joint_Link6_Link7', xyz: [0.0363, 0, 0.0499], rpy: [0, 0, 0], axis: [0, 1, 0] },
  { name: 'joint_Link6_Link8', xyz: [-0.0363, 0, 0.047], rpy: [0, 0, 0], axis: [0, 1, 0] },
];

/* 正运动学链：base_link → Link6（6 个 revolute + 1 个 fixed），与 JOINT_DEFS 一致，用于工作空间逐点采样 */
const FK_ORIGINS = [
  [0, 0, 0],
  [-0.0105, 0, 0.0699],
  [-0.2247, 0, 0.8662],
  [0.1835, 0, 0.0157],
  [0, 0, 0],
  [0, -0.0091, 0.3784],
  [-0.0113, 0.0091, 0.2516],
];
const FK_AXES = [
  [1, 0, 0],
  [0, 1, 0],
  [0, 1, 0],
  null,
  [0, 0, 1],
  [0, 1, 0],
  [0, 0, 1],
];
const FK_FIXED_RPY = [0, -3.97248, 0];

const JOINT_LIMIT_MIN = -Math.PI;
const JOINT_LIMIT_MAX = Math.PI;
const SAMPLE_STEPS = 4;

function fkChain(jointAngles) {
  const total = new THREE.Matrix4().identity();
  const pos = new THREE.Vector3();
  const quat = new THREE.Quaternion();
  const scale = new THREE.Vector3(1, 1, 1);
  let ji = 0;
  for (let i = 0; i < FK_ORIGINS.length; i++) {
    const origin = FK_ORIGINS[i];
    let M;
    if (FK_AXES[i] !== null) {
      const angle = jointAngles[ji++];
      const ax = FK_AXES[i];
      const R = new THREE.Matrix4().makeRotationAxis(
        new THREE.Vector3(ax[0], ax[1], ax[2]),
        angle
      );
      M = new THREE.Matrix4().makeTranslation(origin[0], origin[1], origin[2]).multiply(R);
    } else {
      quat.setFromEuler(
        new THREE.Euler(FK_FIXED_RPY[0], FK_FIXED_RPY[1], FK_FIXED_RPY[2], 'XYZ')
      );
      M = new THREE.Matrix4().compose(
        new THREE.Vector3(origin[0], origin[1], origin[2]),
        quat,
        scale
      );
    }
    total.multiply(M);
  }
  pos.set(0, 0, 0);
  pos.applyMatrix4(total);
  return pos;
}

function sampleWorkspace() {
  const min = { x: Infinity, y: Infinity, z: Infinity };
  const max = { x: -Infinity, y: -Infinity, z: -Infinity };
  const step = (JOINT_LIMIT_MAX - JOINT_LIMIT_MIN) / (SAMPLE_STEPS - 1);
  const angles = new Array(6);
  for (let i0 = 0; i0 < SAMPLE_STEPS; i0++) {
    angles[0] = JOINT_LIMIT_MIN + i0 * step;
    for (let i1 = 0; i1 < SAMPLE_STEPS; i1++) {
      angles[1] = JOINT_LIMIT_MIN + i1 * step;
      for (let i2 = 0; i2 < SAMPLE_STEPS; i2++) {
        angles[2] = JOINT_LIMIT_MIN + i2 * step;
        for (let i3 = 0; i3 < SAMPLE_STEPS; i3++) {
          angles[3] = JOINT_LIMIT_MIN + i3 * step;
          for (let i4 = 0; i4 < SAMPLE_STEPS; i4++) {
            angles[4] = JOINT_LIMIT_MIN + i4 * step;
            for (let i5 = 0; i5 < SAMPLE_STEPS; i5++) {
              angles[5] = JOINT_LIMIT_MIN + i5 * step;
              const p = fkChain(angles);
              if (p.x < min.x) min.x = p.x;
              if (p.y < min.y) min.y = p.y;
              if (p.z < min.z) min.z = p.z;
              if (p.x > max.x) max.x = p.x;
              if (p.y > max.y) max.y = p.y;
              if (p.z > max.z) max.z = p.z;
            }
          }
        }
      }
    }
  }
  return { min, max };
}

let WORKSPACE_URDF_CACHED = null;
function getWorkspaceUrdf() {
  if (WORKSPACE_URDF_CACHED) return WORKSPACE_URDF_CACHED;
  const { min, max } = sampleWorkspace();
  const margin = 0.02;
  WORKSPACE_URDF_CACHED = {
    minX: min.x - margin,
    maxX: max.x + margin,
    minY: min.y - margin,
    maxY: max.y + margin,
    minZ: min.z - margin,
    maxZ: max.z + margin,
  };
  return WORKSPACE_URDF_CACHED;
}

export function getWorkspaceBoundsScene() {
  const u = getWorkspaceUrdf();
  return {
    min: { x: u.minX, y: u.minZ, z: -u.maxY },
    max: { x: u.maxX, y: u.maxZ, z: -u.minY },
  };
}
export function getWorkspaceBoundsForDoc() {
  const u = getWorkspaceUrdf();
  return {
    urdf_frame: {
      x_m: { min: u.minX, max: u.maxX },
      y_m: { min: u.minY, max: u.maxY },
      z_m: { min: u.minZ, max: u.maxZ },
    },
    from_kinematics: true,
  };
}

function loadSTL(url) {
  return new Promise((resolve, reject) => {
    const loader = new STLLoader();
    loader.load(url, resolve, undefined, reject);
  });
}

/* 工业 UI：金属感灰蓝 #C7D2FE */
const ROBOT_MATERIAL_COLOR = 0xc7d2fe;
const ROBOT_METALNESS = 0.4;
const ROBOT_ROUGHNESS = 0.6;
const COLLISION_DEBUG_COLOR = 0xef4444;

function addMesh(group, linkName) {
  return loadSTL(MESH_DIR + linkName + '.stl').then((geometry) => {
    const mat = new THREE.MeshStandardMaterial({
      color: ROBOT_MATERIAL_COLOR,
      metalness: ROBOT_METALNESS,
      roughness: ROBOT_ROUGHNESS,
    });
    const mesh = new THREE.Mesh(geometry, mat);
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    group.add(mesh);
    const edges = new THREE.EdgesGeometry(geometry, 15);
    const line = new THREE.LineSegments(
      edges,
      new THREE.LineBasicMaterial({ color: COLLISION_DEBUG_COLOR })
    );
    line.name = 'collision_debug';
    line.visible = false;
    group.add(line);
  }).catch((err) => {
    console.warn('RobotModelLoader: ' + linkName + ' 加载失败', err);
  });
}

function rpyToQuaternion(rpy) {
  return new THREE.Quaternion().setFromEuler(
    new THREE.Euler(rpy[0], rpy[1], rpy[2], 'XYZ')
  );
}

function makeJointGroup(jointDef) {
  const g = new THREE.Group();
  g.name = jointDef.name;
  g.position.set(jointDef.xyz[0], jointDef.xyz[1], jointDef.xyz[2]);
  if (jointDef.rpy && (jointDef.rpy[0] || jointDef.rpy[1] || jointDef.rpy[2])) {
    g.quaternion.copy(rpyToQuaternion(jointDef.rpy));
  }
  return g;
}

export function loadRobotModel() {
  const root = new THREE.Group();
  root.name = 'orion';
  const jointTransforms = [];

  return addMesh(root, 'base_link').then(() => {
    let parent = root;
    let ji = 0;

    function addJointAndLink(linkName) {
      if (ji >= JOINT_DEFS.length) return Promise.resolve();
      const j = JOINT_DEFS[ji++];
      const jointGroup = makeJointGroup(j);
      if (j.axis) jointTransforms.push({ name: j.name, group: jointGroup, axis: j.axis });
      parent.add(jointGroup);
      const p = linkName ? addMesh(jointGroup, linkName) : Promise.resolve();
      return p.then(() => {
        parent = jointGroup;
        return parent;
      });
    }

    return addJointAndLink('Link1')
      .then(() => addJointAndLink('Link2'))
      .then(() => addJointAndLink('Link3'))
      .then(() => addJointAndLink(null))
      .then(() => addJointAndLink('Link4'))
      .then(() => addJointAndLink('Link5'))
      .then(() => addJointAndLink('Link6'));
  }).then(() => {
    const link6Group = root.getObjectByName('joint_Link5_Link6');
    if (!link6Group) return root;
    const j7 = JOINT_DEFS[7];
    const j8 = JOINT_DEFS[8];
    const g7 = makeJointGroup(j7);
    const g8 = makeJointGroup(j8);
    jointTransforms.push({ name: j7.name, group: g7, axis: j7.axis });
    jointTransforms.push({ name: j8.name, group: g8, axis: j8.axis });
    link6Group.add(g7);
    link6Group.add(g8);
    return addMesh(g7, 'Link7').then(() => addMesh(g8, 'Link8')).then(() => root);
  }).then(() => {
    root.rotation.x = -Math.PI / 2;
    root.userData.jointTransforms = jointTransforms;
    root.setJointValues = function (names, positions) {
      if (!names || !positions) return;
      const map = {};
      for (let i = 0; i < names.length; i++) map[names[i]] = positions[i];
      jointTransforms.forEach(({ name, group, axis }) => {
        const v = map[name];
        if (v === undefined || v === null) return;
        group.quaternion.setFromAxisAngle(
          new THREE.Vector3(axis[0], axis[1], axis[2]),
          v
        );
      });
    };
    return root;
  });
}
