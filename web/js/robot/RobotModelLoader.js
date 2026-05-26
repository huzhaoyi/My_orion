/**
 * 从 web/robot/ 加载 Orion STL，按 URDF 层级构建（保持 URDF 坐标），整机做 Z-up -> Y-up
 */

import * as THREE from 'three';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';
import { FEASIBILITY_WORKSPACE } from '../data/feasibilityWorkspace.js';

const MESH_DIR = '/robot/meshes/stl/';

/* 夹爪：SRDF open≈±0.4 rad 时实机已全开；STL 按 URDF 1:1 显示偏「夹紧」。仅用于 3D 视觉放大（不影响 FK 采样）。 */
const GRIPPER_JOINT_NAMES = new Set(['joint_Link6_Link7', 'joint_Link6_Link8']);
const GRIPPER_MECH_FULL_OPEN_RAD = 0.4;
const GRIPPER_VISUAL_MAX_RAD = Math.PI * 0.62;

/**
 * @param {number} rad /joint_states 读到的角（Link7 正开、Link8 负开）
 * @returns {number} 用于 mesh 显示的角
 */
function gripperAngleForDisplay(rad) {
  if (!Number.isFinite(rad)) {
    return 0.0;
  }
  const sign = rad >= 0.0 ? 1.0 : -1.0;
  const mag = Math.min(Math.abs(rad), GRIPPER_MECH_FULL_OPEN_RAD * 1.5);
  const t = mag / GRIPPER_MECH_FULL_OPEN_RAD;
  return sign * Math.min(t * GRIPPER_VISUAL_MAX_RAD, GRIPPER_VISUAL_MAX_RAD);
}

const JOINT_DEFS = [
  { name: 'joint_arm_base_link_Link1', xyz: [0, 0, 0], rpy: [0, 0, 0], axis: [1, 0, 0] },
  { name: 'joint_Link1_Link2', xyz: [-0.0105, 0, 0.0699], rpy: [0, 0, 0], axis: [0, 1, 0] },
  { name: 'joint_Link2_Link3', xyz: [-0.2247, 0, 0.8662], rpy: [0, 0, 0], axis: [0, 1, 0] },
  { name: 'joint_Link3_LinkVirtual', xyz: [0.1835, 0, 0.0157], rpy: [0, -3.97248, 0], axis: null },
  { name: 'joint_LinkVirtual_Link4', xyz: [0, 0, 0], rpy: [0, 0, 0], axis: [0, 0, 1] },
  { name: 'joint_Link4_Link5', xyz: [0, -0.0091, 0.3784], rpy: [0, 0, 0], axis: [0, 1, 0] },
  { name: 'joint_Link5_Link6', xyz: [-0.0113, 0.0091, 0.2516], rpy: [0, 0, 0], axis: [0, 0, 1] },
  { name: 'joint_Link6_Link7', xyz: [0.0363, 0, 0.0499], rpy: [0, 0, 0], axis: [0, 1, 0] },
  { name: 'joint_Link6_Link8', xyz: [-0.0363, 0, 0.047], rpy: [0, 0, 0], axis: [0, 1, 0] },
];

/* 正运动学链：arm_base_link → gripper_tcp（5×revolute + 连续腕 roll + Link3 固定偏置 + TCP），与 orion.urdf 一致；采样后按 feasibility 硬限过滤 */
const FK_ORIGINS = [
  [0, 0, 0],
  [-0.0105, 0, 0.0699],
  [-0.2247, 0, 0.8662],
  [0.1835, 0, 0.0157],
  [0, 0, 0],
  [0, -0.0091, 0.3784],
  [-0.0113, 0.0091, 0.2516],
  /* gripper_tcp：Link6 局部 +Z，与 orion.urdf joint_Link6_gripper_tcp 一致 */
  [0, 0, 0.148],
];
const FK_AXES = [
  [1, 0, 0],
  [0, 1, 0],
  [0, 1, 0],
  null,
  [0, 0, 1],
  [0, 1, 0],
  [0, 0, 1],
  null,
];
const FK_FIXED_RPY = [0, -3.97248, 0];

/* 与 sealien_ctrlpilot_manipulator_orion_description/urdf/orion.urdf 标称一致：±120°、J4 ±180°、J6 continuous（包络采样取一周 [-π,π]） */
const JOINT_SAMPLE_BOUNDS_RAD = [
  { min: (-2.0 * Math.PI) / 3.0, max: (2.0 * Math.PI) / 3.0 },
  { min: (-2.0 * Math.PI) / 3.0, max: (2.0 * Math.PI) / 3.0 },
  { min: (-2.0 * Math.PI) / 3.0, max: (2.0 * Math.PI) / 3.0 },
  { min: -Math.PI, max: Math.PI },
  { min: (-2.0 * Math.PI) / 3.0, max: (2.0 * Math.PI) / 3.0 },
  { min: -Math.PI, max: Math.PI },
];
const SAMPLE_STEPS = 4;

/** 简化的 7+1 段正运动学链：输出末端 4x4 累计变换（与 URDF 链一致，用于工作空间粗采样）。 */
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
    } else if (i === 3) {
      quat.setFromEuler(
        new THREE.Euler(FK_FIXED_RPY[0], FK_FIXED_RPY[1], FK_FIXED_RPY[2], 'XYZ')
      );
      M = new THREE.Matrix4().compose(
        new THREE.Vector3(origin[0], origin[1], origin[2]),
        quat,
        scale
      );
    } else {
      M = new THREE.Matrix4().makeTranslation(origin[0], origin[1], origin[2]);
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
  const bounds = JOINT_SAMPLE_BOUNDS_RAD;
  const stepK = (SAMPLE_STEPS - 1) > 0 ? (k) => (bounds[k].max - bounds[k].min) / (SAMPLE_STEPS - 1) : () => 0;
  const angles = new Array(6);
  const F = FEASIBILITY_WORKSPACE;
  for (let i0 = 0; i0 < SAMPLE_STEPS; i0++) {
    angles[0] = bounds[0].min + i0 * stepK(0);
    for (let i1 = 0; i1 < SAMPLE_STEPS; i1++) {
      angles[1] = bounds[1].min + i1 * stepK(1);
      for (let i2 = 0; i2 < SAMPLE_STEPS; i2++) {
        angles[2] = bounds[2].min + i2 * stepK(2);
        for (let i3 = 0; i3 < SAMPLE_STEPS; i3++) {
          angles[3] = bounds[3].min + i3 * stepK(3);
          for (let i4 = 0; i4 < SAMPLE_STEPS; i4++) {
            angles[4] = bounds[4].min + i4 * stepK(4);
            for (let i5 = 0; i5 < SAMPLE_STEPS; i5++) {
              angles[5] = bounds[5].min + i5 * stepK(5);
              const p = fkChain(angles);
              const r = Math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
              if (r < F.min_reach_safe_m || r > F.max_reach_hard_m) continue;
              if (p.z < F.z_min_m || p.z > F.z_max_m) continue;
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
  if (!Number.isFinite(min.x)) {
    return {
      min: { x: -F.max_reach_hard_m, y: -F.max_reach_hard_m, z: F.z_min_m },
      max: { x: F.max_reach_hard_m, y: F.max_reach_hard_m, z: F.z_max_m },
    };
  }
  return { min, max };
}

let WORKSPACE_URDF_CACHED = null;

/** 基于 FK 粗采样 + feasibility 过滤后的工作空间 AABB（URDF Z-up），带少量 margin。 */
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

/** 右栏「工作空间」文案：URDF 框 + feasibility 径向/软限（与 yaml 同源常量）。 */
export function getWorkspaceBoundsForDoc() {
  const u = getWorkspaceUrdf();
  const F = FEASIBILITY_WORKSPACE;
  return {
    urdf_frame: {
      x_m: { min: u.minX, max: u.maxX },
      y_m: { min: u.minY, max: u.maxY },
      z_m: { min: u.minZ, max: u.maxZ },
    },
    /** 与后端 object_pose 校验一致：arm_base_link 原点到目标中心距离 */
    radial_m: { min: F.min_reach_safe_m, max: F.max_reach_hard_m },
    soft_reach_m: F.max_reach_soft_m,
    from_kinematics: true,
    from_feasibility: true,
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

/** 异步链式加载 arm_base_link…Link8 的 STL，挂接 setJointValues；根节点已做 Z-up→Y-up 倾斜。 */
export function loadRobotModel() {
  const root = new THREE.Group();
  root.name = 'orion';
  const jointTransforms = [];

  return addMesh(root, 'arm_base_link').then(() => {
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
    const axisVec = new THREE.Vector3();
    root.setJointValues = function (names, positions) {
      if (!names || !positions) return;
      const map = {};
      const lim = Math.min(names.length, positions.length);
      for (let i = 0; i < lim; i += 1) {
        map[names[i]] = positions[i];
      }
      jointTransforms.forEach(({ name, group, axis }) => {
        const raw = map[name];
        if (raw === undefined || raw === null) return;
        const angle = Number(raw);
        if (!Number.isFinite(angle)) return;
        const displayAngle = GRIPPER_JOINT_NAMES.has(name) ? gripperAngleForDisplay(angle) : angle;
        axisVec.set(axis[0], axis[1], axis[2]);
        group.quaternion.setFromAxisAngle(axisVec, displayAngle);
      });
    };
    return root;
  });
}
