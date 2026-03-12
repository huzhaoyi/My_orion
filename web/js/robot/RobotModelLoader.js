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

function loadSTL(url) {
  return new Promise((resolve, reject) => {
    const loader = new STLLoader();
    loader.load(url, resolve, undefined, reject);
  });
}

/* 深灰金属，与背景区分、轮廓清晰 */
const ROBOT_MATERIAL_COLOR = 0x3a3f4b;
const ROBOT_METALNESS = 0.55;
const ROBOT_ROUGHNESS = 0.45;

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
