/**
 * Three.js 场景骨架：world / robot / targets / overlays 分层
 * 机械臂由 STL 加载（RobotModelLoader），joint_states 驱动
 */

import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { loadRobotModel } from './RobotModelLoader.js';

const LAYER_NAMES = {
  showAxes: 'show_axes',
  showWorldObject: 'show_world_object',
  showAttachedObject: 'show_attached_object',
  showCollision: 'show_collision',
  showTrajectory: 'show_trajectory',
  showTargets: 'show_targets',
  showWorkspace: 'show_workspace',
};

/* 工业主题色；背景用 CSS 渐变，此处透明 */
const GRID_COLOR = 0x334155;
const AXIS_SIZE = 0.07;
const PICK_MARKER_COLOR = 0x22d3ee;
const PICK_MARKER_RADIUS = 0.04;
const PLACE_MARKER_RADIUS = 0.04;
const OUTLINE_COLOR = 0x64748b;

function createScene(containerEl) {
  const width = containerEl.clientWidth;
  const height = containerEl.clientHeight;
  const scene = new THREE.Scene();
  scene.background = null;

  const camera = new THREE.PerspectiveCamera(50, width / height, 0.01, 100);
  camera.position.set(1.5, 1.2, 1.5);
  camera.lookAt(0, 0, 0);

  const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
  renderer.setSize(width, height);
  renderer.setPixelRatio(window.devicePixelRatio);
  renderer.setClearColor(0x000000, 0);
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  containerEl.appendChild(renderer.domElement);

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.05;
  controls.screenSpacePanning = true;
  controls.minDistance = 0.3;
  controls.maxDistance = 5;

  /* 光照：环境光 0.5 + 方向光 0.8 + 半球光，立体感更强 */
  const ambient = new THREE.AmbientLight(0xffffff, 0.5);
  scene.add(ambient);
  const directional = new THREE.DirectionalLight(0xffffff, 0.8);
  directional.position.set(2, 3, 2);
  directional.castShadow = true;
  scene.add(directional);
  const hemi = new THREE.HemisphereLight(0x94a3b8, 0x334155, 0.35);
  scene.add(hemi);

  /* 对象树：world / robot / targets / overlays */
  const world = new THREE.Group();
  world.name = 'world';
  const robot = new THREE.Group();
  robot.name = 'robot';
  const targets = new THREE.Group();
  targets.name = 'targets';
  const overlays = new THREE.Group();
  overlays.name = 'overlays';

  scene.add(world);
  scene.add(robot);
  scene.add(targets);
  scene.add(overlays);

  /* 地面网格 - 与机械臂底座同高，更淡不抢眼 */
  const GRID_Y = 0;
  const grid = new THREE.GridHelper(2, 20, GRID_COLOR, GRID_COLOR);
  grid.material.opacity = 0.15;
  grid.material.transparent = true;
  grid.position.y = GRID_Y;
  world.add(grid);
  robot.position.y = GRID_Y;

  /* 基坐标系 - ROS 配色 X 红 Y 绿 Z 蓝，尺寸 0.06 */
  const baseAxes = new THREE.AxesHelper(AXIS_SIZE);
  baseAxes.name = 'base_axes';
  world.add(baseAxes);

  /* 机械臂 STL 模型（异步加载）+ 轮廓线 */
  let robotModel = null;
  function addOutlineToModel(model) {
    model.traverse((child) => {
      if (child.isMesh && child.geometry) {
        const edges = new THREE.EdgesGeometry(child.geometry, 12);
        const line = new THREE.LineSegments(
          edges,
          new THREE.LineBasicMaterial({ color: OUTLINE_COLOR })
        );
        child.add(line);
      }
    });
  }
  loadRobotModel().then((model) => {
    robotModel = model;
    robot.add(model);
    addOutlineToModel(model);
  }).catch((err) => {
    console.warn('RobotScene: 机械臂模型加载失败，使用占位', err);
    const linkGeo = new THREE.CylinderGeometry(0.03, 0.04, 0.3, 16);
    const linkMat = new THREE.MeshStandardMaterial({ color: 0x4a90d9 });
    const link = new THREE.Mesh(linkGeo, linkMat);
    link.rotation.z = Math.PI / 2;
    link.position.set(0.15, 0, 0);
    robot.add(link);
  });

  /* TCP 坐标系 - 明显尺寸，ROS 红 X 绿 Y 蓝 Z */
  const tcpAxes = new THREE.AxesHelper(AXIS_SIZE);
  tcpAxes.name = 'tcp_axes';
  tcpAxes.position.set(0.3, 0, 0);
  robot.add(tcpAxes);

  /* 目标点：pick 青 #22D3EE 稍大，place 紫 */
  const pickMarker = new THREE.Mesh(
    new THREE.SphereGeometry(PICK_MARKER_RADIUS, 20, 20),
    new THREE.MeshBasicMaterial({ color: PICK_MARKER_COLOR })
  );
  pickMarker.name = 'pick_target';
  pickMarker.visible = false;
  targets.add(pickMarker);

  const placeMarker = new THREE.Mesh(
    new THREE.SphereGeometry(PLACE_MARKER_RADIUS, 20, 20),
    new THREE.MeshBasicMaterial({ color: 0xa78bfa })
  );
  placeMarker.name = 'place_target';
  placeMarker.visible = false;
  targets.add(placeMarker);

  /* 轨迹线（空） */
  const trajectoryLine = new THREE.Line(
    new THREE.BufferGeometry(),
    new THREE.LineBasicMaterial({ color: 0x5a9fd4, linewidth: 2 })
  );
  trajectoryLine.name = 'trajectory';
  trajectoryLine.visible = false;
  overlays.add(trajectoryLine);

  let followTcp = false;
  const tcpTarget = new THREE.Vector3();
  let animationId = null;
  function animate() {
    animationId = requestAnimationFrame(animate);
    if (followTcp) {
      const tcp = robot.getObjectByName('tcp_axes');
      if (tcp) {
        tcp.getWorldPosition(tcpTarget);
        controls.target.copy(tcpTarget);
      }
    }
    controls.update();
    renderer.render(scene, camera);
  }
  animate();

  function setFollowTcp(enabled) {
    followTcp = !!enabled;
  }

  function resize() {
    const w = containerEl.clientWidth;
    const h = containerEl.clientHeight;
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
    renderer.setSize(w, h);
  }

  window.addEventListener('resize', resize);

  function dispose() {
    window.removeEventListener('resize', resize);
    if (animationId) cancelAnimationFrame(animationId);
    renderer.dispose();
    controls.dispose();
  }

  function setRobotJointValues(names, positions) {
    if (robotModel && robotModel.setJointValues) robotModel.setJointValues(names, positions);
  }

  return {
    scene,
    camera,
    renderer,
    controls,
    world,
    robot,
    targets,
    overlays,
    pickMarker,
    placeMarker,
    trajectoryLine,
    setRobotJointValues,
    setFollowTcp,
    resize,
    dispose,
    LAYER_NAMES,
  };
}

export default { createScene, LAYER_NAMES };
