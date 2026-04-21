/**
 * Three.js 场景骨架：world / robot / targets / overlays 分层
 * 机械臂由 STL 加载（RobotModelLoader），joint_states 驱动
 */

import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';
import { loadRobotModel, getWorkspaceBoundsScene } from './RobotModelLoader.js';

const LAYER_NAMES = {
  showAxes: 'show_axes',
  showWorldObject: 'show_world_object',
  showAttachedObject: 'show_attached_object',
  showCollision: 'show_collision',
  showTrajectory: 'show_trajectory',
  showTargets: 'show_targets',
  showWorkspace: 'show_workspace',
};

/* 工业主题：与页面灰阶层级一致 */
const GRID_COLOR = 0x1f2a36;
const AXIS_SIZE = 0.28;
const AXIS_LABEL_SCALE = 0.06;
const PICK_MARKER_COLOR = 0x22d3ee;
const PICK_MARKER_RADIUS = 0.04;
const OUTLINE_COLOR = 0x64748b;
const AXIS_COLOR_X = 0xe53935;
const AXIS_COLOR_Y = 0x43a047;
const AXIS_COLOR_Z = 0x1e88e5;
const TARGET_SENSOR_STL_URL = '/robot/meshes/stl/target_new.stl';
/* TargetSensor 模型本体局部轴向修正：独立 XYZ 角，便于在线调参。 */
const TARGET_SENSOR_AXIS_FLIP_RX = Math.PI;
const TARGET_SENSOR_AXIS_FLIP_RY = Math.PI;
const TARGET_SENSOR_AXIS_FLIP_RZ = Math.PI;
const TARGET_INSERT_HOLE_DIAMETER_M = 0.128;
const TARGET_INSERT_HOLE_INNER_RADIUS = TARGET_INSERT_HOLE_DIAMETER_M * 0.5;
const TARGET_INSERT_HOLE_RING_THICKNESS = 0.006;
const TARGET_INSERT_HOLE_OUTER_RADIUS = TARGET_INSERT_HOLE_INNER_RADIUS + TARGET_INSERT_HOLE_RING_THICKNESS;
const TARGET_INSERT_HOLE_THICKNESS = 0.012;
/* 插孔模型局部轴向补偿（可调参）。 */
const TARGET_INSERT_HOLE_AXIS_FLIP_RX = 0.0;
const TARGET_INSERT_HOLE_AXIS_FLIP_RY = 0.0;
const TARGET_INSERT_HOLE_AXIS_FLIP_RZ = 0.0;

/**
 * 将 target_new.stl 几何长轴对齐到网格局部 +Y（Three 场景 Y-up）。
 * 约定：细圆柱类 peg 在 CAD 里常见沿 X 或 Z；只要 X 向包络不小于 Y，按「主轴沿 X」绕 Z 转 90° 对齐到 +Y。
 */
function getTargetSensorAlignQuaternionByGeometry(geometry) {
  if (!geometry || !geometry.boundingBox) {
    return new THREE.Quaternion();
  }
  const size = new THREE.Vector3();
  geometry.boundingBox.getSize(size);
  const ex = size.x;
  const ey = size.y;
  const ez = size.z;
  const qRz90 = new THREE.Quaternion().setFromEuler(new THREE.Euler(0, 0, Math.PI / 2));
  const qRxN90 = new THREE.Quaternion().setFromEuler(new THREE.Euler(-Math.PI / 2, 0, 0));
  /* X 不小于 Y：主轴视为沿 X → 绕 Z +90° 到 +Y */
  if (ex >= ey) {
    return qRz90;
  }
  /* 否则 Z 为主轴时沿 Z → 绕 X -90° 到 +Y */
  if (ez >= ey && ez >= ex) {
    return qRxN90;
  }
  /* 真正 Y 最长：与 perception 叠加后常见仍像沿场景 X，再绕局部 Z +90° */
  return qRz90;
}

function makeAxisLabel(text, hexColor) {
  const canvas = document.createElement('canvas');
  canvas.width = 64;
  canvas.height = 64;
  const ctx = canvas.getContext('2d');
  ctx.font = 'bold 42px sans-serif';
  ctx.fillStyle = '#' + hexColor.toString(16).padStart(6, '0');
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.fillText(text, 32, 32);
  const tex = new THREE.CanvasTexture(canvas);
  tex.needsUpdate = true;
  const mat = new THREE.SpriteMaterial({ map: tex });
  const sprite = new THREE.Sprite(mat);
  sprite.scale.set(AXIS_LABEL_SCALE, AXIS_LABEL_SCALE, 1);
  return sprite;
}

function makeHoleIndexLabel(text) {
  const canvas = document.createElement('canvas');
  canvas.width = 96;
  canvas.height = 96;
  const ctx = canvas.getContext('2d');
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.font = 'bold 66px sans-serif';
  ctx.lineWidth = 8;
  ctx.strokeStyle = 'rgba(15, 23, 42, 0.96)';
  ctx.fillStyle = 'rgba(255, 245, 157, 0.98)';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.strokeText(text, 48, 50);
  ctx.fillText(text, 48, 50);
  const tex = new THREE.CanvasTexture(canvas);
  tex.needsUpdate = true;
  const mat = new THREE.SpriteMaterial({
    map: tex,
    transparent: true,
    depthTest: false,
    depthWrite: false,
    sizeAttenuation: true,
  });
  const sprite = new THREE.Sprite(mat);
  sprite.renderOrder = 999;
  sprite.scale.set(0.09, 0.09, 1);
  return sprite;
}

function loadStlGeometry(url) {
  return new Promise((resolve, reject) => {
    const loader = new STLLoader();
    loader.load(url, resolve, undefined, reject);
  });
}

/**
 * 在 containerEl 内创建 WebGL 画布、灯光、网格、机械臂占位、目标/轨迹/工作空间辅助体与动画循环。
 * @returns {object} scene API（setRobotJointValues、resize、targets、world 等）
 */
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
  renderer.setClearColor(0x0e1621, 0);
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  containerEl.appendChild(renderer.domElement);

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.05;
  controls.screenSpacePanning = true;
  controls.minDistance = 0.3;
  controls.maxDistance = 5;

  /* 工业 3D 舞台感：Ambient 0.4 + Directional 0.8 + Hemisphere 0.5 */
  const ambient = new THREE.AmbientLight(0xffffff, 0.4);
  scene.add(ambient);
  const directional = new THREE.DirectionalLight(0xffffff, 0.8);
  directional.position.set(2, 3, 2);
  directional.castShadow = true;
  scene.add(directional);
  const hemi = new THREE.HemisphereLight(0x94a3b8, 0x334155, 0.5);
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

  /* 基坐标系 - ROS base_link 语义：与 RobotModelLoader 根节点相同，Z-up(URDF) -> 场景 Y-up */
  const ROS_Z_UP_TO_SCENE_Y_UP = -Math.PI / 2;
  const baseAxesGroup = new THREE.Group();
  baseAxesGroup.name = 'base_axes';
  baseAxesGroup.rotation.x = ROS_Z_UP_TO_SCENE_Y_UP;
  const baseAxes = new THREE.AxesHelper(AXIS_SIZE);
  baseAxes.name = 'base_axes_lines';
  baseAxesGroup.add(baseAxes);
  const labelX = makeAxisLabel('X', AXIS_COLOR_X);
  labelX.position.set(AXIS_SIZE, 0, 0);
  baseAxesGroup.add(labelX);
  const labelY = makeAxisLabel('Y', AXIS_COLOR_Y);
  labelY.position.set(0, AXIS_SIZE, 0);
  baseAxesGroup.add(labelY);
  const labelZ = makeAxisLabel('Z', AXIS_COLOR_Z);
  labelZ.position.set(0, 0, AXIS_SIZE);
  baseAxesGroup.add(labelZ);
  world.add(baseAxesGroup);

  /* 机械臂 STL 模型（异步加载）+ 轮廓线；碰撞体显示状态在模型加载后补刷 */
  let robotModel = null;
  /** 加载完成前收到的 joint_states，避免「表格已更新、模型仍零位」 */
  let lastJointNames = null;
  let lastJointPositions = null;
  let collisionDebugVisible = false;
  function applyCollisionDebugVisible() {
    robot.traverse((obj) => {
      if (obj.name === 'collision_debug') obj.visible = collisionDebugVisible;
    });
  }
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
    applyCollisionDebugVisible();
    if (lastJointNames && lastJointPositions && model.setJointValues) {
      model.setJointValues(lastJointNames, lastJointPositions);
    }
    window.dispatchEvent(new CustomEvent('orion:robot-model-ready'));
  }).catch((err) => {
    console.warn('RobotScene: 机械臂模型加载失败，使用占位', err);
    const linkGeo = new THREE.CylinderGeometry(0.03, 0.04, 0.3, 16);
    const linkMat = new THREE.MeshStandardMaterial({
      color: 0xc7d2fe,
      metalness: 0.4,
      roughness: 0.6,
    });
    const link = new THREE.Mesh(linkGeo, linkMat);
    link.rotation.z = Math.PI / 2;
    link.position.set(0.15, 0, 0);
    robot.add(link);
  });

  /* 工作空间示意框：gripper_tcp 粗网格采样 ∩ feasibility 硬限（与 orion_mtc_params 一致） */
  const wsBounds = getWorkspaceBoundsScene();
  const wsMin = new THREE.Vector3(wsBounds.min.x, wsBounds.min.y, wsBounds.min.z);
  const wsMax = new THREE.Vector3(wsBounds.max.x, wsBounds.max.y, wsBounds.max.z);
  const wsSize = new THREE.Vector3().subVectors(wsMax, wsMin);
  const wsCenter = new THREE.Vector3().addVectors(wsMin, wsMax).multiplyScalar(0.5);
  const wsGeo = new THREE.BoxGeometry(wsSize.x, wsSize.y, wsSize.z);
  const wsEdges = new THREE.EdgesGeometry(wsGeo);
  const workspaceBox = new THREE.LineSegments(
    wsEdges,
    new THREE.LineBasicMaterial({ color: 0x94a3b8, transparent: true, opacity: 0.7 })
  );
  workspaceBox.name = 'workspace_box';
  workspaceBox.position.copy(wsCenter);
  workspaceBox.visible = false;
  world.add(workspaceBox);

  /* 场景物体（缆绳目标，位置由 object_pose 更新，来自 CableSensor）- 小方块作后备 */
  const worldObject = new THREE.Mesh(
    new THREE.BoxGeometry(0.04, 0.04, 0.04),
    new THREE.MeshBasicMaterial({ color: 0x94a3b8, transparent: true, opacity: 0.85 })
  );
  worldObject.name = 'world_object';
  worldObject.visible = false;
  worldObject.userData.valid = false;
  targets.add(worldObject);

  /* 缆绳模型：3m 长、直径 5cm 圆柱体（与后端 planning scene 一致） */
  const CABLE_LENGTH = 3.0;
  const CABLE_RADIUS = 0.025;
  const GRASP_SPHERE_RADIUS = 0.012;

  const targetObjectComposed = new THREE.Group();
  targetObjectComposed.name = 'target_object_composed';
  targetObjectComposed.visible = false;
  targetObjectComposed.userData.valid = false;

  const cableGeo = new THREE.CylinderGeometry(CABLE_RADIUS, CABLE_RADIUS, CABLE_LENGTH, 24);
  /* THREE 的 Cylinder 默认沿 Y 轴，旋转到 Z 轴以匹配 ROS/MoveIt 的圆柱轴定义 */
  cableGeo.rotateX(-Math.PI / 2);
  const cableMesh = new THREE.Mesh(
    cableGeo,
    new THREE.MeshBasicMaterial({ color: 0x64748b, transparent: true, opacity: 0.85 })
  );
  cableMesh.position.set(0, 0, 0);
  cableMesh.name = 'target_cable';
  targetObjectComposed.add(cableMesh);

  const graspPointMesh = new THREE.Mesh(
    new THREE.SphereGeometry(GRASP_SPHERE_RADIUS, 20, 20),
    new THREE.MeshBasicMaterial({ color: 0x22d3ee })
  );
  graspPointMesh.position.set(0, 0, 0);
  graspPointMesh.name = 'grasp_point';
  targetObjectComposed.add(graspPointMesh);

  targets.add(targetObjectComposed);

  /* 附着物体（持物，位置由 held 状态更新）- 仅做提示用，不做精确建模 */
  const attachedObject = new THREE.Mesh(
    new THREE.BoxGeometry(0.035, 0.035, 0.035),
    new THREE.MeshBasicMaterial({ color: 0xf59e0b, transparent: true, opacity: 0.9 })
  );
  attachedObject.name = 'attached_object';
  attachedObject.visible = false;
  attachedObject.userData.valid = false;
  targets.add(attachedObject);

  /* 目标点：抓取缆绳目标 青 #22D3EE */
  const pickMarker = new THREE.Mesh(
    new THREE.SphereGeometry(PICK_MARKER_RADIUS, 20, 20),
    new THREE.MeshBasicMaterial({ color: PICK_MARKER_COLOR })
  );
  pickMarker.name = 'pick_target';
  pickMarker.visible = false;
  targets.add(pickMarker);

  /* 融合中心线抓取点（object_pose_fused）— 品红，略大于 legacy 便于区分 */
  const FUSED_MARKER_COLOR = 0xd946ef;
  const pickMarkerFused = new THREE.Mesh(
    new THREE.SphereGeometry(PICK_MARKER_RADIUS * 1.15, 20, 20),
    new THREE.MeshBasicMaterial({ color: FUSED_MARKER_COLOR })
  );
  pickMarkerFused.name = 'pick_target_fused';
  pickMarkerFused.visible = false;
  targets.add(pickMarkerFused);

  /* TargetSensor 目标物（perception_state.target_sensor_object_pose）— 橙球，与缆绳/青抓取点区分 */
  const TARGET_SENSOR_MARKER_COLOR = 0xf97316;
  const pickMarkerTargetSensor = new THREE.Mesh(
    new THREE.SphereGeometry(PICK_MARKER_RADIUS * 1.08, 20, 20),
    new THREE.MeshBasicMaterial({ color: TARGET_SENSOR_MARKER_COLOR })
  );
  pickMarkerTargetSensor.name = 'pick_target_sensor';
  pickMarkerTargetSensor.visible = false;
  targets.add(pickMarkerTargetSensor);

  /*
   * TargetSensor 目标物模型：直接加载 orion_description/target_new.stl（由 sync/start 脚本同步到 web）。
   * 位姿直接使用 perception_state.target_sensor_object_pose，避免前端临时建模导致的轴向偏差。
   */
  const targetSensorObjectComposed = new THREE.Group();
  targetSensorObjectComposed.name = 'target_sensor_object_composed';
  targetSensorObjectComposed.visible = false;
  targetSensorObjectComposed.userData.valid = false;
  const tsStlMaterial = new THREE.MeshStandardMaterial({
    color: 0xb87333,
    metalness: 0.6,
    roughness: 0.35,
  });
  loadStlGeometry(TARGET_SENSOR_STL_URL).then((geometry) => {
    geometry.computeVertexNormals();
    geometry.computeBoundingBox();
    if (geometry.boundingBox) {
      const center = new THREE.Vector3();
      geometry.boundingBox.getCenter(center);
      geometry.translate(-center.x, -center.y, -center.z);
    }
    const tsMesh = new THREE.Mesh(geometry, tsStlMaterial);
    tsMesh.quaternion.setFromEuler(
      new THREE.Euler(
        TARGET_SENSOR_AXIS_FLIP_RX,
        TARGET_SENSOR_AXIS_FLIP_RY,
        TARGET_SENSOR_AXIS_FLIP_RZ
      )
    );
    tsMesh.castShadow = true;
    tsMesh.receiveShadow = true;
    tsMesh.name = 'target_sensor_stl';
    targetSensorObjectComposed.add(tsMesh);
  }).catch((err) => {
    console.warn('RobotScene: target_new.stl 加载失败，回退简化模型', err);
    const fallbackMesh = new THREE.Mesh(
      new THREE.CylinderGeometry(0.015, 0.015, 0.32, 24),
      new THREE.MeshStandardMaterial({
        color: 0xb87333,
        metalness: 0.5,
        roughness: 0.4,
      })
    );
    fallbackMesh.quaternion.setFromEuler(
      new THREE.Euler(
        TARGET_SENSOR_AXIS_FLIP_RX,
        TARGET_SENSOR_AXIS_FLIP_RY,
        TARGET_SENSOR_AXIS_FLIP_RZ
      )
    );
    targetSensorObjectComposed.add(fallbackMesh);
  });
  targets.add(targetSensorObjectComposed);

  /* 插孔调试可视化：环形孔位（base_link->scene 变换后由 Viewport3D 更新位姿）。 */
  const targetInsertHolesGroup = new THREE.Group();
  targetInsertHolesGroup.name = 'target_insert_holes_group';
  targetInsertHolesGroup.visible = false;
  for (let i = 0; i < 7; i += 1) {
    const holePoseNode = new THREE.Group();
    holePoseNode.name = `target_insert_hole_${i}`;
    holePoseNode.visible = false;
    const ring = new THREE.Mesh(
      new THREE.CylinderGeometry(
        TARGET_INSERT_HOLE_OUTER_RADIUS,
        TARGET_INSERT_HOLE_OUTER_RADIUS,
        TARGET_INSERT_HOLE_THICKNESS,
        36,
        1,
        false,
        0,
        Math.PI * 2
      ),
      new THREE.MeshStandardMaterial({
        color: 0xffd400,
        emissive: 0x3b2f00,
        emissiveIntensity: 0.55,
        metalness: 0.25,
        roughness: 0.32,
        transparent: true,
        opacity: 0.92,
      })
    );
    ring.quaternion.setFromEuler(
      new THREE.Euler(
        TARGET_INSERT_HOLE_AXIS_FLIP_RX,
        TARGET_INSERT_HOLE_AXIS_FLIP_RY,
        TARGET_INSERT_HOLE_AXIS_FLIP_RZ
      )
    );
    const inner = new THREE.Mesh(
      new THREE.CylinderGeometry(
        TARGET_INSERT_HOLE_INNER_RADIUS,
        TARGET_INSERT_HOLE_INNER_RADIUS,
        TARGET_INSERT_HOLE_THICKNESS * 1.2,
        28
      ),
      new THREE.MeshBasicMaterial({
        color: 0x0e1621,
      })
    );
    inner.name = 'target_insert_hole_inner';
    ring.add(inner);
    const holeIndexLabel = makeHoleIndexLabel(String(i + 1));
    holeIndexLabel.position.set(0, 0.002, 0);
    ring.add(holeIndexLabel);
    holePoseNode.add(ring);
    targetInsertHolesGroup.add(holePoseNode);
  }
  targets.add(targetInsertHolesGroup);

  /* Keypoints 轨迹：琥珀球（与品红融合点、fused 区分）+ 折线 */
  const KP_TRACE_COLOR = 0xf59e0b;
  const KP_TRACE_RADIUS = 0.028;
  const KP_TRACE_MAX = 32;
  const keypointsTraceGroup = new THREE.Group();
  keypointsTraceGroup.name = 'keypoints_trace';
  const kpTraceSpheres = [];
  for (let i = 0; i < KP_TRACE_MAX; i += 1) {
    const m = new THREE.Mesh(
      new THREE.SphereGeometry(KP_TRACE_RADIUS, 14, 14),
      new THREE.MeshBasicMaterial({ color: KP_TRACE_COLOR })
    );
    m.name = 'kp_trace_' + i;
    m.visible = false;
    keypointsTraceGroup.add(m);
    kpTraceSpheres.push(m);
  }
  targets.add(keypointsTraceGroup);
  const keypointsPolyline = new THREE.Line(
    new THREE.BufferGeometry(),
    new THREE.LineBasicMaterial({
      color: KP_TRACE_COLOR,
      transparent: true,
      opacity: 0.75,
    })
  );
  keypointsPolyline.name = 'keypoints_polyline';
  keypointsPolyline.visible = false;
  targets.add(keypointsPolyline);

  /* ROV 坐标系（base_link 下，由 rov_pose_in_base_link 更新） */
  const ROV_AXIS_SIZE = 0.12;
  const rovAxesGroup = new THREE.Group();
  rovAxesGroup.name = 'rov_axes';
  const rovAxes = new THREE.AxesHelper(ROV_AXIS_SIZE);
  rovAxesGroup.add(rovAxes);
  rovAxesGroup.visible = false;
  world.add(rovAxesGroup);

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
      const tip = robot.getObjectByName('joint_Link6_Link7') || robot.getObjectByName('joint_Link5_Link6');
      if (tip) {
        tip.getWorldPosition(tcpTarget);
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
    const n = Array.isArray(names) ? names : [];
    const p = Array.isArray(positions) ? positions : [];
    if (n.length > 0 && p.length > 0) {
      lastJointNames = n;
      lastJointPositions = p;
    }
    if (robotModel && robotModel.setJointValues) {
      robotModel.setJointValues(n, p);
    }
  }

  function setCollisionDebugVisible(visible) {
    collisionDebugVisible = !!visible;
    applyCollisionDebugVisible();
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
    pickMarkerFused,
    pickMarkerTargetSensor,
    targetSensorObjectComposed,
    targetInsertHolesGroup,
    keypointsTraceGroup,
    kpTraceSpheres,
    keypointsPolyline,
    targetObjectComposed,
    rovAxesGroup,
    trajectoryLine,
    setRobotJointValues,
    setFollowTcp,
    setCollisionDebugVisible,
    resize,
    dispose,
    LAYER_NAMES,
  };
}

export default { createScene, LAYER_NAMES };
