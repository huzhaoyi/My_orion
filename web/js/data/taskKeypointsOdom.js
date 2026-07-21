/**
 * odom 任务关键点：与 config/manipulator_task_keypoints_odom.yaml 数值一致。
 * 抓取由 TargetSensor 世界系 + 侧向抓取系推导；插孔读 catalog 静态 slot。
 */
const catalog = {
  keypoint_frame: 'odom',
  grasp: { offset_along_direction_m: 0.122671 },
  insert_slots: [
    {
      id: 'slot_1',
      position_xyz: [-162.649, 85.66733, -131.1388],
      orientation_xyzw: [0.0, 0.0, -0.70710678, 0.70710678],
    },
    {
      id: 'slot_2',
      position_xyz: [-162.649, 85.43484, -131.1388],
      orientation_xyzw: [0.0, 0.0, -0.70710678, 0.70710678],
    },
    {
      id: 'slot_3',
      position_xyz: [-162.649, 85.20445, -131.1388],
      orientation_xyzw: [0.0, 0.0, -0.70710678, 0.70710678],
    },
  ],
};

export const KEYPOINT_FRAME = catalog.keypoint_frame || 'odom';
export const GRASP_OFFSET_ALONG_DIRECTION_M = Number(
  catalog.grasp?.offset_along_direction_m ?? 0.122671
);
const LEFT_ARM_BASE_IN_ROV = [1.55, 0.5653, -0.283628];

function quatToRotationMatrix(q) {
  const x = Number(q.x);
  const y = Number(q.y);
  const z = Number(q.z);
  const w = Number(q.w);
  return [
    [
      1.0 - 2.0 * (y * y + z * z),
      2.0 * (x * y - z * w),
      2.0 * (x * z + y * w),
    ],
    [
      2.0 * (x * y + z * w),
      1.0 - 2.0 * (x * x + z * z),
      2.0 * (y * z - x * w),
    ],
    [
      2.0 * (x * z - y * w),
      2.0 * (y * z + x * w),
      1.0 - 2.0 * (x * x + y * y),
    ],
  ];
}

function matMul3(a, b) {
  const out = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
  for (let r = 0; r < 3; r += 1) {
    for (let c = 0; c < 3; c += 1) {
      out[r][c] = a[r][0] * b[0][c] + a[r][1] * b[1][c] + a[r][2] * b[2][c];
    }
  }
  return out;
}

function matVec3(m, v) {
  return [
    m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
    m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
    m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
  ];
}

function matTranspose3(m) {
  return [
    [m[0][0], m[1][0], m[2][0]],
    [m[0][1], m[1][1], m[2][1]],
    [m[0][2], m[1][2], m[2][2]],
  ];
}

const sideGraspYHintByIndex = {};

function quatFromRotationMatrix(m) {
  const trace = m[0][0] + m[1][1] + m[2][2];
  let qw;
  let qx;
  let qy;
  let qz;
  if (trace > 0.0) {
    const s = 0.5 / Math.sqrt(trace + 1.0);
    qw = 0.25 / s;
    qx = (m[2][1] - m[1][2]) * s;
    qy = (m[0][2] - m[2][0]) * s;
    qz = (m[1][0] - m[0][1]) * s;
  } else if (m[0][0] > m[1][1] && m[0][0] > m[2][2]) {
    const s = 2.0 * Math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]);
    qw = (m[2][1] - m[1][2]) / s;
    qx = 0.25 * s;
    qy = (m[0][1] + m[1][0]) / s;
    qz = (m[0][2] + m[2][0]) / s;
  } else if (m[1][1] > m[2][2]) {
    const s = 2.0 * Math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]);
    qw = (m[0][2] - m[2][0]) / s;
    qx = (m[0][1] + m[1][0]) / s;
    qy = 0.25 * s;
    qz = (m[1][2] + m[2][1]) / s;
  } else {
    const s = 2.0 * Math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]);
    qw = (m[1][0] - m[0][1]) / s;
    qx = (m[0][2] + m[2][0]) / s;
    qy = (m[1][2] + m[2][1]) / s;
    qz = 0.25 * s;
  }
  return { x: qx, y: qy, z: qz, w: qw };
}

function rotationMatrixSideGraspFromDirection(direction, yAxisHint) {
  let ax = Number(direction[0]);
  let ay = Number(direction[1]);
  let az = Number(direction[2]);
  let n = Math.sqrt(ax * ax + ay * ay + az * az);
  if (!Number.isFinite(n) || n < 1.0e-9) {
    ax = 0.0;
    ay = 0.0;
    az = 1.0;
    n = 1.0;
  } else {
    ax /= n;
    ay /= n;
    az /= n;
  }
  if (az < 0.0) {
    ax = -ax;
    ay = -ay;
    az = -az;
  }
  let yx = 0.0;
  let yy = 0.0;
  let yz = 0.0;
  if (yAxisHint) {
    let hx = Number(yAxisHint[0]);
    let hy = Number(yAxisHint[1]);
    let hz = Number(yAxisHint[2]);
    const hn = Math.sqrt(hx * hx + hy * hy + hz * hz);
    if (Number.isFinite(hn) && hn > 1.0e-9) {
      hx /= hn;
      hy /= hn;
      hz /= hn;
      const dot = hx * ax + hy * ay + hz * az;
      yx = hx - dot * ax;
      yy = hy - dot * ay;
      yz = hz - dot * az;
      const yn = Math.sqrt(yx * yx + yy * yy + yz * yz);
      if (Number.isFinite(yn) && yn > 1.0e-9) {
        yx /= yn;
        yy /= yn;
        yz /= yn;
        const dotY = yx * hx + yy * hy + yz * hz;
        if (dotY < 0.0) {
          yx = -yx;
          yy = -yy;
          yz = -yz;
        }
      } else {
        yx = 0.0;
        yy = 0.0;
        yz = 0.0;
      }
    }
  }
  if (Math.abs(yx) + Math.abs(yy) + Math.abs(yz) < 1.0e-9) {
    let rx = 0.0;
    let ry = 0.0;
    let rz = 1.0;
    if (Math.abs(ax * rx + ay * ry + az * rz) > 0.95) {
      rx = 1.0;
      ry = 0.0;
      rz = 0.0;
    }
    yx = ry * az - rz * ay;
    yy = rz * ax - rx * az;
    yz = rx * ay - ry * ax;
    const yn = Math.sqrt(yx * yx + yy * yy + yz * yz);
    if (Number.isFinite(yn) && yn > 1.0e-9) {
      yx /= yn;
      yy /= yn;
      yz /= yn;
    } else {
      yx = 1.0;
      yy = 0.0;
      yz = 0.0;
    }
  }
  let zx = yy * az - yz * ay;
  let zy = yz * ax - yx * az;
  let zz = yx * ay - yy * ax;
  let zn = Math.sqrt(zx * zx + zy * zy + zz * zz);
  if (!Number.isFinite(zn) || zn < 1.0e-9) {
    return null;
  }
  zx /= zn;
  zy /= zn;
  zz /= zn;
  const xx = yy * zz - yz * zy;
  const xy = yz * zx - yx * zz;
  const xz = yx * zy - yy * zx;
  return [
    [xx, yx, zx],
    [xy, yy, zy],
    [xz, yz, zz],
  ];
}

/**
 * @param {{ num_targets?: number, positions?: number[], directions?: number[] }} targetSensor
 * @param {number} graspIndex 0-based
 * @param {{ position?: object, orientation?: object }} rovPoseWorld /holoocean/rov0/PoseSensor 或 perception_state.rov_pose_in_world
 */
export function computeGraspKeypointOdom(targetSensor, graspIndex, rovPoseWorld) {
  const numTargets = Number(targetSensor?.num_targets ?? 0);
  if (!Number.isFinite(numTargets) || numTargets <= 0) {
    return null;
  }
  if (graspIndex < 0 || graspIndex >= numTargets) {
    return null;
  }
  if (!rovPoseWorld || !rovPoseWorld.position || !rovPoseWorld.orientation) {
    return null;
  }
  const positions = targetSensor.positions || [];
  const directions = targetSensor.directions || [];
  const i = graspIndex * 3;
  if (positions.length < i + 3 || directions.length < i + 3) {
    return null;
  }
  const pxw = Number(positions[i]);
  const pyw = Number(positions[i + 1]);
  const pzw = Number(positions[i + 2]);
  let dxw = Number(directions[i]);
  let dyw = Number(directions[i + 1]);
  let dzw = Number(directions[i + 2]);
  let dn = Math.sqrt(dxw * dxw + dyw * dyw + dzw * dzw);
  if (!Number.isFinite(dn) || dn < 1.0e-9) {
    dxw = 0.0;
    dyw = 0.0;
    dzw = 1.0;
  } else {
    dxw /= dn;
    dyw /= dn;
    dzw /= dn;
  }
  const R_rov = quatToRotationMatrix(rovPoseWorld.orientation);
  const R_rov_t = matTranspose3(R_rov);
  const t_rov = [
    Number(rovPoseWorld.position.x),
    Number(rovPoseWorld.position.y),
    Number(rovPoseWorld.position.z),
  ];
  const p_world = [pxw, pyw, pzw];
  const d_world = [dxw, dyw, dzw];
  const p_rel = [p_world[0] - t_rov[0], p_world[1] - t_rov[1], p_world[2] - t_rov[2]];
  const p_rov = matVec3(R_rov_t, p_rel);
  const p_base = [
    p_rov[0] - LEFT_ARM_BASE_IN_ROV[0],
    p_rov[1] - LEFT_ARM_BASE_IN_ROV[1],
    p_rov[2] - LEFT_ARM_BASE_IN_ROV[2],
  ];
  const d_base = matVec3(R_rov_t, d_world);
  const d_base_n = Math.sqrt(d_base[0] * d_base[0] + d_base[1] * d_base[1] + d_base[2] * d_base[2]);
  const d_base_u = d_base_n > 1.0e-9
    ? [d_base[0] / d_base_n, d_base[1] / d_base_n, d_base[2] / d_base_n]
    : [0.0, 0.0, 1.0];
  const p_grasp_base = [
    p_base[0] + d_base_u[0] * GRASP_OFFSET_ALONG_DIRECTION_M,
    p_base[1] + d_base_u[1] * GRASP_OFFSET_ALONG_DIRECTION_M,
    p_base[2] + d_base_u[2] * GRASP_OFFSET_ALONG_DIRECTION_M,
  ];
  const yHint = sideGraspYHintByIndex[graspIndex];
  const rotBase = rotationMatrixSideGraspFromDirection(d_base_u, yHint);
  if (!rotBase) {
    return null;
  }
  sideGraspYHintByIndex[graspIndex] = [rotBase[0][1], rotBase[1][1], rotBase[2][1]];
  const p_rov_grasp = [
    p_grasp_base[0] + LEFT_ARM_BASE_IN_ROV[0],
    p_grasp_base[1] + LEFT_ARM_BASE_IN_ROV[1],
    p_grasp_base[2] + LEFT_ARM_BASE_IN_ROV[2],
  ];
  const p_odom = matVec3(R_rov, p_rov_grasp);
  p_odom[0] += t_rov[0];
  p_odom[1] += t_rov[1];
  p_odom[2] += t_rov[2];
  const rotOdom = matMul3(R_rov, rotBase);
  const orientation = quatFromRotationMatrix(rotOdom);
  return {
    frame_id: KEYPOINT_FRAME,
    position: { x: p_odom[0], y: p_odom[1], z: p_odom[2] },
    orientation,
    object_id: `target_${graspIndex}`,
  };
}

/** @param {number} slotHumanIndex 1-based，与 UI slot 按钮一致 */
export function getInsertKeypointOdom(slotHumanIndex) {
  const slotIndex = Math.max(0, Math.floor(Number(slotHumanIndex) || 1) - 1);
  const slots = Array.isArray(catalog.insert_slots) ? catalog.insert_slots : [];
  const slot = slots[slotIndex];
  if (!slot || !Array.isArray(slot.position_xyz) || !Array.isArray(slot.orientation_xyzw)) {
    return null;
  }
  return {
    frame_id: KEYPOINT_FRAME,
    slot_id: String(slot.id || `slot_${slotIndex + 1}`),
    position: {
      x: Number(slot.position_xyz[0]),
      y: Number(slot.position_xyz[1]),
      z: Number(slot.position_xyz[2]),
    },
    orientation: {
      x: Number(slot.orientation_xyzw[0]),
      y: Number(slot.orientation_xyzw[1]),
      z: Number(slot.orientation_xyzw[2]),
      w: Number(slot.orientation_xyzw[3]),
    },
  };
}
