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
 */
export function computeGraspKeypointOdom(targetSensor, graspIndex) {
  const numTargets = Number(targetSensor?.num_targets ?? 0);
  if (!Number.isFinite(numTargets) || numTargets <= 0) {
    return null;
  }
  if (graspIndex < 0 || graspIndex >= numTargets) {
    return null;
  }
  const positions = targetSensor.positions || [];
  const directions = targetSensor.directions || [];
  const i = graspIndex * 3;
  if (positions.length < i + 3 || directions.length < i + 3) {
    return null;
  }
  let dx = Number(directions[i]);
  let dy = Number(directions[i + 1]);
  let dz = Number(directions[i + 2]);
  let dn = Math.sqrt(dx * dx + dy * dy + dz * dz);
  if (!Number.isFinite(dn) || dn < 1.0e-9) {
    dx = 0.0;
    dy = 0.0;
    dz = 1.0;
  } else {
    dx /= dn;
    dy /= dn;
    dz /= dn;
  }
  const px = Number(positions[i]) + dx * GRASP_OFFSET_ALONG_DIRECTION_M;
  const py = Number(positions[i + 1]) + dy * GRASP_OFFSET_ALONG_DIRECTION_M;
  const pz = Number(positions[i + 2]) + dz * GRASP_OFFSET_ALONG_DIRECTION_M;
  const yHint = sideGraspYHintByIndex[graspIndex];
  const rot = rotationMatrixSideGraspFromDirection([dx, dy, dz], yHint);
  if (!rot) {
    return null;
  }
  sideGraspYHintByIndex[graspIndex] = [rot[0][1], rot[1][1], rot[2][1]];
  const orientation = quatFromRotationMatrix(rot);
  return {
    frame_id: KEYPOINT_FRAME,
    position: { x: px, y: py, z: pz },
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
