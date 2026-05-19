/**
 * 与 src/orion_mtc/config/orion_mtc_params.yaml → feasibility 块一致（物体/缆绳中心在 arm_base_link）
 * URDF：orion_description/urdf/orion.urdf；变更 yaml 时请同步此处。
 */
export const FEASIBILITY_WORKSPACE = {
  max_reach_hard_m: 1.8,
  max_reach_soft_m: 1.66,
  min_reach_safe_m: 0.14,
  z_min_m: -0.55,
  z_max_m: 1.55,
};
