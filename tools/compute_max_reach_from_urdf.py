#!/usr/bin/env python3
"""
与 orion_mtc reach_kinematics.cpp 一致：沿 base_link←…←gripper_tcp 累加各子连杆
joint 的 origin xyz 范数（MoveIt LinkModel::getJointOriginTransform().translation().norm() 的 URDF 来源）。

URDF 或规划模型变更后重跑本脚本，将输出的 feasibility 数值写回 pick_params.yaml。
"""

from __future__ import annotations

import argparse
import math
import sys
import xml.etree.ElementTree as ET


def _norm_xyz(xyz: list[float]) -> float:
    return math.sqrt(sum(x * x for x in xyz))


def _joint_xyz(joint_el: ET.Element) -> list[float]:
    origin = joint_el.find("origin")
    if origin is None:
        return [0.0, 0.0, 0.0]
    raw = origin.attrib.get("xyz", "0 0 0").split()
    return [float(raw[0]), float(raw[1]), float(raw[2])]


def compute_kinematic_ub_m(urdf_path: str, tip_link: str, base_link: str) -> tuple[float, list[tuple[str, list[float], float]]]:
    root = ET.parse(urdf_path).getroot()
    child_to: dict[str, tuple[str, list[float], str]] = {}
    for j in root.findall("joint"):
        parent = j.find("parent")
        child = j.find("child")
        if parent is None or child is None:
            continue
        pn = parent.attrib["link"]
        cn = child.attrib["link"]
        xyz = _joint_xyz(j)
        child_to[cn] = (pn, xyz, j.attrib.get("name", ""))

    cur = tip_link
    total = 0.0
    steps: list[tuple[str, list[float], float]] = []
    while cur != base_link:
        if cur not in child_to:
            raise ValueError(f"无法从 {tip_link} 沿父链走到 {base_link}：在 link '{cur}' 处断开")
        parent, xyz, jname = child_to[cur]
        n = _norm_xyz(xyz)
        total += n
        steps.append((jname, xyz, n))
        cur = parent

    return total, steps


def main() -> int:
    ap = argparse.ArgumentParser(description="从 URDF 计算 max_reach 建议值（与 orion_mtc 预检一致）")
    ap.add_argument(
        "--urdf",
        default="src/orion_description/urdf/orion.urdf",
        help="URDF 路径（相对仓库根或绝对路径）",
    )
    ap.add_argument("--tip", default="gripper_tcp", help="末端 link 名")
    ap.add_argument("--base", default="base_link", help="臂基 link 名")
    ap.add_argument("--margin", type=float, default=1.08, help="与 feasibility.max_reach_hard_kinematic_margin 一致")
    ap.add_argument("--soft-ratio", type=float, default=0.92, help="soft = hard × 该值")
    ap.add_argument("--verbose", action="store_true", help="打印各关节贡献")
    args = ap.parse_args()

    try:
        ub, steps = compute_kinematic_ub_m(args.urdf, args.tip, args.base)
    except (OSError, ValueError) as e:
        print(str(e), file=sys.stderr)
        return 1

    if args.verbose:
        for jname, xyz, n in steps:
            print(f"{jname}: xyz={xyz}  ||xyz||={n:.6f} m")
        print(f"kinematic_ub = {ub:.6f} m")

    margin = max(args.margin, 1.0)
    hard = ub * margin
    soft = hard * args.soft_ratio

    print(f"# 由 tools/compute_max_reach_from_urdf.py 生成（URDF: {args.urdf}）")
    print(f"  max_reach_hard: {hard:.3f}")
    print(f"  max_reach_soft: {soft:.3f}")
    print(f"  max_reach_hard_kinematic_margin: {margin}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
