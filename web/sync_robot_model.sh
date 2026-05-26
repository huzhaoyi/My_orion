#!/usr/bin/env bash
# 将 src/sealien_ctrlpilot_manipulator_orion_description 的 URDF 与 STL 同步到 web/robot/，便于网页加载机械臂模型
set -e
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="$REPO_ROOT/src/sealien_ctrlpilot_manipulator_orion_description"
DST="$REPO_ROOT/web/robot"
mkdir -p "$DST/meshes/stl"
cp -u "$SRC/meshes/stl/"*.stl "$DST/meshes/stl/" 2>/dev/null || true
cp -u "$SRC/target_new.stl" "$DST/meshes/stl/target_new.stl" 2>/dev/null || true
sed 's|../meshes/stl/|meshes/stl/|g' "$SRC/urdf/orion.urdf" > "$DST/orion.urdf"
echo "已同步 sealien_ctrlpilot_manipulator_orion_description -> web/robot/"
