#!/usr/bin/env bash
# 将 ROV+左臂模型（orion_description + 可选 rov_urdf/meshes）同步到 web/robot/
set -e
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC_DESC="$REPO_ROOT/src/orion_description"
SRC_ROV="$REPO_ROOT/rov_urdf"
DST="$REPO_ROOT/web/robot"
mkdir -p "$DST/meshes/stl"
cp -u "$SRC_DESC/meshes/stl/"*.stl "$DST/meshes/stl/" 2>/dev/null || true
if [ -d "$SRC_ROV/meshes/stl" ]; then
  cp -u "$SRC_ROV/meshes/stl/"*.stl "$DST/meshes/stl/" 2>/dev/null || true
fi
sed 's|../meshes/stl/|meshes/stl/|g' "$SRC_DESC/urdf/orion.urdf" > "$DST/orion.urdf"
echo "已同步 orion_description (+ rov_urdf/meshes/stl) -> web/robot/"
