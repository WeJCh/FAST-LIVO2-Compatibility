#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
  printf '用法：%s RUN_DIR [OUTPUT_DIR]\n' "$0" >&2
  exit 2
fi

project_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
run_dir=$(cd "$1" && pwd)
stage01_dir="$run_dir/scene_pipeline_v4/stage01_pose_correction"
stage02_dir="$run_dir/scene_pipeline_v4/stage02_geometric_evidence_v2_r2"
stage04_handoff="$run_dir/scene_pipeline_v4/stage04_final_handoff"
output_dir=${2:-"$run_dir/scene_pipeline_v4/stage05_visible_scene_objects"}
source_file="$project_root/scene_pipeline/stage05_visible_scene_objects.cpp"
binary=$(mktemp /tmp/fast_livo_scene_stage05.XXXXXX)
trap 'rm -f "$binary"' EXIT

if [[ -e "$output_dir" ]]; then
  printf '阶段5输出目录已存在，拒绝覆盖：%s\n' "$output_dir" >&2
  exit 2
fi

# 先验证阶段4冻结树；失败时绝不混入未冻结或被修改过的阶段4结果。
python3 "$project_root/scene_pipeline/verify_stage04_freeze.py" --handoff "$stage04_handoff"
stage04_r7=$(python3 - "$stage04_handoff/stage04_freeze_manifest.json" <<'PY'
import json
import sys
with open(sys.argv[1], 'r') as handle:
    manifest = json.load(handle)
if manifest.get('status') != 'frozen':
    raise SystemExit('阶段4冻结状态不是 frozen')
print(manifest['accepted_baseline']['stage04_r7'])
PY
)

"${CXX:-g++}" -std=c++14 -O3 -Wall -Wextra -Wpedantic "$source_file" -o "$binary"
"$binary" --run "$run_dir" --stage01 "$stage01_dir" --stage02 "$stage02_dir" \
  --stage04-r7 "$stage04_r7" --output "$output_dir"
