#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
  printf '用法：%s RUN_DIR [OUTPUT_DIR]\n' "$0" >&2
  exit 2
fi

project_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
run_dir=$(cd "$1" && pwd)
stage02="$run_dir/scene_pipeline_v4/stage02_geometric_evidence_v2_r2"
stage03="$run_dir/scene_pipeline_v4/stage03_final_handoff"
stage04_handoff="$run_dir/scene_pipeline_v4/stage04_final_handoff"
stage05_r1="$run_dir/scene_pipeline_v4/stage05_visible_scene_objects"
stage05_r2="$run_dir/scene_pipeline_v4/stage05_navigation_structure_refinement_r2"
output_dir=${2:-"$run_dir/scene_pipeline_v4/stage05_navigation_structure_refinement_r3"}

if [[ -e "$output_dir" ]]; then
  printf '阶段5 R3输出目录已存在，拒绝覆盖：%s\n' "$output_dir" >&2
  exit 2
fi

python3 "$project_root/scene_pipeline/verify_stage04_freeze.py" --handoff "$stage04_handoff"
python3 "$project_root/scene_pipeline/verify_stage05_visible_scene_objects.py" --stage05 "$stage05_r1"
python3 "$project_root/scene_pipeline/verify_stage05_navigation_structure_refinement.py" --stage05-r2 "$stage05_r2"
exec python3 "$project_root/scene_pipeline/stage05_navigation_structure_refinement_r3.py" \
  --stage05-r1 "$stage05_r1" --stage05-r2 "$stage05_r2" --stage02 "$stage02" --stage03 "$stage03" \
  --output "$output_dir"
