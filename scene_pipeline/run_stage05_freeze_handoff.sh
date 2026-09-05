#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
  printf '用法：%s RUN_DIR [OUTPUT_DIR]\n' "$0" >&2
  exit 2
fi

project_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
run_dir=$(cd "$1" && pwd)
stage01="$run_dir/scene_pipeline_v4/stage01_pose_correction"
stage02="$run_dir/scene_pipeline_v4/stage02_geometric_evidence_v2_r2"
stage03="$run_dir/scene_pipeline_v4/stage03_final_handoff"
stage04="$run_dir/scene_pipeline_v4/stage04_final_handoff"
r1="$run_dir/scene_pipeline_v4/stage05_visible_scene_objects"
r2="$run_dir/scene_pipeline_v4/stage05_navigation_structure_refinement_r2"
r3="$run_dir/scene_pipeline_v4/stage05_navigation_structure_refinement_r3"
output_dir=${2:-"$run_dir/scene_pipeline_v4/stage05_final_handoff"}

if [[ -e "$output_dir" ]]; then
  printf '阶段5冻结输出目录已存在，拒绝覆盖：%s\n' "$output_dir" >&2
  exit 2
fi

"$project_root/scene_pipeline/run_verify_stage04_freeze.sh" "$run_dir"
"$project_root/scene_pipeline/run_verify_stage05_visible_scene_objects.sh" "$run_dir"
"$project_root/scene_pipeline/run_verify_stage05_navigation_structure_refinement.sh" "$run_dir"
"$project_root/scene_pipeline/run_verify_stage05_navigation_structure_refinement_r3.sh" "$run_dir"
exec python3 "$project_root/scene_pipeline/stage05_freeze_handoff.py" \
  --project-root "$project_root" --stage01 "$stage01" --stage02 "$stage02" --stage03 "$stage03" \
  --stage04-handoff "$stage04" --stage05-r1 "$r1" --stage05-r2 "$r2" --stage05-r3 "$r3" --output "$output_dir"
