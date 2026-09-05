#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
  printf '用法：%s RUN_DIR [OUTPUT_DIR]\n' "$0" >&2
  exit 2
fi

project_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
run_dir=$(cd "$1" && pwd)
stage02_dir="$run_dir/scene_pipeline_v4/stage02_geometric_evidence_v2_r2"
stage03a_dir="$run_dir/scene_pipeline_v4/stage03a_global_boundary_primitives_r4_conservative_merge"
stage03_dir="$run_dir/scene_pipeline_v4/stage03_final_handoff"
stage04_dir="$run_dir/scene_pipeline_v4/stage04_curb_sidewalk_extraction_r7_low_road_evidence_review"
output_dir=${2:-"$run_dir/scene_pipeline_v4/stage042_inferred_road_edge_constraints"}

if [[ -e "$output_dir" ]]; then
  printf '阶段4.2输出目录已存在，拒绝覆盖：%s\n' "$output_dir" >&2
  exit 2
fi

exec python3 "$project_root/scene_pipeline/stage042_inferred_road_edge_constraints.py" \
  --stage02 "$stage02_dir" --stage03a "$stage03a_dir" --stage03 "$stage03_dir" \
  --stage04 "$stage04_dir" --output "$output_dir"
