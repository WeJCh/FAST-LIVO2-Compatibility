#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
  printf '用法：%s RUN_DIR [OUTPUT_DIR]\n' "$0" >&2
  exit 2
fi

project_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
run_dir=$(cd "$1" && pwd)
stage_root="$run_dir/scene_pipeline_v4"
output_dir=${2:-"$stage_root/stage04_final_handoff"}

if [[ -e "$output_dir" ]]; then
  printf '阶段4冻结交接目录已存在，拒绝覆盖：%s\n' "$output_dir" >&2
  exit 2
fi

exec python3 "$project_root/scene_pipeline/stage04_freeze_handoff.py" \
  --project-root "$project_root" \
  --stage02 "$stage_root/stage02_geometric_evidence_v2_r2" \
  --stage03a "$stage_root/stage03a_global_boundary_primitives_r4_conservative_merge" \
  --stage03 "$stage_root/stage03_final_handoff" \
  --stage04 "$stage_root/stage04_curb_sidewalk_extraction_r7_low_road_evidence_review" \
  --stage042 "$stage_root/stage042_inferred_road_edge_constraints_r2_local_parallel_fix" \
  --output "$output_dir"
