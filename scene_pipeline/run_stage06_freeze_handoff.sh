#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
  printf '用法：%s RUN_DIR [OUTPUT_DIR]\n' "$0" >&2
  exit 2
fi

project_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
run_dir=$(cd "$1" && pwd)
v4="$run_dir/scene_pipeline_v4"
output_dir=${2:-"$v4/stage06_final_handoff"}

if [[ -e "$output_dir" ]]; then
  printf '阶段6冻结输出目录已存在，拒绝覆盖：%s\n' "$output_dir" >&2
  exit 2
fi

"$project_root/scene_pipeline/run_verify_stage04_freeze.sh" "$run_dir"
"$project_root/scene_pipeline/run_verify_stage05_freeze.sh" "$run_dir"
"$project_root/scene_pipeline/run_verify_stage06_vectorization.sh" "$v4/stage06_vector_map_v1"
exec python3 "$project_root/scene_pipeline/stage06_freeze_handoff.py" \
  --project-root "$project_root" \
  --stage03 "$v4/stage03_final_handoff" \
  --stage04-handoff "$v4/stage04_final_handoff" \
  --stage05-handoff "$v4/stage05_final_handoff" \
  --stage06 "$v4/stage06_vector_map_v1" \
  --output "$output_dir"
