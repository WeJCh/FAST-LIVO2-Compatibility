#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
  printf '用法：%s RUN_DIR [OUTPUT_DIR]\n' "$0" >&2
  exit 2
fi

project_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
run_dir=$(cd "$1" && pwd)
stage01_dir="$run_dir/scene_pipeline_v4/stage01_pose_correction"
output_dir=${2:-"$run_dir/scene_pipeline_v4/stage00_input_freeze"}

python3 "$project_root/scene_pipeline/stage00_input_freeze.py" \
  --run-dir "$run_dir" --stage01 "$stage01_dir" --output "$output_dir" \
  --project-root "$project_root" --overwrite
