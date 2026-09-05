#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
  printf 'Usage: %s RUN_DIR [OUTPUT_DIR]\n' "$0" >&2
  exit 2
fi

project_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
run_dir=$(cd "$1" && pwd)
output_dir=${2:-"$run_dir/scene_pipeline_v4/stage01_pose_correction"}

exec python3 "$project_root/scene_pipeline/stage01_pose_correction.py" \
  --run-dir "$run_dir" \
  --output "$output_dir"
