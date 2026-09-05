#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  printf '用法：%s RUN_DIR [OUTPUT_DIR] [--localization-trajectory CSV --raw-rtk-trajectory CSV --trajectory-map-dir MAP_DIR]\n' "$0" >&2
  exit 2
fi

project_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
run_dir=$(cd "$1" && pwd)
shift
output_dir="$run_dir/scene_pipeline_v4/stage07_navigation_2p5d_v4"
if [[ $# -gt 0 && "$1" != --* ]]; then
  output_dir=$1
  shift
fi
"$project_root/scene_pipeline/run_verify_stage06_freeze.sh" "$run_dir"
exec python3 "$project_root/scene_pipeline/stage07_navigation_2p5d.py" \
  --run-dir "$run_dir" --output "$output_dir" "$@"
