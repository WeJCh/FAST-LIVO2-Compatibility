#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
  printf '用法：%s RUN_DIR [OUTPUT_DIR]\n' "$0" >&2
  exit 2
fi

project_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
run_dir=$(cd "$1" && pwd)
if [[ $# -eq 2 ]]; then
  output_dir=$2
else
  output_dir="$run_dir/scene_pipeline_v4/stage06_vector_map_v1"
fi

exec python3 "$project_root/scene_pipeline/stage06_vectorization.py" \
  --run-dir "$run_dir" \
  --output "$output_dir"
