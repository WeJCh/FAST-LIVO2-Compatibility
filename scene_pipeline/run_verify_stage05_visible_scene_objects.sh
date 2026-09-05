#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  printf '用法：%s RUN_DIR\n' "$0" >&2
  exit 2
fi

project_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
run_dir=$(cd "$1" && pwd)

exec python3 "$project_root/scene_pipeline/verify_stage05_visible_scene_objects.py" \
  --stage05 "$run_dir/scene_pipeline_v4/stage05_visible_scene_objects"
