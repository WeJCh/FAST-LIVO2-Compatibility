#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  printf '用法：%s STAGE06_OUTPUT_DIR\n' "$0" >&2
  exit 2
fi

project_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
output_dir=$(cd "$1" && pwd)
exec python3 "$project_root/scene_pipeline/verify_stage06_vectorization.py" --output "$output_dir"
