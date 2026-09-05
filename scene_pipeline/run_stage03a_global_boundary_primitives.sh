#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
  printf '用法：%s RUN_DIR [OUTPUT_DIR]\n' "$0" >&2
  exit 2
fi

project_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
run_dir=$(cd "$1" && pwd)
stage02_dir="$run_dir/scene_pipeline_v4/stage02_geometric_evidence_v2_r2"
# R1 已完成，不能改写。当前脚本在相同的 3A 基元结果之上增加边界带合并和轨迹双边配对。
output_dir=${2:-"$run_dir/scene_pipeline_v4/stage03a_global_boundary_primitives_r4_conservative_merge"}

if [[ -e "$output_dir/stage03a_complete.json" ]]; then
  printf '阶段 3A 完整输出已存在，拒绝覆盖：%s\n' "$output_dir" >&2
  exit 2
fi

python3 "$project_root/scene_pipeline/stage03a_global_boundary_primitives.py" \
  --stage02 "$stage02_dir" --output "$output_dir"
