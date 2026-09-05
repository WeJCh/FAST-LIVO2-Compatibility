#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
  printf '用法：%s RUN_DIR [OUTPUT_DIR]\n' "$0" >&2
  exit 2
fi

project_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
run_dir=$(cd "$1" && pwd)
stage01_dir="$run_dir/scene_pipeline_v4/stage01_pose_correction"
# 每次重要算法修订都写入独立目录；V2、R1 保留为人工验证与问题定位基线。
output_dir=${2:-"$run_dir/scene_pipeline_v4/stage02_geometric_evidence_v2_r2"}
source_file="$project_root/scene_pipeline/stage02_geometric_evidence_v2.cpp"
binary=$(mktemp /tmp/fast_livo_scene_stage02_v2.XXXXXX)
trap 'rm -f "$binary"' EXIT

if [[ -e "$output_dir" ]]; then
  printf '阶段 2 V2 输出已存在，拒绝覆盖：%s\n' "$output_dir" >&2
  exit 2
fi

"${CXX:-g++}" -std=c++14 -O3 -Wall -Wextra -Wpedantic "$source_file" -o "$binary"
"$binary" --run "$run_dir" --stage01 "$stage01_dir" --output "$output_dir"
