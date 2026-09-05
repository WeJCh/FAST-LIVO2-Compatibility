#!/usr/bin/env bash
# 将一个已保留帧级观测证据的 FAST-LIVO2 建图运行目录，处理为可审计的 2.5D 产品。
#
# 该入口故意不接受单个 xyz/xyzrgb PCD：道路、路沿和可见建筑的证据判定还依赖逐帧
# 观测、原始/优化关键帧位姿。接受裸 PCD 会把“未观测”错误地当成“没有”。
set -euo pipefail

usage() {
  cat <<'EOF'
用法：
  bash scene_pipeline/run_2p5d_workflow.sh RUN_DIR
  bash scene_pipeline/run_2p5d_workflow.sh --check-inputs RUN_DIR

RUN_DIR 是一次独立的 FAST-LIVO2 建图输出目录。必须包含：
  scene_evidence/metadata.yaml
  scene_evidence/frame_observations.csv
  dense_rgb_cache/manifest.csv
  dense_rgb_cache/frames/frame_*.pcd
  keyframes/keyframe_poses_imu.txt
  loop_backend/optimized_keyframe_poses_imu.txt
  pcd/all_global_optimized_rgb_dense_full.pcd

RUN_DIR 可以在云端或其他建图机器生成后完整回传到当前机器；本脚本只执行
scene_pipeline 的离线处理，不检查或依赖当前机器的 ROS、GTSAM 或建图环境。

本命令只接受尚不存在 RUN_DIR/scene_pipeline_v4 的新运行目录；这样不会覆盖
任何已生成的产品。成功后最终产品位于：
  RUN_DIR/scene_pipeline_v4/stage07_navigation_2p5d_v4/
  RUN_DIR/scene_pipeline_v4/stage07_final_handoff/

--check-inputs 只检查输入目录契约，不执行计算。
EOF
}

fail() {
  printf '2.5D 工作流失败：%s\n' "$*" >&2
  exit 2
}

require_file() {
  local path=$1
  local label=$2
  [[ -s "$path" ]] || fail "缺少或为空的${label}：${path}"
}

require_directory() {
  local path=$1
  local label=$2
  [[ -d "$path" ]] || fail "缺少${label}目录：${path}"
}

run_stage() {
  local label=$1
  shift
  printf '\n========== %s ==========\n' "$label"
  bash "$@"
}

mode=run
case "${1:-}" in
  -h|--help)
    usage
    exit 0
    ;;
  --check-inputs)
    mode=check
    shift
    ;;
esac

[[ $# -eq 1 ]] || {
  usage >&2
  exit 2
}

run_dir=$(cd "$1" && pwd -P) || fail "运行目录不存在：$1"
project_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)
pipeline_dir="$project_root/scene_pipeline"
stage_root="$run_dir/scene_pipeline_v4"

# 与阶段1的真实读取集合一致；阶段1会继续完成逐帧时间、点数、PCD头和位姿的一致性校验。
require_file "$run_dir/scene_evidence/metadata.yaml" '场景证据元数据'
require_file "$run_dir/scene_evidence/frame_observations.csv" '逐帧观测清单'
require_file "$run_dir/dense_rgb_cache/manifest.csv" 'RGB缓存清单'
require_directory "$run_dir/dense_rgb_cache/frames" 'RGB缓存帧'
require_file "$run_dir/keyframes/keyframe_poses_imu.txt" '原始关键帧位姿'
require_file "$run_dir/loop_backend/optimized_keyframe_poses_imu.txt" '回环优化关键帧位姿'
require_file "$run_dir/pcd/all_global_optimized_rgb_dense_full.pcd" '全局稠密RGB点云图'

if [[ "$mode" == check ]]; then
  printf '2.5D 工作流输入契约检查通过：%s\n' "$run_dir"
  printf '提示：这只是路径/非空检查；完整逐帧一致性校验会在阶段1执行。\n'
  exit 0
fi

[[ ! -e "$stage_root" ]] || fail "检测到既有流程输出，拒绝覆盖或混跑：${stage_root}；请使用新的 RUN_DIR。"

# 阶段1先建立输入库存和 loop map<-odom 校正；阶段0随后将阶段1核验过的输入和 0--3 代码冻结。
run_stage '阶段 1：位姿校正与输入库存' "$pipeline_dir/run_stage01_pose_correction.sh" "$run_dir"
run_stage '阶段 0：输入冻结' "$pipeline_dir/run_stage00_input_freeze.sh" "$run_dir"
run_stage '阶段 2：几何观测证据' "$pipeline_dir/run_stage02_geometric_evidence_v2.sh" "$run_dir"
run_stage '阶段 3A：全局边界基元' "$pipeline_dir/run_stage03a_global_boundary_primitives.sh" "$run_dir"
run_stage '阶段 3.2：受边界约束走廊' "$pipeline_dir/run_stage032_boundary_constrained_corridors.sh" "$run_dir"
run_stage '阶段 3：道路证据汇总' "$pipeline_dir/run_stage03_final_consolidation.sh" "$run_dir"
run_stage '阶段 4：路沿与人行道候选' "$pipeline_dir/run_stage04_curb_sidewalk_extraction.sh" \
  "$run_dir" "$stage_root/stage04_curb_sidewalk_extraction_r7_low_road_evidence_review"
run_stage '阶段 4.2：推断道路边界约束' "$pipeline_dir/run_stage042_inferred_road_edge_constraints.sh" \
  "$run_dir" "$stage_root/stage042_inferred_road_edge_constraints_r2_local_parallel_fix"
run_stage '阶段 4：冻结交接' "$pipeline_dir/run_stage04_freeze_handoff.sh" "$run_dir"
run_stage '阶段 5 R1：可见场景物体' "$pipeline_dir/run_stage05_visible_scene_objects.sh" "$run_dir"
run_stage '阶段 5 R2：导航结构精化' "$pipeline_dir/run_stage05_navigation_structure_refinement.sh" "$run_dir"
run_stage '阶段 5 R3：导航结构恢复' "$pipeline_dir/run_stage05_navigation_structure_refinement_r3.sh" "$run_dir"
run_stage '阶段 5：冻结交接' "$pipeline_dir/run_stage05_freeze_handoff.sh" "$run_dir"
run_stage '阶段 6：证据状态矢量化' "$pipeline_dir/run_stage06_vectorization.sh" "$run_dir"
run_stage '阶段 6：冻结交接' "$pipeline_dir/run_stage06_freeze_handoff.sh" "$run_dir"
run_stage '阶段 7：2.5D 产品渲染' "$pipeline_dir/run_stage07_navigation_2p5d.sh" "$run_dir"
run_stage '阶段 7：最终冻结交接' "$pipeline_dir/run_stage07_freeze_handoff.sh" "$run_dir"

printf '\n2.5D 产品已完成：\n'
printf '  Web 产品：%s\n' "$stage_root/stage07_navigation_2p5d_v4/web/index.html"
printf '  静态总览：%s\n' "$stage_root/stage07_navigation_2p5d_v4/overview/navigation_2p5d_overview.svg"
printf '  最终冻结：%s\n' "$stage_root/stage07_final_handoff/stage07_freeze_manifest.json"
