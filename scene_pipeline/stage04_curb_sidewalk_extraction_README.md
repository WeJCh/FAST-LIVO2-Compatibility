# 阶段 4 / 4.2：路沿、人行道与推断道路边约束

对新地图，使用 [`run_2p5d_workflow.sh`](run_2p5d_workflow.sh) 执行完整链路。本文仅用于单独开发或
定位阶段 4；必须已有阶段 2、3A、3 的正式输出，且目标目录不存在。

## 输入、职责与边界

阶段 4 只读阶段 2 几何网格/连续硬高差、阶段 3A 边界带/层级和阶段 3 道路横断面/道路候选。
它将道路邻近边界保守分类为路沿候选、未知边界和人行道/路肩候选：

- 固定宽度道路候选只用于保持搜索连续，不能确认实测道路宽度或观测路沿；
- 连续硬高差、长边界带和 PCD 颜色均不是语义标签；
- 确认人行道/路肩需要道路相邻、同侧内外平行边界、连续稳定地面及观测支持；
- 证据不足的墙根/绿化或边界必须保留为 `unknown` 或组合候选；
- `low_road_evidence_hard_edge_review` 是人工复核层，不是路沿、道路或渲染输入。

阶段 4.2 只从阶段 4 R7 生成低置信度 `inferred_road_edge_constraint`。它不能生成道路面，也不能把
推断约束升级为观测道路或路沿。

## 单独运行

端到端入口为阶段 4 使用以下规范输出名。需要单独重现时使用相同的名字：

```bash
RUN_DIR=/path/to/RUN_DIR
V4="$RUN_DIR/scene_pipeline_v4"

bash scene_pipeline/run_stage04_curb_sidewalk_extraction.sh \
  "$RUN_DIR" "$V4/stage04_curb_sidewalk_extraction_r7_low_road_evidence_review"
bash scene_pipeline/run_stage042_inferred_road_edge_constraints.sh \
  "$RUN_DIR" "$V4/stage042_inferred_road_edge_constraints_r2_local_parallel_fix"
bash scene_pipeline/run_stage04_freeze_handoff.sh "$RUN_DIR"
bash scene_pipeline/run_verify_stage04_freeze.sh "$RUN_DIR"
```

阶段 4 冻结包 `stage04_final_handoff/` 才是阶段 5/6 的唯一阶段 4 输入；不要将 R7 或 4.2
的未冻结输出直接混用到下游。

## 审计输出

- `evidence/combined_boundary_semantic_records.csv`：边界来源、几何、道路关系、分类、证据状态与拒绝原因；
- `evidence/sidewalk_surface_records.csv`：确认/可能人行道或路肩面；
- `validation/stage04_curb_sidewalk_report.json`：统计、参数、限制和被抑制的可视层；
- `evidence/low_road_evidence_hard_edge_review_*`：仅人工复核；
- 阶段 4.2 的约束 CSV/PCD：阶段 6 的低置信度对齐语境，保留 `inferred` 状态。

局部硬高差诊断脚本 `stage04_local_hard_edge_diagnosis.py` 是开发工具；只读既有阶段输出，不能替代
阶段 4 R7 或冻结交接。
