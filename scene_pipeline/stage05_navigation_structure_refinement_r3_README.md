# 阶段 5 R3：道路反证与绿色层局部平面回收

R3 是阶段 5 最终对象基线。它只读 R1/R2、阶段 2 和阶段 3 正式输出：用双边实测道路面排除道路上方的
动态或树冠候选，并从绿色非平面层保守回收局部贴地平面候选。

```bash
RUN_DIR=/path/to/RUN_DIR
bash scene_pipeline/run_stage05_navigation_structure_refinement_r3.sh "$RUN_DIR"
bash scene_pipeline/run_verify_stage05_navigation_structure_refinement_r3.sh "$RUN_DIR"
```

规范输出是 `RUN_DIR/scene_pipeline_v4/stage05_navigation_structure_refinement_r3/`。审计表
`evidence/navigation_structure_r3_records.csv` 记录道路重叠比例、道路上方比例、R3 决策和来源。

R3 不生成完整建筑块；所有支持点均源于 R1 的真实多帧稳定体素。R3 完成后仍须执行
`run_stage05_freeze_handoff.sh`，由 `stage05_final_handoff/` 作为阶段 6/7 的唯一对象输入。
