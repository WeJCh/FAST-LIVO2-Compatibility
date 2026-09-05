# 阶段 5 R2：导航结构精炼

R2 在已验证的阶段 5 R1 对象上进行导航式结构精炼，不重读逐帧缓存，也不修改阶段 0--4 或 R1。
常规执行仍由 [`run_2p5d_workflow.sh`](run_2p5d_workflow.sh) 完成。

```bash
RUN_DIR=/path/to/RUN_DIR
bash scene_pipeline/run_stage05_navigation_structure_refinement.sh "$RUN_DIR"
bash scene_pipeline/run_verify_stage05_navigation_structure_refinement.sh "$RUN_DIR"
```

输入是 `stage05_visible_scene_objects/`，规范输出是
`RUN_DIR/scene_pipeline_v4/stage05_navigation_structure_refinement_r2/`。

R2 只细分导航建筑标识强/可能候选、植被样拒绝和其他上下文；它不生成完整建筑块。候选仍须保留
`observed` 几何证据、`inferred` 语义和 `partial` 完整性。R3 会在此基础上以道路反证排除道路上方候选，
因此 R2 不是阶段 6/7 的直接输入。
