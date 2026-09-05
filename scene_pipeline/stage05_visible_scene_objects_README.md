# 阶段 5 R1：可见立面与场景对象证据

新地图应由 [`run_2p5d_workflow.sh`](run_2p5d_workflow.sh) 调用本阶段。R1 只在已经完成并验证阶段 4
冻结包后运行，读取阶段 1 的逐帧校正与不可变 RGB 缓存、阶段 2 几何语境和阶段 4 冻结语境。

## 判定边界

R1 先在 `map` 坐标中累计多帧稳定三维体素，再提取局部窄带竖直平面段。它不把颜色、单帧高度跨度
或 PCD 可视化颜色当作建筑标签。

- 几何证据为 `observed`，但 `visible_facade_candidate` / `wall_candidate` 等语义仍为 `inferred`；
- 所有对象保持 `partial`，不输出完整建筑块、背面、屋顶或真实建筑深度；
- 道路、路沿和确认人行道只提供距离语境，不参与对象朝向或类别的反向推断；
- R2/R3 才在 R1 的可审计来源上完成结构精炼与道路反证。

## 单独运行与校验

```bash
RUN_DIR=/path/to/RUN_DIR
bash scene_pipeline/run_stage05_visible_scene_objects.sh "$RUN_DIR"
bash scene_pipeline/run_verify_stage05_visible_scene_objects.sh "$RUN_DIR"
```

规范输出是 `RUN_DIR/scene_pipeline_v4/stage05_visible_scene_objects/`。该目录存在时入口会拒绝覆盖。
R1 的输出不是阶段 6/7 的直接输入；必须完成 R2、R3 和 `stage05_final_handoff/`。

主要审计内容位于 `evidence/` 与 `validation/`：对象记录保留来源稳定体素数、帧支持汇总、可见拟合段、
证据状态和拒绝原因。人工复核应检查对象是否落在真实可见竖直面上，而不是追求完整建筑外观。
