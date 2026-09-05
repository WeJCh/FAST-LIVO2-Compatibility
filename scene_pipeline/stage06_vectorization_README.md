# 阶段 6：版本化矢量化与谨慎规则化

阶段 6 只消费阶段 3 最终交接、阶段 4 冻结和阶段 5 冻结。它输出保留来源和证据状态的 GeoJSON/CSV，
并在运行前递归校验阶段 4/5 冻结包。新地图通常由端到端入口执行。

```bash
RUN_DIR=/path/to/RUN_DIR
bash scene_pipeline/run_stage06_vectorization.sh "$RUN_DIR"
bash scene_pipeline/run_verify_stage06_vectorization.sh \
  "$RUN_DIR/scene_pipeline_v4/stage06_vector_map_v1"
bash scene_pipeline/run_stage06_freeze_handoff.sh "$RUN_DIR"
bash scene_pipeline/run_verify_stage06_freeze.sh "$RUN_DIR"
```

## 输出契约

- `vectors/road_surface_areas.geojson`、中心线和宽度剖面：区分观测道路、桥接和固定宽度候选；
- 路沿、人行道/路肩、路口/开放区及推断道路边约束：均保留证据状态；
- 建筑标识：仅可见、`partial/inferred` 的导航标识；
- `pcd_review/`：CloudCompare 显示副本，不是新的证据，也不是阶段 7 输入。

阶段 6 只做同证据类别内的谨慎合并与规则化；不平行化、正交化、圆弧拟合或用固定宽度补造观测道路。
完成冻结后，`stage06_final_handoff/` 是阶段 7 的唯一输入。
