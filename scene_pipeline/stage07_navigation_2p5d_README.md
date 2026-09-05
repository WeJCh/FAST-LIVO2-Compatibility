# 阶段 7：2.5D 导航产品渲染

阶段 7 只读取 `stage06_final_handoff/` 指定的阶段 6 版本化 `vectors/`。它不读取 `pcd_review/`、阶段 4/5
原始 PCD 或逐帧缓存；常规执行使用 [`run_2p5d_workflow.sh`](run_2p5d_workflow.sh)。

```bash
RUN_DIR=/path/to/RUN_DIR
bash scene_pipeline/run_stage07_navigation_2p5d.sh "$RUN_DIR"
bash scene_pipeline/run_verify_stage07_navigation_2p5d.sh \
  "$RUN_DIR/scene_pipeline_v4/stage07_navigation_2p5d_v4"
bash scene_pipeline/run_stage07_freeze_handoff.sh "$RUN_DIR"
bash scene_pipeline/run_verify_stage07_freeze.sh "$RUN_DIR"
```

输出：

- `web/index.html`：自包含本地交互式 2.5D 产品；
- `overview/navigation_2p5d_overview.svg`：静态总览；
- `render_contract.json`：输入、图层样式、默认显示和禁止解释；
- `validation/stage07_render_report.json`：产品统计与安全检查；
- `stage07_final_handoff/`：最终 SHA-256 冻结交接。

实测道路、桥接候选、固定宽度候选、路沿候选、人行道/路肩、路口/开放区和建筑标识必须按各自
`render_policy` 显示。建筑体块仅是由可见立面线段锚定的导航符号：可见长度、方向和高度来自证据，
符号厚度不代表真实深度。产品层的过滤不回写阶段 5/6 对象分类。

## 可选：定位效果展示

当 FAST-LOCALIZATION 的定位运行实际使用了与 `RUN_DIR` 相同的 FAST-LIVO2 地图时，可在**新的输出目录**生成带红色定位轨迹的产品。若同时确认未筛选 RTK CSV 已转换到同一局部地图坐标系，可追加蓝色 RTK 的 **XY 平面参考**：

```bash
RUN_DIR=/path/to/RUN_DIR
LOC_CSV=/path/to/localization_trajectory_YYYYMMDD_HHMMSS_pidNNN.csv
RAW_RTK_CSV=/path/to/rtk_reference_trajectory.csv
MAP_DIR=/path/to/Log/nav_runs/indoor_01
OUT="$RUN_DIR/scene_pipeline_v4/stage07_navigation_2p5d_with_localization_rtk_xy"

bash scene_pipeline/run_stage07_navigation_2p5d.sh "$RUN_DIR" "$OUT" \
  --localization-trajectory "$LOC_CSV" \
  --raw-rtk-trajectory "$RAW_RTK_CSV" \
  --trajectory-map-dir "$MAP_DIR"
bash scene_pipeline/run_verify_stage07_navigation_2p5d.sh "$OUT"
```

网页增加红蓝轨迹开关、共同时间段滑条、播放、红色四元数航向标记和蓝色同步 RTK 点；SVG 总览显示完整红线与蓝线。
蓝线严格只读取 `timestamp_sec,map_x,map_y`，不读取或嵌入任何 Z 字段；`render_contract.json` 会记录两份 CSV、地图 `metadata.yaml` 和所用 IMU 位姿文件的 SHA-256。

这不是精度评估功能：蓝线必须标为“未筛选 RTK 参考”，不是真值；未施加 GNSS 天线到 IMU 的水平杆臂修正，且未筛选 RTK 可能含异常位置，因此页面禁止 ATE/RMSE 或绝对精度结论。历史 CSV 本身没有记录
`map_dir`，因此只能在确认定位实际加载的地图后传入 `--trajectory-map-dir`；不同地图坐标系不得强行叠加。省略 `--raw-rtk-trajectory` 时产品保持只有红色定位轨迹。
