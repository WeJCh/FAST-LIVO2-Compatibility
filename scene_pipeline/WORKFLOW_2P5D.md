# 3D 点云地图到 2.5D 产品：可复现工作流

本文件面向后续执行者和维护者。它说明如何把一次**已经完成建图**的 FAST-LIVO2
运行目录转换成 2.5D 导航产品；实际执行以
[`run_2p5d_workflow.sh`](run_2p5d_workflow.sh) 为准。

## 适用范围与边界

云端建图机器负责生成完整的三维场景证据；本机只运行 `scene_pipeline` 的离线处理。
二者可以是不同机器。本机不需要 ROS、GTSAM 或建图环境，但必须完整接收下文定义的
`RUN_DIR`。

本工作流**不接受单个最终 `xyz` / `xyzrgb` PCD** 作为输入。道路、路沿和可见建筑的
提取需要逐帧观测及位姿关系；仅有最终点云时，无法区分“没有物体”和“没有被观测”。

## 1. 云端产物：`RUN_DIR`

`RUN_DIR` 表示一次独立建图运行的根目录。云端完成建图后，需将其整体回传，至少包含：

```text
RUN_DIR/
├── scene_evidence/
│   ├── metadata.yaml
│   └── frame_observations.csv
├── dense_rgb_cache/
│   ├── manifest.csv
│   └── frames/frame_*.pcd
├── keyframes/keyframe_poses_imu.txt
├── loop_backend/optimized_keyframe_poses_imu.txt
└── pcd/all_global_optimized_rgb_dense_full.pcd
```

若云端也使用本项目代码，使用
[`../launch/mapping_robotdog_scene_evidence.launch`](../launch/mapping_robotdog_scene_evidence.launch)
配置一次独立建图输出目录。它把关键帧、回环结果、逐帧 RGB 缓存、观测清单和最终稠密 RGB 图写入上述
固定位置；需在云端完成建图并正常结束后再回传。例如：

```bash
roslaunch fast_livo mapping_robotdog_scene_evidence.launch \
  run_dir:=/remote/path/map_run img_en:=1
```

若云端使用不同的建图入口，也可以，只要最终输出满足本节的目录与文件契约。

各文件的作用如下：

| 内容 | 用途 |
| --- | --- |
| `frame_observations.csv` | 帧时间、前端 odom 位姿、RGB 点云批次与传感器来源。 |
| `dense_rgb_cache/frames/` | 不可变的逐帧 RGB 点云观测。 |
| 原始与优化关键帧位姿 | 阶段 1 构造随时间变化的 `T_map_odom(t)`。 |
| 最终稠密 RGB 图 | 校验逐帧缓存点数总和，并作为全局结果的证据锚点。 |

回传时必须保持目录结构、文件名和二进制 PCD 内容不变。例如：

```bash
rsync -aP USER@CLOUD_HOST:/remote/path/map_run/ /local/path/map_run/
```

这里的命令只是传输示例；云端主机、认证和路径按实际环境替换。

## 2. 本机执行

先检查回传是否完整：

```bash
bash scene_pipeline/run_2p5d_workflow.sh --check-inputs /local/path/map_run
```

检查通过后执行完整流程：

```bash
bash scene_pipeline/run_2p5d_workflow.sh /local/path/map_run
```

入口会按下列顺序执行，并在每个阶段出错时立即停止：

```text
1 位姿校正与输入库存
→ 0 输入冻结
→ 2 几何观测证据
→ 3A 边界基元 → 3.2 走廊候选 → 3 道路证据汇总
→ 4 路沿/人行道候选 → 4.2 推断道路边界约束 → 4 冻结
→ 5 R1/R2/R3 可见场景对象 → 5 冻结
→ 6 证据状态矢量化 → 6 冻结
→ 7 2.5D 渲染 → 7 最终冻结
```

阶段 1 会继续验证逐帧时间、帧号、点数、PCD 头、原始/优化关键帧 ID 与最终点云点数；
`--check-inputs` 只做路径与非空的快速检查。

## 3. 输出与验收

成功后，产品都位于：

```text
RUN_DIR/scene_pipeline_v4/
├── stage07_navigation_2p5d_v4/
│   ├── web/index.html
│   ├── overview/navigation_2p5d_overview.svg
│   ├── render_contract.json
│   └── validation/stage07_render_report.json
└── stage07_final_handoff/
    └── stage07_freeze_manifest.json
```

- `web/index.html`：本地交互式 2.5D 产品；直接用浏览器打开。
- `navigation_2p5d_overview.svg`：静态总览。
- `render_contract.json`：图层、证据状态与禁止解释。
- `stage07_freeze_manifest.json`：最终可复现冻结清单。

可独立复核最终产品和冻结包：

```bash
bash scene_pipeline/run_verify_stage07_navigation_2p5d.sh \
  /local/path/map_run/scene_pipeline_v4/stage07_navigation_2p5d_v4

bash scene_pipeline/run_verify_stage07_freeze.sh /local/path/map_run
```

### 可选：叠加 FAST-LOCALIZATION 定位效果

这一步仅在确认定位运行加载的 `MAP_DIR` 与该 2.5D 产品同属一张 FAST-LIVO2 地图时执行。它不重跑
阶段 1--6，不改写已冻结产品，而是在新目录生成红色定位轨迹展示。若已确认未筛选 RTK CSV 同样属于该局部地图坐标系，可同时生成蓝色 RTK 的 XY 平面参考层：

```bash
LOC_CSV=/path/to/localization_trajectory_YYYYMMDD_HHMMSS_pidNNN.csv
RAW_RTK_CSV=/path/to/rtk_reference_trajectory.csv
MAP_DIR=/path/to/Log/nav_runs/indoor_01
OUT=/local/path/map_run/scene_pipeline_v4/stage07_navigation_2p5d_with_localization_rtk_xy

bash scene_pipeline/run_stage07_navigation_2p5d.sh /local/path/map_run "$OUT" \
  --localization-trajectory "$LOC_CSV" \
  --raw-rtk-trajectory "$RAW_RTK_CSV" \
  --trajectory-map-dir "$MAP_DIR"
bash scene_pipeline/run_verify_stage07_navigation_2p5d.sh "$OUT"
```

网页包含红蓝轨迹开关、四元数航向的红色当前标记、共同时间段滑条和播放控制；SVG 含静态完整红蓝线。蓝线严格只读取
未筛选 RTK CSV 的 `timestamp_sec,map_x,map_y`，不读取或嵌入 Z。输出契约记录两份 CSV 与地图关键文件的 SHA-256。
蓝色 RTK 仅是平面参考：未做质量筛选或 GNSS 天线到 IMU 的水平杆臂修正，不是真值，不产生 ATE/RMSE 或绝对精度结论。
省略 `--raw-rtk-trajectory` 时仅显示红线。

## 4. 不可覆盖与可复现规则

1. 端到端入口只接受没有 `RUN_DIR/scene_pipeline_v4/` 的新运行目录，避免覆盖或混合旧产物。
2. 若一次运行中断或需要改变算法，使用新的 `RUN_DIR` 重新执行；不要删除或手工改写已冻结结果。
3. 不要编辑 `dense_rgb_cache/`、`scene_evidence/`、`keyframes/`、`loop_backend/` 或最终 PCD。阶段 0/1 会记录并核验输入 SHA-256。
4. 阶段 4、5、6、7 的冻结包记录算法代码、运行入口、校验器和接受输出树的 SHA-256。文档编辑不会改变新冻结包的有效性。
5. `pcd_review/` 仅供 CloudCompare 人工复核，阶段 7 只消费阶段 6 的版本化 `vectors/`，不从 PCD 颜色反推语义。

## 5. 产品语义限制

- 稳定地面、连续硬高差和垂直结构是几何证据，不是道路、路沿或建筑标签。
- 实测道路、受限桥接道路和固定宽度道路候选具有不同证据状态，产品中不得混同。
- 建筑只表示可见立面推断，始终是 `partial`；不能解释为完整建筑深度、屋顶或背面。
- 不确定区域保留候选或未知状态，不能通过渲染补全。

## 6. 代码定位

| 目标 | 入口/实现 |
| --- | --- |
| 端到端执行与输入契约 | `run_2p5d_workflow.sh` |
| 帧级坐标校正 | `stage01_pose_correction.py` |
| 多帧几何证据 | `stage02_geometric_evidence_v2.cpp` |
| 路沿与人行道候选 | `stage04_curb_sidewalk_extraction.py` |
| 可见场景对象 | `stage05_visible_scene_objects.cpp` |
| 版本化矢量 | `stage06_vectorization.py` |
| 2.5D Web/SVG 渲染 | `stage07_navigation_2p5d.py` |
| 最终产品校验 | `verify_stage07_navigation_2p5d.py`、`verify_stage07_freeze.py` |

如需理解各阶段的算法规则、字段或人工复核方法，再阅读本目录的 `README.md` 和各阶段
`*_README.md`。
