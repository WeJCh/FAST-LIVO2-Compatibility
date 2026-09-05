# 场景管线：证据优先的 3D → 2.5D 产品化

本目录是 FAST-LIVO2 三维场景证据到 2.5D 导航产品的正式离线实现。执行新地图时，先阅读
[`WORKFLOW_2P5D.md`](WORKFLOW_2P5D.md)，并只使用
[`run_2p5d_workflow.sh`](run_2p5d_workflow.sh) 作为端到端入口。

```text
云端完成建图并回传完整 RUN_DIR
  → 本机阶段 1 位姿校正与阶段 0 输入冻结
  → 阶段 2--5 几何、道路边界与可见对象证据
  → 阶段 6 版本化矢量
  → 阶段 7 2.5D Web/SVG 产品与最终冻结
```

## 当前工作流

`RUN_DIR` 是一次独立建图运行的根目录，不是单个 PCD。它可在云端生成后回传至本机，必须包含
逐帧 RGB 缓存、逐帧观测清单、原始/优化关键帧位姿和最终稠密 RGB 点云。完整文件契约见
[`WORKFLOW_2P5D.md`](WORKFLOW_2P5D.md#1-云端产物run_dir)。

```bash
# 只检查回传目录是否具备基本输入
bash scene_pipeline/run_2p5d_workflow.sh --check-inputs /path/to/RUN_DIR

# 生成完整 2.5D 产品；RUN_DIR/scene_pipeline_v4 必须不存在
bash scene_pipeline/run_2p5d_workflow.sh /path/to/RUN_DIR
```

阶段 1 会进一步校验时间、帧号、点数、PCD 头、关键帧 ID 与最终点云点数。端到端入口按以下顺序执行：

```text
1 → 0 → 2 → 3A → 3.2 → 3 → 4 → 4.2 → 4 freeze
  → 5 R1 → 5 R2 → 5 R3 → 5 freeze
  → 6 → 6 freeze → 7 → 7 final freeze
```

阶段 1 必须先于阶段 0：阶段 0 冻结的是阶段 1 已核验的输入库存与 0--3 代码哈希。

## 阶段职责与代码入口

| 阶段 | 职责 | 主实现/入口 |
| --- | --- | --- |
| 1 / 0 | 逐帧 `map←odom` 校正、输入与代码指纹 | `stage01_pose_correction.py`、`stage00_input_freeze.py` |
| 2 | 多帧稳定地面、硬高差、垂直结构、自由空间等几何证据 | `stage02_geometric_evidence_v2.cpp` |
| 3A / 3.2 / 3 | 边界带、走廊候选与道路证据汇总 | `stage03a_*`、`stage032_*`、`stage03_final_*` |
| 4 / 4.2 | 路沿、人行道/路肩候选与低置信度道路边约束 | `stage04_*`、`stage042_*` |
| 5 R1/R2/R3 | 多帧可见立面、建筑标识候选、道路反证与绿色层回收 | `stage05_*` |
| 6 | 保留证据状态的 GeoJSON/CSV 矢量 | `stage06_vectorization.py` |
| 7 | 自包含 Web 产品、SVG 总览与渲染契约 | `stage07_navigation_2p5d.py` |

阶段专用 `run_stage*.sh` 适用于开发或定位某一阶段；它们假定上游已存在，且大多数输出拒绝覆盖。
普通复现不要手工拼接阶段命令，使用端到端入口。

已确认 FAST-LOCALIZATION 实际使用同一 FAST-LIVO2 地图时，阶段 7 还可在新的输出目录追加红色定位轨迹展示，并可将同一局部地图坐标系中的未筛选 RTK 以仅 XY 的蓝色参考层叠加；
它是产品显示扩展，不会改变阶段 1--6 证据或常规端到端入口。蓝色 RTK 不是真值，也不产生 ATE/RMSE 或绝对精度结论。命令、地图身份校验和限制见
[`stage07_navigation_2p5d_README.md`](stage07_navigation_2p5d_README.md#可选定位效果展示)。

## 产品与验证

成功后，主要结果位于：

```text
RUN_DIR/scene_pipeline_v4/stage07_navigation_2p5d_v4/web/index.html
RUN_DIR/scene_pipeline_v4/stage07_navigation_2p5d_v4/overview/navigation_2p5d_overview.svg
RUN_DIR/scene_pipeline_v4/stage07_final_handoff/stage07_freeze_manifest.json
```

```bash
bash scene_pipeline/run_verify_stage07_navigation_2p5d.sh \
  /path/to/RUN_DIR/scene_pipeline_v4/stage07_navigation_2p5d_v4
bash scene_pipeline/run_verify_stage07_freeze.sh /path/to/RUN_DIR
```

阶段 4--7 的冻结包记录算法、入口、校验器和被接受输出树的 SHA-256。文档不是冻结输入；修改说明
不应使新运行的冻结校验失效。

## 证据边界

- 稳定地面、硬高差和垂直结构是几何事实，不自动等于道路、路沿或建筑。
- 实测道路、受限桥接道路、固定宽度道路候选必须保留不同的证据状态。
- 建筑只表达可见立面推断，保持 `partial`，不能补出背面、屋顶或完整深度。
- `pcd_review/` 只供人工复核；阶段 7 只读取阶段 6 的版本化 `vectors/`。
- 未观测区域保持未知，不能由渲染补齐。

## 文档索引

- [`WORKFLOW_2P5D.md`](WORKFLOW_2P5D.md)：新地图的操作、回传、输出与复现规则。
- `stage04_*_README.md` 至 `stage07_*_README.md`：单阶段算法契约和开发定位说明。
- [`../docs/7-FAST-LIVO2道路场景矢量化与2.5D导航风格渲染需求与实施方案.md`](../docs/7-FAST-LIVO2道路场景矢量化与2.5D导航风格渲染需求与实施方案.md)：项目级技术方案摘要。
- [`../docs/8-FAST-LIVO2三维点云地图3D平面化渲染-会话交接.md`](../docs/8-FAST-LIVO2三维点云地图3D平面化渲染-会话交接.md)：当前 V4 技术交接。
