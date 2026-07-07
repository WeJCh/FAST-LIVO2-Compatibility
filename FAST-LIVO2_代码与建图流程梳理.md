# FAST-LIVO2 代码架构与 SLAM 建图流程梳理

## 一、项目概述

FAST-LIVO2 是一个高效的 **激光雷达-惯性-视觉紧耦合融合 SLAM 系统**，发表于 **IEEE T-RO 2024**，由香港大学火星实验室 (HKU MARS Lab) 郑纯然开发。系统能够在退化环境中完成实时三维重建和机器人定位。

### 核心能力
- **三种工作模式**：纯激光里程计 (ONLY_LO)、激光-惯性里程计 (ONLY_LIO)、激光-惯性-视觉融合里程计 (LIVO)
- **多传感器支持**：支持 Avia, Velodyne, Ouster, Hesai XT32, Pandar128, Robosense 等多款激光雷达
- **松耦合视觉**：视觉采用直接法 (direct method)，基于稀疏图像块的光度误差进行状态更新

---

## 二、整体架构图

```
┌─────────────────────────────────────────────────────────────────┐
│                        main.cpp                                  │
│  LIVMapper mapper(nh);                                          │
│  mapper.initializeSubscribersAndPublishers(nh, it);             │
│  mapper.run();  ← 核心主循环                                      │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                     LIVMapper (核心协调类)                        │
│                                                                  │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────────┐  │
│  │  Preprocess   │  │  ImuProcess  │  │  VoxelMapManager     │  │
│  │  (p_pre)      │  │  (p_imu)     │  │  (voxelmap_manager)  │  │
│  │              │  │              │  │                      │  │
│  │ • LiDAR点云   │  │ • IMU前向传播 │  │ • 八叉树体素建图      │  │
│  │   解析与特征  │  │ • 点云去畸变  │  │ • 平面拟合           │  │
│  │ • 多LiDAR型号 │  │ • IMU初始化   │  │ • 残差构建           │  │
│  │   适配       │  │ • 重力对齐    │  │ • 滑动窗口管理       │  │
│  └──────────────┘  └──────────────┘  └──────────────────────┘  │
│                                                                  │
│  ┌──────────────────────────────────────────────────────────────┐│
│  │                     VIOManager (vio_manager)                 ││
│  │  • 直接法视觉里程计 (direct sparse image alignment)           ││
│  │  • 图像块提取与光度误差计算                                   ││
│  │  • 投影雅可比 → EKF 状态更新                                  ││
│  │  • 视觉地图点管理                                            ││
│  └──────────────────────────────────────────────────────────────┘│
│                                                                  │
│  ┌──────────────────────────────────────────────────────────────┐│
│  │              EKF 状态向量 (19维)                              ││
│  │  [R(3) | t(3) | inv_expo(1) | v(3) | bg(3) | ba(3) | g(3)] ││
│  └──────────────────────────────────────────────────────────────┘│
│                                                                  │
│  ┌──────────────────────────────────────────────────────────────┐│
│  │  数据缓冲区:                                                  ││
│  │  • lid_raw_data_buffer:    LiDAR点云队列                     ││
│  │  • imu_buffer:             IMU 数据队列                      ││
│  │  • img_buffer:             图像数据队列                      ││
│  │  • LidarMeasures.measures: 同步后的测量组队列                ││
│  └──────────────────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────────────┘
```

---

## 三、系统状态与模式

### 3.1 EKF 状态向量 (StatesGroup, 19维)

| 维度 | 符号 | 物理含义 |
|------|------|----------|
| 0-2  | R    | 姿态 (SO(3) 旋转矩阵) |
| 3-5  | t    | 位置 (世界坐标系) |
| 6    | τ    | 逆曝光时间 (inverse exposure time) |
| 7-9  | v    | 速度 (世界坐标系) |
| 10-12 | b_g | 陀螺仪零偏 |
| 13-15 | b_a | 加速度计零偏 |
| 16-18 | g   | 重力加速度 |

### 3.2 SLAM 模式 (SLAM_MODE)

```cpp
enum SLAM_MODE {
  ONLY_LO  = 0,  // 纯激光里程计
  ONLY_LIO = 1,  // 激光 + 惯性里程计
  LIVO     = 2   // 激光 + 惯性 + 视觉融合里程计
};
```

### 3.3 EKF 执行状态 (EKF_STATE)

```cpp
enum EKF_STATE {
  WAIT = 0,  // 等待数据
  VIO  = 1,  // 执行视觉-惯性更新
  LIO  = 2,  // 执行激光-惯性更新
  LO   = 3   // 执行纯激光更新
};
```

---

## 四、完整建图流程 (以 LIVO 模式为例)

### 4.1 启动流程

```
main()
  │
  ├─ LIVMapper(nh)
  │   ├─ readParameters(): 读取 YAML 配置 (传感器外参、体素参数、模式等)
  │   └─ initializeComponents(): 初始化子模块
  │       ├─ Preprocess: LiDAR 预处理
  │       ├─ ImuProcess: IMU 处理器
  │       ├─ VoxelMapManager: 体素地图管理器
  │       └─ VIOManager: 视觉惯性管理器
  │           ├─ 加载相机内参 (vikit camera model)
  │           ├─ 设置 LiDAR→IMU→Camera 外参链
  │           ├─ 初始化图像网格 (grid_size, grid_n_width, grid_n_height)
  │           └─ 初始化 SubSparseMap (视觉稀疏子图)
  │
  ├─ initializeSubscribersAndPublishers()
  │   ├─ 订阅: LiDAR点云、IMU、图像
  │   └─ 发布者:
  │       ├─ /LaserCloudFullRes: 世界坐标系点云 (带颜色)
  │       ├─ /Odometry: 里程计位姿
  │       ├─ /path: 轨迹路径
  │       ├─ /rgb_image: RGB图像
  │       ├─ /visual_sub_map: 视觉子地图
  │       └─ /LaserCloudEffect: 有效特征点
  │
  └─ run() 【主循环】
```

### 4.2 主循环 run() 详细流程

```
while (ros::ok())
{
  // ========== 第1步: 数据同步 ==========
  if (!sync_packages(LidarMeasures))  continue;

  // ========== 第2步: 重力对齐 (仅首帧) ==========
  if (!gravity_align_finished)
    gravityAlignment();

  // ========== 第3步: 状态估计与建图 ==========
  if (imu_need_init)  // IMU初始化阶段
  {
    processImu();  // IMU初始化
  }
  else  // 正常EKF更新阶段
  {
    stateEstimationAndMapping();
  }

  // ========== 第4步: 发布结果 ==========
  publish_frame_world(pubLaserCloudFullRes, vio_manager);  // 带颜色点云
  publish_visual_sub_map(pubSubVisualMap);                  // 视觉子地图
  publish_odometry(pubOdomAftMapped);                       // 里程计
  publish_path(pubPath);                                     // 轨迹
  publish_img_rgb(pubImage, vio_manager);                   // RGB图像
  if (pub_effect_point_en)
    publish_effect_world(pubLaserCloudEffect, ...);         // 有效特征点
}
```

### 4.3 数据同步 (sync_packages) —— LIVO 模式

这是整个系统的核心调度逻辑，确保 LiDAR/IMU/Image 三种传感器的时间对齐：

```
sync_packages(LidarMeasureGroup &meas) [LIVO模式]
│
├─ 根据条件选择执行 VIO 或 LIO:
│
├─ 条件1: 执行 VIO (视觉-惯性更新)
│   ├─ 检查: img_buffer、lid_raw_data_buffer 非空
│   ├─ 检查: 图像时间戳 < 当前激光帧起始时间 + 曝光补偿
│   ├─ 创建 MeasureGroup m:
│   │   ├─ m.vio_time = img_capture_time
│   │   ├─ m.lio_time = last_lio_update_time
│   │   ├─ m.img = img_buffer.front()
│   │   └─ m.imu ← 对应时间段内的IMU数据
│   ├─ meas.measures.push_back(m)
│   ├─ meas.lio_vio_flg = VIO
│   └─ return true
│
├─ 条件2: 执行 LIO (激光-惯性更新)
│   ├─ 检查: 激光扫描完整 (lidar_pushed==true)
│   ├─ 检查: 激光帧结束时间 < 最早的图像时间
│   ├─ 创建 MeasureGroup m:
│   │   └─ m.lio_time = meas.lidar_frame_end_time
│   ├─ meas.measures.push_back(m)
│   ├─ meas.lio_vio_flg = LIO
│   └─ return true
│
└─ 条件3: 等待
    └─ return false
```

**同步策略总结**：
- VIO 和 LIO 交替执行
- VIO 以图像帧率运行 (如 30Hz)
- LIO 以激光帧率运行 (如 10Hz)
- 当图像时间戳早于当前激光帧起始时间时执行 VIO
- 当激光帧扫描完成且时间早于下一张图像时执行 LIO

### 4.4 状态估计与建图 (stateEstimationAndMapping)

```
stateEstimationAndMapping()
│
├─ for (auto &meas : LidarMeasures.measures)
│   {
│     if (meas.lio_vio_flg == VIO:)
│       └─ handleVIO()  → VIOManager::processFrame()
│
│     if (meas.lio_vio_flg == LIO:)
│       └─ handleLIO()  → VoxelMapManager::StateEstimation()
│   }
│
└─ 清空已处理的 measures
```

---

## 五、子模块详解

### 5.1 LiDAR 预处理 (Preprocess)

```
Preprocess::process(livox/standard_msg, pcl_out)
│
├─ 根据 lidar_type 选择处理器:
│   ├─ avia_handler:     Livox Avia 固态激光雷达
│   ├─ oust64_handler:   Ouster OS1-64
│   ├─ velodyne_handler: Velodyne VLP-16
│   ├─ xt32_handler:     Hesai XT32
│   ├─ Pandar128_handler:Hesai Pandar128
│   ├─ robosense_handler: Robosense Airy
│   └─ l515_handler:     Intel RealSense L515
│
├─ 提取 LiDAR 特征 (give_feature):
│   ├─ 判断平面点 (plane_judge): 分析相邻点的几何关系
│   ├─ 判断边缘/跳跃 (edge_jump_judge): 检测深度不连续
│   ├─ 特征类型: Nor, Poss_Plane, Real_Plane, Edge_Jump, Edge_Plane, Wire, ZeroPoint
│   └─ 过滤盲区点 (blind filtering)
│
├─ 时间戳提取:
│   ├─ Avia: msg->points[i].offset_time / 1e9
│   ├─ Velodyne/XT32: point.time (存储在 curvature 字段)
│   └─ Ouster: point.t / 1e9
│
└─ 输出: PointCloudXYZI (包含 xyz + intensity + curvature(时间戳))
```

### 5.2 IMU 处理 (IMU_Processing)

```
ImuProcess::Process2(LidarMeasureGroup &meas, StatesGroup &stat, pcl_out)
│
├─ 第1阶段: IMU 初始化 (imu_need_init == true)
│   ├─ 收集多帧IMU数据
│   ├─ 估计初始位姿 (通过静止假设)
│   ├─ 估计陀螺仪零偏、加速度计零偏
│   ├─ 估计重力方向
│   └─ 初始化完成 → imu_need_init = false
│
├─ 第2阶段: IMU前向传播 + 点云去畸变 (正常模式)
│   │
│   ├─ a) IMU前向传播 (EKF预测步骤):
│   │   ├─ 遍历每个 IMU 测量
│   │   ├─ prop_imu_once() × N:
│   │   │   ├─ 中值积分 (mid-point integration)
│   │   │   ├─ 状态更新:
│   │   │   │   ├─ pos_end  += vel × dt + 0.5 × acc × dt²
│   │   │   │   ├─ vel_end  += acc × dt
│   │   │   │   └─ rot_end   = rot × Exp(angvel × dt)
│   │   │   └─ 协方差传播 (EKF predict)
│   │   └─ 记录每个IMU时刻的状态 → IMUpose[]
│   │
│   └─ b) LiDAR点云去畸变 (UndistortPcl):
│       ├─ 对每个激光点:
│       │   ├─ 读取点的原始时间戳 (curvature字段)
│       │   ├─ 在 IMUpose[] 中找到对应时刻的IMU位姿
│       │   ├─ 插值计算该点的位姿
│       │   └─ 将点从body系变换到扫描结束时刻的body系
│       └─ 输出: 去畸变后的点云 (所有点对齐到扫描结束时刻)
│
└─ 输出: stat (传播后的状态), pcl_out (去畸变点云)
```

### 5.3 激光-惯性建图 (VoxelMapManager)

这是 **LiDAR 建图的核心模块**，采用基于八叉树的体素平面地图。

#### 5.3.1 数据结构

```
VoxelOctoTree (八叉树节点)
├─ voxel_center_[3]:      体素中心坐标
├─ quater_length_:        体素四分之一边长
├─ layer_:                当前层数
├─ octo_state_:           0=叶子节点, 1=需继续分裂
├─ leaves_[8]:            8个子节点指针
├─ plane_ptr_ → VoxelPlane:
│   ├─ center_:           平面中心
│   ├─ normal_:           平面法向量
│   ├─ covariance_:       点云协方差矩阵
│   ├─ eigen_values_:     特征值 (判断平面度)
│   ├─ points_size_:      包含点数量
│   └─ is_plane_:         是否已拟合为平面
├─ temp_points_:          暂存点 (pointWithVar)
└─ new_points_:           新加点数量
```

```
VOXEL_LOCATION → 体素空间索引
├─ x, y, z: int64_t (体素坐标)
└─ hash函数: ((z * 116101 % 1e10 + y) * 116101 % 1e10 + x)
```

```
voxel_map_: unordered_map<VOXEL_LOCATION, VoxelOctoTree*>
  全局体素地图 (所有已建图的体素)
```

#### 5.3.2 LIO 状态估计流程

```
VoxelMapManager::StateEstimation(StatesGroup &state_propagat)
│
├─ Step 1: 降采样 (VoxelGrid)
│   └─ 将去畸变点云体素滤波 → feats_down_body_
│
├─ Step 2: 构建残差列表
│   ├─ a) 点云坐标变换: body → world
│   │   └─ pw = R * pb + t (使用 state_propagat 的预测位姿)
│   │
│   ├─ b) 构建 pv_list (pointWithVar 列表):
│   │   └─ 每个点: point_b, point_w, body_var, point_crossmat
│   │
│   └─ c) BuildResidualListOMP(pv_list, ptpl_list):
│       ├─ 并行遍历每个点 (OMP parallel):
│       │   ├─ 根据 point_w 计算所在的体素位置 (VOXEL_LOCATION)
│       │   ├─ 在 voxel_map_ 中查找对应体素
│       │   ├─ 若体素存在 → find_correspond(pw):
│       │   │   ├─ 递归搜索八叉树到叶子节点
│       │   │   ├─ 判断叶子是否有拟合的平面
│       │   │   └─ 若有平面 → 计算点到平面距离作为残差
│       │   └─ build_single_residual():
│       │       ├─ 残差: r = n^T · (pw - plane_center)
│       │       ├─ 构建 PointToPlane (包含 point_w, normal, center, plane_var等)
│       │       └─ 加入 ptpl_list (有效测量列表)
│       └─ 输出: effct_feat_num (有效特征数量)
│
├─ Step 3: IEKF 迭代更新
│   ├─ for (iter = 0; iter < max_iterations_; iter++)
│   │   ├─ 遍历 ptpl_list 中的每个残差:
│   │   │   ├─ 计算点到平面距离: d = n^T · (R*pb + t - center)
│   │   │   ├─ 计算雅可比:
│   │   │   │   ├─ ∂d/∂R = n^T · [-R·pb]×  (对旋转的导数)
│   │   │   │   └─ ∂d/∂t = n^T             (对平移的导数)
│   │   │   └─ 构建 H^T·H 和 H^T·r (信息矩阵)
│   │   │
│   │   ├─ 求解 δx = -(H^T·H + P⁻¹)⁻¹ · (H^T·r) (IEKF更新)
│   │   ├─ 更新状态: state = state ⊕ δx
│   │   └─ 检查收敛: 若 |δx| < threshold → 退出迭代
│   │
│   └─ 更新协方差: P = (I - K·H)·P
│
├─ Step 4: 更新体素地图
│   ├─ UpdateVoxelMap(pv_list):
│   │   ├─ 对每个残差点:
│   │   │   ├─ 将点插入对应体素的 temp_points_
│   │   │   └─ 若超过阈值 (如 20个点) → 尝试初始化平面
│   │   │       ├─ init_plane(): PCA分解得到法向量
│   │   │       └─ 判断平面度 (最小特征值 < planer_threshold_)
│   │   │
│   │   └─ UpdateOctoTree() / cut_octo_tree():
│   │       ├─ 若点数过大 (如 > 50) → 分裂为8个子体素
│   │       └─ 递归构建多层八叉树
│   │
│   └─ 地图状态: 每个体素最多分裂到 max_layer_ 层
│       └─ 叶子节点含一个平面 (VoxelPlane) 或等待更多点
│
└─ Step 5: 地图滑动窗口管理 (MapSliding)
    ├─ 检查当前位姿是否超出滑动窗口范围
    ├─ 若超出 → 清除窗口外的体素
    └─ 保持地图在局部范围内 (如 +-50m)
```

### 5.4 视觉-惯性里程计 (VIOManager)

VIOManager 实现 **直接法稀疏视觉里程计**，与 LiDAR 建图松耦合 —— 视觉使用独立的视觉稀疏地图，通过光度误差约束 EKF 状态。

#### 5.4.1 视觉帧处理流程

```
VIOManager::processFrame(img, pg, voxel_map, img_time)
│
├─ Step 1: 创建新帧
│   ├─ new_frame_ → Frame(cam, img)
│   ├─ 设置帧的姿态: T_f_w_ = state 预测值
│   ├─ 创建图像金字塔 (4层)
│   └─ frame_count_++
│
├─ Step 2: 对齐视觉稀疏地图 (retrieveFromVisualSparseMap)
│   ├─ 将 LiDAR 点投影到图像:
│   │   └─ pc = camera.world2cam(T_f_w_ * pw)
│   │
│   ├─ 构建 2D 网格 (grid_size × grid_size 像素)
│   ├─ 每个网格选一个最优 LiDAR 点:
│   │   ├─ 优先选择离相机最近的点
│   │   └─ 若网格无 LiDAR 点 → 检查地图中是否有视觉点投影到该网格
│   │
│   └─ 生成 visual_submap (包含待跟踪的视觉点)
│
├─ Step 3: 预计算参考图像块 (projectPatchFromRefToCur)
│   ├─ 对每个待跟踪视觉点:
│   │   ├─ 计算仿射变换矩阵 (Warp Affine):
│   │   │   └─ A_cur_ref = 参考帧与当前帧的相对姿态 + 平面法向量
│   │   ├─ 选择最佳搜索层级 (getBestSearchLevel)
│   │   ├─ 从参考帧提取图像块 (warpAffine):
│   │   │   └─ 使用仿射变换将参考帧图像块变换到当前帧视角
│   │   └─ 存储到 visual_submap
│   └─ precomputeReferencePatches(): 预计算各层参考块
│
├─ Step 4: EKF 状态更新 (多层级迭代)
│   └─ for level = max_level down to 0:
│       └─ updateState(img, level) 或 updateStateInverse(img, level)
│
├─ Step 5: 更新视觉地图点 (updateVisualMapPoints)
│   ├─ 检查每个跟踪点的收敛状态
│   ├─ 若收敛(光度误差小) → insertPointIntoVoxelMap()
│   │   └─ 创建 VisualPoint → 存入 feat_map
│   └─ 生成新的候选点 (generateVisualMapPoints)
│       └─ 从LiDAR点中选择未覆盖区域的候选视觉点
│
└─ Step 6: 更新参考图像块 (updateReferencePatch)
    └─ 关键帧策略: 若跟踪点比例过低 → 更新参考帧
```

#### 5.4.2 EKF 视觉更新 (updateState)

```
updateState(img, level)
│
├─ 当前层姿态预测:
│   └─ T_f_w_prior_ = state_propagat 的位姿 (IMU传播)
│
├─ resetGrid(): 重置网格占用标记
│
├─ 并行遍历 visual_submap 中的每个点:
│   ├─ 投影当前点: px_cur = cam.world2cam(T * pw)
│   ├─ 检查网格独占 (每个网格只保留一个最优观测)
│   ├─ 提取当前帧图像块 (getImagePatch)
│   ├─ 计算光度误差:
│   │   └─ error = Σ(scale × I_cur(x) - I_ref(x))²
│   └─ 异常值检查 (outlier_threshold, NCC)
│
├─ 构建 EKF 观测方程:
│   ├─ 对每个有效观测:
│   │   ├─ 雅可比 ∂e/∂δx:
│   │   │   ├─ ∂I/∂u, ∂I/∂v: 图像梯度
│   │   │   ├─ ∂π/∂p: 相机投影导数
│   │   │   ├─ ∂p/∂T: 位姿对3D点的导数 (链式法则)
│   │   │   └─ ∂T/∂δx: 状态对位姿的导数
│   │   ├─ 权重: W = (σ_img² + σ_pt²)⁻¹
│   │   └─ 累加 H^T·W·H 和 H^T·W·r
│   │
│   └─ IEKF 求解:
│       ├─ K = P⁻¹·H^T·(H·P⁻¹·H^T + R)⁻¹
│       ├─ δx = K · r
│       ├─ state += δx
│       └─ P = (I - K·H)·P⁻¹
│
└─ 输出: 更新后的 state
```

---

## 六、关键算法细节

### 6.1 点云去畸变

由于激光雷达扫描周期内（约 0.1s），传感器自身在运动，每个激光点的坐标是在不同时刻采集的。去畸变将所有点统一到扫描结束时刻的 body 坐标系：

```cpp
// 对每个激光点
for (point in lidar_scan) {
  timestamp = point.curvature;  // 点的采集时间 (相对扫描周期的偏移)
  
  // 找到该时刻的 IMU 位姿
  for (imu_pose in IMUpose[]) {
    if (imu_pose.time >= timestamp) {
      Rt = interpolate(imu_pose_k, imu_pose_k+1, timestamp);
      break;
    }
  }
  
  // 变换到扫描结束时刻的 body 系
  point_undistort = Rt_end.inverse() * Rt * point_raw;
}
```

### 6.2 体素平面拟合

```cpp
VoxelOctoTree::init_plane(points, plane) {
  // 1. 计算中心点
  center = mean(points);
  
  // 2. PCA: 计算协方差矩阵
  for (point : points) {
    d = point - center;
    cov += d * d^T;
  }
  
  // 3. 特征值分解
  eigenvalues = eigen(cov);  // λ1 < λ2 < λ3
  
  // 4. 判断平面度
  if (λ1 / (λ1 + λ2 + λ3) < threshold) {
    plane.normal = eigenvector(λ1);  // 最小特征值对应的特征向量 = 法向量
    plane.is_plane = true;
  }
}
```

### 6.3 直接法视觉 (Direct Sparse Image Alignment)

与特征点法不同，FAST-LIVO2 使用 **直接法**：

```
特征点法 (ORB-SLAM等):
  提取特征点 → 计算描述子 → 特征匹配 → PnP/Bundle Adjustment

直接法 (FAST-LIVO2):
  LiDAR 深度 → 投影到图像 → 提取图像块 → 光度误差最小化 ↗ EKF更新

优点:
  - 无需特征提取和匹配，节省计算
  - 在纹理较弱区域也能工作 (利用整体图像梯度)
  - 图像块的仿射变换补偿了视角变化
```

### 6.4 IEKF (迭代扩展卡尔曼滤波)

FAST-LIVO2 使用 IEKF 而非标准 EKF：

```
标准 EKF:
  predict → update (一次线性化)

IEKF:
  predict → for (iter):
              compute residual at current estimate
              linearize around current estimate
              solve for δx
              update state
              if converged: break
           end
```

IEKF 的优势是可以多次线性化，减少线性化误差，对非线性系统收敛性更好。

---

## 七、坐标系变换链

```
LiDAR → IMU → Camera → World

外参 (YAML 配置):
├─ extR, extT:     IMU → LiDAR  (R_li, t_li)
│   └─ P_lidar = R_li^T * (P_imu - t_li)
│
├─ cameraextrinR, cameraextrinT: Camera → LiDAR (R_cl, t_cl)
│   └─ P_camera = R_cl * P_lidar + t_cl
│
姿态传播:
├─ state.rot_end:   世界系 → IMU 系 的旋转 (R_w_i)
├─ state.pos_end:   世界系下 IMU 的位置 (t_w_i)
│
点坐标变换:
  点从 LiDAR 系 → 世界系:
    pw = R_w_i * (R_li * pb + t_li) + t_w_i
  
  世界系 → 相机像素:
    pc = K * (R_cl * R_li^T * R_w_i^T * (pw - t_w_i) - R_cl * t_li + t_cl)
```

---

## 八、多传感器时间同步策略

```
Timeline:
-------|-----[LiDAR Scan]------|--------|------[LiDAR Scan 2]------|---
  IMU IMU IMU IMU IMU IMU IMU IMU IMU IMU IMU IMU IMU IMU IMU IMU IMU
  Img1  ..........  Img2  ..........  Img3  ..........  Img4 ......

同步逻辑:
  VIO触发: 当 Img.time < LiDAR帧.start_time + offset
    → 取图像时间之前的IMU做传播
    → 执行视觉EKF更新

  LIO触发: 当 LiDAR扫描完成 && LiDAR帧.end_time < 下一张图像.time
    → 执行激光EKF更新
    → 更新体素地图

  VIO和LIO交替执行，保证时间顺序
```

---

## 九、发布的数据与建图输出

### 9.1 实时发布

| Topic | 内容 |
|-------|------|
| `/LaserCloudFullRes` | 世界系 RGB 点云 (LiDAR点+相机着色) |
| `/Odometry` | 实时里程计位姿 |
| `/path` | 运动轨迹 |
| `/rgb_image` | RGB 图像 |
| `/visual_sub_map` | 视觉稀疏地图 |
| `/LaserCloudEffect` | 用于建图的有效特征点 |

### 9.2 保存输出

```cpp
// PCD 文件保存
Log/pcd/<timestamp>.pcd    // 世界系带颜色点云
Log/lidar_pos.txt          // LiDAR 位姿轨迹
Log/visual_pos.txt          // 视觉位姿轨迹

// 图像保存 (可选)
Log/image/<timestamp>.png  // 图像帧
```

### 9.3 为 Colmap 输出 (colmap_output_en)

当启用 `colmap_output_en` 时，系统会输出 Colmap 兼容格式的数据，用于离线全局BA优化。

---

## 十、配置文件关键参数 (YAML)

```yaml
# 传感器设置
lid_topic: "/livox/lidar"
imu_topic: "/imu/data"
img_topic: "/camera/image_raw"

# 外参
extrinsic_T: [0.0, 0.0, 0.0]      # IMU→LiDAR 平移
extrinsic_R: [1, 0, 0, 0, 1, 0, 0, 0, 1]  # IMU→LiDAR 旋转
camer_extrinsic_T: [0.0, 0.0, 0.0]
camer_extrinsic_R: [1, 0, 0, 0, 1, 0, 0, 0, 1]

# 体素地图参数
max_layer: 3               # 八叉树最大层数
max_points_num: 50         # 体素最大点数
planer_threshold: 0.01     # 平面判断阈值
voxel_size: 1.0            # 体素尺寸

# 视觉参数
grid_size: 30               # 图像网格大小
patch_size: 8               # 图像块尺寸
max_iterations: 5           # 迭代次数
outlier_threshold: 0.2      # 异常值阈值

# 模式
SLAM_MODE: 2                # 0=LO, 1=LIO, 2=LIVO
dense_map_en: false         # 稠密建图开关
```

---

## 十一、总结

### 核心设计理念

1. **紧耦合 LiDAR-惯性-视觉融合**：所有传感器在同一 EKF 框架下，使用相同的 19 维状态向量，通过 IMU 前向传播进行预测，分别用 LiDAR 点到平面残差和视觉光度残差进行更新。

2. **松耦合视觉子图**：视觉使用独立的稀疏地图，不同于直接融合到 LiDAR 体素地图中。视觉提供高频 (30Hz) 的姿态约束，LiDAR 提供精确 (10Hz) 的结构约束。

3. **增量式八叉树体素地图**：体素地图自适应分裂，叶子节点存储平面参数。点到平面距离作为 LiDAR 残差，同时不断更新地图。

4. **直接法视觉**：避免了特征提取/匹配的开销，利用 LiDAR 提供深度先验，通过图像块对齐实现高效的视觉跟踪。

### 建图流程全景

```
传感器数据 → 预处理 → IMU前向传播(预测) → 点云去畸变
                                            ↓
                    ┌── VIO(视觉): 直接法EKF更新 ← 图像
                    │
                    ├── LIO(激光): 点到平面EKF更新 ← 体素地图匹配
                    │
                    └── 地图更新: 八叉树体素平面拟合

        输出: 世界系RGB点云 + 里程计 + 轨迹
```
