# FAST-LIVO2 RobotDog 建图工程

本仓库是面向当前机器狗传感器布置与数据接口适配的 FAST-LIVO2 建图工程。它以 4 线前置激光雷达、雷达内置 IMU 和前置相机为输入，生成可交给独立 `FAST-LOCALIZATION` 工程使用的三维先验地图。

本仓库的定位配套代码不包含在这里；建图和定位应作为两个独立 Git 仓库维护。二者通过一次建图运行目录中的关键帧点云与优化位姿衔接。

## 1. 上游项目资料与适配说明

本工程基于香港大学 MARS Lab 的 FAST-LIVO2。上游系统是紧耦合的激光—惯性—视觉里程计与建图系统，面向实时三维重建和机载/车载定位。当前仓库在此基础上增加了机器狗传感器话题、外参、数据回放和建图—定位交接适配；这些改动不取代上游算法与依赖。

![FAST-LIVO2 框架](pics/Framework.png)

- 上游论文：[FAST-LIVO2](https://arxiv.org/pdf/2408.14035)、[Resource-Constrained Platforms](https://arxiv.org/pdf/2501.13876)、[FAST-LIVO](https://arxiv.org/pdf/2203.00893)；
- 上游演示视频：[Bilibili](https://www.bilibili.com/video/BV1Ezxge7EEi)、[YouTube](https://youtu.be/SecDzgbtlY)；
- 相机—雷达外参标定可参考上游推荐的 [FAST-Calib](https://github.com/hku-mars/FAST-Calib)；
- 上游公开评测数据见 [FAST-LIVO2-Dataset](https://connecthkuhk-my.sharepoint.com/:f:/g/personal/zhengcr_connect_hku_hk/ErdFNQtjMxZOorYKDTtK4ugBkogXfq1OfDm90GECouuIQA?e=KngY9Z)。该数据集的传感器配置不等同于当前机器狗，不能直接复用本仓库的 `robotdog.yaml`。

## 2. 当前工程能做什么

- 订阅机器狗的 `/front_lidar`、`/front_lidar/imu` 和 `/front_camera/image_compressed` 话题；
- 使用激光—惯性里程计，并在相机可用时使用视觉信息辅助建图；
- 保存关键帧、局部点云、优化后的关键帧位姿和最终稠密点云；
- 在编译到 GTSAM 后启用回环后端，对关键帧位姿进行图优化；
- 从 ROS 2 的 rosbag2（SQLite `.db3`）回放机器狗的激光、IMU 和压缩图像到 ROS 1；
- 可选地从一次三维建图结果生成二维导航栅格，或导出场景取证/可视化的旁路数据。

二维导航栅格和场景可视化是三维建图结果的派生产品，不是 `FAST-LOCALIZATION` 的地图输入。

## 3. 机器狗接口与配置

当前机器狗参数位于 [config/robotdog.yaml](config/robotdog.yaml)，默认话题如下：


| 传感器       | ROS 话题                         | 说明                                           |
| ------------ | -------------------------------- | ---------------------------------------------- |
| 前置激光雷达 | `/front_lidar`                   | `sensor_msgs/PointCloud2`，当前配置为 4 线雷达 |
| 激光雷达 IMU | `/front_lidar/imu`               | `sensor_msgs/Imu`                              |
| 前置相机     | `/front_camera/image_compressed` | `sensor_msgs/CompressedImage`                  |

雷达到 IMU、相机到雷达的外参均由该配置文件提供。更换传感器安装位置、话题名称、点云字段或时间戳单位时，必须同时核对建图和定位仓库中的 `robotdog.yaml`；两个工程的雷达—IMU 外参不一致会导致定位地图坐标关系错误。

## 4. 部署：依赖安装、源码放置与编译

### 4.1 系统与 ROS

上游 README 的支持范围为 Ubuntu 18.04–20.04；当前机器狗配置和启动器以 **Ubuntu 20.04 + ROS Noetic + catkin** 为运行目标。较早的 ROS 发行版没有在这套机器狗话题、rosbag2 回放与配置上验证，因此不应仅因上游声明兼容就直接用于交付实验。

### 4.2 基础库

需要安装或自行提供以下依赖：

- PCL >= 1.8；
- Eigen >= 3.3.4；
- OpenCV >= 4.2；
- Boost（含 `thread` 组件）；
- `cv_bridge`、`image_transport`、`vikit_common`、`vikit_ros` 等 ROS/catkin 依赖。

使用自定义版本 PCL、OpenCV 或 Sophus 时，应保证 CMake 实际找到的是同一套 ABI 兼容的库；仅把库安装到系统中但没有让 CMake 找到，仍会导致编译或运行失败。

### 4.3 Sophus

上游工程使用非模板化、double-only 版本 Sophus。若系统中尚未安装兼容版本，可按上游建议构建指定提交：

```bash
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout a621ff
mkdir build && cd build
cmake ..
make
sudo make install
```

安装后可用 `pkg-config` 或 CMake 配置阶段确认 `Sophus` 能被找到。`sudo make install` 会写入系统目录，应仅在确认目标机器允许系统级安装时执行。

### 4.4 Vikit

Vikit 提供本工程所需的相机模型、数学和插值功能，且应作为 catkin 包放在工作空间 `src` 下：

```bash
cd "$WORKSPACE/src"
git clone https://github.com/xuankuzcr/rpg_vikit.git
```

其中 `$WORKSPACE` 是 catkin 工作空间根目录。该仓库使用的 Vikit 与部分旧版 FAST-LIVO 工程不同，混用其他 Vikit 分支可能出现编译接口不兼容。

### 4.5 获取源码并编译

将本仓库放在 `$WORKSPACE/src/FAST-LIVO2`。首次克隆时，将下面的占位地址替换为你发布到 GitHub 的建图仓库地址：

```bash
cd "$WORKSPACE/src"
git clone https://github.com/WeJCh/FAST-LIVO2-Compatibility FAST-LIVO2
cd "$WORKSPACE"
source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_LOOP_BACKEND=ON -j4
source devel/setup.bash
```

回环后端还需要 GTSAM；若 CMake 没有找到 GTSAM，主建图程序仍可编译，但不会启用回环优化。二维导航图构建依赖 OctoMap，只有需要该派生产品时才应以 `-DBUILD_NAV_MAP_BUILDER=ON` 重新配置。动态物体剔除模块默认关闭，且当前机器狗交付不将其视为已验证的建图结果。

## 5. 常规建图与数据回放

先启动建图节点，再另开终端回放 rosbag2 数据。这样可以确保启动阶段的 IMU 初始化和全部激光数据被接收。

终端 1：

```bash
source /opt/ros/noetic/setup.bash
source "$WORKSPACE/devel/setup.bash"
roslaunch fast_livo mapping_robotdog.launch rviz:=false
```

终端 2：

```bash
source /opt/ros/noetic/setup.bash
source "$WORKSPACE/devel/setup.bash"
python3 "$WORKSPACE/src/FAST-LIVO2/scripts/play_ros2_robotdog_to_ros1.py" \
  "$DATASET_DIR" --rate 1.0 --wait-subscribers
```

`$DATASET_DIR` 是包含 rosbag2 `.db3` 与元数据文件的目录。正常回放结束后，让节点完成最终点云导出；需要停止时只发送一次 `Ctrl-C` 并等待进程退出，避免中断最终地图写出。

默认启动器为 [launch/mapping_robotdog.launch](launch/mapping_robotdog.launch)。针对一张可独立交付的先验地图，可使用 [launch/mapping_robotdog_nav_map.launch](launch/mapping_robotdog_nav_map.launch) 并指定一个从未使用过的运行目录：

```bash
source /opt/ros/noetic/setup.bash
source "$WORKSPACE/devel/setup.bash"
roslaunch fast_livo mapping_robotdog_nav_map.launch \
  run_dir:="$MAP_DIR" rviz:=false
```

其中 `$MAP_DIR` 例如为 `$WORKSPACE/src/FAST-LIVO2/Log/nav_runs/<map_id>`。不要把不同采集批次写入同一运行目录，否则关键帧、位姿和缓存文件会混在一起，无法可靠复现。

## 6. 建图输出与定位交接

一次可用于定位的 FAST-LIVO2 地图运行目录至少应保留以下内容：

```text
<map_id>/
├── keyframes/
│   ├── metadata.yaml
│   └── 关键帧局部激光点云（零填充编号 .pcd）
├── loop_backend/
│   └── optimized_keyframe_poses_imu.txt
├── pcd/
│   └── 最终稠密点云等建图产物
└── 运行日志、缓存和报告文件
```

`FAST-LOCALIZATION` 的 `map_format:=fast_livo2` 会直接读取 `keyframes/metadata.yaml`、关键帧局部点云和优化后的 IMU 位姿；不需要把点云转换成 FAST-LOCALIZATION 的旧格式。交付地图前，应确认 `optimized_keyframe_poses_imu.txt` 已生成，且关键帧数量与点云文件编号匹配。

最终稠密点云便于检查和展示，但定位初始化所需的是上述关键帧地图。`nav_map_terrain_final` 等二维栅格仅供导航或展示，不可替代三维关键帧地图。

## 7. 场景证据与二维地图（可选）

[launch/mapping_robotdog_scene_evidence.launch](launch/mapping_robotdog_scene_evidence.launch) 用于离线回放时导出图像帧、观测记录、回环优化位姿与稠密点云，供 [scene_pipeline/README.md](scene_pipeline/README.md) 的 2.5D 场景可视化流程使用。该流程是结果展示旁路，不参与在线建图或定位状态估计。

二维导航图同样是从三维建图结果投影得到的产品。其占据、未知区域比例和覆盖范围应以实际运行结果为准，不能据此推断三维地图已完整覆盖环境。

## 8. 结果检查与可复现边界

- 建图后的主要检查对象是关键帧数、回环后端日志、优化位姿文件、最终点云和 RViz 中的轨迹/点云一致性；
- RTK 文件可作为离线轨迹对照数据，但当前工程不把 RTK 作为建图或定位在线输入；
- 未完成坐标基准、杆臂和时间同步的独立标定前，不应把离线对照误差表述为绝对定位精度或 ATE；
- 采集数据、生成的点云、缓存、运行日志和大体积结果不应随源码直接提交。建议将其作为版本化附件或发布资产，并在实验说明中记录数据集版本、配置版本和命令行参数。

具体的启动顺序、输出目录含义与本机示例可参考 [运行指令.md](运行指令.md) 和 [输出目录说明.md](输出目录说明.md)。

## 9. 上游来源与许可证

本工程基于香港大学 MARS Lab 的 FAST-LIVO2 二次适配，保留上游版权、许可证和依赖声明。源码按仓库中的 [LICENSE](LICENSE) 发布；在公开仓库、分发地图或附带第三方数据前，还应分别确认上游许可证、数据授权和隐私要求。
