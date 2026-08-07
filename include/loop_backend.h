#ifndef FAST_LIVO_LOOP_BACKEND_H
#define FAST_LIVO_LOOP_BACKEND_H

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "utils/types.h"

struct LoopBackendConfig
{
  double candidate_radius_m = 10.0;
  double min_time_separation_s = 30.0;
  int min_keyframe_index_gap = 20;
  int history_keyframes_each_side = 20;

  double icp_voxel_leaf_m = 0.4;
  double icp_max_correspondence_m = 150.0;
  int icp_max_iterations = 100;
  double icp_fitness_threshold = 0.4;
  int min_current_points = 100;
  int min_history_points = 300;

  double odom_rotation_sigma_rad = 0.02;
  double odom_translation_sigma_m = 0.10;
  double loop_rotation_sigma_rad = 0.10;
  double loop_translation_sigma_min_m = 0.05;
  double loop_translation_sigma_max_m = 1.0;
  double global_map_leaf_m = 0.20;
};

struct LoopKeyframe
{
  int id = -1;
  double timestamp = 0.0;
  pcl::PointCloud<PointType>::Ptr cloud_lidar;
  Eigen::Isometry3d T_odom_imu = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T_odom_lidar = Eigen::Isometry3d::Identity();
};

struct LoopEdgeReport
{
  int current_id = -1;
  int history_id = -1;
  double current_timestamp = 0.0;
  double history_timestamp = 0.0;
  double candidate_distance_m = 0.0;
  double icp_fitness = 0.0;
  Eigen::Isometry3d T_current_to_history_imu = Eigen::Isometry3d::Identity();
};

// 在线后端向 ROS 发布层提供的不可变结果快照。
// cloud_lidar 仍是关键帧局部点云；global_keyframe_map 已处于 map 坐标系。
struct LoopBackendResult
{
  std::vector<LoopKeyframe> keyframes;
  std::vector<Eigen::Isometry3d> optimized_imu_poses;
  std::vector<LoopEdgeReport> loop_edges;
  pcl::PointCloud<PointType>::Ptr global_keyframe_map;
  std::uint64_t generation = 0;
};

class LoopBackend
{
public:
  explicit LoopBackend(const LoopBackendConfig &config);

  // 在线接口：前端只调用 enqueueOnlineKeyframe()，后台线程调用 runOnce()。
  // 该函数会深拷贝点云，确保前端复用 feats_down_body 时不会影响后台。
  void setImuLidarExtrinsic(const Eigen::Isometry3d &T_imu_lidar);
  bool enqueueOnlineKeyframe(int id, double timestamp,
                             const pcl::PointCloud<PointType>::ConstPtr &cloud_lidar,
                             const Eigen::Isometry3d &T_odom_imu,
                             std::string *error = nullptr);
  std::size_t pendingOnlineKeyframeCount() const;
  bool runOnce(std::string *error = nullptr);
  bool getLatestResult(LoopBackendResult *result) const;
  // 导出最近一次完成优化的在线快照。
  bool exportLatestOnlineResult(const std::string &output_dir, std::string *error = nullptr) const;

private:
  bool optimize(std::string *error = nullptr);
  bool writeResults(const std::string &output_dir, std::string *error = nullptr) const;
  pcl::PointCloud<PointType>::Ptr makeHistorySubmap(std::size_t center_index) const;
  pcl::PointCloud<PointType>::Ptr buildOptimizedMap() const;

  LoopBackendConfig config_;
  Eigen::Isometry3d T_imu_lidar_ = Eigen::Isometry3d::Identity();
  std::vector<LoopKeyframe> keyframes_;
  std::vector<Eigen::Isometry3d> optimized_imu_poses_;
  std::vector<LoopEdgeReport> loop_edges_;

  // 仅在线接口访问以下成员。优化工作在独立快照对象中完成，避免阻塞前端。
  mutable std::mutex online_mutex_;
  std::vector<LoopKeyframe> pending_online_keyframes_;
  LoopBackendResult latest_online_result_;
  std::uint64_t online_generation_ = 0;
};

#endif
