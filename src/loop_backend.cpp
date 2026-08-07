#include "loop_backend.h"

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <tbb/global_control.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <sys/stat.h>

namespace
{
bool createDirectoryRecursively(const std::string &path)
{
  if (path.empty()) return false;
  std::string current;
  std::size_t start = 0;
  if (path.front() == '/')
  {
    current = "/";
    start = 1;
  }
  while (start <= path.size())
  {
    const std::size_t end = path.find('/', start);
    const std::string component = path.substr(start, end - start);
    if (!component.empty())
    {
      if (!current.empty() && current.back() != '/') current += '/';
      current += component;
      if (::mkdir(current.c_str(), 0755) != 0 && errno != EEXIST)
      {
        return false;
      }
    }
    if (end == std::string::npos) break;
    start = end + 1;
  }
  return true;
}

gtsam::Pose3 toGtsam(const Eigen::Isometry3d &pose)
{
  return gtsam::Pose3(gtsam::Rot3(pose.rotation()),
                      gtsam::Point3(pose.translation().x(), pose.translation().y(), pose.translation().z()));
}

Eigen::Isometry3d fromGtsam(const gtsam::Pose3 &pose)
{
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.linear() = pose.rotation().matrix();
  result.translation() = Eigen::Vector3d(pose.translation().x(), pose.translation().y(), pose.translation().z());
  return result;
}

pcl::PointCloud<PointType>::Ptr transformCloud(const pcl::PointCloud<PointType>::Ptr &cloud,
                                                const Eigen::Isometry3d &pose)
{
  pcl::PointCloud<PointType>::Ptr transformed(new pcl::PointCloud<PointType>());
  pcl::transformPointCloud(*cloud, *transformed, pose.matrix().cast<float>());
  return transformed;
}

pcl::PointCloud<PointType>::Ptr voxelDownsample(const pcl::PointCloud<PointType>::Ptr &cloud, double leaf_size)
{
  pcl::PointCloud<PointType>::Ptr output(new pcl::PointCloud<PointType>());
  if (leaf_size <= 0.0)
  {
    *output = *cloud;
    return output;
  }
  pcl::VoxelGrid<PointType> filter;
  filter.setInputCloud(cloud);
  filter.setLeafSize(static_cast<float>(leaf_size), static_cast<float>(leaf_size), static_cast<float>(leaf_size));
  filter.filter(*output);
  return output;
}

} // namespace

LoopBackend::LoopBackend(const LoopBackendConfig &config) : config_(config) {}

void LoopBackend::setImuLidarExtrinsic(const Eigen::Isometry3d &T_imu_lidar)
{
  std::lock_guard<std::mutex> lock(online_mutex_);
  T_imu_lidar_ = T_imu_lidar;
}

bool LoopBackend::enqueueOnlineKeyframe(int id, double timestamp,
                                         const pcl::PointCloud<PointType>::ConstPtr &cloud_lidar,
                                         const Eigen::Isometry3d &T_odom_imu,
                                         std::string *error)
{
  if (!cloud_lidar || cloud_lidar->empty())
  {
    if (error) *error = "Online keyframe cloud is empty.";
    return false;
  }

  // 关键帧点云必须在前端线程内完成深拷贝，后台不能持有可被下一帧覆盖的指针。
  LoopKeyframe keyframe;
  keyframe.id = id;
  keyframe.timestamp = timestamp;
  keyframe.cloud_lidar.reset(new pcl::PointCloud<PointType>(*cloud_lidar));
  keyframe.T_odom_imu = T_odom_imu;

  std::lock_guard<std::mutex> lock(online_mutex_);
  if (!keyframes_.empty() && id <= keyframes_.back().id)
  {
    if (error) *error = "Online keyframe id is not strictly increasing.";
    return false;
  }
  if (!pending_online_keyframes_.empty() && id <= pending_online_keyframes_.back().id)
  {
    if (error) *error = "Online keyframe id duplicates a queued keyframe.";
    return false;
  }
  keyframe.T_odom_lidar = keyframe.T_odom_imu * T_imu_lidar_;
  pending_online_keyframes_.push_back(keyframe);
  return true;
}

std::size_t LoopBackend::pendingOnlineKeyframeCount() const
{
  std::lock_guard<std::mutex> lock(online_mutex_);
  return pending_online_keyframes_.size();
}

bool LoopBackend::getLatestResult(LoopBackendResult *result) const
{
  if (result == nullptr) return false;
  std::lock_guard<std::mutex> lock(online_mutex_);
  if (latest_online_result_.generation == 0 || latest_online_result_.optimized_imu_poses.empty()) return false;
  *result = latest_online_result_;
  return true;
}

bool LoopBackend::exportLatestOnlineResult(const std::string &output_dir, std::string *error) const
{
  LoopBackendResult snapshot;
  Eigen::Isometry3d snapshot_extrinsic = Eigen::Isometry3d::Identity();
  {
    std::lock_guard<std::mutex> lock(online_mutex_);
    if (latest_online_result_.generation == 0 || latest_online_result_.optimized_imu_poses.empty())
    {
      if (error) *error = "No completed online optimization result is available for export.";
      return false;
    }
    snapshot = latest_online_result_;
    snapshot_extrinsic = T_imu_lidar_;
  }

  // 复用统一写出逻辑，确保后台快照与最终交付文件一致。
  LoopBackend exporter(config_);
  exporter.T_imu_lidar_ = snapshot_extrinsic;
  exporter.keyframes_ = snapshot.keyframes;
  exporter.optimized_imu_poses_ = snapshot.optimized_imu_poses;
  exporter.loop_edges_ = snapshot.loop_edges;
  return exporter.writeResults(output_dir, error);
}

bool LoopBackend::runOnce(std::string *error)
{
  std::vector<LoopKeyframe> snapshot_keyframes;
  Eigen::Isometry3d snapshot_extrinsic = Eigen::Isometry3d::Identity();
  std::uint64_t generation = 0;
  {
    std::lock_guard<std::mutex> lock(online_mutex_);
    if (pending_online_keyframes_.empty()) return false;

    keyframes_.insert(keyframes_.end(), pending_online_keyframes_.begin(), pending_online_keyframes_.end());
    pending_online_keyframes_.clear();
    snapshot_keyframes = keyframes_;
    snapshot_extrinsic = T_imu_lidar_;
  }

  // 在独立对象中处理完整快照：ICP、iSAM2 和全局点云生成都不会占用前端提交锁。
  LoopBackend snapshot_backend(config_);
  snapshot_backend.T_imu_lidar_ = snapshot_extrinsic;
  snapshot_backend.keyframes_ = snapshot_keyframes;
  if (!snapshot_backend.optimize(error)) return false;

  pcl::PointCloud<PointType>::Ptr global_map = snapshot_backend.buildOptimizedMap();
  if (!global_map)
  {
    if (error) *error = "Failed to construct the optimized online keyframe map.";
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(online_mutex_);
    latest_online_result_.keyframes = snapshot_keyframes;
    latest_online_result_.optimized_imu_poses = snapshot_backend.optimized_imu_poses_;
    latest_online_result_.loop_edges = snapshot_backend.loop_edges_;
    latest_online_result_.global_keyframe_map = global_map;
    generation = ++online_generation_;
    latest_online_result_.generation = generation;
  }

  std::cout << "[loop-backend] Online generation " << generation
            << ": " << snapshot_keyframes.size() << " nodes, "
            << snapshot_backend.loop_edges_.size() << " loop edges." << std::endl;
  return true;
}

pcl::PointCloud<PointType>::Ptr LoopBackend::makeHistorySubmap(std::size_t center_index) const
{
  pcl::PointCloud<PointType>::Ptr submap(new pcl::PointCloud<PointType>());
  const int first = std::max(0, static_cast<int>(center_index) - config_.history_keyframes_each_side);
  const int last = std::min(static_cast<int>(keyframes_.size()) - 1,
                            static_cast<int>(center_index) + config_.history_keyframes_each_side);
  for (int index = first; index <= last; ++index)
  {
    pcl::PointCloud<PointType>::Ptr transformed =
        transformCloud(keyframes_[static_cast<std::size_t>(index)].cloud_lidar,
                       keyframes_[static_cast<std::size_t>(index)].T_odom_lidar);
    *submap += *transformed;
  }
  return voxelDownsample(submap, config_.icp_voxel_leaf_m);
}

bool LoopBackend::optimize(std::string *error)
{
  if (keyframes_.size() < 2)
  {
    if (error) *error = "Keyframes must be loaded before optimization.";
    return false;
  }

  // The online optimizer shares a process with the OpenMP LIO frontend.  In
  // the deployed GTSAM/TBB combination, parallel factor linearization corrupts
  // the allocator state (see gtsam::NoiseModelFactor::linearize in the abort
  // backtrace).  Keep this small background graph serial: it does not affect
  // the LIO state or map, and avoids spawning TBB worker threads here.
  const tbb::global_control serial_factor_linearization(
      tbb::global_control::max_allowed_parallelism, 1);

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial;
  gtsam::Vector odom_sigmas(6);
  odom_sigmas << config_.odom_rotation_sigma_rad, config_.odom_rotation_sigma_rad, config_.odom_rotation_sigma_rad,
                 config_.odom_translation_sigma_m, config_.odom_translation_sigma_m, config_.odom_translation_sigma_m;
  const auto odom_noise = gtsam::noiseModel::Diagonal::Sigmas(odom_sigmas);

  gtsam::Vector prior_sigmas(6);
  prior_sigmas << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6;
  graph.add(gtsam::PriorFactor<gtsam::Pose3>(0, toGtsam(keyframes_[0].T_odom_imu),
                                             gtsam::noiseModel::Diagonal::Sigmas(prior_sigmas)));
  initial.insert(0, toGtsam(keyframes_[0].T_odom_imu));

  for (std::size_t index = 1; index < keyframes_.size(); ++index)
  {
    const Eigen::Isometry3d relative = keyframes_[index - 1].T_odom_imu.inverse() * keyframes_[index].T_odom_imu;
    graph.add(gtsam::BetweenFactor<gtsam::Pose3>(index - 1, index, toGtsam(relative), odom_noise));
    initial.insert(index, toGtsam(keyframes_[index].T_odom_imu));
  }

  loop_edges_.clear();
  for (std::size_t current = 0; current < keyframes_.size(); ++current)
  {
    if (static_cast<int>(current) < config_.min_keyframe_index_gap) continue;

    int history = -1;
    double best_distance = std::numeric_limits<double>::infinity();
    for (std::size_t candidate = 0; candidate < current; ++candidate)
    {
      if (static_cast<int>(current - candidate) < config_.min_keyframe_index_gap) continue;
      const double time_difference = std::abs(keyframes_[current].timestamp - keyframes_[candidate].timestamp);
      if (time_difference < config_.min_time_separation_s) continue;
      const double distance = (keyframes_[current].T_odom_lidar.translation() -
                               keyframes_[candidate].T_odom_lidar.translation()).norm();
      if (distance <= config_.candidate_radius_m && distance < best_distance)
      {
        history = static_cast<int>(candidate);
        best_distance = distance;
      }
    }
    if (history < 0) continue;

    pcl::PointCloud<PointType>::Ptr current_cloud = transformCloud(
        keyframes_[current].cloud_lidar, keyframes_[current].T_odom_lidar);
    current_cloud = voxelDownsample(current_cloud, config_.icp_voxel_leaf_m);
    pcl::PointCloud<PointType>::Ptr history_cloud = makeHistorySubmap(static_cast<std::size_t>(history));
    if (static_cast<int>(current_cloud->size()) < config_.min_current_points ||
        static_cast<int>(history_cloud->size()) < config_.min_history_points)
    {
      continue;
    }

    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setInputSource(current_cloud);
    icp.setInputTarget(history_cloud);
    icp.setMaxCorrespondenceDistance(config_.icp_max_correspondence_m);
    icp.setMaximumIterations(config_.icp_max_iterations);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);
    pcl::PointCloud<PointType> aligned;
    icp.align(aligned);
    const double fitness = icp.getFitnessScore();
    if (!icp.hasConverged() || !std::isfinite(fitness) || fitness > config_.icp_fitness_threshold)
    {
      continue;
    }

    Eigen::Isometry3d correction = Eigen::Isometry3d::Identity();
    correction.matrix() = icp.getFinalTransformation().cast<double>();
    const Eigen::Isometry3d corrected_current_lidar = correction * keyframes_[current].T_odom_lidar;
    const Eigen::Isometry3d z_lidar = corrected_current_lidar.inverse() * keyframes_[static_cast<std::size_t>(history)].T_odom_lidar;
    const Eigen::Isometry3d z_imu = T_imu_lidar_ * z_lidar * T_imu_lidar_.inverse();

    const double position_sigma = std::max(config_.loop_translation_sigma_min_m,
                                           std::min(config_.loop_translation_sigma_max_m, std::sqrt(fitness)));
    gtsam::Vector loop_sigmas(6);
    loop_sigmas << config_.loop_rotation_sigma_rad, config_.loop_rotation_sigma_rad, config_.loop_rotation_sigma_rad,
                   position_sigma, position_sigma, position_sigma;
    const auto base_loop_noise = gtsam::noiseModel::Diagonal::Sigmas(loop_sigmas);
    const auto robust_loop_noise = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(1.345), base_loop_noise);
    graph.add(gtsam::BetweenFactor<gtsam::Pose3>(current, static_cast<std::size_t>(history),
                                                 toGtsam(z_imu), robust_loop_noise));

    LoopEdgeReport edge;
    edge.current_id = keyframes_[current].id;
    edge.history_id = keyframes_[static_cast<std::size_t>(history)].id;
    edge.current_timestamp = keyframes_[current].timestamp;
    edge.history_timestamp = keyframes_[static_cast<std::size_t>(history)].timestamp;
    edge.candidate_distance_m = best_distance;
    edge.icp_fitness = fitness;
    edge.T_current_to_history_imu = z_imu;
    loop_edges_.push_back(edge);
    std::cout << "[loop-backend] Accepted loop " << edge.current_id << " -> " << edge.history_id
              << ", fitness=" << edge.icp_fitness << std::endl;
  }

  gtsam::ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.01;
  parameters.relinearizeSkip = 1;
  gtsam::ISAM2 isam(parameters);
  isam.update(graph, initial);
  isam.update();
  if (!loop_edges_.empty())
  {
    isam.update();
    isam.update();
    isam.update();
  }
  const gtsam::Values optimized = isam.calculateBestEstimate();
  optimized_imu_poses_.clear();
  optimized_imu_poses_.reserve(keyframes_.size());
  for (std::size_t index = 0; index < keyframes_.size(); ++index)
  {
    optimized_imu_poses_.push_back(fromGtsam(optimized.at<gtsam::Pose3>(index)));
  }

  std::cout << "[loop-backend] Optimization complete: " << keyframes_.size() << " nodes, "
            << loop_edges_.size() << " accepted loop edges." << std::endl;
  return true;
}

bool LoopBackend::writeResults(const std::string &output_dir, std::string *error) const
{
  if (optimized_imu_poses_.size() != keyframes_.size())
  {
    if (error) *error = "Optimization must succeed before export.";
    return false;
  }
  if (!createDirectoryRecursively(output_dir))
  {
    if (error) *error = "Cannot create output directory: " + output_dir + " (" + std::strerror(errno) + ")";
    return false;
  }

  std::ofstream pose_output(output_dir + "/optimized_keyframe_poses_imu.txt", std::ios::out | std::ios::trunc);
  std::ofstream edge_output(output_dir + "/loop_edges.csv", std::ios::out | std::ios::trunc);
  if (!pose_output.is_open() || !edge_output.is_open())
  {
    if (error) *error = "Cannot create pose or loop-edge output file.";
    return false;
  }

  pose_output << std::fixed << std::setprecision(9);
  for (std::size_t index = 0; index < keyframes_.size(); ++index)
  {
    const Eigen::Quaterniond q(optimized_imu_poses_[index].rotation());
    const Eigen::Vector3d t = optimized_imu_poses_[index].translation();
    pose_output << keyframes_[index].id << ' ' << keyframes_[index].timestamp << ' '
                << t.x() << ' ' << t.y() << ' ' << t.z() << ' '
                << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << '\n';
  }

  edge_output << "current_id,history_id,current_timestamp,history_timestamp,candidate_distance_m,icp_fitness,"
              << "tx,ty,tz,qx,qy,qz,qw\n";
  edge_output << std::fixed << std::setprecision(9);
  for (const LoopEdgeReport &edge : loop_edges_)
  {
    const Eigen::Quaterniond q(edge.T_current_to_history_imu.rotation());
    const Eigen::Vector3d t = edge.T_current_to_history_imu.translation();
    edge_output << edge.current_id << ',' << edge.history_id << ','
                << edge.current_timestamp << ',' << edge.history_timestamp << ','
                << edge.candidate_distance_m << ',' << edge.icp_fitness << ','
                << t.x() << ',' << t.y() << ',' << t.z() << ','
                << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w() << '\n';
  }

  pcl::PointCloud<PointType>::Ptr optimized_cloud = buildOptimizedMap();
  if (!optimized_cloud)
  {
    if (error) *error = "Unable to build optimized global cloud.";
    return false;
  }
  pcl::PCDWriter writer;
  const std::string cloud_path = output_dir + "/all_global_optimized.pcd";
  if (writer.writeBinary(cloud_path, *optimized_cloud) != 0)
  {
    if (error) *error = "Cannot write optimized global PCD: " + cloud_path;
    return false;
  }

  std::ofstream report(output_dir + "/report.yaml", std::ios::out | std::ios::trunc);
  if (report.is_open())
  {
    report << "keyframes: " << keyframes_.size() << '\n'
           << "accepted_loop_edges: " << loop_edges_.size() << '\n'
           << "candidate_radius_m: " << config_.candidate_radius_m << '\n'
           << "min_time_separation_s: " << config_.min_time_separation_s << '\n'
           << "min_keyframe_index_gap: " << config_.min_keyframe_index_gap << '\n'
           << "icp_fitness_threshold: " << config_.icp_fitness_threshold << '\n'
           << "global_map_leaf_m: " << config_.global_map_leaf_m << '\n'
           << "output_points: " << optimized_cloud->size() << '\n';
  }
  std::cout << "[loop-backend] Wrote " << cloud_path << " with " << optimized_cloud->size() << " points." << std::endl;
  return true;
}

pcl::PointCloud<PointType>::Ptr LoopBackend::buildOptimizedMap() const
{
  if (optimized_imu_poses_.size() != keyframes_.size()) return pcl::PointCloud<PointType>::Ptr();

  pcl::PointCloud<PointType>::Ptr global_cloud(new pcl::PointCloud<PointType>());
  for (std::size_t index = 0; index < keyframes_.size(); ++index)
  {
    const Eigen::Isometry3d T_map_lidar = optimized_imu_poses_[index] * T_imu_lidar_;
    pcl::PointCloud<PointType>::Ptr transformed = transformCloud(keyframes_[index].cloud_lidar, T_map_lidar);
    *global_cloud += *transformed;
  }
  return voxelDownsample(global_cloud, config_.global_map_leaf_m);
}
