#include "dense_rgb_reprojector.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <exception>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <unordered_map>
#include <vector>

#include <sys/stat.h>

namespace
{
struct TimedPose
{
  int id = -1;
  double timestamp = 0.0;
  Eigen::Isometry3d T_world_imu = Eigen::Isometry3d::Identity();
};

struct PosePair
{
  double timestamp = 0.0;
  Eigen::Isometry3d T_map_odom = Eigen::Isometry3d::Identity();
};

struct CachedFrame
{
  uint64_t id = 0;
  double timestamp = 0.0;
  uint64_t point_count = 0;
  std::string path;
};

struct VoxelKey
{
  int64_t x = 0;
  int64_t y = 0;
  int64_t z = 0;

  bool operator==(const VoxelKey &other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey &key) const
  {
    const std::size_t h1 = std::hash<int64_t>()(key.x);
    const std::size_t h2 = std::hash<int64_t>()(key.y);
    const std::size_t h3 = std::hash<int64_t>()(key.z);
    return h1 ^ (h2 << 1) ^ (h3 << 7);
  }
};

struct VoxelAccumulator
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double r = 0.0;
  double g = 0.0;
  double b = 0.0;
  uint64_t count = 0;
};

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
      if (::mkdir(current.c_str(), 0755) != 0 && errno != EEXIST) return false;
    }
    if (end == std::string::npos) break;
    start = end + 1;
  }
  return true;
}

std::string parentDirectory(const std::string &path)
{
  const std::string::size_type slash = path.find_last_of('/');
  if (slash == std::string::npos) return std::string();
  if (slash == 0) return "/";
  return path.substr(0, slash);
}

bool isAbsolutePath(const std::string &path)
{
  return !path.empty() && path.front() == '/';
}

bool parsePoseFile(const std::string &path, std::map<int, TimedPose> *poses, std::string *error)
{
  std::ifstream input(path);
  if (!input.is_open())
  {
    if (error) *error = "Cannot open pose file: " + path;
    return false;
  }

  std::string line;
  std::size_t line_number = 0;
  while (std::getline(input, line))
  {
    ++line_number;
    if (line.empty() || line.front() == '#') continue;
    std::istringstream stream(line);
    TimedPose pose;
    double tx, ty, tz, qx, qy, qz, qw;
    if (!(stream >> pose.id >> pose.timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw))
    {
      if (error) *error = "Invalid pose at " + path + ":" + std::to_string(line_number);
      return false;
    }
    Eigen::Quaterniond q(qw, qx, qy, qz);
    if (q.norm() < 1e-12)
    {
      if (error) *error = "Zero quaternion at " + path + ":" + std::to_string(line_number);
      return false;
    }
    q.normalize();
    pose.T_world_imu.linear() = q.toRotationMatrix();
    pose.T_world_imu.translation() = Eigen::Vector3d(tx, ty, tz);
    if (!poses->insert(std::make_pair(pose.id, pose)).second)
    {
      if (error) *error = "Duplicate keyframe id " + std::to_string(pose.id) + " in " + path;
      return false;
    }
  }
  if (poses->empty())
  {
    if (error) *error = "No poses found in: " + path;
    return false;
  }
  return true;
}

bool loadPosePairs(const DenseRgbReprojectConfig &config, std::vector<PosePair> *pairs, std::string *error)
{
  std::map<int, TimedPose> raw_poses;
  std::map<int, TimedPose> optimized_poses;
  if (!parsePoseFile(config.raw_keyframe_pose_path, &raw_poses, error) ||
      !parsePoseFile(config.optimized_keyframe_pose_path, &optimized_poses, error))
  {
    return false;
  }

  for (const std::pair<const int, TimedPose> &entry : raw_poses)
  {
    const auto optimized = optimized_poses.find(entry.first);
    if (optimized == optimized_poses.end()) continue;
    PosePair pair;
    pair.timestamp = entry.second.timestamp;
    pair.T_map_odom = optimized->second.T_world_imu * entry.second.T_world_imu.inverse();
    pairs->push_back(pair);
  }
  if (pairs->empty())
  {
    if (error) *error = "Raw and optimized pose files have no matching keyframe ids.";
    return false;
  }
  std::sort(pairs->begin(), pairs->end(), [](const PosePair &a, const PosePair &b) {
    return a.timestamp < b.timestamp;
  });
  return true;
}

bool loadManifest(const std::string &cache_dir, std::vector<CachedFrame> *frames, std::string *error)
{
  const std::string manifest_path = cache_dir + "/manifest.csv";
  std::ifstream input(manifest_path);
  if (!input.is_open())
  {
    if (error) *error = "Cannot open RGB cache manifest: " + manifest_path;
    return false;
  }

  std::string line;
  std::getline(input, line);  // CSV header
  std::size_t line_number = 1;
  while (std::getline(input, line))
  {
    ++line_number;
    if (line.empty()) continue;
    std::stringstream stream(line);
    std::string id_text, timestamp_text, count_text, path;
    if (!std::getline(stream, id_text, ',') || !std::getline(stream, timestamp_text, ',') ||
        !std::getline(stream, count_text, ',') || !std::getline(stream, path))
    {
      if (error) *error = "Invalid cache manifest line " + std::to_string(line_number);
      return false;
    }
    CachedFrame frame;
    try
    {
      frame.id = static_cast<uint64_t>(std::stoull(id_text));
      frame.timestamp = std::stod(timestamp_text);
      frame.point_count = static_cast<uint64_t>(std::stoull(count_text));
    }
    catch (const std::exception &)
    {
      if (error) *error = "Invalid numeric value in cache manifest line " + std::to_string(line_number);
      return false;
    }
    frame.path = isAbsolutePath(path) ? path : cache_dir + "/" + path;
    frames->push_back(frame);
  }
  if (frames->empty())
  {
    if (error) *error = "RGB cache manifest contains no frames: " + manifest_path;
    return false;
  }
  std::sort(frames->begin(), frames->end(), [](const CachedFrame &a, const CachedFrame &b) {
    return a.timestamp < b.timestamp;
  });
  return true;
}

Eigen::Isometry3d interpolateCorrection(const std::vector<PosePair> &pairs, double timestamp, std::size_t *lower_index)
{
  while (*lower_index + 1 < pairs.size() && pairs[*lower_index + 1].timestamp <= timestamp) ++(*lower_index);
  const std::size_t lower = *lower_index;
  const std::size_t upper = std::min(lower + 1, pairs.size() - 1);
  if (lower == upper) return pairs[lower].T_map_odom;

  const double dt = pairs[upper].timestamp - pairs[lower].timestamp;
  const double alpha = dt > 1e-9 ? std::max(0.0, std::min(1.0, (timestamp - pairs[lower].timestamp) / dt)) : 0.0;
  Eigen::Quaterniond q0(pairs[lower].T_map_odom.rotation());
  Eigen::Quaterniond q1(pairs[upper].T_map_odom.rotation());
  q0.normalize();
  q1.normalize();

  Eigen::Isometry3d correction = Eigen::Isometry3d::Identity();
  correction.linear() = q0.slerp(alpha, q1).normalized().toRotationMatrix();
  correction.translation() = (1.0 - alpha) * pairs[lower].T_map_odom.translation() +
                             alpha * pairs[upper].T_map_odom.translation();
  return correction;
}

bool writePcdHeader(std::ofstream &output, uint64_t point_count)
{
  output << "# .PCD v0.7 - Point Cloud Data file format\n"
         << "VERSION 0.7\n"
         << "FIELDS x y z rgb\n"
         << "SIZE 4 4 4 4\n"
         << "TYPE F F F U\n"
         << "COUNT 1 1 1 1\n"
         << "WIDTH " << point_count << "\n"
         << "HEIGHT 1\n"
         << "VIEWPOINT 0 0 0 1 0 0 0\n"
         << "POINTS " << point_count << "\n"
         << "DATA binary\n";
  return output.good();
}

bool writePoint(std::ofstream &output, float x, float y, float z, uint8_t r, uint8_t g, uint8_t b)
{
  const uint32_t rgb = (static_cast<uint32_t>(r) << 16) |
                       (static_cast<uint32_t>(g) << 8) |
                       static_cast<uint32_t>(b);
  output.write(reinterpret_cast<const char *>(&x), sizeof(x));
  output.write(reinterpret_cast<const char *>(&y), sizeof(y));
  output.write(reinterpret_cast<const char *>(&z), sizeof(z));
  output.write(reinterpret_cast<const char *>(&rgb), sizeof(rgb));
  return output.good();
}

bool isFinite(const Eigen::Vector3d &point)
{
  return std::isfinite(point.x()) && std::isfinite(point.y()) && std::isfinite(point.z());
}
}  // namespace

bool reprojectDenseRgbMap(const DenseRgbReprojectConfig &config, std::string *error)
{
  if (config.cache_dir.empty() || config.raw_keyframe_pose_path.empty() ||
      config.optimized_keyframe_pose_path.empty() || config.output_path.empty())
  {
    if (error) *error = "cache_dir, raw_keyframe_pose_path, optimized_keyframe_pose_path and output_path are required.";
    return false;
  }
  if (config.voxel_leaf_m < 0.0)
  {
    if (error) *error = "voxel_leaf_m must be zero or positive.";
    return false;
  }

  std::vector<PosePair> pose_pairs;
  std::vector<CachedFrame> frames;
  if (!loadPosePairs(config, &pose_pairs, error) || !loadManifest(config.cache_dir, &frames, error)) return false;

  const std::string output_dir = parentDirectory(config.output_path);
  if (!output_dir.empty() && !createDirectoryRecursively(output_dir))
  {
    if (error) *error = "Cannot create output directory: " + output_dir + " (" + std::strerror(errno) + ")";
    return false;
  }

  std::cout << "[dense-rgb] Loaded " << frames.size() << " RGB batches and " << pose_pairs.size()
            << " raw/final keyframe pose pairs." << std::endl;

  if (config.voxel_leaf_m == 0.0)
  {
    uint64_t source_points = 0;
    for (std::size_t frame_index = 0; frame_index < frames.size(); ++frame_index)
    {
      const CachedFrame &frame = frames[frame_index];
      pcl::PointCloud<pcl::PointXYZRGB> cloud;
      if (pcl::io::loadPCDFile(frame.path, cloud) != 0)
      {
        if (error) *error = "Cannot read cached RGB frame: " + frame.path;
        return false;
      }
      source_points += static_cast<uint64_t>(cloud.size());
      if ((frame_index + 1) % 500 == 0 || frame_index + 1 == frames.size())
      {
        std::cout << "[dense-rgb] Scanned " << (frame_index + 1) << "/" << frames.size()
                  << " RGB batches for output size." << std::endl;
      }
    }

    std::ofstream output(config.output_path, std::ios::binary | std::ios::trunc);
    if (!output.is_open() || !writePcdHeader(output, source_points))
    {
      if (error) *error = "Cannot create output PCD: " + config.output_path;
      return false;
    }

    std::size_t lower_index = 0;
    uint64_t written_points = 0;
    for (std::size_t frame_index = 0; frame_index < frames.size(); ++frame_index)
    {
      const CachedFrame &frame = frames[frame_index];
      const Eigen::Isometry3d correction = interpolateCorrection(pose_pairs, frame.timestamp, &lower_index);
      pcl::PointCloud<pcl::PointXYZRGB> cloud;
      if (pcl::io::loadPCDFile(frame.path, cloud) != 0)
      {
        if (error) *error = "Cannot read cached RGB frame: " + frame.path;
        return false;
      }
      for (const pcl::PointXYZRGB &point : cloud.points)
      {
        const Eigen::Vector3d corrected = correction * Eigen::Vector3d(point.x, point.y, point.z);
        if (!isFinite(corrected))
        {
          if (error) *error = "Encountered non-finite point while reprojecting: " + frame.path;
          return false;
        }
        if (!writePoint(output, static_cast<float>(corrected.x()), static_cast<float>(corrected.y()),
                        static_cast<float>(corrected.z()), point.r, point.g, point.b))
        {
          if (error) *error = "Write failure for output PCD: " + config.output_path;
          return false;
        }
        ++written_points;
      }
      if ((frame_index + 1) % 500 == 0 || frame_index + 1 == frames.size())
      {
        std::cout << "[dense-rgb] Reprojected " << (frame_index + 1) << "/" << frames.size()
                  << " RGB batches." << std::endl;
      }
    }
    std::cout << "[dense-rgb] Wrote " << config.output_path << " with " << written_points
              << " points (no downsampling)." << std::endl;
    return true;
  }

  std::unordered_map<VoxelKey, VoxelAccumulator, VoxelKeyHash> voxels;
  std::size_t lower_index = 0;
  uint64_t source_points = 0;
  for (std::size_t frame_index = 0; frame_index < frames.size(); ++frame_index)
  {
    const CachedFrame &frame = frames[frame_index];
    const Eigen::Isometry3d correction = interpolateCorrection(pose_pairs, frame.timestamp, &lower_index);
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    if (pcl::io::loadPCDFile(frame.path, cloud) != 0)
    {
      if (error) *error = "Cannot read cached RGB frame: " + frame.path;
      return false;
    }
    source_points += static_cast<uint64_t>(cloud.size());
    for (const pcl::PointXYZRGB &point : cloud.points)
    {
      const Eigen::Vector3d corrected = correction * Eigen::Vector3d(point.x, point.y, point.z);
      if (!isFinite(corrected)) continue;
      const VoxelKey key{
          static_cast<int64_t>(std::floor(corrected.x() / config.voxel_leaf_m)),
          static_cast<int64_t>(std::floor(corrected.y() / config.voxel_leaf_m)),
          static_cast<int64_t>(std::floor(corrected.z() / config.voxel_leaf_m))};
      VoxelAccumulator &voxel = voxels[key];
      voxel.x += corrected.x();
      voxel.y += corrected.y();
      voxel.z += corrected.z();
      voxel.r += point.r;
      voxel.g += point.g;
      voxel.b += point.b;
      ++voxel.count;
    }
    if ((frame_index + 1) % 500 == 0 || frame_index + 1 == frames.size())
    {
      std::cout << "[dense-rgb] Reprojected " << (frame_index + 1) << "/" << frames.size()
                << " RGB batches into voxels." << std::endl;
    }
  }
  if (voxels.empty())
  {
    if (error) *error = "No finite RGB points remained after reprojection.";
    return false;
  }

  std::ofstream output(config.output_path, std::ios::binary | std::ios::trunc);
  if (!output.is_open() || !writePcdHeader(output, static_cast<uint64_t>(voxels.size())))
  {
    if (error) *error = "Cannot create output PCD: " + config.output_path;
    return false;
  }
  for (const std::pair<const VoxelKey, VoxelAccumulator> &entry : voxels)
  {
    const VoxelAccumulator &voxel = entry.second;
    const double inverse_count = 1.0 / static_cast<double>(voxel.count);
    const uint8_t r = static_cast<uint8_t>(std::round(std::max(0.0, std::min(255.0, voxel.r * inverse_count))));
    const uint8_t g = static_cast<uint8_t>(std::round(std::max(0.0, std::min(255.0, voxel.g * inverse_count))));
    const uint8_t b = static_cast<uint8_t>(std::round(std::max(0.0, std::min(255.0, voxel.b * inverse_count))));
    if (!writePoint(output, static_cast<float>(voxel.x * inverse_count), static_cast<float>(voxel.y * inverse_count),
                    static_cast<float>(voxel.z * inverse_count), r, g, b))
    {
      if (error) *error = "Write failure for output PCD: " + config.output_path;
      return false;
    }
  }
  std::cout << "[dense-rgb] Wrote " << config.output_path << " with " << voxels.size()
            << " points (" << source_points << " source points, voxel leaf " << config.voxel_leaf_m << " m)." << std::endl;
  return true;
}
