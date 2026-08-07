#include <octomap/OcTree.h>
#include <octomap/Pointcloud.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace fs = std::filesystem;

namespace
{
struct Options
{
  std::string keyframe_dir;
  std::string optimized_pose_path;
  std::string output_dir;
  double resolution = 0.10;
  double terrain_resolution = 0.25;
  double ground_seed_resolution = 0.50;
  double terrain_radius = 1.25;
  double ground_quantile = 0.20;
  double ground_min_below_sensor = 0.10;
  double ground_outlier_threshold = 0.15;
  double ground_max_slope_deg = 30.0;
  double min_ground_spread = 0.20;
  int min_ground_observations = 12;
  int min_ground_cells = 6;
  int min_ground_inliers = 6;
  double terrain_gap_fill_radius = 0.75;
  double terrain_gap_fill_max_rms = 0.05;
  double terrain_gap_fill_coverage = 0.10;
  double fallback_terrain_radius = 2.00;
  double fallback_max_rms = 0.05;
  double fallback_coverage = 0.25;
  int fallback_min_ground_cells = 10;
  int min_fallback_free_keyframes = 2;
  double fallback_free_ray_voxel = 0.25;
  double obstacle_min_height = 0.05;
  double obstacle_max_height = 0.65;
  double obstacle_min_occupancy = 0.80;
  int min_obstacle_keyframes = 2;
  double obstacle_support_tolerance = 0.15;
  double max_slope_deg = 15.0;
  double max_roughness = 0.20;
  double max_ray_range = 20.0;
  double padding = 1.0;
};

struct TimedPose
{
  double timestamp = 0.0;
  Eigen::Isometry3d T_map_imu = Eigen::Isometry3d::Identity();
};

enum class CellState : unsigned char
{
  kUnknown = 205,
  kFree = 254,
  kOccupied = 0,
};

struct GroundKey
{
  int64_t x = 0;
  int64_t y = 0;

  bool operator==(const GroundKey &other) const { return x == other.x && y == other.y; }
};

struct GroundKeyHash
{
  std::size_t operator()(const GroundKey &key) const
  {
    return std::hash<int64_t>()(key.x) ^ (std::hash<int64_t>()(key.y) << 1);
  }
};

struct VoxelKey
{
  int64_t x = 0;
  int64_t y = 0;
  int64_t z = 0;

  bool operator==(const VoxelKey &other) const { return x == other.x && y == other.y && z == other.z; }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey &key) const
  {
    const std::size_t first = std::hash<int64_t>()(key.x);
    const std::size_t second = std::hash<int64_t>()(key.y);
    const std::size_t third = std::hash<int64_t>()(key.z);
    return first ^ (second << 1) ^ (third << 2);
  }
};

struct TerrainSample
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct PlaneModel
{
  double slope_x = 0.0;
  double slope_y = 0.0;
  double ground_z = 0.0;
  double rms = 0.0;
  int inlier_count = 0;
};

struct TerrainCell
{
  bool valid = false;
  double ground_z = 0.0;
  double slope_deg = 0.0;
  double rms = 0.0;
  int inlier_count = 0;
  bool interpolated = false;
  bool fallback = false;
};

struct TerrainGrid
{
  double resolution = 0.25;
  double origin_x = 0.0;
  double origin_y = 0.0;
  int width = 0;
  int height = 0;
  std::vector<TerrainCell> cells;

  const TerrainCell *at(const double x, const double y) const
  {
    const int column = static_cast<int>(std::floor((x - origin_x) / resolution));
    const int row = static_cast<int>(std::floor((y - origin_y) / resolution));
    if (column < 0 || row < 0 || column >= width || row >= height) return nullptr;
    return &cells[static_cast<std::size_t>(row) * width + column];
  }
};

void printUsage(const char *program)
{
  std::cout
      << "Usage:\n  " << program
      << " --keyframes DIR --poses FILE --output DIR [options]\n\n"
      << "Builds a static 2D ROS map from FAST-LIVO2 loop-optimized keyframes.\n"
      << "The keyframe directory must contain metadata.yaml and <id>.pcd files;\n"
      << "the pose file must be optimized_keyframe_poses_imu.txt.\n\n"
      << "Options:\n"
      << "  --resolution METERS           OctoMap and PGM resolution (default: 0.10)\n"
      << "  --terrain-resolution METERS   Local-ground grid resolution (default: 0.25)\n"
      << "  --ground-seed-resolution M   Coarse grid used to build ground seeds (default: 0.50)\n"
      << "  --terrain-radius METERS       Local plane-fit radius (default: 1.25)\n"
      << "  --ground-quantile VALUE       Low-point quantile used as ground seed (default: 0.20)\n"
      << "  --ground-min-below-sensor M  Ignore candidate points too close to/above LiDAR (default: 0.10)\n"
      << "  --ground-outlier-threshold M  Residual kept by robust plane fit (default: 0.15)\n"
      << "  --ground-max-slope-deg DEG    Reject implausibly steep ground hypotheses (default: 30)\n"
      << "  --min-ground-spread M         Required two-dimensional support spread (default: 0.20)\n"
      << "  --min-ground-observations N  Minimum point observations per terrain seed (default: 12)\n"
      << "  --min-ground-cells N         Minimum seed cells per local plane (default: 6)\n"
      << "  --min-ground-inliers N       Minimum inliers after robust plane fit (default: 6)\n"
      << "  --terrain-gap-fill-radius M  Fill only short gaps enclosed by terrain (default: 0.75)\n"
      << "  --terrain-gap-fill-max-rms M Maximum RMS accepted for a filled gap (default: 0.05)\n"
      << "  --terrain-gap-fill-coverage M Required support on each XY side (default: 0.10)\n"
      << "  --fallback-terrain-radius M  Larger radius for failed terrain cells (default: 2.00)\n"
      << "  --fallback-max-rms M         Maximum RMS accepted by fallback terrain (default: 0.05)\n"
      << "  --fallback-coverage M        Required seed support on each XY side (default: 0.25)\n"
      << "  --fallback-min-ground-cells N Minimum ground seeds for fallback terrain (default: 10)\n"
      << "  --min-fallback-free-keyframes N Independent ground/free observations required for fallback free cells (default: 2)\n"
      << "  --fallback-free-ray-voxel M Local voxel used to decimate fallback free rays (default: 0.25)\n"
      << "  --max-slope-deg DEGREES       Mark steeper terrain untraversable (default: 15)\n"
      << "  --max-roughness METERS        Mark rough terrain untraversable (default: 0.20)\n"
      << "  --obstacle-min-height METERS Height above ground to begin obstacles (default: 0.05)\n"
      << "  --obstacle-max-height METERS Height above ground to end obstacles (default: 0.65)\n"
      << "  Unconfirmed obstacle evidence is retained only above the per-cell adaptive\n"
      << "  height max(obstacle-min-height, 0.5 * resolution + terrain RMS).\n"
      << "  --obstacle-min-occupancy P   Require this OctoMap confidence for an obstacle (default: 0.80)\n"
      << "  --min-obstacle-keyframes N  Independent keyframes required for an obstacle (default: 2)\n"
      << "  --obstacle-support-tolerance M Neighbor tolerance for cross-frame support (default: 0.15)\n"
      << "  --max-ray-range METERS       Ignore ray endpoints beyond this range (default: 20.0)\n"
      << "  --padding METERS             Empty border around map bounds (default: 1.0)\n"
      << "  --help\n";
}

double parseDouble(const std::string &value, const char *name)
{
  char *end = nullptr;
  const double parsed = std::strtod(value.c_str(), &end);
  if (end == value.c_str() || *end != '\0' || !std::isfinite(parsed))
  {
    throw std::runtime_error(std::string("Invalid ") + name + ": " + value);
  }
  return parsed;
}

int parsePositiveInt(const std::string &value, const char *name)
{
  char *end = nullptr;
  const long parsed = std::strtol(value.c_str(), &end, 10);
  if (end == value.c_str() || *end != '\0' || parsed < 1 || parsed > std::numeric_limits<int>::max())
  {
    throw std::runtime_error(std::string("Invalid ") + name + ": " + value);
  }
  return static_cast<int>(parsed);
}

Options parseOptions(int argc, char **argv)
{
  Options options;
  for (int index = 1; index < argc; ++index)
  {
    const std::string option(argv[index]);
    if (option == "--help")
    {
      printUsage(argv[0]);
      std::exit(0);
    }
    if (index + 1 >= argc)
    {
      throw std::runtime_error("Missing value for " + option);
    }
    const std::string value(argv[++index]);
    if (option == "--keyframes") options.keyframe_dir = value;
    else if (option == "--poses") options.optimized_pose_path = value;
    else if (option == "--output") options.output_dir = value;
    else if (option == "--resolution") options.resolution = parseDouble(value, "--resolution");
    else if (option == "--terrain-resolution") options.terrain_resolution = parseDouble(value, "--terrain-resolution");
    else if (option == "--ground-seed-resolution") options.ground_seed_resolution = parseDouble(value, "--ground-seed-resolution");
    else if (option == "--terrain-radius") options.terrain_radius = parseDouble(value, "--terrain-radius");
    else if (option == "--ground-quantile") options.ground_quantile = parseDouble(value, "--ground-quantile");
    else if (option == "--ground-min-below-sensor") options.ground_min_below_sensor = parseDouble(value, "--ground-min-below-sensor");
    else if (option == "--ground-outlier-threshold") options.ground_outlier_threshold = parseDouble(value, "--ground-outlier-threshold");
    else if (option == "--ground-max-slope-deg") options.ground_max_slope_deg = parseDouble(value, "--ground-max-slope-deg");
    else if (option == "--min-ground-spread") options.min_ground_spread = parseDouble(value, "--min-ground-spread");
    else if (option == "--min-ground-observations") options.min_ground_observations = parsePositiveInt(value, "--min-ground-observations");
    else if (option == "--min-ground-cells") options.min_ground_cells = parsePositiveInt(value, "--min-ground-cells");
    else if (option == "--min-ground-inliers") options.min_ground_inliers = parsePositiveInt(value, "--min-ground-inliers");
    else if (option == "--terrain-gap-fill-radius") options.terrain_gap_fill_radius = parseDouble(value, "--terrain-gap-fill-radius");
    else if (option == "--terrain-gap-fill-max-rms") options.terrain_gap_fill_max_rms = parseDouble(value, "--terrain-gap-fill-max-rms");
    else if (option == "--terrain-gap-fill-coverage") options.terrain_gap_fill_coverage = parseDouble(value, "--terrain-gap-fill-coverage");
    else if (option == "--fallback-terrain-radius") options.fallback_terrain_radius = parseDouble(value, "--fallback-terrain-radius");
    else if (option == "--fallback-max-rms") options.fallback_max_rms = parseDouble(value, "--fallback-max-rms");
    else if (option == "--fallback-coverage") options.fallback_coverage = parseDouble(value, "--fallback-coverage");
    else if (option == "--fallback-min-ground-cells") options.fallback_min_ground_cells = parsePositiveInt(value, "--fallback-min-ground-cells");
    else if (option == "--min-fallback-free-keyframes") options.min_fallback_free_keyframes = parsePositiveInt(value, "--min-fallback-free-keyframes");
    else if (option == "--fallback-free-ray-voxel") options.fallback_free_ray_voxel = parseDouble(value, "--fallback-free-ray-voxel");
    else if (option == "--obstacle-min-height") options.obstacle_min_height = parseDouble(value, "--obstacle-min-height");
    else if (option == "--obstacle-max-height") options.obstacle_max_height = parseDouble(value, "--obstacle-max-height");
    else if (option == "--obstacle-min-occupancy") options.obstacle_min_occupancy = parseDouble(value, "--obstacle-min-occupancy");
    else if (option == "--min-obstacle-keyframes") options.min_obstacle_keyframes = parsePositiveInt(value, "--min-obstacle-keyframes");
    else if (option == "--obstacle-support-tolerance") options.obstacle_support_tolerance = parseDouble(value, "--obstacle-support-tolerance");
    else if (option == "--max-slope-deg") options.max_slope_deg = parseDouble(value, "--max-slope-deg");
    else if (option == "--max-roughness") options.max_roughness = parseDouble(value, "--max-roughness");
    else if (option == "--max-ray-range") options.max_ray_range = parseDouble(value, "--max-ray-range");
    else if (option == "--padding") options.padding = parseDouble(value, "--padding");
    else throw std::runtime_error("Unknown option: " + option);
  }

  if (options.keyframe_dir.empty() || options.optimized_pose_path.empty() || options.output_dir.empty())
  {
    throw std::runtime_error("--keyframes, --poses and --output are required.");
  }
  if (options.resolution <= 0.0 || options.terrain_resolution <= 0.0 || options.ground_seed_resolution <= 0.0 ||
      options.terrain_radius <= 0.0 || options.ground_quantile <= 0.0 || options.ground_quantile >= 0.5 ||
      options.ground_min_below_sensor < 0.0 || options.ground_outlier_threshold <= 0.0 ||
      options.ground_max_slope_deg <= 0.0 || options.ground_max_slope_deg >= 89.0 || options.min_ground_spread <= 0.0 ||
      options.min_ground_observations < 1 || options.min_ground_cells < 3 || options.min_ground_inliers < 3 ||
      options.terrain_gap_fill_radius <= 0.0 || options.terrain_gap_fill_max_rms <= 0.0 ||
      options.terrain_gap_fill_coverage <= 0.0 ||
      options.fallback_terrain_radius <= options.terrain_radius || options.fallback_max_rms <= 0.0 ||
      options.fallback_coverage <= 0.0 || options.fallback_min_ground_cells < options.min_ground_cells ||
      options.min_fallback_free_keyframes < 1 || options.fallback_free_ray_voxel <= 0.0 ||
      options.obstacle_min_height < 0.0 ||
      options.obstacle_max_height <= options.obstacle_min_height ||
      options.max_slope_deg <= 0.0 ||
      options.max_roughness <= 0.0 || options.obstacle_min_occupancy <= 0.5 || options.obstacle_min_occupancy > 1.0 ||
      options.min_obstacle_keyframes < 1 || options.obstacle_support_tolerance < 0.0 ||
      options.max_ray_range <= 0.0 || options.padding < 0.0)
  {
    throw std::runtime_error("Map resolution/ranges are invalid.");
  }
  if (options.ground_max_slope_deg < options.max_slope_deg)
  {
    throw std::runtime_error("--ground-max-slope-deg must be at least --max-slope-deg.");
  }
  if (options.terrain_gap_fill_coverage > options.terrain_gap_fill_radius)
  {
    throw std::runtime_error("--terrain-gap-fill-coverage must not exceed --terrain-gap-fill-radius.");
  }
  if (options.fallback_coverage > options.fallback_terrain_radius)
  {
    throw std::runtime_error("--fallback-coverage must not exceed --fallback-terrain-radius.");
  }
  return options;
}

std::vector<double> numericValues(std::string value)
{
  for (char &character : value)
  {
    const bool numeric = (character >= '0' && character <= '9') || character == '-' || character == '+' ||
                         character == '.' || character == 'e' || character == 'E';
    if (!numeric) character = ' ';
  }
  std::vector<double> values;
  std::istringstream stream(value);
  double number = 0.0;
  while (stream >> number) values.push_back(number);
  return values;
}

Eigen::Isometry3d loadImuLidarExtrinsic(const fs::path &metadata_path)
{
  std::ifstream input(metadata_path);
  if (!input.is_open()) throw std::runtime_error("Cannot open " + metadata_path.string());

  std::vector<double> translation;
  std::vector<double> rotation;
  std::string line;
  while (std::getline(input, line))
  {
    const std::size_t separator = line.find(':');
    if (separator == std::string::npos) continue;
    const std::string key = line.substr(0, separator);
    if (key == "T_imu_lidar_translation") translation = numericValues(line.substr(separator + 1));
    if (key == "T_imu_lidar_rotation_row_major") rotation = numericValues(line.substr(separator + 1));
  }
  if (translation.size() != 3 || rotation.size() != 9)
  {
    throw std::runtime_error("metadata.yaml does not provide a valid T_imu_lidar transform.");
  }

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(translation[0], translation[1], translation[2]);
  transform.linear() << rotation[0], rotation[1], rotation[2],
                        rotation[3], rotation[4], rotation[5],
                        rotation[6], rotation[7], rotation[8];
  return transform;
}

std::map<int, TimedPose> loadOptimizedPoses(const fs::path &pose_path)
{
  std::ifstream input(pose_path);
  if (!input.is_open()) throw std::runtime_error("Cannot open " + pose_path.string());

  std::map<int, TimedPose> poses;
  std::string line;
  std::size_t line_number = 0;
  while (std::getline(input, line))
  {
    ++line_number;
    if (line.empty() || line.front() == '#') continue;
    int id = -1;
    TimedPose pose;
    double tx, ty, tz, qx, qy, qz, qw;
    std::istringstream stream(line);
    if (!(stream >> id >> pose.timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw))
    {
      throw std::runtime_error("Invalid optimized pose at line " + std::to_string(line_number));
    }
    Eigen::Quaterniond quaternion(qw, qx, qy, qz);
    if (quaternion.norm() < 1e-12) throw std::runtime_error("Zero quaternion at line " + std::to_string(line_number));
    quaternion.normalize();
    pose.T_map_imu.linear() = quaternion.toRotationMatrix();
    pose.T_map_imu.translation() = Eigen::Vector3d(tx, ty, tz);
    if (!poses.emplace(id, pose).second) throw std::runtime_error("Duplicate keyframe id " + std::to_string(id));
  }
  if (poses.empty()) throw std::runtime_error("No optimized poses found in " + pose_path.string());
  return poses;
}

fs::path keyframePath(const fs::path &directory, int id)
{
  std::ostringstream name;
  name << std::setw(6) << std::setfill('0') << id << ".pcd";
  return directory / name.str();
}

pcl::PointCloud<pcl::PointXYZ> loadCloud(const fs::path &path)
{
  pcl::PCLPointCloud2 blob;
  if (pcl::io::loadPCDFile(path.string(), blob) != 0)
  {
    throw std::runtime_error("Cannot read keyframe PCD " + path.string());
  }
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromPCLPointCloud2(blob, cloud);
  return cloud;
}

bool finite(const pcl::PointXYZ &point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

GroundKey groundKeyFor(const double x, const double y, const double resolution)
{
  return GroundKey{static_cast<int64_t>(std::floor(x / resolution)),
                   static_cast<int64_t>(std::floor(y / resolution))};
}

bool hasSpatialSupport(const std::vector<TerrainSample> &samples, const double minimum_spread)
{
  if (samples.size() < 3) return false;
  double mean_x = 0.0;
  double mean_y = 0.0;
  for (const TerrainSample &sample : samples)
  {
    mean_x += sample.x;
    mean_y += sample.y;
  }
  mean_x /= static_cast<double>(samples.size());
  mean_y /= static_cast<double>(samples.size());

  double xx = 0.0;
  double xy = 0.0;
  double yy = 0.0;
  for (const TerrainSample &sample : samples)
  {
    const double dx = sample.x - mean_x;
    const double dy = sample.y - mean_y;
    xx += dx * dx;
    xy += dx * dy;
    yy += dy * dy;
  }
  const double inverse_count = 1.0 / static_cast<double>(samples.size());
  xx *= inverse_count;
  xy *= inverse_count;
  yy *= inverse_count;
  const double discriminant = std::max(0.0, (xx - yy) * (xx - yy) + 4.0 * xy * xy);
  const double minor_eigenvalue = 0.5 * (xx + yy - std::sqrt(discriminant));
  return std::sqrt(std::max(0.0, minor_eigenvalue)) >= minimum_spread;
}

bool slopeIsPlausible(const PlaneModel &model, const double max_slope_deg)
{
  const double slope_deg = std::atan(std::hypot(model.slope_x, model.slope_y)) * 180.0 / std::acos(-1.0);
  return slope_deg <= max_slope_deg;
}

bool fitPlaneAt(const std::vector<TerrainSample> &samples, const double query_x, const double query_y,
                const double outlier_threshold, const int minimum_inliers, const double max_slope_deg,
                const double minimum_spread, PlaneModel *model)
{
  if (model == nullptr || static_cast<int>(samples.size()) < minimum_inliers) return false;
  const auto solve = [query_x, query_y](const std::vector<TerrainSample> &input, PlaneModel *output) {
    Eigen::Matrix3d normal = Eigen::Matrix3d::Zero();
    Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
    for (const TerrainSample &sample : input)
    {
      const Eigen::Vector3d row(sample.x - query_x, sample.y - query_y, 1.0);
      normal.noalias() += row * row.transpose();
      rhs.noalias() += row * sample.z;
    }
    if (std::abs(normal.determinant()) < 1e-10) return false;
    const Eigen::Vector3d solution = normal.ldlt().solve(rhs);
    if (!solution.allFinite()) return false;
    output->slope_x = solution.x();
    output->slope_y = solution.y();
    output->ground_z = solution.z();
    return true;
  };

  // The former implementation fitted all low points first and then accepted
  // any three remaining points.  Three arbitrary points always form a plane,
  // which made vertical surfaces look like very low-RMS terrain.  Generate
  // bounded RANSAC-style hypotheses instead and retain the widest supported
  // near-horizontal surface.
  constexpr std::size_t kMaxHypotheses = 96;
  std::vector<TerrainSample> best_inliers;
  double best_error = std::numeric_limits<double>::infinity();
  const std::size_t hypothesis_count = std::min(
      kMaxHypotheses, samples.size() * (samples.size() - 1) * (samples.size() - 2) / 6);
  for (std::size_t hypothesis_index = 0; hypothesis_index < hypothesis_count; ++hypothesis_index)
  {
    // Deterministic pseudo-random sampling avoids letting the fixed grid scan
    // order bias every hypothesis toward the same first seed cell.
    uint64_t state = static_cast<uint64_t>(hypothesis_index + 1);
    const auto next_index = [&state, &samples]() {
      state = state * 6364136223846793005ULL + 1442695040888963407ULL;
      return static_cast<std::size_t>(state % samples.size());
    };
    const std::size_t first = next_index();
    std::size_t second = next_index();
    while (second == first) second = next_index();
    std::size_t third = next_index();
    while (third == first || third == second) third = next_index();

    const Eigen::Vector3d point_a(samples[first].x, samples[first].y, samples[first].z);
    const Eigen::Vector3d point_b(samples[second].x, samples[second].y, samples[second].z);
    const Eigen::Vector3d point_c(samples[third].x, samples[third].y, samples[third].z);
    const Eigen::Vector3d normal = (point_b - point_a).cross(point_c - point_a);
    if (normal.squaredNorm() < 1e-12 || std::abs(normal.z()) < 1e-8) continue;

    PlaneModel hypothesis;
    hypothesis.slope_x = -normal.x() / normal.z();
    hypothesis.slope_y = -normal.y() / normal.z();
    hypothesis.ground_z = samples[first].z - hypothesis.slope_x * (samples[first].x - query_x) -
                          hypothesis.slope_y * (samples[first].y - query_y);
    if (!slopeIsPlausible(hypothesis, max_slope_deg)) continue;

    std::vector<TerrainSample> inliers;
    inliers.reserve(samples.size());
    double absolute_error = 0.0;
    for (const TerrainSample &sample : samples)
    {
      const double predicted = hypothesis.slope_x * (sample.x - query_x) +
                               hypothesis.slope_y * (sample.y - query_y) + hypothesis.ground_z;
      const double residual = std::abs(sample.z - predicted);
      if (residual > outlier_threshold) continue;
      inliers.push_back(sample);
      absolute_error += residual;
    }
    if (static_cast<int>(inliers.size()) < minimum_inliers || !hasSpatialSupport(inliers, minimum_spread)) continue;
    if (inliers.size() > best_inliers.size() ||
        (inliers.size() == best_inliers.size() && absolute_error < best_error))
    {
      best_inliers = std::move(inliers);
      best_error = absolute_error;
    }
  }
  if (static_cast<int>(best_inliers.size()) < minimum_inliers) return false;

  PlaneModel refined;
  if (!solve(best_inliers, &refined) || !slopeIsPlausible(refined, max_slope_deg)) return false;
  std::vector<TerrainSample> refined_inliers;
  refined_inliers.reserve(best_inliers.size());
  for (const TerrainSample &sample : samples)
  {
    const double predicted = refined.slope_x * (sample.x - query_x) +
                             refined.slope_y * (sample.y - query_y) + refined.ground_z;
    if (std::abs(sample.z - predicted) <= outlier_threshold) refined_inliers.push_back(sample);
  }
  if (static_cast<int>(refined_inliers.size()) < minimum_inliers ||
      !hasSpatialSupport(refined_inliers, minimum_spread)) return false;
  if (!solve(refined_inliers, model) || !slopeIsPlausible(*model, max_slope_deg)) return false;

  double squared_error = 0.0;
  for (const TerrainSample &sample : refined_inliers)
  {
    const double predicted = model->slope_x * (sample.x - query_x) +
                             model->slope_y * (sample.y - query_y) + model->ground_z;
    const double residual = sample.z - predicted;
    squared_error += residual * residual;
  }
  model->rms = std::sqrt(squared_error / static_cast<double>(refined_inliers.size()));
  model->inlier_count = static_cast<int>(refined_inliers.size());
  return true;
}

using GroundObservations = std::unordered_map<GroundKey, std::vector<float>, GroundKeyHash>;
using GroundSeeds = std::unordered_map<GroundKey, double, GroundKeyHash>;

GroundSeeds buildGroundSeeds(const Options &options, const GroundObservations &ground_observations)
{
  GroundSeeds seeds;
  seeds.reserve(ground_observations.size());
  for (const auto &entry : ground_observations)
  {
    std::vector<float> heights = entry.second;
    if (static_cast<int>(heights.size()) < options.min_ground_observations) continue;
    const std::size_t index = static_cast<std::size_t>(std::floor(options.ground_quantile * (heights.size() - 1)));
    std::nth_element(heights.begin(), heights.begin() + index, heights.end());
    seeds.emplace(entry.first, heights[index]);
  }
  if (seeds.empty()) throw std::runtime_error("No local-ground seed cells could be estimated from the keyframes.");
  return seeds;
}

TerrainGrid buildTerrainGrid(const Options &options, const GroundSeeds &seeds,
    const double map_origin_x, const double map_origin_y, const int map_width, const int map_height)
{
  TerrainGrid terrain;
  terrain.resolution = options.terrain_resolution;
  terrain.origin_x = std::floor(map_origin_x / terrain.resolution) * terrain.resolution;
  terrain.origin_y = std::floor(map_origin_y / terrain.resolution) * terrain.resolution;
  terrain.width = std::max(1, static_cast<int>(std::ceil(
      (map_origin_x + map_width * options.resolution - terrain.origin_x) / terrain.resolution)));
  terrain.height = std::max(1, static_cast<int>(std::ceil(
      (map_origin_y + map_height * options.resolution - terrain.origin_y) / terrain.resolution)));
  terrain.cells.resize(static_cast<std::size_t>(terrain.width) * terrain.height);

  const int radius_cells = static_cast<int>(std::ceil(options.terrain_radius / options.ground_seed_resolution));
  const double radius_squared = options.terrain_radius * options.terrain_radius;
  for (int row = 0; row < terrain.height; ++row)
  {
    for (int column = 0; column < terrain.width; ++column)
    {
      const double x = terrain.origin_x + (static_cast<double>(column) + 0.5) * terrain.resolution;
      const double y = terrain.origin_y + (static_cast<double>(row) + 0.5) * terrain.resolution;
      const GroundKey center = groundKeyFor(x, y, options.ground_seed_resolution);
      std::vector<TerrainSample> samples;
      samples.reserve(static_cast<std::size_t>((2 * radius_cells + 1) * (2 * radius_cells + 1)));
      for (int dy = -radius_cells; dy <= radius_cells; ++dy)
      {
        for (int dx = -radius_cells; dx <= radius_cells; ++dx)
        {
          const GroundKey key{center.x + dx, center.y + dy};
          const auto seed = seeds.find(key);
          if (seed == seeds.end()) continue;
          const double sample_x = (static_cast<double>(key.x) + 0.5) * options.ground_seed_resolution;
          const double sample_y = (static_cast<double>(key.y) + 0.5) * options.ground_seed_resolution;
          const double delta_x = sample_x - x;
          const double delta_y = sample_y - y;
          if (delta_x * delta_x + delta_y * delta_y > radius_squared) continue;
          samples.push_back(TerrainSample{sample_x, sample_y, seed->second});
        }
      }
      if (static_cast<int>(samples.size()) < options.min_ground_cells) continue;

      PlaneModel plane;
      if (!fitPlaneAt(samples, x, y, options.ground_outlier_threshold, options.min_ground_inliers,
                      options.ground_max_slope_deg, options.min_ground_spread, &plane))
      {
        continue;
      }
      TerrainCell &cell = terrain.cells[static_cast<std::size_t>(row) * terrain.width + column];
      cell.valid = true;
      cell.ground_z = plane.ground_z;
      cell.slope_deg = std::atan(std::sqrt(plane.slope_x * plane.slope_x + plane.slope_y * plane.slope_y)) *
                       180.0 / std::acos(-1.0);
      cell.rms = plane.rms;
      cell.inlier_count = plane.inlier_count;
    }
  }
  return terrain;
}

bool enclosesQuery(const std::vector<TerrainSample> &samples, const double query_x, const double query_y,
                   const double coverage)
{
  bool left = false;
  bool right = false;
  bool below = false;
  bool above = false;
  for (const TerrainSample &sample : samples)
  {
    left = left || sample.x <= query_x - coverage;
    right = right || sample.x >= query_x + coverage;
    below = below || sample.y <= query_y - coverage;
    above = above || sample.y >= query_y + coverage;
  }
  return left && right && below && above;
}

std::size_t fillTerrainGaps(const Options &options, TerrainGrid *terrain)
{
  if (terrain == nullptr) return 0;

  // Fill from the original, directly observed terrain only.  This deliberately
  // prevents repeated passes from growing a narrow road model into the large
  // unobserved centre of the loop.
  const std::vector<TerrainCell> observed_cells = terrain->cells;
  const int radius_cells = static_cast<int>(std::ceil(options.terrain_gap_fill_radius / terrain->resolution));
  const double radius_squared = options.terrain_gap_fill_radius * options.terrain_gap_fill_radius;
  std::size_t filled = 0;
  for (int row = 0; row < terrain->height; ++row)
  {
    for (int column = 0; column < terrain->width; ++column)
    {
      const std::size_t index = static_cast<std::size_t>(row) * terrain->width + column;
      if (observed_cells[index].valid) continue;

      const double x = terrain->origin_x + (static_cast<double>(column) + 0.5) * terrain->resolution;
      const double y = terrain->origin_y + (static_cast<double>(row) + 0.5) * terrain->resolution;
      std::vector<TerrainSample> samples;
      samples.reserve(static_cast<std::size_t>((2 * radius_cells + 1) * (2 * radius_cells + 1)));
      for (int dy = -radius_cells; dy <= radius_cells; ++dy)
      {
        const int neighbor_row = row + dy;
        if (neighbor_row < 0 || neighbor_row >= terrain->height) continue;
        for (int dx = -radius_cells; dx <= radius_cells; ++dx)
        {
          const int neighbor_column = column + dx;
          if (neighbor_column < 0 || neighbor_column >= terrain->width) continue;
          const TerrainCell &neighbor = observed_cells[static_cast<std::size_t>(neighbor_row) * terrain->width + neighbor_column];
          if (!neighbor.valid) continue;
          const double sample_x = terrain->origin_x + (static_cast<double>(neighbor_column) + 0.5) * terrain->resolution;
          const double sample_y = terrain->origin_y + (static_cast<double>(neighbor_row) + 0.5) * terrain->resolution;
          const double delta_x = sample_x - x;
          const double delta_y = sample_y - y;
          if (delta_x * delta_x + delta_y * delta_y > radius_squared) continue;
          samples.push_back(TerrainSample{sample_x, sample_y, neighbor.ground_z});
        }
      }
      if (static_cast<int>(samples.size()) < options.min_ground_inliers ||
          !enclosesQuery(samples, x, y, options.terrain_gap_fill_coverage))
      {
        continue;
      }

      PlaneModel plane;
      if (!fitPlaneAt(samples, x, y, options.ground_outlier_threshold, options.min_ground_inliers,
                      options.ground_max_slope_deg, options.min_ground_spread, &plane) ||
          plane.rms > options.terrain_gap_fill_max_rms)
      {
        continue;
      }
      TerrainCell &target = terrain->cells[index];
      target.valid = true;
      target.ground_z = plane.ground_z;
      target.slope_deg = std::atan(std::hypot(plane.slope_x, plane.slope_y)) * 180.0 / std::acos(-1.0);
      target.rms = plane.rms;
      target.inlier_count = plane.inlier_count;
      target.interpolated = true;
      ++filled;
    }
  }
  return filled;
}

std::size_t fillTerrainFallback(const Options &options, const GroundSeeds &seeds, TerrainGrid *terrain)
{
  if (terrain == nullptr) return 0;

  // This is a one-shot, seed-based fallback for cells that remain invalid
  // after the short-gap fill.  It never uses newly created fallback cells as
  // input, so it cannot grow an assumed road into an unobserved region.
  const int radius_cells = static_cast<int>(std::ceil(options.fallback_terrain_radius /
                                                       options.ground_seed_resolution));
  const double radius_squared = options.fallback_terrain_radius * options.fallback_terrain_radius;
  std::size_t filled = 0;
  for (int row = 0; row < terrain->height; ++row)
  {
    for (int column = 0; column < terrain->width; ++column)
    {
      const std::size_t index = static_cast<std::size_t>(row) * terrain->width + column;
      if (terrain->cells[index].valid) continue;
      const double x = terrain->origin_x + (static_cast<double>(column) + 0.5) * terrain->resolution;
      const double y = terrain->origin_y + (static_cast<double>(row) + 0.5) * terrain->resolution;
      const GroundKey center = groundKeyFor(x, y, options.ground_seed_resolution);
      std::vector<TerrainSample> samples;
      samples.reserve(static_cast<std::size_t>((2 * radius_cells + 1) * (2 * radius_cells + 1)));
      for (int dy = -radius_cells; dy <= radius_cells; ++dy)
      {
        for (int dx = -radius_cells; dx <= radius_cells; ++dx)
        {
          const GroundKey key{center.x + dx, center.y + dy};
          const auto seed = seeds.find(key);
          if (seed == seeds.end()) continue;
          const double sample_x = (static_cast<double>(key.x) + 0.5) * options.ground_seed_resolution;
          const double sample_y = (static_cast<double>(key.y) + 0.5) * options.ground_seed_resolution;
          const double delta_x = sample_x - x;
          const double delta_y = sample_y - y;
          if (delta_x * delta_x + delta_y * delta_y > radius_squared) continue;
          samples.push_back(TerrainSample{sample_x, sample_y, seed->second});
        }
      }
      if (static_cast<int>(samples.size()) < options.fallback_min_ground_cells ||
          !enclosesQuery(samples, x, y, options.fallback_coverage))
      {
        continue;
      }
      PlaneModel plane;
      if (!fitPlaneAt(samples, x, y, options.ground_outlier_threshold, options.min_ground_inliers,
                      options.ground_max_slope_deg, options.min_ground_spread, &plane) ||
          plane.rms > options.fallback_max_rms)
      {
        continue;
      }
      TerrainCell &target = terrain->cells[index];
      target.valid = true;
      target.ground_z = plane.ground_z;
      target.slope_deg = std::atan(std::hypot(plane.slope_x, plane.slope_y)) * 180.0 / std::acos(-1.0);
      target.rms = plane.rms;
      target.inlier_count = plane.inlier_count;
      target.fallback = true;
      ++filled;
    }
  }
  return filled;
}

std::vector<uint16_t> collectObstacleKeyframeSupport(
    const Options &options, const fs::path &keyframe_dir, const std::map<int, TimedPose> &optimized_poses,
    const Eigen::Isometry3d &T_imu_lidar, const TerrainGrid &terrain, const double origin_x,
    const double origin_y, const int width, const int height)
{
  const std::size_t total_cells = static_cast<std::size_t>(width) * height;
  std::vector<uint16_t> support(total_cells, 0);
  const int support_radius_cells = static_cast<int>(std::ceil(options.obstacle_support_tolerance / options.resolution));
  const double support_tolerance_squared = options.obstacle_support_tolerance * options.obstacle_support_tolerance;
  std::size_t processed = 0;
  for (const auto &entry : optimized_poses)
  {
    const int id = entry.first;
    const fs::path cloud_path = keyframePath(keyframe_dir, id);
    const Eigen::Isometry3d T_map_lidar = entry.second.T_map_imu * T_imu_lidar;
    const pcl::PointCloud<pcl::PointXYZ> cloud_lidar = loadCloud(cloud_path);

    // Each frame contributes at most one vote per target cell.  The small
    // neighborhood tolerance absorbs sub-voxel loop/registration offsets, but
    // a dense scanline from one frame still cannot cast more than one vote.
    std::unordered_set<std::size_t> frame_cells;
    frame_cells.reserve(cloud_lidar.points.size() / 8 + 1);
    for (const pcl::PointXYZ &point : cloud_lidar.points)
    {
      if (!finite(point)) continue;
      const Eigen::Vector3d transformed = T_map_lidar * Eigen::Vector3d(point.x, point.y, point.z);
      if (!std::isfinite(transformed.x()) || !std::isfinite(transformed.y()) || !std::isfinite(transformed.z())) continue;

      const int column = static_cast<int>(std::floor((transformed.x() - origin_x) / options.resolution));
      const int row = static_cast<int>(std::floor((transformed.y() - origin_y) / options.resolution));
      if (column < 0 || row < 0 || column >= width || row >= height) continue;
      const TerrainCell *terrain_cell = terrain.at(transformed.x(), transformed.y());
      if (terrain_cell == nullptr || !terrain_cell->valid || terrain_cell->slope_deg > options.max_slope_deg ||
          terrain_cell->rms > options.max_roughness)
      {
        continue;
      }
      const double height_above_ground = transformed.z() - terrain_cell->ground_z;
      if (height_above_ground < options.obstacle_min_height || height_above_ground > options.obstacle_max_height) continue;
      for (int dy = -support_radius_cells; dy <= support_radius_cells; ++dy)
      {
        const int supported_row = row + dy;
        if (supported_row < 0 || supported_row >= height) continue;
        for (int dx = -support_radius_cells; dx <= support_radius_cells; ++dx)
        {
          const int supported_column = column + dx;
          if (supported_column < 0 || supported_column >= width) continue;
          const double offset_x = dx * options.resolution;
          const double offset_y = dy * options.resolution;
          if (offset_x * offset_x + offset_y * offset_y > support_tolerance_squared) continue;
          frame_cells.insert(static_cast<std::size_t>(supported_row) * width + supported_column);
        }
      }
    }
    for (const std::size_t index : frame_cells)
    {
      if (support[index] < std::numeric_limits<uint16_t>::max()) ++support[index];
    }
    ++processed;
    if (processed % 50 == 0 || processed == optimized_poses.size())
    {
      std::cout << "[nav-map] Counted obstacle support from " << processed << '/' << optimized_poses.size()
                << " keyframes.\n";
    }
  }
  return support;
}

std::vector<uint16_t> collectFallbackFreeKeyframeSupport(
    const Options &options, const fs::path &keyframe_dir, const std::map<int, TimedPose> &optimized_poses,
    const Eigen::Isometry3d &T_imu_lidar, const TerrainGrid &terrain, const double origin_x,
    const double origin_y, const int width, const int height)
{
  const std::size_t total_cells = static_cast<std::size_t>(width) * height;
  std::vector<uint16_t> support(total_cells, 0);
  const int support_radius_cells = static_cast<int>(std::ceil(options.obstacle_support_tolerance / options.resolution));
  const double support_tolerance_squared = options.obstacle_support_tolerance * options.obstacle_support_tolerance;
  std::size_t processed = 0;
  for (const auto &entry : optimized_poses)
  {
    const Eigen::Isometry3d T_map_lidar = entry.second.T_map_imu * T_imu_lidar;
    const Eigen::Vector3d sensor_origin = T_map_lidar.translation();
    const pcl::PointCloud<pcl::PointXYZ> cloud_lidar = loadCloud(keyframePath(keyframe_dir, entry.first));
    std::unordered_set<std::size_t> frame_cells;
    frame_cells.reserve(cloud_lidar.points.size() / 8 + 1);
    std::unordered_set<VoxelKey, VoxelKeyHash> ray_voxels;
    ray_voxels.reserve(cloud_lidar.points.size() / 4 + 1);
    for (const pcl::PointXYZ &point : cloud_lidar.points)
    {
      if (!finite(point)) continue;
      const VoxelKey voxel{static_cast<int64_t>(std::floor(point.x / options.fallback_free_ray_voxel)),
                           static_cast<int64_t>(std::floor(point.y / options.fallback_free_ray_voxel)),
                           static_cast<int64_t>(std::floor(point.z / options.fallback_free_ray_voxel))};
      if (!ray_voxels.insert(voxel).second) continue;
      const Eigen::Vector3d transformed = T_map_lidar * Eigen::Vector3d(point.x, point.y, point.z);
      if (!transformed.allFinite()) continue;
      Eigen::Vector3d delta = transformed - sensor_origin;
      const double ray_length = delta.norm();
      if (ray_length <= 1e-6) continue;
      if (ray_length > options.max_ray_range) delta *= options.max_ray_range / ray_length;
      const double horizontal_length = std::hypot(delta.x(), delta.y());
      const int steps = static_cast<int>(std::ceil(horizontal_length / options.terrain_resolution));
      if (steps < 2) continue;
      for (int step = 1; step < steps; ++step)
      {
        const double ratio = static_cast<double>(step) / static_cast<double>(steps);
        const Eigen::Vector3d sample = sensor_origin + ratio * delta;
        const TerrainCell *terrain_cell = terrain.at(sample.x(), sample.y());
        if (terrain_cell == nullptr || !terrain_cell->valid || !terrain_cell->fallback ||
            terrain_cell->slope_deg > options.max_slope_deg || terrain_cell->rms > options.fallback_max_rms)
        {
          continue;
        }
        const double height_above_ground = sample.z() - terrain_cell->ground_z;
        if (height_above_ground < options.obstacle_min_height || height_above_ground > options.obstacle_max_height)
        {
          continue;
        }
        const int column = static_cast<int>(std::floor((sample.x() - origin_x) / options.resolution));
        const int row = static_cast<int>(std::floor((sample.y() - origin_y) / options.resolution));
        if (column < 0 || row < 0 || column >= width || row >= height) continue;
        for (int dy = -support_radius_cells; dy <= support_radius_cells; ++dy)
        {
          const int supported_row = row + dy;
          if (supported_row < 0 || supported_row >= height) continue;
          for (int dx = -support_radius_cells; dx <= support_radius_cells; ++dx)
          {
            const int supported_column = column + dx;
            if (supported_column < 0 || supported_column >= width) continue;
            const double offset_x = dx * options.resolution;
            const double offset_y = dy * options.resolution;
            if (offset_x * offset_x + offset_y * offset_y > support_tolerance_squared) continue;
            frame_cells.insert(static_cast<std::size_t>(supported_row) * width + supported_column);
          }
        }
      }
    }
    for (const std::size_t index : frame_cells)
    {
      if (support[index] < std::numeric_limits<uint16_t>::max()) ++support[index];
    }
    ++processed;
    if (processed % 50 == 0 || processed == optimized_poses.size())
    {
      std::cout << "[nav-map] Counted fallback free-ray support from " << processed << '/'
                << optimized_poses.size() << " keyframes.\n";
    }
  }
  return support;
}

std::vector<unsigned char> visualizeObstacleSupport(const std::vector<uint16_t> &support)
{
  std::vector<unsigned char> visualization;
  visualization.reserve(support.size());
  for (const uint16_t count : support)
  {
    visualization.push_back(static_cast<unsigned char>(std::min(255, static_cast<int>(count) * 32)));
  }
  return visualization;
}

void writePgm(const fs::path &path, const std::vector<unsigned char> &cells, int width, int height)
{
  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  if (!output.is_open()) throw std::runtime_error("Cannot write " + path.string());
  output << "P5\n" << width << ' ' << height << "\n255\n";
  for (int row = height - 1; row >= 0; --row)
  {
    for (int column = 0; column < width; ++column)
    {
      const unsigned char value = cells[static_cast<std::size_t>(row) * width + column];
      output.write(reinterpret_cast<const char *>(&value), 1);
    }
  }
  if (!output.good()) throw std::runtime_error("Write failed for " + path.string());
}

void writePgm(const fs::path &path, const std::vector<CellState> &cells, int width, int height)
{
  std::vector<unsigned char> raw_cells;
  raw_cells.reserve(cells.size());
  for (const CellState cell : cells) raw_cells.push_back(static_cast<unsigned char>(cell));
  writePgm(path, raw_cells, width, height);
}

void writeMapYaml(const fs::path &path, double resolution, double origin_x, double origin_y)
{
  std::ofstream output(path, std::ios::trunc);
  if (!output.is_open()) throw std::runtime_error("Cannot write " + path.string());
  output << std::fixed << std::setprecision(9)
         << "image: map.pgm\n"
         << "resolution: " << resolution << "\n"
         << "origin: [" << origin_x << ", " << origin_y << ", 0.0]\n"
         << "negate: 0\n"
         << "occupied_thresh: 0.65\n"
         << "free_thresh: 0.196\n";
}

void writeTerrainCsv(const fs::path &path, const TerrainGrid &terrain)
{
  std::ofstream output(path, std::ios::trunc);
  if (!output.is_open()) throw std::runtime_error("Cannot write " + path.string());
  output << "x,y,ground_z,slope_deg,rms,inlier_count,interpolated,fallback,valid\n";
  output << std::fixed << std::setprecision(6);
  for (int row = 0; row < terrain.height; ++row)
  {
    for (int column = 0; column < terrain.width; ++column)
    {
      const TerrainCell &cell = terrain.cells[static_cast<std::size_t>(row) * terrain.width + column];
      const double x = terrain.origin_x + (static_cast<double>(column) + 0.5) * terrain.resolution;
      const double y = terrain.origin_y + (static_cast<double>(row) + 0.5) * terrain.resolution;
      output << x << ',' << y << ',' << cell.ground_z << ',' << cell.slope_deg << ',' << cell.rms << ','
             << cell.inlier_count << ',' << (cell.interpolated ? 1 : 0) << ',' << (cell.fallback ? 1 : 0) << ','
             << (cell.valid ? 1 : 0) << '\n';
    }
  }
}

void writeMetadata(const fs::path &path, const Options &options, std::size_t keyframes, std::size_t inserted_points,
                   std::size_t terrain_valid, std::size_t terrain_interpolated, std::size_t slope_blocked,
                   std::size_t roughness_blocked, std::size_t weak_obstacle, std::size_t occupied,
                   std::size_t free, std::size_t unknown, std::size_t obstacle_support_cells,
                   std::size_t obstacle_confirmed_support_cells, std::size_t below_adaptive_weak_ignored,
                   std::size_t weak_obstacle_free_override, double adaptive_weak_height_min,
                   double adaptive_weak_height_max, std::size_t terrain_fallback,
                   std::size_t fallback_free_confirmed, std::size_t fallback_free_insufficient,
                   std::size_t fallback_free_support_cells, std::size_t fallback_free_confirmed_support_cells,
                   int width, int height)
{
  std::ofstream output(path, std::ios::trunc);
  if (!output.is_open()) throw std::runtime_error("Cannot write " + path.string());
  output << std::fixed << std::setprecision(6)
         << "keyframes_inserted: " << keyframes << "\n"
         << "points_inserted: " << inserted_points << "\n"
         << "resolution_m: " << options.resolution << "\n"
         << "terrain_resolution_m: " << options.terrain_resolution << "\n"
         << "ground_seed_resolution_m: " << options.ground_seed_resolution << "\n"
         << "terrain_radius_m: " << options.terrain_radius << "\n"
         << "ground_quantile: " << options.ground_quantile << "\n"
         << "ground_min_below_sensor_m: " << options.ground_min_below_sensor << "\n"
         << "ground_outlier_threshold_m: " << options.ground_outlier_threshold << "\n"
         << "ground_max_slope_deg: " << options.ground_max_slope_deg << "\n"
         << "min_ground_spread_m: " << options.min_ground_spread << "\n"
         << "min_ground_observations: " << options.min_ground_observations << "\n"
         << "min_ground_cells: " << options.min_ground_cells << "\n"
         << "min_ground_inliers: " << options.min_ground_inliers << "\n"
         << "terrain_gap_fill_radius_m: " << options.terrain_gap_fill_radius << "\n"
         << "terrain_gap_fill_max_rms_m: " << options.terrain_gap_fill_max_rms << "\n"
         << "terrain_gap_fill_coverage_m: " << options.terrain_gap_fill_coverage << "\n"
         << "fallback_terrain_radius_m: " << options.fallback_terrain_radius << "\n"
         << "fallback_max_rms_m: " << options.fallback_max_rms << "\n"
         << "fallback_coverage_m: " << options.fallback_coverage << "\n"
         << "fallback_min_ground_cells: " << options.fallback_min_ground_cells << "\n"
         << "min_fallback_free_keyframes: " << options.min_fallback_free_keyframes << "\n"
         << "fallback_free_ray_voxel_m: " << options.fallback_free_ray_voxel << "\n"
         << "obstacle_min_height_m: " << options.obstacle_min_height << "\n"
         << "obstacle_max_height_m: " << options.obstacle_max_height << "\n"
         << "weak_obstacle_adaptive_height_rule: \"max(obstacle_min_height_m, 0.5 * resolution_m + terrain_rms_m)\"\n"
         << "weak_obstacle_adaptive_height_min_m: " << adaptive_weak_height_min << "\n"
         << "weak_obstacle_adaptive_height_max_m: " << adaptive_weak_height_max << "\n"
         << "obstacle_min_occupancy: " << options.obstacle_min_occupancy << "\n"
         << "min_obstacle_keyframes: " << options.min_obstacle_keyframes << "\n"
         << "obstacle_support_tolerance_m: " << options.obstacle_support_tolerance << "\n"
         << "max_slope_deg: " << options.max_slope_deg << "\n"
         << "max_roughness_m: " << options.max_roughness << "\n"
         << "max_ray_range_m: " << options.max_ray_range << "\n"
         << "width: " << width << "\n"
         << "height: " << height << "\n"
         << "terrain_valid_cells: " << terrain_valid << "\n"
         << "terrain_interpolated_cells: " << terrain_interpolated << "\n"
         << "terrain_fallback_grid_cells: " << terrain_fallback << "\n"
         << "slope_blocked_cells: " << slope_blocked << "\n"
         << "roughness_blocked_cells: " << roughness_blocked << "\n"
         << "weak_obstacle_cells: " << weak_obstacle << "\n"
         << "occupied_cells: " << occupied << "\n"
         << "free_cells: " << free << "\n"
         << "unknown_cells: " << unknown << "\n"
         << "obstacle_support_cells: " << obstacle_support_cells << "\n"
         << "obstacle_confirmed_support_cells: " << obstacle_confirmed_support_cells << "\n"
         << "below_adaptive_weak_obstacle_ignored_cells: " << below_adaptive_weak_ignored << "\n"
         << "weak_obstacle_free_override_cells: " << weak_obstacle_free_override << "\n"
         << "fallback_free_confirmed_cells: " << fallback_free_confirmed << "\n"
         << "fallback_free_insufficient_cells: " << fallback_free_insufficient << "\n"
         << "fallback_free_support_cells: " << fallback_free_support_cells << "\n"
         << "fallback_free_confirmed_support_cells: " << fallback_free_confirmed_support_cells << "\n";
}
}  // namespace

int main(int argc, char **argv)
{
  try
  {
    const Options options = parseOptions(argc, argv);
    const fs::path keyframe_dir = fs::absolute(options.keyframe_dir);
    const fs::path pose_path = fs::absolute(options.optimized_pose_path);
    const fs::path output_dir = fs::absolute(options.output_dir);
    if (!fs::is_directory(keyframe_dir)) throw std::runtime_error("Keyframe directory does not exist: " + keyframe_dir.string());

    const Eigen::Isometry3d T_imu_lidar = loadImuLidarExtrinsic(keyframe_dir / "metadata.yaml");
    const std::map<int, TimedPose> optimized_poses = loadOptimizedPoses(pose_path);
    fs::create_directories(output_dir);

    octomap::OcTree tree(options.resolution);
    GroundObservations ground_observations;
    std::size_t inserted_keyframes = 0;
    std::size_t inserted_points = 0;
    for (const auto &entry : optimized_poses)
    {
      const int id = entry.first;
      const fs::path cloud_path = keyframePath(keyframe_dir, id);
      if (!fs::exists(cloud_path)) throw std::runtime_error("Missing keyframe PCD for optimized id " + std::to_string(id) + ": " + cloud_path.string());

      const Eigen::Isometry3d T_map_lidar = entry.second.T_map_imu * T_imu_lidar;
      const pcl::PointCloud<pcl::PointXYZ> cloud_lidar = loadCloud(cloud_path);
      const Eigen::Vector3d origin_eigen = T_map_lidar.translation();
      const double maximum_ground_candidate_z = origin_eigen.z() - options.ground_min_below_sensor;
      // ROS Noetic's OctoMap library exposes insertPointCloud() for its own
      // Pointcloud type, rather than a PCL overload.  Keep the source PCD in
      // PCL only for decoding, then explicitly transform into map coordinates.
      octomap::Pointcloud cloud_map;
      for (const pcl::PointXYZ &point : cloud_lidar.points)
      {
        if (!finite(point)) continue;
        const Eigen::Vector3d transformed = T_map_lidar * Eigen::Vector3d(point.x, point.y, point.z);
        if (!std::isfinite(transformed.x()) || !std::isfinite(transformed.y()) || !std::isfinite(transformed.z())) continue;
        cloud_map.push_back(static_cast<float>(transformed.x()), static_cast<float>(transformed.y()),
                            static_cast<float>(transformed.z()));
        // A point at LiDAR height or above is a wall/ceiling/object candidate,
        // never a ground seed.  The remaining points are still fused across
        // keyframes, but only into coarse cells with substantial support.
        if (transformed.z() <= maximum_ground_candidate_z)
        {
          ground_observations[groundKeyFor(transformed.x(), transformed.y(), options.ground_seed_resolution)]
              .push_back(static_cast<float>(transformed.z()));
        }
      }
      const octomap::point3d origin(static_cast<float>(origin_eigen.x()), static_cast<float>(origin_eigen.y()),
                                    static_cast<float>(origin_eigen.z()));
      tree.insertPointCloud(cloud_map, origin, options.max_ray_range, false, true);
      ++inserted_keyframes;
      inserted_points += cloud_map.size();
      std::cout << "[nav-map] Inserted keyframe " << id << " (" << cloud_map.size() << " finite points)\n";
    }
    if (inserted_points == 0) throw std::runtime_error("All optimized keyframes are empty or contain invalid points.");
    tree.updateInnerOccupancy();
    if (!tree.writeBinary((output_dir / "map.bt").string()))
    {
      throw std::runtime_error("Cannot write OctoMap binary.");
    }

    double min_x, min_y, min_z, max_x, max_y, max_z;
    tree.getMetricMin(min_x, min_y, min_z);
    tree.getMetricMax(max_x, max_y, max_z);
    if (!(max_x >= min_x && max_y >= min_y)) throw std::runtime_error("OctoMap has invalid metric bounds.");
    const double origin_x = std::floor((min_x - options.padding) / options.resolution) * options.resolution;
    const double origin_y = std::floor((min_y - options.padding) / options.resolution) * options.resolution;
    const int width = std::max(1, static_cast<int>(std::ceil((max_x + options.padding - origin_x) / options.resolution)));
    const int height = std::max(1, static_cast<int>(std::ceil((max_y + options.padding - origin_y) / options.resolution)));
    const std::size_t total_cells = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
    std::vector<CellState> cells(total_cells, CellState::kUnknown);
    // Diagnostic image: white/directly observed free, 230/interpolated free,
    // black/confirmed obstacle, dark gray/slope, medium gray/roughness,
    // 180/weak obstacle evidence, and light gray/unknown terrain.
    std::vector<unsigned char> traversability(total_cells, static_cast<unsigned char>(CellState::kUnknown));
    const GroundSeeds ground_seeds = buildGroundSeeds(options, ground_observations);
    TerrainGrid terrain = buildTerrainGrid(options, ground_seeds, origin_x, origin_y, width, height);
    const std::size_t filled_terrain_cells = fillTerrainGaps(options, &terrain);
    const std::size_t fallback_terrain_cells = fillTerrainFallback(options, ground_seeds, &terrain);
    writeTerrainCsv(output_dir / "terrain.csv", terrain);
    const std::vector<uint16_t> obstacle_support = collectObstacleKeyframeSupport(
        options, keyframe_dir, optimized_poses, T_imu_lidar, terrain, origin_x, origin_y, width, height);
    writePgm(output_dir / "obstacle_support.pgm", visualizeObstacleSupport(obstacle_support), width, height);
    const std::vector<uint16_t> fallback_free_support = collectFallbackFreeKeyframeSupport(
        options, keyframe_dir, optimized_poses, T_imu_lidar, terrain, origin_x, origin_y, width, height);
    writePgm(output_dir / "fallback_free_support.pgm", visualizeObstacleSupport(fallback_free_support), width, height);
    std::size_t obstacle_support_cells = 0;
    std::size_t obstacle_confirmed_support_cells = 0;
    std::size_t fallback_free_support_cells = 0;
    std::size_t fallback_free_confirmed_support_cells = 0;
    for (const uint16_t support : obstacle_support)
    {
      if (support == 0) continue;
      ++obstacle_support_cells;
      if (support >= options.min_obstacle_keyframes) ++obstacle_confirmed_support_cells;
    }
    for (const uint16_t support : fallback_free_support)
    {
      if (support == 0) continue;
      ++fallback_free_support_cells;
      if (support >= options.min_fallback_free_keyframes) ++fallback_free_confirmed_support_cells;
    }

    std::size_t terrain_valid_count = 0;
    std::size_t terrain_interpolated_count = 0;
    std::size_t slope_blocked_count = 0;
    std::size_t roughness_blocked_count = 0;
    std::size_t weak_obstacle_count = 0;
    std::size_t below_adaptive_weak_obstacle_ignored_count = 0;
    std::size_t weak_obstacle_free_override_count = 0;
    std::size_t fallback_free_confirmed_count = 0;
    std::size_t fallback_free_insufficient_count = 0;
    double adaptive_weak_height_min = std::numeric_limits<double>::infinity();
    double adaptive_weak_height_max = 0.0;
    std::size_t occupied_count = 0;
    std::size_t free_count = 0;
    std::size_t unknown_count = 0;
    for (int row = 0; row < height; ++row)
    {
      for (int column = 0; column < width; ++column)
      {
        const double x = origin_x + (static_cast<double>(column) + 0.5) * options.resolution;
        const double y = origin_y + (static_cast<double>(row) + 0.5) * options.resolution;
        const TerrainCell *terrain_cell = terrain.at(x, y);
        const std::size_t cell_index = static_cast<std::size_t>(row) * width + column;
        if (terrain_cell == nullptr || !terrain_cell->valid)
        {
          ++unknown_count;
          continue;
        }
        ++terrain_valid_count;
        if (terrain_cell->interpolated) ++terrain_interpolated_count;
        if (terrain_cell->slope_deg > options.max_slope_deg)
        {
          cells[cell_index] = CellState::kOccupied;
          traversability[cell_index] = 64;
          ++occupied_count;
          ++slope_blocked_count;
          continue;
        }
        if (terrain_cell->rms > options.max_roughness)
        {
          cells[cell_index] = CellState::kOccupied;
          traversability[cell_index] = 128;
          ++occupied_count;
          ++roughness_blocked_count;
          continue;
        }
        const double z_min = terrain_cell->ground_z + options.obstacle_min_height;
        const double z_max = terrain_cell->ground_z + options.obstacle_max_height;
        const double adaptive_weak_height = std::min(
            options.obstacle_max_height,
            std::max(options.obstacle_min_height, 0.5 * options.resolution + terrain_cell->rms));
        adaptive_weak_height_min = std::min(adaptive_weak_height_min, adaptive_weak_height);
        adaptive_weak_height_max = std::max(adaptive_weak_height_max, adaptive_weak_height);
        const double weak_obstacle_z_min = terrain_cell->ground_z + adaptive_weak_height;
        bool occupied = false;
        bool free = false;
        bool weak_occupied = false;
        bool below_adaptive_weak_occupied = false;
        for (double z = z_min; z <= z_max + 0.5 * options.resolution; z += options.resolution)
        {
          const octomap::OcTreeNode *node = tree.search(x, y, z);
          if (node == nullptr) continue;
          if (tree.isNodeOccupied(node))
          {
            if (node->getOccupancy() >= options.obstacle_min_occupancy &&
                obstacle_support[cell_index] >= options.min_obstacle_keyframes)
            {
              occupied = true;
              break;
            }
            if (z >= weak_obstacle_z_min) weak_occupied = true;
            else below_adaptive_weak_occupied = true;
            continue;
          }
          free = true;
        }
        if (below_adaptive_weak_occupied) ++below_adaptive_weak_obstacle_ignored_count;
        if (occupied)
        {
          cells[cell_index] = CellState::kOccupied;
          traversability[cell_index] = static_cast<unsigned char>(CellState::kOccupied);
          ++occupied_count;
        }
        else
        {
          const bool fallback_free_confirmed = !terrain_cell->fallback ||
              fallback_free_support[cell_index] >= options.min_fallback_free_keyframes;
          if (terrain_cell->fallback && free && !fallback_free_confirmed)
          {
            ++fallback_free_insufficient_count;
          }
          if ((free && fallback_free_confirmed) || terrain_cell->interpolated)
          {
            // A larger-scale terrain estimate is allowed to create free
            // navigation space only after independent free-ray support.
            cells[cell_index] = CellState::kFree;
            traversability[cell_index] = terrain_cell->interpolated ? 230 : static_cast<unsigned char>(CellState::kFree);
            if (terrain_cell->fallback) ++fallback_free_confirmed_count;
            if (weak_occupied) ++weak_obstacle_free_override_count;
            ++free_count;
          }
          else if (weak_occupied)
          {
            // Keep unresolved evidence at and above the configured height as
            // unknown when no free-space observation is available.
            traversability[cell_index] = 180;
            ++weak_obstacle_count;
            ++unknown_count;
          }
          else ++unknown_count;
        }
      }
    }

    writePgm(output_dir / "map.pgm", cells, width, height);
    writePgm(output_dir / "traversability.pgm", traversability, width, height);
    writeMapYaml(output_dir / "map.yaml", options.resolution, origin_x, origin_y);
    writeMetadata(output_dir / "map_metadata.yaml", options, inserted_keyframes, inserted_points,
                  terrain_valid_count, terrain_interpolated_count, slope_blocked_count, roughness_blocked_count,
                  weak_obstacle_count, occupied_count, free_count, unknown_count, obstacle_support_cells,
                  obstacle_confirmed_support_cells, below_adaptive_weak_obstacle_ignored_count,
                  weak_obstacle_free_override_count,
                  std::isfinite(adaptive_weak_height_min) ? adaptive_weak_height_min : 0.0,
                  adaptive_weak_height_max, fallback_terrain_cells, fallback_free_confirmed_count,
                  fallback_free_insufficient_count, fallback_free_support_cells,
                  fallback_free_confirmed_support_cells, width, height);
    std::cout << "[nav-map] Completed: " << output_dir << "\n"
              << "[nav-map] Filled " << filled_terrain_cells << " short terrain-grid gaps.\n"
              << "[nav-map] Recovered " << fallback_terrain_cells << " terrain-grid cells with fallback fitting.\n"
              << "[nav-map] " << width << "x" << height << " cells; occupied=" << occupied_count
              << ", free=" << free_count << ", unknown=" << unknown_count << "\n";
    return 0;
  }
  catch (const std::exception &error)
  {
    std::cerr << "[nav-map] ERROR: " << error.what() << '\n';
    return 1;
  }
}
