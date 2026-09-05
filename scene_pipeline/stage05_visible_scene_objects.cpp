/*
 * 阶段 5：可见建筑立面与场景对象的证据提取。
 *
 * 输入是阶段 1 回环校正后的逐帧 RGB 缓存点云。程序先将每个点变换到 map，
 * 在三维体素中累积跨帧可复现观测；随后仅从具有真实垂直覆盖的体素列提取
 * 局部竖直平面段。它不使用道路距离或道路走向来生成对象，更不会补出建筑
 * 背面、顶部或完整轮廓。
 *
 * 阶段 4 的路沿/人行道仅提供“最近邻语境”审计字段，绝不参与对象分类。
 * 因而 visible_facade_candidate / wall_candidate 都表示“观测到的平面候选”，
 * 不是对完整建筑物或其不可见部分的断言。
 */

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <sys/stat.h>
#include <unistd.h>

namespace {

const double kEpsilon = 1e-9;
const uint32_t kNoFrame = std::numeric_limits<uint32_t>::max();

struct Quaternion { double x = 0.0, y = 0.0, z = 0.0, w = 1.0; };
struct Transform { double tx = 0.0, ty = 0.0, tz = 0.0; Quaternion q; };
struct Frame {
  uint32_t id = 0;
  uint64_t point_count = 0;
  std::string cache_path, support;
  Transform map_odom, map_imu;
};

struct Key2 {
  int x = 0, y = 0;
  bool operator==(const Key2 &other) const { return x == other.x && y == other.y; }
};
struct Key3 {
  int x = 0, y = 0, z = 0;
  bool operator==(const Key3 &other) const { return x == other.x && y == other.y && z == other.z; }
};
struct Key2Hash {
  std::size_t operator()(const Key2 &key) const {
    return std::hash<int>()(key.x) ^ (std::hash<int>()(key.y) << 1);
  }
};
struct Key3Hash {
  std::size_t operator()(const Key3 &key) const {
    return std::hash<int>()(key.x) ^ (std::hash<int>()(key.y) << 1) ^ (std::hash<int>()(key.z) << 2);
  }
};

struct Voxel {
  uint32_t point_count = 0, frame_count = 0, last_frame = kNoFrame;
  double frame_weight = 0.0;
  double red_sum = 0.0, green_sum = 0.0, blue_sum = 0.0;
  // 从体素指向传感器的平均视线。仅用来给局部平面法向定向，不用于类别判断。
  double to_sensor_x_sum = 0.0, to_sensor_y_sum = 0.0;
};
struct Column {
  uint32_t stable_voxels = 0, frame_support_sum = 0;
  double min_z = std::numeric_limits<double>::infinity();
  double max_z = -std::numeric_limits<double>::infinity();
  double red_sum = 0.0, green_sum = 0.0, blue_sum = 0.0, color_weight = 0.0;
};
struct GroundCell { double z = 0.0; int frames = 0, free_frames = 0; double confidence = 0.0; };
struct ContextPoint { double x = 0.0, y = 0.0; };
struct PcdLayout {
  uint64_t points = 0;
  std::size_t point_step = 0, x_offset = 0, y_offset = 0, z_offset = 0, rgb_offset = 0;
  bool x_found = false, y_found = false, z_found = false, rgb_found = false;
  std::streampos data_offset;
};
struct Options {
  std::string run_dir, stage01_dir, stage02_dir, stage04_r7_dir, output_dir;
  double voxel_m = 0.25;
  int min_voxel_frames = 3;
  double min_voxel_weight = 2.0;
  double min_range_m = 1.5, max_range_m = 35.0;
  double min_local_z = -1.8, max_local_z = 10.0;
};
struct Object {
  int id = -1;
  std::string type, geometry_state, semantic_state, completeness, reject_reason;
  double x = 0.0, y = 0.0, base_z = 0.0, top_z = 0.0;
  double tangent_x = 1.0, tangent_y = 0.0, normal_x = 0.0, normal_y = 1.0;
  double length = 0.0, width_rms = 0.0, height = 0.0, coverage = 0.0, confidence = 0.0;
  double ground_distance = -1.0, ground_z = std::numeric_limits<double>::quiet_NaN();
  double curb_distance = -1.0, sidewalk_distance = -1.0;
  uint32_t source_columns = 0, source_voxels = 0, max_voxel_frame_count = 0;
  uint64_t voxel_frame_support_sum = 0;
  double red_sum = 0.0, green_sum = 0.0, blue_sum = 0.0, color_weight = 0.0;
  double to_sensor_x_sum = 0.0, to_sensor_y_sum = 0.0;
};
struct Stats {
  uint64_t source_points = 0, valid_points = 0, selected_points = 0;
  uint64_t raw_voxels = 0, stable_voxels = 0, candidate_columns = 0;
  uint64_t components = 0, observed_vertical_voxels = 0;
};

void fail(const std::string &message) { throw std::runtime_error(message); }
bool fileExists(const std::string &path) { struct stat s; return ::stat(path.c_str(), &s) == 0 && S_ISREG(s.st_mode) && s.st_size > 0; }
bool directoryExists(const std::string &path) { struct stat s; return ::stat(path.c_str(), &s) == 0 && S_ISDIR(s.st_mode); }
bool createDirectories(const std::string &path) {
  if (path.empty()) return false;
  std::string current; std::size_t begin = 0;
  if (path.front() == '/') { current = "/"; begin = 1; }
  while (begin <= path.size()) {
    const std::size_t end = path.find('/', begin); const std::string part = path.substr(begin, end - begin);
    if (!part.empty()) { if (!current.empty() && current.back() != '/') current += '/'; current += part; if (::mkdir(current.c_str(), 0755) && errno != EEXIST) return false; }
    if (end == std::string::npos) break;
    begin = end + 1;
  }
  return true;
}
std::string absolutePath(const std::string &path) {
  if (path.empty() || path.front() == '/') return path;
  char buffer[4096]; if (!::getcwd(buffer, sizeof(buffer))) fail("无法解析当前目录");
  return std::string(buffer) + "/" + path;
}
std::vector<std::string> split(const std::string &line, char separator) {
  std::vector<std::string> result; std::stringstream stream(line); std::string item;
  while (std::getline(stream, item, separator)) result.push_back(item);
  if (!line.empty() && line.back() == separator) result.push_back("");
  return result;
}
double number(const std::string &text, const std::string &label) {
  char *end = nullptr; const double value = std::strtod(text.c_str(), &end);
  if (!end || end == text.c_str() || *end || !std::isfinite(value)) fail("无效数值（" + label + "）：" + text);
  return value;
}
uint64_t wholeNumber(const std::string &text, const std::string &label) {
  char *end = nullptr; const unsigned long long value = std::strtoull(text.c_str(), &end, 10);
  if (!end || end == text.c_str() || *end) fail("无效整数（" + label + "）：" + text);
  return static_cast<uint64_t>(value);
}
std::unordered_map<std::string, std::size_t> csvHeader(const std::string &line) {
  std::unordered_map<std::string, std::size_t> result; const std::vector<std::string> names = split(line, ',');
  for (std::size_t i = 0; i < names.size(); ++i) if (!result.insert(std::make_pair(names[i], i)).second) fail("重复 CSV 列：" + names[i]);
  return result;
}
const std::string &field(const std::vector<std::string> &row, const std::unordered_map<std::string, std::size_t> &cols, const std::string &name) {
  const auto it = cols.find(name); if (it == cols.end() || it->second >= row.size()) fail("缺少 CSV 字段：" + name); return row[it->second];
}
Quaternion normalized(Quaternion q) {
  const double norm = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w); if (norm < kEpsilon) fail("零四元数");
  q.x /= norm; q.y /= norm; q.z /= norm; q.w /= norm; return q;
}
void rotate(const Quaternion &q, double x, double y, double z, double *rx, double *ry, double *rz) {
  const double tx = 2.0 * (q.y*z - q.z*y), ty = 2.0 * (q.z*x - q.x*z), tz = 2.0 * (q.x*y - q.y*x);
  *rx = x + q.w*tx + (q.y*tz - q.z*ty); *ry = y + q.w*ty + (q.z*tx - q.x*tz); *rz = z + q.w*tz + (q.x*ty - q.y*tx);
}
void apply(const Transform &t, double x, double y, double z, double *ox, double *oy, double *oz) {
  rotate(t.q, x, y, z, ox, oy, oz); *ox += t.tx; *oy += t.ty; *oz += t.tz;
}
void inverseApply(const Transform &t, double x, double y, double z, double *ox, double *oy, double *oz) {
  const Quaternion inverse{-t.q.x, -t.q.y, -t.q.z, t.q.w}; rotate(inverse, x - t.tx, y - t.ty, z - t.tz, ox, oy, oz);
}
double frameWeight(const std::string &support) { return support == "interpolated" ? 1.0 : 0.5; }
Key3 key3Of(double x, double y, double z, double grid) { return Key3{static_cast<int>(std::floor(x/grid)), static_cast<int>(std::floor(y/grid)), static_cast<int>(std::floor(z/grid))}; }

std::vector<Frame> loadFrames(const Options &options) {
  const std::string path = options.stage01_dir + "/pose_correction/frame_map_corrections.csv";
  if (!fileExists(path)) fail("缺少阶段1逐帧校正清单：" + path);
  std::ifstream input(path); std::string line; if (!std::getline(input, line)) fail("阶段1逐帧校正清单为空"); const auto columns = csvHeader(line);
  const char *need[] = {"frame_id", "point_count", "cache_pcd", "map_odom_tx", "map_odom_ty", "map_odom_tz", "map_odom_qx", "map_odom_qy", "map_odom_qz", "map_odom_qw", "map_imu_tx", "map_imu_ty", "map_imu_tz", "map_imu_qx", "map_imu_qy", "map_imu_qz", "map_imu_qw", "support"};
  for (const char *name : need) if (columns.find(name) == columns.end()) fail("阶段1清单缺少列：" + std::string(name));
  std::vector<Frame> frames; uint32_t previous = 0; bool first = true;
  while (std::getline(input, line)) {
    if (line.empty()) continue;
    const std::vector<std::string> row = split(line, ','); Frame frame;
    frame.id = static_cast<uint32_t>(wholeNumber(field(row, columns, "frame_id"), "frame_id"));
    if (!first && frame.id != previous + 1) fail("阶段1帧编号不连续");
    previous = frame.id;
    first = false;
    frame.point_count = wholeNumber(field(row, columns, "point_count"), "point_count"); frame.cache_path = field(row, columns, "cache_pcd"); frame.support = field(row, columns, "support");
    frame.map_odom.tx = number(field(row, columns, "map_odom_tx"), "map_odom_tx"); frame.map_odom.ty = number(field(row, columns, "map_odom_ty"), "map_odom_ty"); frame.map_odom.tz = number(field(row, columns, "map_odom_tz"), "map_odom_tz");
    frame.map_odom.q = normalized(Quaternion{number(field(row, columns, "map_odom_qx"), "map_odom_qx"), number(field(row, columns, "map_odom_qy"), "map_odom_qy"), number(field(row, columns, "map_odom_qz"), "map_odom_qz"), number(field(row, columns, "map_odom_qw"), "map_odom_qw")});
    frame.map_imu.tx = number(field(row, columns, "map_imu_tx"), "map_imu_tx"); frame.map_imu.ty = number(field(row, columns, "map_imu_ty"), "map_imu_ty"); frame.map_imu.tz = number(field(row, columns, "map_imu_tz"), "map_imu_tz");
    frame.map_imu.q = normalized(Quaternion{number(field(row, columns, "map_imu_qx"), "map_imu_qx"), number(field(row, columns, "map_imu_qy"), "map_imu_qy"), number(field(row, columns, "map_imu_qz"), "map_imu_qz"), number(field(row, columns, "map_imu_qw"), "map_imu_qw")});
    frames.push_back(frame);
  }
  if (frames.empty()) fail("阶段1中没有可处理帧");
  return frames;
}

PcdLayout parsePcdLayout(const std::string &path) {
  std::ifstream input(path, std::ios::binary); if (!input) fail("无法打开PCD：" + path); PcdLayout layout;
  std::vector<std::string> names, types; std::vector<std::size_t> sizes, counts; uint64_t width = 0, height = 0; std::string line; bool found_data = false;
  while (std::getline(input, line)) {
    const std::vector<std::string> tokens = split(line, ' '); if (tokens.empty() || tokens[0].empty() || tokens[0][0] == '#') continue;
    if (tokens[0] == "FIELDS") names.assign(tokens.begin()+1, tokens.end());
    else if (tokens[0] == "SIZE") for (std::size_t i = 1; i < tokens.size(); ++i) sizes.push_back(static_cast<std::size_t>(wholeNumber(tokens[i], "PCD SIZE")));
    else if (tokens[0] == "TYPE") types.assign(tokens.begin()+1, tokens.end());
    else if (tokens[0] == "COUNT") for (std::size_t i = 1; i < tokens.size(); ++i) counts.push_back(static_cast<std::size_t>(wholeNumber(tokens[i], "PCD COUNT")));
    else if (tokens[0] == "WIDTH" && tokens.size() == 2) width = wholeNumber(tokens[1], "PCD WIDTH");
    else if (tokens[0] == "HEIGHT" && tokens.size() == 2) height = wholeNumber(tokens[1], "PCD HEIGHT");
    else if (tokens[0] == "POINTS" && tokens.size() == 2) layout.points = wholeNumber(tokens[1], "PCD POINTS");
    else if (tokens[0] == "DATA") { if (tokens.size() != 2 || tokens[1] != "binary") fail("只支持binary PCD：" + path); layout.data_offset = input.tellg(); found_data = true; break; }
  }
  if (!found_data || names.empty() || names.size() != sizes.size() || names.size() != types.size()) fail("无效PCD头：" + path);
  if (counts.empty()) counts.assign(names.size(), 1);
  if (counts.size() != names.size()) fail("无效PCD COUNT：" + path);
  for (std::size_t i = 0; i < names.size(); ++i) {
    if (counts[i] != 1 || sizes[i] != 4) fail("PCD字段必须为标量32位：" + path);
    const std::size_t offset = layout.point_step;
    if (names[i] == "x") { layout.x_offset = offset; layout.x_found = true; }
    if (names[i] == "y") { layout.y_offset = offset; layout.y_found = true; }
    if (names[i] == "z") { layout.z_offset = offset; layout.z_found = true; }
    if (names[i] == "rgb") { layout.rgb_offset = offset; layout.rgb_found = true; }
    layout.point_step += sizes[i];
  }
  if (!layout.points) layout.points = width * height;
  if (!layout.points || !layout.x_found || !layout.y_found || !layout.z_found || !layout.rgb_found || layout.point_step != 16) fail("PCD必须为x y z rgb四个32位字段：" + path);
  return layout;
}
float floatAt(const std::vector<char> &buffer, std::size_t offset) { float value; std::memcpy(&value, buffer.data()+offset, sizeof(value)); return value; }
uint32_t rgbAt(const std::vector<char> &buffer, std::size_t offset) { uint32_t value; std::memcpy(&value, buffer.data()+offset, sizeof(value)); return value; }

bool stableVoxel(const Voxel &voxel, const Options &options) {
  return voxel.frame_count >= static_cast<uint32_t>(options.min_voxel_frames) && voxel.frame_weight >= options.min_voxel_weight;
}

void processFrame(const Frame &frame, const Options &options, std::unordered_map<Key3, Voxel, Key3Hash> *voxels, Stats *stats) {
  const std::string path = options.run_dir + "/" + frame.cache_path; if (!fileExists(path)) fail("缺少缓存PCD：" + path);
  const PcdLayout layout = parsePcdLayout(path); if (layout.points != frame.point_count) fail("PCD点数与阶段1清单不一致：" + path);
  std::ifstream input(path, std::ios::binary); input.seekg(layout.data_offset); std::vector<char> buffer(layout.point_step);
  for (uint64_t i = 0; i < layout.points; ++i) {
    ++stats->source_points; input.read(buffer.data(), static_cast<std::streamsize>(buffer.size())); if (!input) fail("截断的PCD：" + path);
    const float rx = floatAt(buffer, layout.x_offset), ry = floatAt(buffer, layout.y_offset), rz = floatAt(buffer, layout.z_offset);
    if (!std::isfinite(rx) || !std::isfinite(ry) || !std::isfinite(rz)) continue;
    double mx, my, mz, lx, ly, lz; apply(frame.map_odom, rx, ry, rz, &mx, &my, &mz); inverseApply(frame.map_imu, mx, my, mz, &lx, &ly, &lz);
    if (!std::isfinite(mx) || !std::isfinite(my) || !std::isfinite(mz) || !std::isfinite(lx) || !std::isfinite(ly) || !std::isfinite(lz)) continue;
    ++stats->valid_points;
    const double range = std::hypot(lx, ly);
    if (range < options.min_range_m || range > options.max_range_m || lz < options.min_local_z || lz > options.max_local_z) continue;
    ++stats->selected_points;
    Voxel &voxel = (*voxels)[key3Of(mx, my, mz, options.voxel_m)];
    ++voxel.point_count;
    if (voxel.last_frame != frame.id) { voxel.last_frame = frame.id; ++voxel.frame_count; voxel.frame_weight += frameWeight(frame.support); }
    const uint32_t rgb = rgbAt(buffer, layout.rgb_offset); voxel.red_sum += (rgb >> 16) & 0xffU; voxel.green_sum += (rgb >> 8) & 0xffU; voxel.blue_sum += rgb & 0xffU;
    voxel.to_sensor_x_sum += frame.map_imu.tx - mx; voxel.to_sensor_y_sum += frame.map_imu.ty - my;
  }
}

std::unordered_map<Key2, GroundCell, Key2Hash> loadGround(const std::string &path) {
  if (!fileExists(path)) fail("缺少阶段2几何观测网格：" + path);
  std::ifstream input(path); std::string line; if (!std::getline(input, line)) fail("阶段2几何观测网格为空"); const auto columns = csvHeader(line);
  const char *need[] = {"cell_ix", "cell_iy", "ground_mean_z", "stable_ground", "ground_frame_count", "free_frame_count", "observation_confidence"};
  for (const char *name : need) if (columns.find(name) == columns.end()) fail("阶段2网格字段不兼容：" + std::string(name));
  std::unordered_map<Key2, GroundCell, Key2Hash> result;
  while (std::getline(input, line)) {
    if (line.empty()) continue;
    const std::vector<std::string> row = split(line, ',');
    if (field(row, columns, "stable_ground") != "1" || wholeNumber(field(row, columns, "free_frame_count"), "free_frame_count") < 1) continue;
    const Key2 key{static_cast<int>(std::strtol(field(row, columns, "cell_ix").c_str(), nullptr, 10)), static_cast<int>(std::strtol(field(row, columns, "cell_iy").c_str(), nullptr, 10))};
    result[key] = GroundCell{number(field(row, columns, "ground_mean_z"), "ground_mean_z"), static_cast<int>(wholeNumber(field(row, columns, "ground_frame_count"), "ground_frame_count")), static_cast<int>(wholeNumber(field(row, columns, "free_frame_count"), "free_frame_count")), number(field(row, columns, "observation_confidence"), "observation_confidence")};
  }
  if (result.empty()) fail("阶段2中没有可用稳定地面");
  return result;
}

std::vector<ContextPoint> readContextPcd(const std::string &path) {
  if (!fileExists(path)) fail("缺少阶段4语境PCD：" + path);
  const PcdLayout layout = parsePcdLayout(path); std::ifstream input(path, std::ios::binary); input.seekg(layout.data_offset); std::vector<char> buffer(layout.point_step); std::vector<ContextPoint> result; result.reserve(layout.points);
  for (uint64_t i = 0; i < layout.points; ++i) { input.read(buffer.data(), static_cast<std::streamsize>(buffer.size())); if (!input) fail("截断的阶段4语境PCD：" + path); result.push_back(ContextPoint{floatAt(buffer, layout.x_offset), floatAt(buffer, layout.y_offset)}); }
  return result;
}
class ContextIndex {
 public:
  explicit ContextIndex(const std::vector<ContextPoint> &points, double grid_m = 1.0) : grid_m_(grid_m) {
    for (const ContextPoint &point : points) cells_[key(point.x, point.y)].push_back(point);
  }
  double nearest(double x, double y, double maximum_m) const {
    const Key2 base = key(x, y); const int radius = static_cast<int>(std::ceil(maximum_m / grid_m_)); double best_sq = maximum_m * maximum_m; bool found = false;
    for (int dx = -radius; dx <= radius; ++dx) for (int dy = -radius; dy <= radius; ++dy) {
      const auto it = cells_.find(Key2{base.x+dx, base.y+dy}); if (it == cells_.end()) continue;
      for (const ContextPoint &point : it->second) { const double px = point.x-x, py = point.y-y, distance_sq = px*px+py*py; if (distance_sq <= best_sq) { best_sq = distance_sq; found = true; } }
    }
    return found ? std::sqrt(best_sq) : -1.0;
  }
 private:
  Key2 key(double x, double y) const { return Key2{static_cast<int>(std::floor(x/grid_m_)), static_cast<int>(std::floor(y/grid_m_))}; }
  double grid_m_; std::unordered_map<Key2, std::vector<ContextPoint>, Key2Hash> cells_;
};

double nearestGround(const std::unordered_map<Key2, GroundCell, Key2Hash> &ground, double x, double y, double *z) {
  const Key2 base{static_cast<int>(std::floor(x/0.2)), static_cast<int>(std::floor(y/0.2))}; double best_sq = 2.0 * 2.0; bool found = false;
  for (int dx = -10; dx <= 10; ++dx) for (int dy = -10; dy <= 10; ++dy) {
    const auto it = ground.find(Key2{base.x+dx, base.y+dy}); if (it == ground.end()) continue;
    const double gx = (base.x+dx+0.5)*0.2, gy = (base.y+dy+0.5)*0.2, vx = gx-x, vy = gy-y, distance_sq = vx*vx+vy*vy;
    if (distance_sq <= best_sq) { best_sq = distance_sq; *z = it->second.z; found = true; }
  }
  return found ? std::sqrt(best_sq) : -1.0;
}

struct PlaneFit { double x = 0.0, y = 0.0, tx = 1.0, ty = 0.0, width = 0.0, length = 0.0, coverage = 0.0; };
PlaneFit fitPlane(const std::vector<Key2> &keys, double grid_m) {
  PlaneFit fit; if (keys.empty()) return fit;
  for (const Key2 &key : keys) { fit.x += (key.x+0.5)*grid_m; fit.y += (key.y+0.5)*grid_m; }
  fit.x /= keys.size(); fit.y /= keys.size(); double xx = 0.0, xy = 0.0, yy = 0.0;
  for (const Key2 &key : keys) { const double dx = (key.x+0.5)*grid_m-fit.x, dy = (key.y+0.5)*grid_m-fit.y; xx += dx*dx; xy += dx*dy; yy += dy*dy; }
  const double angle = 0.5*std::atan2(2.0*xy, xx-yy); fit.tx = std::cos(angle); fit.ty = std::sin(angle);
  double minimum = std::numeric_limits<double>::infinity(), maximum = -std::numeric_limits<double>::infinity(), residual_sq = 0.0; std::unordered_set<int> bins;
  for (const Key2 &key : keys) {
    const double dx = (key.x+0.5)*grid_m-fit.x, dy = (key.y+0.5)*grid_m-fit.y, projection = dx*fit.tx+dy*fit.ty, residual = -dx*fit.ty+dy*fit.tx;
    minimum = std::min(minimum, projection); maximum = std::max(maximum, projection); residual_sq += residual*residual; bins.insert(static_cast<int>(std::floor(projection/grid_m)));
  }
  fit.length = std::max(0.0, maximum-minimum+grid_m); fit.width = std::sqrt(residual_sq/keys.size()); fit.coverage = fit.length <= kEpsilon ? 0.0 : std::min(1.0, static_cast<double>(bins.size())/std::max(1.0, std::ceil(fit.length/grid_m)));
  return fit;
}

struct Segment { std::vector<Key2> keys; PlaneFit fit; };
std::vector<Segment> extractSegments(const std::vector<Key2> &component, double grid_m, uint32_t seed) {
  std::vector<Key2> remaining = component; std::vector<Segment> result; uint64_t state = static_cast<uint64_t>(seed) * 6364136223846793005ULL + 1442695040888963407ULL;
  for (int pass = 0; pass < 6 && remaining.size() >= 7; ++pass) {
    // 限制随机采样次数：阶段 5 提取的是局部平面，不应把大面积植被块做成无界全局拟合。
    std::vector<Key2> best; double best_score = 0.0; const int iterations = std::min(180, std::max(60, static_cast<int>(remaining.size())*2));
    for (int it = 0; it < iterations; ++it) {
      state = state*2862933555777941757ULL + 3037000493ULL; const Key2 &a = remaining[static_cast<std::size_t>((state >> 16) % remaining.size())];
      state = state*2862933555777941757ULL + 3037000493ULL; const Key2 &b = remaining[static_cast<std::size_t>((state >> 16) % remaining.size())];
      const double ax = (a.x+0.5)*grid_m, ay = (a.y+0.5)*grid_m, bx = (b.x+0.5)*grid_m, by = (b.y+0.5)*grid_m, dx = bx-ax, dy = by-ay, norm = std::hypot(dx, dy);
      if (norm < 0.7) continue;
      const double nx = -dy/norm, ny = dx/norm; std::vector<Key2> inliers; inliers.reserve(remaining.size());
      for (const Key2 &key : remaining) { const double px = (key.x+0.5)*grid_m-ax, py = (key.y+0.5)*grid_m-ay; if (std::abs(px*nx+py*ny) <= 0.32) inliers.push_back(key); }
      if (inliers.size() < 7) continue;
      const PlaneFit fit = fitPlane(inliers, grid_m);
      if (fit.length < 1.4 || fit.coverage < 0.48 || fit.width > 0.32) continue;
      const double score = inliers.size()*fit.coverage*std::min(1.0, fit.length/3.0)*(1.0-fit.width/0.35);
      if (score > best_score) { best_score = score; best.swap(inliers); }
    }
    if (best.empty()) break;
    Segment segment; segment.keys.swap(best); segment.fit = fitPlane(segment.keys, grid_m); result.push_back(segment);
    std::unordered_set<Key2, Key2Hash> selected(segment.keys.begin(), segment.keys.end()); std::vector<Key2> next; next.reserve(remaining.size()-selected.size());
    for (const Key2 &key : remaining) if (selected.find(key) == selected.end()) next.push_back(key);
    remaining.swap(next);
  }
  return result;
}

double componentHeight(const std::vector<Key2> &keys, const std::unordered_map<Key2, Column, Key2Hash> &columns, double *base, double *top) {
  *base = std::numeric_limits<double>::infinity(); *top = -std::numeric_limits<double>::infinity();
  for (const Key2 &key : keys) { const Column &column = columns.at(key); *base = std::min(*base, column.min_z); *top = std::max(*top, column.max_z); }
  return keys.empty() ? 0.0 : *top-*base;
}

std::string lineType(const PlaneFit &fit, double height, uint32_t columns, double confidence) {
  // facade 与 wall 的区分只能是几何候选：墙也可能很高，因此低置信度一律保留不确定。
  if (fit.length >= 3.0 && height >= 2.2 && columns >= 10 && confidence >= 0.55) return "visible_facade_candidate";
  if (fit.length >= 1.4 && height >= 1.2 && confidence >= 0.40) return "wall_candidate";
  return "uncertain_vertical_structure";
}

Object makeObject(int id, const std::vector<Key2> &keys, const std::unordered_map<Key2, Column, Key2Hash> &columns, double grid_m, bool planar, const std::string &forced_type) {
  Object object; object.id = id; object.source_columns = static_cast<uint32_t>(keys.size()); const PlaneFit fit = fitPlane(keys, grid_m); object.x = fit.x; object.y = fit.y; object.tangent_x = fit.tx; object.tangent_y = fit.ty; object.normal_x = -fit.ty; object.normal_y = fit.tx; object.length = fit.length; object.width_rms = fit.width; object.coverage = fit.coverage; object.height = componentHeight(keys, columns, &object.base_z, &object.top_z);
  double mean_frames = 0.0; for (const Key2 &key : keys) mean_frames += columns.at(key).frame_support_sum/std::max(1u, columns.at(key).stable_voxels); mean_frames /= std::max<std::size_t>(1, keys.size());
  const double support = std::min(1.0, mean_frames/5.0), planarity = planar ? std::max(0.0, 1.0-object.width_rms/0.40) : 0.25, height_score = std::min(1.0, object.height/2.5);
  object.confidence = support*std::max(0.25, object.coverage)*planarity*height_score;
  object.type = forced_type.empty() ? lineType(fit, object.height, object.source_columns, object.confidence) : forced_type;
  object.geometry_state = "observed"; object.semantic_state = object.type == "uncertain_vertical_structure" || object.type == "vegetation_or_nonplanar" || object.type == "pole_or_trunk_candidate" ? "unknown" : "inferred"; object.completeness = "partial";
  if (object.type == "vegetation_or_nonplanar") object.reject_reason = "未满足窄带竖直平面条件；不作为立面或围墙候选";
  if (object.type == "uncertain_vertical_structure") object.reject_reason = "垂直观测存在，但平面长度/高度/连续性不足以作立面或围墙判断";
  return object;
}

uint32_t packedRgb(uint8_t red, uint8_t green, uint8_t blue) { return (static_cast<uint32_t>(red)<<16) | (static_cast<uint32_t>(green)<<8) | blue; }
struct VisualPoint { float x, y, z; uint32_t rgb; };
static_assert(sizeof(VisualPoint) == 16, "二进制PCD点记录必须为16字节");
void writePcdHeader(std::ofstream *output, uint64_t count) { *output << "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F U\nCOUNT 1 1 1 1\nWIDTH " << count << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS " << count << "\nDATA binary\n"; }
bool matchesType(const Object &object, const std::string &type) { return object.type == type; }
void writeLayer(const std::string &path, const std::unordered_map<Key3, Voxel, Key3Hash> &voxels, const std::unordered_map<Key2, int, Key2Hash> &assignments, const std::vector<Object> &objects, const Options &options, const std::string &type, uint32_t color, bool original_color) {
  uint64_t count = 0;
  for (const auto &item : voxels) if (stableVoxel(item.second, options)) { const auto owner = assignments.find(Key2{item.first.x, item.first.y}); if (owner != assignments.end() && (type.empty() || matchesType(objects[owner->second], type))) ++count; }
  std::ofstream output(path, std::ios::binary); if (!output) fail("无法写入PCD：" + path); writePcdHeader(&output, count);
  for (const auto &item : voxels) {
    if (!stableVoxel(item.second, options)) continue;
    const auto owner = assignments.find(Key2{item.first.x, item.first.y});
    if (owner == assignments.end() || (!type.empty() && !matchesType(objects[owner->second], type))) continue;
    uint32_t rgb = color; if (original_color && item.second.point_count) rgb = packedRgb(static_cast<uint8_t>(std::min(255.0, item.second.red_sum/item.second.point_count)), static_cast<uint8_t>(std::min(255.0, item.second.green_sum/item.second.point_count)), static_cast<uint8_t>(std::min(255.0, item.second.blue_sum/item.second.point_count)));
    const VisualPoint point{static_cast<float>((item.first.x+0.5)*options.voxel_m), static_cast<float>((item.first.y+0.5)*options.voxel_m), static_cast<float>((item.first.z+0.5)*options.voxel_m), rgb}; output.write(reinterpret_cast<const char *>(&point), sizeof(point));
  }
}

void writeRecords(const std::string &path, const std::vector<Object> &objects) {
  std::ofstream output(path); if (!output) fail("无法写入对象审计CSV：" + path); output << std::fixed << std::setprecision(6);
  output << "object_id,object_type,geometry_evidence_state,semantic_evidence_state,completeness,center_x,center_y,base_z,top_z,tangent_x,tangent_y,normal_x,normal_y,visible_length_m,plane_width_rms_m,visible_height_m,projection_coverage,confidence,source_column_count,source_stable_voxel_count,max_voxel_frame_count,voxel_frame_support_sum,mean_red,mean_green,mean_blue,nearest_stable_ground_distance_m,nearest_stable_ground_z,nearest_stage04_curb_distance_m,nearest_stage04_confirmed_sidewalk_distance_m,reject_reason\n";
  for (const Object &object : objects) {
    const double weight = std::max(1.0, object.color_weight);
    output << object.id << ',' << object.type << ',' << object.geometry_state << ',' << object.semantic_state << ',' << object.completeness << ',' << object.x << ',' << object.y << ',' << object.base_z << ',' << object.top_z << ',' << object.tangent_x << ',' << object.tangent_y << ',' << object.normal_x << ',' << object.normal_y << ',' << object.length << ',' << object.width_rms << ',' << object.height << ',' << object.coverage << ',' << object.confidence << ',' << object.source_columns << ',' << object.source_voxels << ',' << object.max_voxel_frame_count << ',' << object.voxel_frame_support_sum << ',' << object.red_sum/weight << ',' << object.green_sum/weight << ',' << object.blue_sum/weight << ',' << object.ground_distance << ',';
    if (std::isfinite(object.ground_z)) output << object.ground_z;
    output << ',' << object.curb_distance << ',' << object.sidewalk_distance << ',' << '"' << object.reject_reason << '"' << '\n';
  }
}
void writeReport(const std::string &path, const Options &options, const Stats &stats, const std::vector<Object> &objects) {
  std::unordered_map<std::string, int> counts; for (const Object &object : objects) ++counts[object.type];
  std::ofstream output(path); if (!output) fail("无法写入阶段5报告：" + path);
  output << std::fixed << std::setprecision(6) << "{\n  \"schema\": \"fast_livo_scene_pipeline_stage05_visible_scene_objects/v1\",\n  \"status\": \"complete\",\n  \"purpose\": \"从回环校正后的多帧真实观测中提取局部可见竖直对象；不补完整建筑。\",\n  \"parameters\": {\"voxel_m\": " << options.voxel_m << ", \"min_voxel_frames\": " << options.min_voxel_frames << ", \"min_voxel_effective_weight\": " << options.min_voxel_weight << "},\n  \"source_points_read\": " << stats.source_points << ",\n  \"valid_transformed_points\": " << stats.valid_points << ",\n  \"range_filtered_points\": " << stats.selected_points << ",\n  \"raw_voxels\": " << stats.raw_voxels << ",\n  \"stable_multiframe_voxels\": " << stats.stable_voxels << ",\n  \"vertical_candidate_columns\": " << stats.candidate_columns << ",\n  \"connected_components\": " << stats.components << ",\n  \"classified_observed_vertical_voxels\": " << stats.observed_vertical_voxels << ",\n  \"object_type_counts\": {";
  bool first = true; for (const auto &item : counts) { if (!first) output << ','; first = false; output << "\n    \"" << item.first << "\": " << item.second; } output << "\n  },\n  \"limits\": [\"visible_facade_candidate 与 wall_candidate 是由局部平面几何得到的语义候选，不能据此补全建筑轮廓。\", \"阶段4路沿和人行道仅写入最近邻语境字段，不参与对象类别或置信度计算。\", \"persistent_vertical_support.pcd 不作为本阶段的建筑输入；本阶段直接复核阶段1校正后的逐帧点云。\"]\n}\n";
}
void writeInputContract(const std::string &path, const Options &options) {
  std::ofstream output(path); if (!output) fail("无法写入输入契约：" + path);
  output << "{\n  \"schema\": \"fast_livo_scene_pipeline_stage05_input_contract/v1\",\n  \"read_only_inputs\": [\n    \"" << options.stage01_dir << "/pose_correction/frame_map_corrections.csv\",\n    \"" << options.run_dir << "/dense_rgb_cache/frames/*.pcd\",\n    \"" << options.stage02_dir << "/evidence/geometric_observation_grid.csv\",\n    \"" << options.stage04_r7_dir << "/evidence/curb_candidate_support.pcd\",\n    \"" << options.stage04_r7_dir << "/evidence/sidewalk_confirmed_support.pcd\"\n  ],\n  \"stage04_policy\": \"仅记录邻接语境；不得从道路、路沿、人行道推断建筑位置或方向。\"\n}\n";
}
void usage(const char *program) { std::cerr << "用法：" << program << " --run RUN_DIR --stage01 STAGE01_DIR --stage02 STAGE02_DIR --stage04-r7 STAGE04_R7_DIR --output OUTPUT_DIR [--voxel M] [--min-frames N]\n"; }
bool parseArguments(int argc, char **argv, Options *options) {
  for (int i = 1; i < argc; ++i) { const std::string key(argv[i]); if (key == "--help") { usage(argv[0]); std::exit(0); } if (i+1 >= argc) return false; const std::string value(argv[++i]); if (key == "--run") options->run_dir = value; else if (key == "--stage01") options->stage01_dir = value; else if (key == "--stage02") options->stage02_dir = value; else if (key == "--stage04-r7") options->stage04_r7_dir = value; else if (key == "--output") options->output_dir = value; else if (key == "--voxel") options->voxel_m = number(value, "voxel"); else if (key == "--min-frames") options->min_voxel_frames = static_cast<int>(wholeNumber(value, "min-frames")); else return false; }
  return !options->run_dir.empty() && !options->stage01_dir.empty() && !options->stage02_dir.empty() && !options->stage04_r7_dir.empty() && !options->output_dir.empty() && options->voxel_m >= 0.15 && options->voxel_m <= 0.50 && options->min_voxel_frames >= 2;
}

}  // namespace

int main(int argc, char **argv) {
  try {
    Options options; if (!parseArguments(argc, argv, &options)) { usage(argv[0]); return 2; }
    options.run_dir = absolutePath(options.run_dir); options.stage01_dir = absolutePath(options.stage01_dir); options.stage02_dir = absolutePath(options.stage02_dir); options.stage04_r7_dir = absolutePath(options.stage04_r7_dir); options.output_dir = absolutePath(options.output_dir);
    if (!directoryExists(options.run_dir) || !directoryExists(options.stage01_dir) || !directoryExists(options.stage02_dir) || !directoryExists(options.stage04_r7_dir)) fail("RUN_DIR或阶段1/2/4输入目录不存在");
    if (directoryExists(options.output_dir) || fileExists(options.output_dir)) fail("阶段5输出目录已存在，拒绝覆盖：" + options.output_dir);
    if (options.output_dir.find(options.run_dir + "/") != 0) fail("阶段5输出必须位于RUN_DIR内部");
    const std::vector<Frame> frames = loadFrames(options);
    const auto ground = loadGround(options.stage02_dir + "/evidence/geometric_observation_grid.csv");
    const ContextIndex curbs(readContextPcd(options.stage04_r7_dir + "/evidence/curb_candidate_support.pcd"));
    const ContextIndex sidewalks(readContextPcd(options.stage04_r7_dir + "/evidence/sidewalk_confirmed_support.pcd"));
    std::unordered_map<Key3, Voxel, Key3Hash> voxels; voxels.reserve(1800000); Stats stats;
    std::cout << "[scene-stage05] 开始读取并回环校正 " << frames.size() << " 帧。" << std::endl;
    for (std::size_t i = 0; i < frames.size(); ++i) { processFrame(frames[i], options, &voxels, &stats); if ((i+1)%500 == 0 || i+1 == frames.size()) std::cout << "[scene-stage05] 已处理 " << i+1 << '/' << frames.size() << " 帧，三维体素=" << voxels.size() << std::endl; }
    if (stats.source_points != stats.valid_points) fail("检测到无效点；当前阶段5拒绝在不完整点云上产生对象结论");
    stats.raw_voxels = voxels.size(); std::unordered_map<Key2, Column, Key2Hash> columns; columns.reserve(voxels.size()/3);
    for (const auto &item : voxels) {
      const Voxel &voxel = item.second; if (!stableVoxel(voxel, options)) continue; ++stats.stable_voxels;
      Column &column = columns[Key2{item.first.x, item.first.y}]; ++column.stable_voxels; column.frame_support_sum += voxel.frame_count; column.min_z = std::min(column.min_z, (item.first.z+0.5)*options.voxel_m); column.max_z = std::max(column.max_z, (item.first.z+0.5)*options.voxel_m); column.red_sum += voxel.red_sum; column.green_sum += voxel.green_sum; column.blue_sum += voxel.blue_sum; column.color_weight += voxel.point_count;
    }
    std::unordered_set<Key2, Key2Hash> candidates; candidates.reserve(columns.size());
    for (const auto &item : columns) if (item.second.stable_voxels >= 4 && item.second.max_z-item.second.min_z >= 1.0) candidates.insert(item.first);
    stats.candidate_columns = candidates.size();
    std::cout << "[scene-stage05] 稳定体素=" << stats.stable_voxels << "，垂直候选列=" << stats.candidate_columns << "。" << std::endl;
    // 只在 10 m 局部块内连通和拟合。这样立面拐角、树冠和远处对象不会被错误拼成一个全局候选。
    const int local_tile_cells = std::max(1, static_cast<int>(std::floor(10.0/options.voxel_m)));
    const std::size_t max_planar_component_columns = 600;
    std::unordered_set<Key2, Key2Hash> visited; visited.reserve(candidates.size()); std::unordered_map<Key2, int, Key2Hash> assignments; assignments.reserve(candidates.size()); std::vector<Object> objects;
    for (const Key2 &start : candidates) {
      if (!visited.insert(start).second) continue;
      std::queue<Key2> queue;
      queue.push(start);
      std::vector<Key2> component;
      const int tile_x = static_cast<int>(std::floor(static_cast<double>(start.x)/local_tile_cells));
      const int tile_y = static_cast<int>(std::floor(static_cast<double>(start.y)/local_tile_cells));
      while (!queue.empty()) {
        const Key2 current = queue.front(); queue.pop(); component.push_back(current);
        for (int dx = -1; dx <= 1; ++dx) for (int dy = -1; dy <= 1; ++dy) {
          if (!dx && !dy) continue;
          const Key2 next{current.x+dx, current.y+dy};
          if (static_cast<int>(std::floor(static_cast<double>(next.x)/local_tile_cells)) != tile_x || static_cast<int>(std::floor(static_cast<double>(next.y)/local_tile_cells)) != tile_y) continue;
          if (candidates.find(next) != candidates.end() && visited.insert(next).second) queue.push(next);
        }
      }
      ++stats.components;
      if (component.size() > max_planar_component_columns) {
        // 极大、面状连通块更符合树冠/灌木或混合遮挡；保留观测，但不从中强制剥离“建筑平面”。
        Object object = makeObject(static_cast<int>(objects.size()), component, columns, options.voxel_m, false, "vegetation_or_nonplanar");
        for (const Key2 &key : component) assignments[key] = object.id;
        objects.push_back(object);
        continue;
      }
      const std::vector<Segment> segments = extractSegments(component, options.voxel_m, static_cast<uint32_t>(stats.components)); std::unordered_set<Key2, Key2Hash> segmented;
      for (const Segment &segment : segments) { Object object = makeObject(static_cast<int>(objects.size()), segment.keys, columns, options.voxel_m, true, ""); for (const Key2 &key : segment.keys) { assignments[key] = object.id; segmented.insert(key); } objects.push_back(object); }
      std::vector<Key2> remainder; remainder.reserve(component.size()); for (const Key2 &key : component) if (segmented.find(key) == segmented.end()) remainder.push_back(key);
      if (!remainder.empty()) {
        double base = 0.0, top = 0.0; const double height = componentHeight(remainder, columns, &base, &top); const PlaneFit fit = fitPlane(remainder, options.voxel_m); std::string type;
        if (fit.length <= 1.2 && height >= 1.2) type = "pole_or_trunk_candidate";
        else if (fit.width >= 0.45 || remainder.size() >= 16) type = "vegetation_or_nonplanar";
        else type = "uncertain_vertical_structure";
        Object object = makeObject(static_cast<int>(objects.size()), remainder, columns, options.voxel_m, false, type); for (const Key2 &key : remainder) assignments[key] = object.id; objects.push_back(object);
      }
      if (stats.components % 2000 == 0) std::cout << "[scene-stage05] 已处理局部垂直连通块=" << stats.components << "。" << std::endl;
    }
    for (const auto &item : voxels) {
      if (!stableVoxel(item.second, options)) continue;
      const auto owner = assignments.find(Key2{item.first.x, item.first.y});
      if (owner == assignments.end()) continue;
      Object &object = objects[owner->second]; const Voxel &voxel = item.second;
      ++object.source_voxels; object.max_voxel_frame_count = std::max(object.max_voxel_frame_count, voxel.frame_count); object.voxel_frame_support_sum += voxel.frame_count; object.red_sum += voxel.red_sum; object.green_sum += voxel.green_sum; object.blue_sum += voxel.blue_sum; object.color_weight += voxel.point_count; object.to_sensor_x_sum += voxel.to_sensor_x_sum; object.to_sensor_y_sum += voxel.to_sensor_y_sum; ++stats.observed_vertical_voxels;
    }
    for (Object &object : objects) {
      // 法向朝向平均传感器视点；法向的正反是观测关系，不是道路关系。
      if (object.normal_x*object.to_sensor_x_sum + object.normal_y*object.to_sensor_y_sum < 0.0) { object.normal_x = -object.normal_x; object.normal_y = -object.normal_y; }
      object.ground_distance = nearestGround(ground, object.x, object.y, &object.ground_z); object.curb_distance = curbs.nearest(object.x, object.y, 10.0); object.sidewalk_distance = sidewalks.nearest(object.x, object.y, 10.0);
    }
    if (!createDirectories(options.output_dir + "/evidence") || !createDirectories(options.output_dir + "/validation")) fail("无法创建阶段5输出目录");
    writeLayer(options.output_dir + "/evidence/observed_vertical_voxel_support.pcd", voxels, assignments, objects, options, "", 0, true);
    writeLayer(options.output_dir + "/evidence/visible_facade_candidate_support.pcd", voxels, assignments, objects, options, "visible_facade_candidate", packedRgb(238,122,45), false);
    writeLayer(options.output_dir + "/evidence/wall_candidate_support.pcd", voxels, assignments, objects, options, "wall_candidate", packedRgb(245,205,65), false);
    writeLayer(options.output_dir + "/evidence/pole_or_trunk_candidate_support.pcd", voxels, assignments, objects, options, "pole_or_trunk_candidate", packedRgb(54,183,198), false);
    writeLayer(options.output_dir + "/evidence/vegetation_or_nonplanar_support.pcd", voxels, assignments, objects, options, "vegetation_or_nonplanar", packedRgb(65,178,93), false);
    writeLayer(options.output_dir + "/evidence/uncertain_vertical_structure_support.pcd", voxels, assignments, objects, options, "uncertain_vertical_structure", packedRgb(164,92,202), false);
    writeRecords(options.output_dir + "/evidence/visible_scene_object_records.csv", objects); writeInputContract(options.output_dir + "/stage05_input_contract.json", options); writeReport(options.output_dir + "/validation/stage05_visible_scene_objects_report.json", options, stats, objects);
    std::ofstream complete(options.output_dir + "/stage05_complete.json"); complete << "{\n  \"status\": \"complete\",\n  \"stage\": \"05_visible_scene_objects\",\n  \"object_records\": " << objects.size() << ",\n  \"report\": \"validation/stage05_visible_scene_objects_report.json\"\n}\n";
    std::cout << "[scene-stage05] 完成：对象=" << objects.size() << "，稳定体素=" << stats.stable_voxels << "。" << std::endl;
  } catch (const std::exception &error) { std::cerr << "阶段5失败：" << error.what() << std::endl; return 1; }
  return 0;
}
