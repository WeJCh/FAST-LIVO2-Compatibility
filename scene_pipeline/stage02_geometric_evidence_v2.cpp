/*
 * 阶段 2 V2：基于逐帧局部几何的一致性证据。
 *
 * 本程序不生成道路、路沿、人行道或建筑语义。它只保留三类更严格的事实：
 *   1. 贴合每帧局部地面模型、且在全局格中稳定复现的地面观测；
 *   2. 两侧地面均稳定、并沿切向连续的硬高差边；
 *   3. 在同一全局格中多帧保持相近上下端高度的垂直观测。
 *
 * V1 直接对全局格累积 z_min/z_max，动态幻影和墙面局部切片会被错误放大。
 * V2 必须先在 IMU 局部坐标系中估计近平地面，再做跨帧聚合。未知和遮挡
 * 区域仍然保持未知，绝不为了视觉连续性虚构地面。
 */

#include <algorithm>
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
const uint64_t kNoFrame = std::numeric_limits<uint64_t>::max();

struct Quaternion { double x = 0.0, y = 0.0, z = 0.0, w = 1.0; };
struct Transform { double tx = 0.0, ty = 0.0, tz = 0.0; Quaternion q; };

struct Frame {
  uint64_t id = 0, point_count = 0;
  double timestamp = 0.0;
  std::string cache_path, support;
  Transform map_odom, map_imu;
};

struct CellKey {
  int64_t x = 0, y = 0;
  bool operator==(const CellKey &other) const { return x == other.x && y == other.y; }
};
struct CellKeyHash {
  std::size_t operator()(const CellKey &key) const {
    return std::hash<int64_t>()(key.x) ^ (std::hash<int64_t>()(key.y) << 1);
  }
};

struct GroundEvidence {
  uint64_t point_count = 0, frame_count = 0, last_frame = kNoFrame;
  double effective_weight = 0.0;
  double z_min = std::numeric_limits<double>::infinity();
  double z_max = -std::numeric_limits<double>::infinity();
  double z_sum = 0.0, r_sum = 0.0, g_sum = 0.0, b_sum = 0.0;
  bool stable = false;
};

struct VerticalEvidence {
  uint64_t vertical_frame_count = 0, last_vertical_frame = kNoFrame;
  double lower_min = std::numeric_limits<double>::infinity();
  double lower_max = -std::numeric_limits<double>::infinity();
  double upper_min = std::numeric_limits<double>::infinity();
  double upper_max = -std::numeric_limits<double>::infinity();
  double lower_sum = 0.0, upper_sum = 0.0;
  bool persistent = false;
};

struct Cell {
  uint64_t point_count = 0, point_frame_count = 0, last_point_frame = kNoFrame;
  double effective_weight = 0.0;
  uint64_t free_ray_samples = 0, free_frame_count = 0, last_free_frame = kNoFrame;
  // 同一平面格可能同时看到道路与较高的人行道。必须按高度层累计，不能混为均值。
  std::vector<GroundEvidence> ground_layers;
  GroundEvidence ground;
  VerticalEvidence vertical;
};

struct FrameGround {
  uint64_t count = 0;
  double z_sum = 0.0, r_sum = 0.0, g_sum = 0.0, b_sum = 0.0;
};
struct FrameColumn {
  uint64_t count = 0;
  double z_min = std::numeric_limits<double>::infinity();
  double z_max = -std::numeric_limits<double>::infinity();
};
struct Point {
  double map_x, map_y, map_z, local_x, local_y, local_z;
  uint32_t rgb;
};
struct Plane { double a = 0.0, b = 0.0, c = 0.0; uint64_t inliers = 0; bool valid = false; };
struct EdgeKey {
  int64_t x = 0, y = 0;
  int orientation = 0;  // 0：跨 x 邻格的边；1：跨 y 邻格的边。
  bool operator==(const EdgeKey &other) const { return x == other.x && y == other.y && orientation == other.orientation; }
};
struct EdgeKeyHash {
  std::size_t operator()(const EdgeKey &key) const {
    return CellKeyHash()(CellKey{key.x, key.y}) ^ (std::hash<int>()(key.orientation) << 2);
  }
};
struct Edge { EdgeKey key; double z = 0.0; };

struct PcdLayout {
  uint64_t points = 0;
  std::size_t point_step = 0, x_offset = 0, y_offset = 0, z_offset = 0, rgb_offset = 0;
  bool x_found = false, y_found = false, z_found = false, rgb_found = false;
  std::streampos data_offset;
};
struct Options {
  std::string run_dir, stage01_dir, output_dir;
  double grid_m = 0.20;
  double local_ground_grid_m = 0.75;
  double plane_residual_m = 0.10;
  int ground_min_frames = 3;
  int vertical_min_frames = 5;
  double ray_step_m = 0.40, ray_max_m = 20.0;
  uint64_t ray_stride = 64;
};
struct Stats { uint64_t frames = 0, source_points = 0, valid_points = 0, frames_with_ground_plane = 0, sampled_rays = 0; };

typedef std::unordered_map<CellKey, Cell, CellKeyHash> Cells;

void fail(const std::string &message) { throw std::runtime_error(message); }
bool fileExists(const std::string &path) { struct stat s; return ::stat(path.c_str(), &s) == 0 && S_ISREG(s.st_mode) && s.st_size > 0; }
bool directoryExists(const std::string &path) { struct stat s; return ::stat(path.c_str(), &s) == 0 && S_ISDIR(s.st_mode); }
bool createDirectories(const std::string &path) {
  if (path.empty()) return false;
  std::string now; std::size_t begin = 0;
  if (path.front() == '/') { now = "/"; begin = 1; }
  while (begin <= path.size()) {
    const std::size_t end = path.find('/', begin); const std::string part = path.substr(begin, end - begin);
    if (!part.empty()) { if (!now.empty() && now.back() != '/') now += '/'; now += part; if (::mkdir(now.c_str(), 0755) && errno != EEXIST) return false; }
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
  if (!end || end == text.c_str() || *end || !std::isfinite(value)) {
    fail("无效数值（" + label + "）：" + text);
  }
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
  const double n = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w); if (n < kEpsilon) fail("零四元数");
  q.x /= n; q.y /= n; q.z /= n; q.w /= n; return q;
}
void rotate(const Quaternion &q, double x, double y, double z, double *rx, double *ry, double *rz) {
  const double tx = 2.0 * (q.y*z - q.z*y), ty = 2.0 * (q.z*x - q.x*z), tz = 2.0 * (q.x*y - q.y*x);
  *rx = x + q.w*tx + (q.y*tz - q.z*ty); *ry = y + q.w*ty + (q.z*tx - q.x*tz); *rz = z + q.w*tz + (q.x*ty - q.y*tx);
}
void apply(const Transform &t, double x, double y, double z, double *ox, double *oy, double *oz) { rotate(t.q, x, y, z, ox, oy, oz); *ox += t.tx; *oy += t.ty; *oz += t.tz; }
void inverseApply(const Transform &t, double x, double y, double z, double *ox, double *oy, double *oz) {
  Quaternion inv{-t.q.x, -t.q.y, -t.q.z, t.q.w}; rotate(inv, x - t.tx, y - t.ty, z - t.tz, ox, oy, oz);
}
CellKey keyOf(double x, double y, double grid) { return CellKey{static_cast<int64_t>(std::floor(x / grid)), static_cast<int64_t>(std::floor(y / grid))}; }
double frameWeight(const std::string &support) { return support == "interpolated" ? 1.0 : 0.5; }

std::vector<Frame> loadFrames(const Options &options) {
  const std::string path = options.stage01_dir + "/pose_correction/frame_map_corrections.csv";
  if (!fileExists(path)) fail("缺少阶段 1 校正清单：" + path);
  std::ifstream in(path); std::string line; if (!std::getline(in, line)) fail("阶段 1 校正清单为空"); const auto cols = csvHeader(line);
  const char *need[] = {"frame_id", "vio_timestamp", "point_count", "cache_pcd", "map_odom_tx", "map_odom_ty", "map_odom_tz", "map_odom_qx", "map_odom_qy", "map_odom_qz", "map_odom_qw", "map_imu_tx", "map_imu_ty", "map_imu_tz", "map_imu_qx", "map_imu_qy", "map_imu_qz", "map_imu_qw", "support"};
  for (const char *name : need) if (cols.find(name) == cols.end()) fail("阶段 1 清单缺少列：" + std::string(name));
  std::vector<Frame> frames; uint64_t previous = 0; bool first = true;
  while (std::getline(in, line)) {
    if (line.empty()) continue;
    const std::vector<std::string> row = split(line, ',');
    Frame f;
    f.id = wholeNumber(field(row, cols, "frame_id"), "frame_id"); f.timestamp = number(field(row, cols, "vio_timestamp"), "vio_timestamp");
    f.point_count = wholeNumber(field(row, cols, "point_count"), "point_count"); f.cache_path = field(row, cols, "cache_pcd"); f.support = field(row, cols, "support");
    f.map_odom.tx = number(field(row, cols, "map_odom_tx"), "map_odom_tx"); f.map_odom.ty = number(field(row, cols, "map_odom_ty"), "map_odom_ty"); f.map_odom.tz = number(field(row, cols, "map_odom_tz"), "map_odom_tz");
    f.map_odom.q = normalized(Quaternion{number(field(row, cols, "map_odom_qx"), "map_odom_qx"), number(field(row, cols, "map_odom_qy"), "map_odom_qy"), number(field(row, cols, "map_odom_qz"), "map_odom_qz"), number(field(row, cols, "map_odom_qw"), "map_odom_qw")});
    f.map_imu.tx = number(field(row, cols, "map_imu_tx"), "map_imu_tx"); f.map_imu.ty = number(field(row, cols, "map_imu_ty"), "map_imu_ty"); f.map_imu.tz = number(field(row, cols, "map_imu_tz"), "map_imu_tz");
    f.map_imu.q = normalized(Quaternion{number(field(row, cols, "map_imu_qx"), "map_imu_qx"), number(field(row, cols, "map_imu_qy"), "map_imu_qy"), number(field(row, cols, "map_imu_qz"), "map_imu_qz"), number(field(row, cols, "map_imu_qw"), "map_imu_qw")});
    if (!first && f.id != previous + 1) fail("阶段 1 帧编号不连续");
    previous = f.id;
    first = false;
    frames.push_back(f);
  }
  if (frames.empty()) fail("阶段 1 中没有可处理帧");
  return frames;
}

PcdLayout parsePcdLayout(const std::string &path) {
  std::ifstream in(path, std::ios::binary); if (!in) fail("无法打开 PCD：" + path); PcdLayout l;
  std::vector<std::string> names, types; std::vector<std::size_t> sizes, counts; uint64_t width = 0, height = 0; std::string line; bool data = false;
  while (std::getline(in, line)) {
    const std::vector<std::string> t = split(line, ' '); if (t.empty() || t[0].empty() || t[0][0] == '#') continue;
    if (t[0] == "FIELDS") names.assign(t.begin() + 1, t.end()); else if (t[0] == "SIZE") for (std::size_t i = 1; i < t.size(); ++i) sizes.push_back(static_cast<std::size_t>(wholeNumber(t[i], "PCD SIZE")));
    else if (t[0] == "TYPE") types.assign(t.begin() + 1, t.end()); else if (t[0] == "COUNT") for (std::size_t i = 1; i < t.size(); ++i) counts.push_back(static_cast<std::size_t>(wholeNumber(t[i], "PCD COUNT")));
    else if (t[0] == "WIDTH" && t.size() == 2) width = wholeNumber(t[1], "PCD WIDTH"); else if (t[0] == "HEIGHT" && t.size() == 2) height = wholeNumber(t[1], "PCD HEIGHT"); else if (t[0] == "POINTS" && t.size() == 2) l.points = wholeNumber(t[1], "PCD POINTS");
    else if (t[0] == "DATA") { if (t.size() != 2 || t[1] != "binary") fail("只支持 binary PCD：" + path); l.data_offset = in.tellg(); data = true; break; }
  }
  if (!data || names.empty() || names.size() != sizes.size() || names.size() != types.size()) fail("无效 PCD 头：" + path);
  if (counts.empty()) counts.assign(names.size(), 1);
  if (counts.size() != names.size()) fail("无效 PCD COUNT：" + path);
  for (std::size_t i = 0; i < names.size(); ++i) { if (counts[i] != 1 || sizes[i] != 4) fail("PCD 字段必须为标量 32 位：" + path); const std::size_t off = l.point_step; if (names[i] == "x") { l.x_offset = off; l.x_found = true; } if (names[i] == "y") { l.y_offset = off; l.y_found = true; } if (names[i] == "z") { l.z_offset = off; l.z_found = true; } if (names[i] == "rgb") { l.rgb_offset = off; l.rgb_found = true; } l.point_step += sizes[i]; }
  if (!l.points) l.points = width * height;
  if (!l.points || !l.x_found || !l.y_found || !l.z_found || !l.rgb_found || l.point_step != 16) fail("PCD 必须为 x y z rgb 四个 32 位字段：" + path);
  return l;
}
float floatAt(const std::vector<char> &b, std::size_t off) { float v; std::memcpy(&v, b.data() + off, sizeof(v)); return v; }
uint32_t rgbAt(const std::vector<char> &b, std::size_t off) { uint32_t v; std::memcpy(&v, b.data() + off, sizeof(v)); return v; }

// 每个局部 0.75 m 格只保留最低候选，防止高密度墙面主导 RANSAC 抽样。
Plane estimateLocalGround(const std::vector<Point> &points, const Options &o, uint64_t seed, const Plane *excluded_plane) {
  std::unordered_map<CellKey, std::size_t, CellKeyHash> lowest;
  for (std::size_t i = 0; i < points.size(); ++i) {
    const Point &p = points[i]; const double r = std::hypot(p.local_x, p.local_y);
    if (r < 1.5 || r > 30.0 || p.local_z < -1.5 || p.local_z > 0.25) continue;
    // 第二个地形面只能从与首平面明确分离的候选中拟合，避免 RANSAC 重复同一平面。
    if (excluded_plane && std::abs(p.local_z - (excluded_plane->a * p.local_x + excluded_plane->b * p.local_y + excluded_plane->c)) <= 0.13) continue;
    const CellKey key = keyOf(p.local_x, p.local_y, o.local_ground_grid_m); const auto it = lowest.find(key);
    if (it == lowest.end() || p.local_z < points[it->second].local_z) lowest[key] = i;
  }
  std::vector<std::size_t> candidates; candidates.reserve(lowest.size()); for (const auto &e : lowest) candidates.push_back(e.second);
  if (candidates.size() < 24) return Plane();
  Plane best; uint64_t state = seed * 6364136223846793005ULL + 1442695040888963407ULL;
  const int iterations = std::min(240, std::max(80, static_cast<int>(candidates.size() / 2)));
  for (int it = 0; it < iterations; ++it) {
    auto next = [&state, &candidates]() { state = state * 2862933555777941757ULL + 3037000493ULL; return candidates[static_cast<std::size_t>((state >> 16) % candidates.size())]; };
    const Point &p1 = points[next()], &p2 = points[next()], &p3 = points[next()];
    const double ux = p2.local_x-p1.local_x, uy = p2.local_y-p1.local_y, uz = p2.local_z-p1.local_z;
    const double vx = p3.local_x-p1.local_x, vy = p3.local_y-p1.local_y, vz = p3.local_z-p1.local_z;
    double nx = uy*vz-uz*vy, ny = uz*vx-ux*vz, nz = ux*vy-uy*vx; const double norm = std::sqrt(nx*nx+ny*ny+nz*nz);
    if (norm < kEpsilon) continue;
    nx /= norm; ny /= norm; nz /= norm;
    if (nz < 0.0) { nx = -nx; ny = -ny; nz = -nz; }
    if (nz < 0.90) continue;
    const double a = -nx/nz, b = -ny/nz, c = (nx*p1.local_x + ny*p1.local_y + nz*p1.local_z) / nz;
    if (c < -1.5 || c > -0.05) continue;
    uint64_t inliers = 0; for (const std::size_t index : candidates) { const Point &p = points[index]; if (std::abs(p.local_z - (a*p.local_x + b*p.local_y + c)) <= o.plane_residual_m) ++inliers; }
    if (inliers > best.inliers) { best.a = a; best.b = b; best.c = c; best.inliers = inliers; best.valid = true; }
  }
  if (!best.valid || best.inliers < 20) return Plane();
  return best;
}

void addFreeRay(Cells *cells, const Frame &frame, double end_x, double end_y, const Options &o) {
  const double dx = end_x - frame.map_imu.tx, dy = end_y - frame.map_imu.ty, d = std::hypot(dx, dy); if (d < o.ray_step_m || !std::isfinite(d)) return;
  const double limit = std::min(o.ray_max_m, std::max(0.0, d - 0.20));
  for (double t = o.ray_step_m; t < limit; t += o.ray_step_m) { Cell &c = (*cells)[keyOf(frame.map_imu.tx + dx*t/d, frame.map_imu.ty + dy*t/d, o.grid_m)]; ++c.free_ray_samples; if (c.last_free_frame != frame.id) { c.last_free_frame = frame.id; ++c.free_frame_count; } }
}

void processFrame(const Frame &frame, Cells *cells, const Options &o, Stats *stats) {
  const std::string path = o.run_dir + "/" + frame.cache_path; if (!fileExists(path)) fail("缺少缓存 PCD：" + path); const PcdLayout layout = parsePcdLayout(path); if (layout.points != frame.point_count) fail("PCD 点数与阶段 1 清单不一致：" + path);
  std::ifstream in(path, std::ios::binary); in.seekg(layout.data_offset); std::vector<char> buffer(layout.point_step); std::vector<Point> points; points.reserve(layout.points);
  for (uint64_t i = 0; i < layout.points; ++i) {
    ++stats->source_points; in.read(buffer.data(), static_cast<std::streamsize>(buffer.size())); if (!in) fail("截断的 PCD：" + path);
    const float rx = floatAt(buffer, layout.x_offset), ry = floatAt(buffer, layout.y_offset), rz = floatAt(buffer, layout.z_offset); if (!std::isfinite(rx) || !std::isfinite(ry) || !std::isfinite(rz)) continue;
    Point p; apply(frame.map_odom, rx, ry, rz, &p.map_x, &p.map_y, &p.map_z); if (!std::isfinite(p.map_x) || !std::isfinite(p.map_y) || !std::isfinite(p.map_z)) continue;
    inverseApply(frame.map_imu, p.map_x, p.map_y, p.map_z, &p.local_x, &p.local_y, &p.local_z); p.rgb = rgbAt(buffer, layout.rgb_offset); points.push_back(p); ++stats->valid_points;
  }
  const Plane primary_plane = estimateLocalGround(points, o, frame.id, nullptr);
  const Plane secondary_plane = primary_plane.valid ? estimateLocalGround(points, o, frame.id + 0x9e3779b97f4a7c15ULL, &primary_plane) : Plane();
  if (primary_plane.valid) ++stats->frames_with_ground_plane;
  std::unordered_map<CellKey, FrameColumn, CellKeyHash> columns; std::unordered_map<CellKey, FrameGround, CellKeyHash> grounds; columns.reserve(points.size()); grounds.reserve(points.size() / 4);
  for (std::size_t i = 0; i < points.size(); ++i) {
    const Point &p = points[i]; const CellKey key = keyOf(p.map_x, p.map_y, o.grid_m); Cell &cell = (*cells)[key]; ++cell.point_count; if (cell.last_point_frame != frame.id) { cell.last_point_frame = frame.id; ++cell.point_frame_count; cell.effective_weight += frameWeight(frame.support); }
    const double range = std::hypot(p.local_x, p.local_y); if (range >= 1.0 && range <= 30.0) { FrameColumn &col = columns[key]; ++col.count; col.z_min = std::min(col.z_min, p.map_z); col.z_max = std::max(col.z_max, p.map_z); }
    if (o.ray_stride && i % o.ray_stride == 0) { addFreeRay(cells, frame, p.map_x, p.map_y, o); ++stats->sampled_rays; }
    if ((!primary_plane.valid && !secondary_plane.valid) || range < 1.5 || range > 30.0) continue;
    const bool near_primary = primary_plane.valid && std::abs(p.local_z - (primary_plane.a*p.local_x + primary_plane.b*p.local_y + primary_plane.c)) <= o.plane_residual_m;
    const bool near_secondary = secondary_plane.valid && std::abs(p.local_z - (secondary_plane.a*p.local_x + secondary_plane.b*p.local_y + secondary_plane.c)) <= o.plane_residual_m;
    if (!near_primary && !near_secondary) continue;
    FrameGround &g = grounds[key]; ++g.count; g.z_sum += p.map_z; g.r_sum += (p.rgb >> 16) & 0xffU; g.g_sum += (p.rgb >> 8) & 0xffU; g.b_sum += p.rgb & 0xffU;
  }
  for (const auto &item : columns) {
    const FrameColumn &col = item.second;
    if (col.count < 4 || col.z_max - col.z_min < 0.80) continue;
    VerticalEvidence &v = (*cells)[item.first].vertical;
    if (v.last_vertical_frame == frame.id) continue;
    v.last_vertical_frame = frame.id;
    ++v.vertical_frame_count;
    v.lower_min = std::min(v.lower_min, col.z_min);
    v.lower_max = std::max(v.lower_max, col.z_min);
    v.upper_min = std::min(v.upper_min, col.z_max);
    v.upper_max = std::max(v.upper_max, col.z_max);
    v.lower_sum += col.z_min;
    v.upper_sum += col.z_max;
  }
  for (const auto &item : grounds) {
    const FrameGround &src = item.second;
    Cell &cell = (*cells)[item.first];
    const double z = src.z_sum / src.count;
    GroundEvidence *layer = nullptr;
    double closest_height = std::numeric_limits<double>::infinity();
    for (GroundEvidence &candidate : cell.ground_layers) {
      const double candidate_z = candidate.z_sum / candidate.frame_count;
      const double height_difference = std::abs(z - candidate_z);
      if (height_difference < closest_height) {
        closest_height = height_difference;
        layer = &candidate;
      }
    }
    // 8 cm 小于路沿常见高差，也足以容纳同一静态地面的帧间估计误差。
    if (!layer || closest_height > 0.08) {
      cell.ground_layers.push_back(GroundEvidence());
      layer = &cell.ground_layers.back();
    }
    layer->point_count += src.count;
    if (layer->last_frame != frame.id) {
      layer->last_frame = frame.id;
      ++layer->frame_count;
      layer->effective_weight += frameWeight(frame.support);
      layer->z_min = std::min(layer->z_min, z);
      layer->z_max = std::max(layer->z_max, z);
      layer->z_sum += z;
      layer->r_sum += src.r_sum / src.count;
      layer->g_sum += src.g_sum / src.count;
      layer->b_sum += src.b_sum / src.count;
    }
  }
}

bool stableGround(const GroundEvidence &g, const Options &o) { return g.frame_count >= static_cast<uint64_t>(o.ground_min_frames) && g.effective_weight >= 2.0 && std::isfinite(g.z_min) && g.z_max - g.z_min <= 0.15; }
bool persistentVertical(const VerticalEvidence &v, const Options &o) {
  return v.vertical_frame_count >= static_cast<uint64_t>(o.vertical_min_frames) && std::isfinite(v.lower_min) && v.upper_max - v.lower_min >= 0.80 && v.lower_max - v.lower_min <= 0.25 && v.upper_max - v.upper_min <= 0.25;
}
void classify(Cells *cells, const Options &o) {
  for (auto &item : *cells) {
    Cell &cell = item.second;
    cell.ground = GroundEvidence();
    for (GroundEvidence &layer : cell.ground_layers) {
      layer.stable = stableGround(layer, o);
      if (!layer.stable) continue;
      if (!cell.ground.stable || layer.frame_count > cell.ground.frame_count ||
          (layer.frame_count == cell.ground.frame_count && layer.effective_weight > cell.ground.effective_weight)) {
        cell.ground = layer;
      }
    }
    cell.vertical.persistent = persistentVertical(cell.vertical, o);
  }
}

std::vector<Edge> findContinuousEdges(const Cells &cells) {
  std::unordered_map<EdgeKey, Edge, EdgeKeyHash> candidates;
  for (const auto &item : cells) {
    const CellKey a = item.first; const GroundEvidence &ga = item.second.ground; if (!ga.stable) continue;
    for (int axis = 0; axis < 2; ++axis) {
      const CellKey b{a.x + (axis == 0 ? 1 : 0), a.y + (axis == 1 ? 1 : 0)}; const auto neighbor = cells.find(b); if (neighbor == cells.end() || !neighbor->second.ground.stable) continue;
      const double za = ga.z_sum / ga.frame_count, zb = neighbor->second.ground.z_sum / neighbor->second.ground.frame_count, delta = std::abs(za-zb); if (delta < 0.06 || delta > 0.35) continue;
      candidates.insert(std::make_pair(EdgeKey{a.x, a.y, axis}, Edge{EdgeKey{a.x, a.y, axis}, (za+zb)*0.5}));
    }
  }
  std::vector<Edge> candidate_list;
  candidate_list.reserve(candidates.size());
  for (const auto &item : candidates) candidate_list.push_back(item.second);
  // 用半网格单位索引边中点；避免在大量候选边上做 O(N²) 的两两距离比较。
  std::unordered_map<CellKey, std::vector<std::size_t>, CellKeyHash> centers;
  centers.reserve(candidate_list.size());
  for (std::size_t index = 0; index < candidate_list.size(); ++index) {
    const EdgeKey &key = candidate_list[index].key;
    const CellKey center{2 * key.x + (key.orientation == 0 ? 2 : 1),
                         2 * key.y + (key.orientation == 1 ? 2 : 1)};
    centers[center].push_back(index);
  }
  std::vector<Edge> result;
  std::vector<bool> visited(candidate_list.size(), false);
  for (std::size_t index = 0; index < candidate_list.size(); ++index) {
    if (visited[index]) continue;
    std::vector<std::size_t> stack(1, index);
    std::vector<Edge> component;
    visited[index] = true;
    while (!stack.empty()) {
      const std::size_t now = stack.back();
      stack.pop_back();
      component.push_back(candidate_list[now]);
      const Edge &edge = candidate_list[now];
      const EdgeKey &edge_key = edge.key;
      const CellKey center{2 * edge_key.x + (edge_key.orientation == 0 ? 2 : 1),
                           2 * edge_key.y + (edge_key.orientation == 1 ? 2 : 1)};
      for (int du = -3; du <= 3; ++du) {
        for (int dv = -3; dv <= 3; ++dv) {
          // 半网格整数坐标下，平方距离不超过 12 对应约 0.35 m。
          if (du * du + dv * dv > 12) continue;
          const auto nearby = centers.find(CellKey{center.x + du, center.y + dv});
          if (nearby == centers.end()) continue;
          for (const std::size_t other : nearby->second) {
            if (visited[other]) continue;
            const Edge &candidate = candidate_list[other];
            // 允许斜向“台阶状”边界相连，但要求边界高程本身一致。
            if (std::abs(edge.z - candidate.z) <= 0.15) {
              visited[other] = true;
              stack.push_back(other);
            }
          }
        }
      }
    }
    // 少于 0.8 m 的孤立变化更可能是噪声、枝叶或错配，不作为硬边证据输出。
    if (component.size() >= 4) result.insert(result.end(), component.begin(), component.end());
  }
  return result;
}

uint32_t packedRgb(uint8_t r, uint8_t g, uint8_t b) { return (static_cast<uint32_t>(r) << 16) | (static_cast<uint32_t>(g) << 8) | b; }
struct VisualPoint { float x, y, z; uint32_t rgb; };
static_assert(sizeof(VisualPoint) == 16, "二进制 PCD 点记录必须为 16 字节");
void writePcd(const std::string &path, const std::vector<VisualPoint> &points) {
  std::ofstream out(path, std::ios::binary); if (!out) fail("无法写入 PCD：" + path);
  out << "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F U\nCOUNT 1 1 1 1\nWIDTH " << points.size() << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS " << points.size() << "\nDATA binary\n";
  for (const VisualPoint &p : points) out.write(reinterpret_cast<const char *>(&p), sizeof(p));
}
void writeGrid(const std::string &path, const Cells &cells, const Options &o) {
  std::ofstream out(path); if (!out) fail("无法写入证据网格：" + path); out << std::fixed << std::setprecision(6);
  out << "cell_ix,cell_iy,x,y,raw_point_count,raw_point_frame_count,effective_frame_weight,ground_point_count,ground_frame_count,ground_effective_frame_weight,ground_mean_z,ground_z_span_m,stable_ground,vertical_frame_count,vertical_mean_lower_z,vertical_mean_upper_z,vertical_lower_jitter_m,vertical_upper_jitter_m,persistent_vertical,free_ray_samples,free_frame_count,observation_confidence\n";
  for (const auto &item : cells) {
    const Cell &c = item.second; const GroundEvidence &g = c.ground; const VerticalEvidence &v = c.vertical; const bool has_g = g.frame_count > 0; const bool has_v = v.vertical_frame_count > 0; const double confidence = std::min(1.0, c.effective_weight / 4.0);
    out << item.first.x << ',' << item.first.y << ',' << (item.first.x+.5)*o.grid_m << ',' << (item.first.y+.5)*o.grid_m << ',' << c.point_count << ',' << c.point_frame_count << ',' << c.effective_weight << ',' << g.point_count << ',' << g.frame_count << ',' << g.effective_weight;
    if (has_g) out << ',' << g.z_sum/g.frame_count << ',' << g.z_max-g.z_min; else out << ",,";
    out << ',' << (g.stable ? 1 : 0) << ',' << v.vertical_frame_count;
    if (has_v) out << ',' << v.lower_sum/v.vertical_frame_count << ',' << v.upper_sum/v.vertical_frame_count << ',' << v.lower_max-v.lower_min << ',' << v.upper_max-v.upper_min; else out << ",,,,";
    out << ',' << (v.persistent ? 1 : 0) << ',' << c.free_ray_samples << ',' << c.free_frame_count << ',' << confidence << '\n';
  }
}
void writeTrajectory(const std::string &path, const std::vector<Frame> &frames) {
  std::ofstream out(path); if (!out) fail("无法写入轨迹：" + path); out << "frame_id,vio_timestamp,map_imu_tx,map_imu_ty,map_imu_tz,pose_correction_support\n" << std::fixed << std::setprecision(9);
  for (const Frame &f : frames) out << f.id << ',' << f.timestamp << ',' << f.map_imu.tx << ',' << f.map_imu.ty << ',' << f.map_imu.tz << ',' << f.support << '\n';
}
void writeReport(const std::string &path, const Options &o, const Stats &s, const Cells &cells, std::size_t edge_count) {
  uint64_t ground = 0, vertical = 0, free = 0; for (const auto &item : cells) { if (item.second.ground.stable) ++ground; if (item.second.vertical.persistent) ++vertical; if (item.second.free_frame_count) ++free; }
  std::ofstream out(path); if (!out) fail("无法写入阶段 2 V2 报告：" + path); out << std::fixed << std::setprecision(6)
    << "{\n  \"schema\": \"fast_livo_scene_pipeline_stage02_geometric_evidence/v2\",\n  \"purpose\": \"逐帧局部几何和跨帧一致性证据；不是道路、路沿或建筑语义。\",\n"
    << "  \"grid_m\": " << o.grid_m << ",\n  \"input_frames\": " << s.frames << ",\n  \"frames_with_local_ground_plane\": " << s.frames_with_ground_plane << ",\n  \"source_points_read\": " << s.source_points << ",\n  \"valid_transformed_points\": " << s.valid_points << ",\n  \"grid_cells\": " << cells.size() << ",\n  \"stable_ground_cells\": " << ground << ",\n  \"continuous_hard_height_edges\": " << edge_count << ",\n  \"persistent_vertical_cells\": " << vertical << ",\n  \"free_space_cells\": " << free << ",\n  \"sampled_free_rays\": " << s.sampled_rays << ",\n"
    << "  \"parameters\": {\"ground_min_frames\": " << o.ground_min_frames << ", \"vertical_min_frames\": " << o.vertical_min_frames << ", \"ground_plane_residual_m\": " << o.plane_residual_m << "},\n"
    << "  \"limits\": [\"严格地面缺失表示未得到足够静态证据，不代表不存在道路\", \"连续硬高差不是路沿语义\", \"持续垂直结构仍可能是树木、杆件或静态非建筑物\", \"动态物只被几何一致性抑制，不能保证完全去除\"]\n}\n";
}
void usage(const char *program) { std::cerr << "用法：" << program << " --run RUN_DIR --stage01 STAGE01_DIR --output OUTPUT_DIR [--grid M] [--ground-min-frames N] [--vertical-min-frames N]\n"; }
bool parseArguments(int argc, char **argv, Options *o) {
  for (int i = 1; i < argc; ++i) { const std::string key(argv[i]); if (key == "--help") { usage(argv[0]); std::exit(0); } if (i+1 >= argc) return false; const std::string value(argv[++i]); if (key == "--run") o->run_dir = value; else if (key == "--stage01") o->stage01_dir = value; else if (key == "--output") o->output_dir = value; else if (key == "--grid") o->grid_m = number(value, "grid"); else if (key == "--ground-min-frames") o->ground_min_frames = static_cast<int>(wholeNumber(value, "ground-min-frames")); else if (key == "--vertical-min-frames") o->vertical_min_frames = static_cast<int>(wholeNumber(value, "vertical-min-frames")); else return false; }
  return !o->run_dir.empty() && !o->stage01_dir.empty() && !o->output_dir.empty() && o->grid_m > 0.0 && o->ground_min_frames >= 2 && o->vertical_min_frames >= 3;
}

}  // 匿名命名空间

int main(int argc, char **argv) {
  try {
    Options o; if (!parseArguments(argc, argv, &o)) { usage(argv[0]); return 2; } o.run_dir = absolutePath(o.run_dir); o.stage01_dir = absolutePath(o.stage01_dir); o.output_dir = absolutePath(o.output_dir);
    if (!directoryExists(o.run_dir) || !directoryExists(o.stage01_dir)) fail("RUN_DIR 或阶段 1 输出目录不存在");
    if (directoryExists(o.output_dir)) fail("阶段 2 V2 输出目录已存在，拒绝覆盖：" + o.output_dir);
    if (o.output_dir.find(o.run_dir + "/") != 0) fail("阶段 2 V2 输出必须位于 RUN_DIR 内部");
    const std::vector<Frame> frames = loadFrames(o); Cells cells; cells.reserve(600000); Stats stats;
    std::cout << "[scene-stage02-v2] 开始处理 " << frames.size() << " 帧。" << std::endl;
    for (std::size_t i = 0; i < frames.size(); ++i) { processFrame(frames[i], &cells, o, &stats); if ((i+1)%500 == 0 || i+1 == frames.size()) std::cout << "[scene-stage02-v2] 已处理 " << i+1 << '/' << frames.size() << " 帧，网格单元=" << cells.size() << std::endl; }
    stats.frames = frames.size();
    if (stats.source_points != stats.valid_points) fail("检测到无效点，当前 V2 要求输入缓存所有点均有效；请先检查缓存完整性");
    classify(&cells, o);
    const std::vector<Edge> edges = findContinuousEdges(cells);
    if (!createDirectories(o.output_dir + "/evidence") || !createDirectories(o.output_dir + "/validation")) fail("无法创建阶段 2 V2 输出目录");
    writeGrid(o.output_dir + "/evidence/geometric_observation_grid.csv", cells, o);
    writeTrajectory(o.output_dir + "/evidence/map_trajectory_samples.csv", frames);
    std::vector<VisualPoint> ground, vertical, hard; ground.reserve(cells.size()/4); vertical.reserve(cells.size()/3); hard.reserve(edges.size());
    for (const auto &item : cells) { const Cell &c = item.second; const float x = static_cast<float>((item.first.x+.5)*o.grid_m), y = static_cast<float>((item.first.y+.5)*o.grid_m); if (c.ground.stable) ground.push_back(VisualPoint{x,y,static_cast<float>(c.ground.z_sum/c.ground.frame_count),packedRgb(45,190,100)}); if (c.vertical.persistent) { const double lo = c.vertical.lower_sum/c.vertical.vertical_frame_count, hi = c.vertical.upper_sum/c.vertical.vertical_frame_count; for (double z = lo; z <= hi + 0.01; z += 0.20) vertical.push_back(VisualPoint{x,y,static_cast<float>(z),packedRgb(128,84,210)}); } }
    for (const Edge &e : edges) { const float x = static_cast<float>((e.key.x + (e.key.orientation == 0 ? 1.0 : .5))*o.grid_m), y = static_cast<float>((e.key.y + (e.key.orientation == 1 ? 1.0 : .5))*o.grid_m); hard.push_back(VisualPoint{x,y,static_cast<float>(e.z),packedRgb(244,150,45)}); }
    writePcd(o.output_dir + "/evidence/stable_ground_support.pcd", ground); writePcd(o.output_dir + "/evidence/continuous_hard_height_edge_support.pcd", hard); writePcd(o.output_dir + "/evidence/persistent_vertical_support.pcd", vertical); writeReport(o.output_dir + "/validation/stage02_geometric_evidence_v2_report.json", o, stats, cells, edges.size());
    std::ofstream complete(o.output_dir + "/stage02_v2_complete.json"); complete << "{\n  \"status\": \"complete\",\n  \"stage\": \"2_geometric_evidence_v2\",\n  \"source_stage01\": \"" << o.stage01_dir << "\",\n  \"pcd_policy\": \"只读逐帧缓存，按每帧姿态惰性校正；不改写建图输出。\"\n}\n";
    std::cout << "[scene-stage02-v2] 完成：有效点=" << stats.valid_points << "，严格地面/连续硬边/持续垂直证据已写入 " << o.output_dir << std::endl; return 0;
  } catch (const std::exception &error) { std::cerr << "[scene-stage02-v2] " << error.what() << std::endl; return 1; }
}
