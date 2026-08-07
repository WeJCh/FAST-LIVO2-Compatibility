#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <unordered_map>
#include <vector>

namespace
{
struct Point
{
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
  uint32_t rgb = 0;
};

struct BinaryPcdLayout
{
  uint64_t point_count = 0;
  std::streampos data_offset = 0;
  std::size_t point_step = 0;
  std::size_t x_offset = 0;
  std::size_t y_offset = 0;
  std::size_t z_offset = 0;
  std::size_t rgb_offset = 0;
};

struct GridKey
{
  int64_t x = 0;
  int64_t y = 0;

  bool operator==(const GridKey &other) const { return x == other.x && y == other.y; }
};

struct GridKeyHash
{
  std::size_t operator()(const GridKey &key) const
  {
    return std::hash<int64_t>()(key.x) ^ (std::hash<int64_t>()(key.y) << 1);
  }
};

struct GroundCell
{
  float lowest_z = std::numeric_limits<float>::infinity();
};

using GroundGrid = std::unordered_map<GridKey, GroundCell, GridKeyHash>;

bool isFinite(const Point &point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

GridKey keyForPoint(const Point &point, const double grid_size_m)
{
  return GridKey{static_cast<int64_t>(std::floor(point.x / grid_size_m)),
                 static_cast<int64_t>(std::floor(point.y / grid_size_m))};
}

bool parsePositiveDouble(const std::string &text, double *value)
{
  if (value == nullptr) return false;
  char *end = nullptr;
  const double parsed = std::strtod(text.c_str(), &end);
  if (end == text.c_str() || *end != '\0' || !std::isfinite(parsed) || parsed <= 0.0) return false;
  *value = parsed;
  return true;
}

bool parseHeights(const std::string &text, std::vector<double> *heights)
{
  if (heights == nullptr) return false;
  heights->clear();
  std::stringstream stream(text);
  std::string item;
  while (std::getline(stream, item, ','))
  {
    double height = 0.0;
    if (!parsePositiveDouble(item, &height)) return false;
    heights->push_back(height);
  }
  if (heights->empty()) return false;
  std::sort(heights->begin(), heights->end());
  heights->erase(std::unique(heights->begin(), heights->end()), heights->end());
  return true;
}

std::string heightLabel(const double height)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(2) << height;
  std::string label = stream.str();
  while (!label.empty() && label.back() == '0') label.pop_back();
  if (!label.empty() && label.back() == '.') label.pop_back();
  std::replace(label.begin(), label.end(), '.', 'p');
  return label;
}

void printUsage(const char *program)
{
  std::cerr
      << "Usage:\n  " << program
      << " <input.pcd> <output_directory> [--heights 1.5,2.0,2.5] [--grid 0.5]\n\n"
      << "The tool streams an uncompressed binary RGB PCD. For each XY grid cell it\n"
      << "uses the lowest finite point as local ground, then keeps points no more than\n"
      << "each requested height above that local reference.\n\n"
      << "Defaults: --heights 1.5,2.0,2.5 --grid 0.5\n";
}

bool createDirectoryRecursively(const std::string &path, std::string *error)
{
  if (path.empty())
  {
    if (error) *error = "path is empty";
    return false;
  }
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
    const std::string component = path.substr(start, end == std::string::npos ? std::string::npos : end - start);
    if (!component.empty())
    {
      if (!current.empty() && current.back() != '/') current += '/';
      current += component;
      struct stat info;
      if (stat(current.c_str(), &info) != 0)
      {
        if (errno != ENOENT || mkdir(current.c_str(), 0755) != 0)
        {
          if (error) *error = std::strerror(errno);
          return false;
        }
      }
      else if (!S_ISDIR(info.st_mode))
      {
        if (error) *error = "an existing path component is not a directory";
        return false;
      }
    }
    if (end == std::string::npos) break;
    start = end + 1;
  }
  return true;
}

std::string joinPath(const std::string &directory, const std::string &filename)
{
  return directory.back() == '/' ? directory + filename : directory + "/" + filename;
}

bool parseBinaryPcdLayout(const std::string &path, BinaryPcdLayout *layout, std::string *error)
{
  if (layout == nullptr) return false;
  std::ifstream input(path.c_str(), std::ios::binary);
  if (!input.is_open())
  {
    if (error) *error = "cannot open input";
    return false;
  }

  std::vector<std::string> fields;
  std::vector<std::size_t> sizes;
  std::vector<std::string> types;
  std::vector<std::size_t> counts;
  uint64_t width = 0;
  uint64_t height = 0;
  uint64_t points = 0;
  bool data_found = false;
  std::string line;
  while (std::getline(input, line))
  {
    std::istringstream stream(line);
    std::string keyword;
    stream >> keyword;
    if (keyword.empty() || keyword[0] == '#') continue;
    if (keyword == "FIELDS")
    {
      fields.clear();
      std::string value;
      while (stream >> value) fields.push_back(value);
    }
    else if (keyword == "SIZE")
    {
      sizes.clear();
      std::size_t value = 0;
      while (stream >> value) sizes.push_back(value);
    }
    else if (keyword == "TYPE")
    {
      types.clear();
      std::string value;
      while (stream >> value) types.push_back(value);
    }
    else if (keyword == "COUNT")
    {
      counts.clear();
      std::size_t value = 0;
      while (stream >> value) counts.push_back(value);
    }
    else if (keyword == "WIDTH") stream >> width;
    else if (keyword == "HEIGHT") stream >> height;
    else if (keyword == "POINTS") stream >> points;
    else if (keyword == "DATA")
    {
      std::string encoding;
      stream >> encoding;
      if (encoding != "binary")
      {
        if (error) *error = "only uncompressed DATA binary PCD files are supported";
        return false;
      }
      layout->data_offset = input.tellg();
      data_found = true;
      break;
    }
  }

  if (!data_found || fields.empty() || sizes.size() != fields.size() || types.size() != fields.size())
  {
    if (error) *error = "invalid PCD header";
    return false;
  }
  if (counts.empty()) counts.assign(fields.size(), 1);
  if (counts.size() != fields.size())
  {
    if (error) *error = "invalid PCD COUNT field";
    return false;
  }
  layout->point_step = 0;
  bool have_x = false, have_y = false, have_z = false, have_rgb = false;
  for (std::size_t index = 0; index < fields.size(); ++index)
  {
    if (sizes[index] == 0 || counts[index] != 1)
    {
      if (error) *error = "only scalar PCD fields are supported";
      return false;
    }
    const std::size_t offset = layout->point_step;
    if (fields[index] == "x" && sizes[index] == 4 && types[index] == "F")
    {
      layout->x_offset = offset;
      have_x = true;
    }
    else if (fields[index] == "y" && sizes[index] == 4 && types[index] == "F")
    {
      layout->y_offset = offset;
      have_y = true;
    }
    else if (fields[index] == "z" && sizes[index] == 4 && types[index] == "F")
    {
      layout->z_offset = offset;
      have_z = true;
    }
    else if (fields[index] == "rgb" && sizes[index] == 4 && (types[index] == "F" || types[index] == "U"))
    {
      layout->rgb_offset = offset;
      have_rgb = true;
    }
    layout->point_step += sizes[index];
  }
  if (!have_x || !have_y || !have_z || !have_rgb)
  {
    if (error) *error = "PCD must contain scalar x y z rgb fields";
    return false;
  }
  layout->point_count = points != 0 ? points : width * height;
  if (layout->point_count == 0 || layout->point_step == 0)
  {
    if (error) *error = "PCD contains no points";
    return false;
  }
  return true;
}

bool readPoint(std::ifstream &input, const BinaryPcdLayout &layout, std::vector<char> *record, Point *point)
{
  if (record == nullptr || point == nullptr) return false;
  input.read(record->data(), static_cast<std::streamsize>(layout.point_step));
  if (!input) return false;
  std::memcpy(&point->x, record->data() + layout.x_offset, sizeof(point->x));
  std::memcpy(&point->y, record->data() + layout.y_offset, sizeof(point->y));
  std::memcpy(&point->z, record->data() + layout.z_offset, sizeof(point->z));
  std::memcpy(&point->rgb, record->data() + layout.rgb_offset, sizeof(point->rgb));
  return true;
}

template <typename Callback>
bool forEachPoint(const std::string &path, const BinaryPcdLayout &layout, Callback callback, std::string *error)
{
  std::ifstream input(path.c_str(), std::ios::binary);
  if (!input.is_open())
  {
    if (error) *error = "cannot reopen input";
    return false;
  }
  input.seekg(layout.data_offset);
  std::vector<char> record(layout.point_step);
  for (uint64_t index = 0; index < layout.point_count; ++index)
  {
    Point point;
    if (!readPoint(input, layout, &record, &point))
    {
      if (error) *error = "unexpected end of input at point " + std::to_string(index);
      return false;
    }
    callback(point);
  }
  return true;
}

bool isWithinHeight(const Point &point, const GroundGrid &ground_grid,
                    const double grid_size_m, const double height_m)
{
  if (!isFinite(point)) return false;
  const auto cell = ground_grid.find(keyForPoint(point, grid_size_m));
  return cell != ground_grid.end() && static_cast<double>(point.z - cell->second.lowest_z) <= height_m;
}

bool writePcdHeader(std::ofstream &output, const uint64_t point_count)
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
  return static_cast<bool>(output);
}

bool writePoint(std::ofstream &output, const Point &point)
{
  output.write(reinterpret_cast<const char *>(&point.x), sizeof(point.x));
  output.write(reinterpret_cast<const char *>(&point.y), sizeof(point.y));
  output.write(reinterpret_cast<const char *>(&point.z), sizeof(point.z));
  output.write(reinterpret_cast<const char *>(&point.rgb), sizeof(point.rgb));
  return static_cast<bool>(output);
}
}  // namespace

int main(int argc, char **argv)
{
  if (argc < 3)
  {
    printUsage(argv[0]);
    return 2;
  }
  const std::string input_path = argv[1];
  const std::string output_directory = argv[2];
  std::vector<double> heights{1.5, 2.0, 2.5};
  double grid_size_m = 0.5;
  for (int index = 3; index < argc; ++index)
  {
    const std::string option = argv[index];
    if (option == "--heights" && index + 1 < argc)
    {
      if (!parseHeights(argv[++index], &heights))
      {
        std::cerr << "Invalid --heights value. Example: 1.5,2.0,2.5\n";
        return 2;
      }
    }
    else if (option == "--grid" && index + 1 < argc)
    {
      if (!parsePositiveDouble(argv[++index], &grid_size_m) || grid_size_m < 0.1 || grid_size_m > 5.0)
      {
        std::cerr << "--grid must be between 0.1 and 5.0 metres.\n";
        return 2;
      }
    }
    else
    {
      std::cerr << "Unknown or incomplete option: " << option << '\n';
      printUsage(argv[0]);
      return 2;
    }
  }

  BinaryPcdLayout layout;
  std::string error;
  if (!parseBinaryPcdLayout(input_path, &layout, &error))
  {
    std::cerr << "[road-filter] Cannot read input header: " << error << '\n';
    return 1;
  }
  std::cout << "[road-filter] Streaming " << layout.point_count << " points from " << input_path << " ..." << std::endl;

  GroundGrid ground_grid;
  ground_grid.reserve(100000);
  uint64_t finite_input_points = 0;
  if (!forEachPoint(input_path, layout, [&](const Point &point) {
        if (!isFinite(point)) return;
        GroundCell &cell = ground_grid[keyForPoint(point, grid_size_m)];
        cell.lowest_z = std::min(cell.lowest_z, point.z);
        ++finite_input_points;
      }, &error))
  {
    std::cerr << "[road-filter] Failed while estimating local ground: " << error << '\n';
    return 1;
  }
  if (ground_grid.empty())
  {
    std::cerr << "[road-filter] Input PCD has no finite XYZ points.\n";
    return 1;
  }
  if (!createDirectoryRecursively(output_directory, &error))
  {
    std::cerr << "[road-filter] Cannot create output directory " << output_directory << ": " << error << '\n';
    return 1;
  }
  std::cout << "[road-filter] " << finite_input_points << " finite points, " << ground_grid.size()
            << " local cells (" << grid_size_m << " m grid)." << std::endl;

  for (const double height_m : heights)
  {
    uint64_t output_point_count = 0;
    if (!forEachPoint(input_path, layout, [&](const Point &point) {
          if (isWithinHeight(point, ground_grid, grid_size_m, height_m)) ++output_point_count;
        }, &error))
    {
      std::cerr << "[road-filter] Failed while counting output points: " << error << '\n';
      return 1;
    }
    const std::string output_path = joinPath(output_directory, "road_height_le_" + heightLabel(height_m) + "m.pcd");
    std::ofstream output(output_path.c_str(), std::ios::binary | std::ios::trunc);
    if (!output.is_open() || !writePcdHeader(output, output_point_count))
    {
      std::cerr << "[road-filter] Cannot write " << output_path << '\n';
      return 1;
    }
    bool write_ok = true;
    if (!forEachPoint(input_path, layout, [&](const Point &point) {
          if (write_ok && isWithinHeight(point, ground_grid, grid_size_m, height_m)) write_ok = writePoint(output, point);
        }, &error) || !write_ok)
    {
      std::cerr << "[road-filter] Write failure for " << output_path << (error.empty() ? "" : ": " + error) << '\n';
      return 1;
    }
    output.close();
    std::cout << "[road-filter] Wrote " << output_path << " (" << output_point_count
              << " points, local height <= " << height_m << " m)." << std::endl;
  }
  std::cout << "[road-filter] Done. Open the three PCDs in CloudCompare and select the best threshold." << std::endl;
  return 0;
}
