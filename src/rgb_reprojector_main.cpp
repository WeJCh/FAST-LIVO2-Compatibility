#include "dense_rgb_reprojector.h"

#include <exception>
#include <iostream>
#include <string>

namespace
{
void printUsage(const char *program)
{
  std::cout << "Usage:\n  " << program
            << " --cache_dir <dense_rgb_cache_dir> --raw_keyframe_poses <keyframe_poses_imu.txt>"
            << " --optimized_keyframe_poses <optimized_keyframe_poses_imu.txt> --output <output.pcd>"
            << " [--voxel_leaf <meters>]\n\n"
            << "Example:\n  " << program
            << " --cache_dir Log/dense_rgb_cache_robotdog --raw_keyframe_poses Log/pcd/keyframes_robotdog/keyframe_poses_imu.txt"
            << " --optimized_keyframe_poses Log/loop_online_final_robotdog/optimized_keyframe_poses_imu.txt"
            << " --output Log/loop_online_final_robotdog/all_global_optimized_rgb_dense.pcd --voxel_leaf 0.10\n"
            << "Use --voxel_leaf 0 to preserve every cached RGB point.\n";
}

bool readArgument(int argc, char **argv, int *index, std::string *value)
{
  if (*index + 1 >= argc) return false;
  *value = argv[++(*index)];
  return true;
}
}  // namespace

int main(int argc, char **argv)
{
  DenseRgbReprojectConfig config;
  for (int index = 1; index < argc; ++index)
  {
    const std::string argument = argv[index];
    std::string value;
    if (argument == "--help" || argument == "-h")
    {
      printUsage(argv[0]);
      return 0;
    }
    if (argument == "--cache_dir" && readArgument(argc, argv, &index, &value)) config.cache_dir = value;
    else if (argument == "--raw_keyframe_poses" && readArgument(argc, argv, &index, &value)) config.raw_keyframe_pose_path = value;
    else if (argument == "--optimized_keyframe_poses" && readArgument(argc, argv, &index, &value)) config.optimized_keyframe_pose_path = value;
    else if (argument == "--output" && readArgument(argc, argv, &index, &value)) config.output_path = value;
    else if (argument == "--voxel_leaf" && readArgument(argc, argv, &index, &value))
    {
      try { config.voxel_leaf_m = std::stod(value); }
      catch (const std::exception &) { std::cerr << "Invalid --voxel_leaf value: " << value << std::endl; return 2; }
    }
    else
    {
      std::cerr << "Unknown or incomplete option: " << argument << std::endl;
      printUsage(argv[0]);
      return 2;
    }
  }

  std::string error;
  if (!reprojectDenseRgbMap(config, &error))
  {
    std::cerr << "[dense-rgb] Failed: " << error << std::endl;
    return 1;
  }
  return 0;
}
