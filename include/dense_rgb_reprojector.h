/*
 * Dense RGB global-map exporter used by the automatic loop-closure workflow.
 *
 * Cached RGB clouds are already expressed in FAST-LIVO2's original odom
 * (camera_init) world frame.  This tool interpolates the correction field
 * T_map_odom(t) from raw/final keyframe pose pairs and warps every batch.
 */
#ifndef FAST_LIVO_DENSE_RGB_REPROJECTOR_H
#define FAST_LIVO_DENSE_RGB_REPROJECTOR_H

#include <string>

struct DenseRgbReprojectConfig
{
  std::string cache_dir;
  std::string raw_keyframe_pose_path;
  std::string optimized_keyframe_pose_path;
  std::string output_path;
  // 0 means keep every source point.  A positive value performs streaming
  // voxel aggregation, avoiding a full dense source cloud in RAM.
  double voxel_leaf_m = 0.10;
};

bool reprojectDenseRgbMap(const DenseRgbReprojectConfig &config, std::string *error);

#endif  // FAST_LIVO_DENSE_RGB_REPROJECTOR_H
