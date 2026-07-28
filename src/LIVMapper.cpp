/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "LIVMapper.h"

#include <cerrno>
#include <cstring>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>

namespace
{
bool createDirectoryRecursively(const std::string &path)
{
  if (path.empty()) return false;

  std::string current;
  size_t start = 0;
  if (path.front() == '/')
  {
    current = "/";
    start = 1;
  }

  while (start <= path.size())
  {
    const size_t end = path.find('/', start);
    const std::string component = path.substr(start, end - start);
    if (!component.empty())
    {
      if (!current.empty() && current.back() != '/') current += '/';
      current += component;
      if (::mkdir(current.c_str(), 0755) != 0 && errno != EEXIST)
      {
        ROS_ERROR("Failed to create keyframe directory '%s': %s", current.c_str(), strerror(errno));
        return false;
      }
    }
    if (end == std::string::npos) break;
    start = end + 1;
  }
  return true;
}

bool isManagedLogPath(const std::string &path)
{
  const std::string log_root = std::string(ROOT_DIR) + "Log/";
  return path.size() > log_root.size() && path.compare(0, log_root.size(), log_root) == 0;
}

bool removePathRecursively(const std::string &path)
{
  if (!isManagedLogPath(path))
  {
    ROS_ERROR("Refusing to remove a path outside the generated Log directory: %s", path.c_str());
    return false;
  }

  struct stat status;
  if (::lstat(path.c_str(), &status) != 0)
  {
    return errno == ENOENT;
  }
  if (!S_ISDIR(status.st_mode)) return ::unlink(path.c_str()) == 0;

  DIR *directory = ::opendir(path.c_str());
  if (!directory)
  {
    ROS_ERROR("Cannot open generated directory for cleanup: %s", path.c_str());
    return false;
  }
  bool success = true;
  while (dirent *entry = ::readdir(directory))
  {
    const std::string name(entry->d_name);
    if (name == "." || name == "..") continue;
    if (!removePathRecursively(path + "/" + name)) success = false;
  }
  ::closedir(directory);
  if (::rmdir(path.c_str()) != 0)
  {
    ROS_ERROR("Cannot remove generated directory: %s", path.c_str());
    success = false;
  }
  return success;
}

bool copyGeneratedFile(const std::string &source, const std::string &target)
{
  if (!isManagedLogPath(source) || !isManagedLogPath(target))
  {
    ROS_ERROR("Refusing to copy a result outside the generated Log directory.");
    return false;
  }
  std::ifstream input(source, std::ios::binary);
  std::ofstream output(target, std::ios::binary | std::ios::trunc);
  if (!input.is_open() || !output.is_open())
  {
    ROS_ERROR("Cannot copy final trajectory from %s to %s", source.c_str(), target.c_str());
    return false;
  }
  output << input.rdbuf();
  return !input.bad() && output.good();
}

bool isPathInsideDirectory(const std::string &path, const std::string &directory)
{
  if (path.size() <= directory.size()) return false;
  if (path.compare(0, directory.size(), directory) != 0) return false;
  return directory.back() == '/' || path[directory.size()] == '/';
}
} // namespace

LIVMapper::LIVMapper(ros::NodeHandle &nh)
    : extT(0, 0, 0),
      extR(M3D::Identity())
{
  extrinT.assign(3, 0.0);
  extrinR.assign(9, 0.0);
  cameraextrinT.assign(3, 0.0);
  cameraextrinR.assign(9, 0.0);

  p_pre.reset(new Preprocess());
  p_imu.reset(new ImuProcess());

  readParameters(nh);
  VoxelMapConfig voxel_config;
  loadVoxelConfig(nh, voxel_config);

  visual_sub_map.reset(new PointCloudXYZI());
  feats_undistort.reset(new PointCloudXYZI());
  feats_down_body.reset(new PointCloudXYZI());
  feats_down_world.reset(new PointCloudXYZI());
  pcl_w_wait_pub.reset(new PointCloudXYZI());
  pcl_wait_pub.reset(new PointCloudXYZI());
  pcl_wait_save.reset(new PointCloudXYZRGB());
  pcl_wait_save_intensity.reset(new PointCloudXYZI());
  voxelmap_manager.reset(new VoxelMapManager(voxel_config, voxel_map));
  vio_manager.reset(new VIOManager());
  root_dir = ROOT_DIR;
  initializeFiles();
  initializeComponents();
  initializeKeyframeOutput();
  initializeDenseRgbCache();
#ifdef FAST_LIVO_HAS_LOOP_BACKEND
  initializeOnlineLoopBackend();
#endif
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "camera_init";
}

LIVMapper::~LIVMapper()
{
#ifdef FAST_LIVO_HAS_LOOP_BACKEND
  stopOnlineLoopBackend();
#endif
}

void LIVMapper::readParameters(ros::NodeHandle &nh)
{
  nh.param<string>("common/lid_topic", lid_topic, "/livox/lidar");
  nh.param<string>("common/imu_topic", imu_topic, "/livox/imu");
  nh.param<bool>("common/ros_driver_bug_fix", ros_driver_fix_en, false);
  nh.param<int>("common/img_en", img_en, 1);
  nh.param<int>("common/lidar_en", lidar_en, 1);
  nh.param<string>("common/img_topic", img_topic, "/left_camera/image");
  nh.param<bool>("common/img_compressed", img_compressed, false); // 是否直接订阅压缩图像，机器狗数据使用 true

  nh.param<bool>("vio/normal_en", normal_en, true);
  nh.param<bool>("vio/inverse_composition_en", inverse_composition_en, false);
  nh.param<int>("vio/max_iterations", max_iterations, 5);
  nh.param<double>("vio/img_point_cov", IMG_POINT_COV, 100);
  nh.param<bool>("vio/raycast_en", raycast_en, false);
  nh.param<bool>("vio/exposure_estimate_en", exposure_estimate_en, true);
  nh.param<double>("vio/inv_expo_cov", inv_expo_cov, 0.2);
  nh.param<int>("vio/grid_size", grid_size, 5);
  nh.param<int>("vio/grid_n_height", grid_n_height, 17);
  nh.param<int>("vio/patch_pyrimid_level", patch_pyrimid_level, 3);
  nh.param<int>("vio/patch_size", patch_size, 8);
  nh.param<double>("vio/outlier_threshold", outlier_threshold, 1000);

  nh.param<double>("time_offset/exposure_time_init", exposure_time_init, 0.0);
  nh.param<double>("time_offset/img_time_offset", img_time_offset, 0.0);
  nh.param<double>("time_offset/imu_time_offset", imu_time_offset, 0.0);
  nh.param<double>("time_offset/lidar_time_offset", lidar_time_offset, 0.0);
  nh.param<bool>("uav/imu_rate_odom", imu_prop_enable, false);
  nh.param<bool>("uav/gravity_align_en", gravity_align_en, false);

  nh.param<string>("evo/seq_name", seq_name, "01");
  nh.param<bool>("evo/pose_output_en", pose_output_en, false);
  nh.param<double>("imu/gyr_cov", gyr_cov, 1.0);
  nh.param<double>("imu/acc_cov", acc_cov, 1.0);
  nh.param<int>("imu/imu_int_frame", imu_int_frame, 3);
  nh.param<bool>("imu/imu_en", imu_en, false);
  nh.param<bool>("imu/gravity_est_en", gravity_est_en, true);
  nh.param<bool>("imu/ba_bg_est_en", ba_bg_est_en, true);

  nh.param<double>("preprocess/blind", p_pre->blind, 0.01);
  nh.param<double>("preprocess/filter_size_surf", filter_size_surf_min, 0.5);
  nh.param<bool>("preprocess/hilti_en", hilti_en, false);
  nh.param<int>("preprocess/lidar_type", p_pre->lidar_type, AVIA);
  nh.param<int>("preprocess/scan_line", p_pre->N_SCANS, 6);
  nh.param<int>("preprocess/point_filter_num", p_pre->point_filter_num, 3);
  nh.param<bool>("preprocess/feature_extract_enabled", p_pre->feature_enabled, false);

  nh.param<int>("pcd_save/interval", pcd_save_interval, -1);
  nh.param<bool>("pcd_save/pcd_save_en", pcd_save_en, false);
  nh.param<int>("pcd_save/type", pcd_save_type, 0);
  nh.param<bool>("image_save/img_save_en", img_save_en, false);
  nh.param<int>("image_save/interval", img_save_interval, 1);

  nh.param<bool>("pcd_save/colmap_output_en", colmap_output_en, false);
  nh.param<double>("pcd_save/filter_size_pcd", filter_size_pcd, 0.5);

  nh.param<bool>("loop_closure/keyframe_save_en", keyframe_save_en, false);
  nh.param<string>("loop_closure/keyframe_dir", keyframe_dir, "Log/pcd/keyframes");
  nh.param<bool>("loop_closure/keyframe_overwrite", keyframe_overwrite, false);
  nh.param<double>("loop_closure/keyframe_translation_m", keyframe_translation_thresh, 1.5);
  nh.param<double>("loop_closure/keyframe_rotation_deg", keyframe_rotation_thresh_deg, 10.0);
  nh.param<double>("loop_closure/keyframe_min_interval_s", keyframe_min_interval, 0.5);
  nh.param<double>("loop_closure/skip_initialization_s", keyframe_skip_initialization, 5.0);
  nh.param<bool>("dense_global_map/enabled", dense_rgb_cache_enabled, true);
  nh.param<int>("dense_global_map/cache_queue_size", dense_rgb_cache_queue_size, 8);
  nh.param<string>("dense_global_map/cache_dir", dense_rgb_cache_dir, "Log/dense_rgb_cache");
  nh.param<bool>("dense_global_map/auto_export_on_shutdown", dense_rgb_auto_export_on_shutdown, false);
  nh.param<double>("dense_global_map/auto_export_voxel_leaf_m", dense_rgb_auto_export_voxel_leaf_m, 0.0);
  nh.param<string>("dense_global_map/output_path", dense_rgb_output_path, "");
  nh.param<bool>("dense_global_map/cleanup_intermediate_on_success",
                 dense_rgb_cleanup_intermediate_on_success, false);
#ifdef FAST_LIVO_HAS_LOOP_BACKEND
  nh.param<bool>("loop_backend/online_enable", online_loop_enable, false);
  nh.param<double>("loop_backend/online_frequency_hz", online_loop_frequency_hz, 0.2);
  nh.param<int>("loop_backend/online_min_new_keyframes", online_loop_min_new_keyframes, 5);
  nh.param<bool>("loop_backend/export_on_shutdown", online_loop_export_on_shutdown, true);
  nh.param<string>("loop_backend/output_dir", online_loop_output_dir, "Log/loop_online_final");
  nh.param<bool>("loop_backend/output_overwrite", online_loop_output_overwrite, false);
#endif
  nh.param<vector<double>>("extrin_calib/extrinsic_T", extrinT, vector<double>());
  nh.param<vector<double>>("extrin_calib/extrinsic_R", extrinR, vector<double>());
  nh.param<vector<double>>("extrin_calib/Pcl", cameraextrinT, vector<double>());
  nh.param<vector<double>>("extrin_calib/Rcl", cameraextrinR, vector<double>());
  nh.param<double>("debug/plot_time", plot_time, -10);
  nh.param<int>("debug/frame_cnt", frame_cnt, 6);

  nh.param<double>("publish/blind_rgb_points", blind_rgb_points, 0.01);
  nh.param<int>("publish/pub_scan_num", pub_scan_num, 1);
  nh.param<bool>("publish/pub_effect_point_en", pub_effect_point_en, false);
  nh.param<bool>("publish/dense_map_en", dense_map_en, false);

  p_pre->blind_sqr = p_pre->blind * p_pre->blind;
}

void LIVMapper::initializeComponents() 
{
  downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
  extT << VEC_FROM_ARRAY(extrinT);
  extR << MAT_FROM_ARRAY(extrinR);

  voxelmap_manager->extT_ << VEC_FROM_ARRAY(extrinT);
  voxelmap_manager->extR_ << MAT_FROM_ARRAY(extrinR);

  if (!vk::camera_loader::loadFromRosNs("laserMapping", vio_manager->cam)) throw std::runtime_error("Camera model not correctly specified.");

  vio_manager->grid_size = grid_size;
  vio_manager->patch_size = patch_size;
  vio_manager->outlier_threshold = outlier_threshold;
  vio_manager->setImuToLidarExtrinsic(extT, extR);
  vio_manager->setLidarToCameraExtrinsic(cameraextrinR, cameraextrinT);
  vio_manager->state = &_state;
  vio_manager->state_propagat = &state_propagat;
  vio_manager->max_iterations = max_iterations;
  vio_manager->img_point_cov = IMG_POINT_COV;
  vio_manager->normal_en = normal_en;
  vio_manager->inverse_composition_en = inverse_composition_en;
  vio_manager->raycast_en = raycast_en;
  vio_manager->grid_n_width = grid_n_width;
  vio_manager->grid_n_height = grid_n_height;
  vio_manager->patch_pyrimid_level = patch_pyrimid_level;
  vio_manager->exposure_estimate_en = exposure_estimate_en;
  vio_manager->colmap_output_en = colmap_output_en;
  vio_manager->initializeVIO();

  p_imu->set_extrinsic(extT, extR);
  p_imu->set_gyr_cov_scale(V3D(gyr_cov, gyr_cov, gyr_cov));
  p_imu->set_acc_cov_scale(V3D(acc_cov, acc_cov, acc_cov));
  p_imu->set_inv_expo_cov(inv_expo_cov);
  p_imu->set_gyr_bias_cov(V3D(0.0001, 0.0001, 0.0001));
  p_imu->set_acc_bias_cov(V3D(0.0001, 0.0001, 0.0001));
  p_imu->set_imu_init_frame_num(imu_int_frame);

  if (!imu_en) p_imu->disable_imu();
  if (!gravity_est_en) p_imu->disable_gravity_est();
  if (!ba_bg_est_en) p_imu->disable_bias_est();
  if (!exposure_estimate_en) p_imu->disable_exposure_est();

  slam_mode_ = (img_en && lidar_en) ? LIVO : imu_en ? ONLY_LIO : ONLY_LO;
}

void LIVMapper::initializeFiles() 
{
  if (pcd_save_en && colmap_output_en)
  {
      const std::string folderPath = std::string(ROOT_DIR) + "/scripts/colmap_output.sh";
      
      std::string chmodCommand = "chmod +x " + folderPath;
      
      int chmodRet = system(chmodCommand.c_str());  
      if (chmodRet != 0) {
          std::cerr << "Failed to set execute permissions for the script." << std::endl;
          return;
      }

      int executionRet = system(folderPath.c_str());
      if (executionRet != 0) {
          std::cerr << "Failed to execute the script." << std::endl;
          return;
      }
  }
  if(colmap_output_en) fout_points.open(std::string(ROOT_DIR) + "Log/Colmap/sparse/0/points3D.txt", std::ios::out);
  if(pcd_save_en) fout_lidar_pos.open(std::string(ROOT_DIR) + "Log/pcd/lidar_poses.txt", std::ios::out);
  if(img_save_en) fout_visual_pos.open(std::string(ROOT_DIR) + "Log/image/image_poses.txt", std::ios::out);
  fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"), std::ios::out);
  fout_out.open(DEBUG_FILE_DIR("mat_out.txt"), std::ios::out);
}

void LIVMapper::initializeKeyframeOutput()
{
  if (!keyframe_save_en) return;

  keyframe_output_dir = keyframe_dir;
  if (keyframe_output_dir.empty())
  {
    ROS_ERROR("loop_closure/keyframe_dir is empty; keyframe recording is disabled.");
    keyframe_save_en = false;
    return;
  }
  if (keyframe_output_dir.front() != '/') keyframe_output_dir = std::string(ROOT_DIR) + keyframe_output_dir;

  const std::string pose_path = keyframe_output_dir + "/keyframe_poses_imu.txt";
  if (!keyframe_overwrite && std::ifstream(pose_path).good())
  {
    ROS_ERROR("Keyframe pose file already exists: %s. Choose a new loop_closure/keyframe_dir or set keyframe_overwrite=true.", pose_path.c_str());
    keyframe_save_en = false;
    return;
  }
  if (!createDirectoryRecursively(keyframe_output_dir))
  {
    keyframe_save_en = false;
    return;
  }

  fout_keyframe_pose.open(pose_path, std::ios::out);
  if (!fout_keyframe_pose.is_open())
  {
    ROS_ERROR("Failed to open keyframe pose file: %s", pose_path.c_str());
    keyframe_save_en = false;
    return;
  }

  std::ofstream metadata(keyframe_output_dir + "/metadata.yaml", std::ios::out);
  metadata << "cloud_frame: lidar\n"
           << "pose_frame: camera_init_to_imu\n"
           << "point_cloud: '<id>.pcd, undistorted and downsampled in the LiDAR frame'\n"
           << "pose_file: keyframe_poses_imu.txt\n"
           << "pose_format: 'id timestamp tx ty tz qx qy qz qw'\n"
           << "composition: 'T_camera_init_lidar = T_camera_init_imu * T_imu_lidar'\n"
           << "T_imu_lidar_translation: [" << extT[0] << ", " << extT[1] << ", " << extT[2] << "]\n"
           << "T_imu_lidar_rotation_row_major: [" << extR(0, 0) << ", " << extR(0, 1) << ", " << extR(0, 2) << ", "
           << extR(1, 0) << ", " << extR(1, 1) << ", " << extR(1, 2) << ", "
           << extR(2, 0) << ", " << extR(2, 1) << ", " << extR(2, 2) << "]\n";

  keyframe_output_ready = true;
  ROS_INFO("Loop-closure keyframes will be saved to: %s", keyframe_output_dir.c_str());
}

void LIVMapper::initializeDenseRgbCache()
{
  if (!dense_rgb_cache_enabled) return;
  if (!img_en)
  {
    ROS_WARN("[dense-rgb] dense_global_map/enabled is true but common/img_en is false; cache recording is disabled.");
    dense_rgb_cache_enabled = false;
    return;
  }
  if (dense_rgb_cache_dir.empty()) dense_rgb_cache_dir = "Log/dense_rgb_cache";
  if (dense_rgb_cache_dir.front() != '/') dense_rgb_cache_dir = std::string(ROOT_DIR) + dense_rgb_cache_dir;
  if (!dense_rgb_output_path.empty() && dense_rgb_output_path.front() != '/')
  {
    dense_rgb_output_path = std::string(ROOT_DIR) + dense_rgb_output_path;
  }
  if (dense_rgb_cache_queue_size < 1) dense_rgb_cache_queue_size = 1;
  if (dense_rgb_auto_export_voxel_leaf_m < 0.0)
  {
    ROS_WARN("[dense-rgb] auto_export_voxel_leaf_m is negative; using 0 (no downsampling).");
    dense_rgb_auto_export_voxel_leaf_m = 0.0;
  }
}

void LIVMapper::startDenseRgbCacheWriter()
{
  if (!dense_rgb_cache_enabled || dense_rgb_cache_writer_thread.joinable()) return;

  const std::string frames_dir = dense_rgb_cache_dir + "/frames";
  if (!createDirectoryRecursively(frames_dir))
  {
    ROS_ERROR("[dense-rgb] Cannot create cache directory: %s", frames_dir.c_str());
    dense_rgb_cache_enabled = false;
    return;
  }

  // manifest.csv is the complete input contract of the offline C1 exporter.
  // Old frame files are harmless: only entries from this newly created manifest are read.
  std::ofstream manifest(dense_rgb_cache_dir + "/manifest.csv", std::ios::out | std::ios::trunc);
  if (!manifest.is_open())
  {
    ROS_ERROR("[dense-rgb] Cannot create cache manifest in: %s", dense_rgb_cache_dir.c_str());
    dense_rgb_cache_enabled = false;
    return;
  }
  manifest << "frame_id,timestamp,point_count,pcd_path\n";
  manifest.close();

  {
    std::lock_guard<std::mutex> lock(dense_rgb_cache_mutex);
    dense_rgb_cache_queue.clear();
    dense_rgb_cache_next_id = 0;
    dense_rgb_cache_stop.store(false);
  }
  dense_rgb_cache_writer_thread = std::thread(&LIVMapper::denseRgbCacheWriterThread, this);
  ROS_INFO("[dense-rgb] Recording frame-addressable RGB batches in: %s", dense_rgb_cache_dir.c_str());
}

void LIVMapper::stopDenseRgbCacheWriter()
{
  if (!dense_rgb_cache_writer_thread.joinable()) return;
  {
    std::lock_guard<std::mutex> lock(dense_rgb_cache_mutex);
    dense_rgb_cache_stop.store(true);
  }
  dense_rgb_cache_cv.notify_all();
  dense_rgb_cache_writer_thread.join();
}

void LIVMapper::enqueueDenseRgbCache(const PointCloudXYZRGB::Ptr &cloud, double timestamp)
{
  if (!dense_rgb_cache_enabled || !cloud || cloud->empty() || !dense_rgb_cache_writer_thread.joinable()) return;

  // The color cloud is a fresh batch created in publish_frame_world and is not modified later.
  // A bounded queue moves PCD I/O off the LIO/VIO thread while never dropping mapping data.
  std::unique_lock<std::mutex> lock(dense_rgb_cache_mutex);
  dense_rgb_cache_cv.wait(lock, [this]() {
    return dense_rgb_cache_stop.load() ||
           static_cast<int>(dense_rgb_cache_queue.size()) < dense_rgb_cache_queue_size;
  });
  if (dense_rgb_cache_stop.load()) return;

  DenseRgbCacheItem item;
  item.id = dense_rgb_cache_next_id++;
  item.timestamp = timestamp;
  item.cloud = cloud;
  dense_rgb_cache_queue.push_back(item);
  lock.unlock();
  dense_rgb_cache_cv.notify_all();
}

void LIVMapper::denseRgbCacheWriterThread()
{
  const std::string frames_dir = dense_rgb_cache_dir + "/frames";
  std::ofstream manifest(dense_rgb_cache_dir + "/manifest.csv", std::ios::out | std::ios::app);
  if (!manifest.is_open())
  {
    ROS_ERROR("[dense-rgb] Cannot append cache manifest: %s", dense_rgb_cache_dir.c_str());
    std::lock_guard<std::mutex> lock(dense_rgb_cache_mutex);
    dense_rgb_cache_stop.store(true);
    dense_rgb_cache_cv.notify_all();
    return;
  }

  pcl::PCDWriter writer;
  while (true)
  {
    DenseRgbCacheItem item;
    {
      std::unique_lock<std::mutex> lock(dense_rgb_cache_mutex);
      dense_rgb_cache_cv.wait(lock, [this]() {
        return dense_rgb_cache_stop.load() || !dense_rgb_cache_queue.empty();
      });
      if (dense_rgb_cache_queue.empty())
      {
        if (dense_rgb_cache_stop.load()) break;
        continue;
      }
      item = dense_rgb_cache_queue.front();
      dense_rgb_cache_queue.pop_front();
    }
    dense_rgb_cache_cv.notify_all();

    std::ostringstream filename;
    filename << frames_dir << "/frame_" << std::setfill('0') << std::setw(6) << item.id << ".pcd";
    if (writer.writeBinary(filename.str(), *item.cloud) != 0)
    {
      ROS_ERROR("[dense-rgb] Failed to write RGB cache batch: %s", filename.str().c_str());
      continue;
    }
    manifest << item.id << ',' << std::setprecision(17) << item.timestamp << ','
             << item.cloud->size() << ',' << filename.str() << '\n';
    manifest.flush();
  }
}

void LIVMapper::exportDenseRgbMapOnShutdown()
{
  dense_rgb_auto_export_succeeded = false;
  dense_rgb_final_output_path.clear();
  if (!dense_rgb_cache_enabled || !dense_rgb_auto_export_on_shutdown) return;

#ifndef FAST_LIVO_HAS_LOOP_BACKEND
  ROS_ERROR("[dense-rgb] Automatic export requires the GTSAM loop backend, which was not built.");
  return;
#else
  if (!online_loop_final_export_succeeded)
  {
    ROS_ERROR("[dense-rgb] Skip automatic RGB export because final loop optimization output is unavailable.");
    return;
  }
  if (keyframe_output_dir.empty())
  {
    ROS_ERROR("[dense-rgb] Skip automatic RGB export because loop keyframe recording was not initialized.");
    return;
  }

  DenseRgbReprojectConfig config;
  config.cache_dir = dense_rgb_cache_dir;
  config.raw_keyframe_pose_path = keyframe_output_dir + "/keyframe_poses_imu.txt";
  config.optimized_keyframe_pose_path = online_loop_final_output_dir + "/optimized_keyframe_poses_imu.txt";
  config.output_path = dense_rgb_output_path.empty()
      ? std::string(ROOT_DIR) + "Log/pcd/all_global_optimized_rgb_dense_full.pcd"
      : dense_rgb_output_path;
  config.voxel_leaf_m = dense_rgb_auto_export_voxel_leaf_m;

  ROS_INFO("[dense-rgb] Automatic final export begins: %s (voxel leaf %.3f m).",
           config.output_path.c_str(), config.voxel_leaf_m);
  std::string error;
  if (!reprojectDenseRgbMap(config, &error))
  {
    ROS_ERROR("[dense-rgb] Automatic final export failed; cache and keyframes are preserved: %s", error.c_str());
    return;
  }
  dense_rgb_auto_export_succeeded = true;
  dense_rgb_final_output_path = config.output_path;
  ROS_INFO("[dense-rgb] Automatic final RGB map export completed: %s", config.output_path.c_str());
#endif
}

void LIVMapper::cleanupC2IntermediateFiles()
{
  if (!dense_rgb_cleanup_intermediate_on_success || !dense_rgb_auto_export_succeeded) return;

  // 仅在最终稠密 PCD 已成功写入后清理，失败时保留所有输入便于离线恢复。
  if (fout_keyframe_pose.is_open()) fout_keyframe_pose.close();
  if (fout_lidar_pos.is_open()) fout_lidar_pos.close();

  const std::string pcd_dir = std::string(ROOT_DIR) + "Log/pcd";
#ifdef FAST_LIVO_HAS_LOOP_BACKEND
  const std::string optimized_pose_source = online_loop_final_output_dir + "/optimized_keyframe_poses_imu.txt";
  const std::string optimized_pose_target = pcd_dir + "/optimized_keyframe_poses_imu.txt";
  if (!online_loop_final_output_dir.empty() && !copyGeneratedFile(optimized_pose_source, optimized_pose_target))
  {
    ROS_ERROR("[dense-rgb] C2 cleanup skipped: final optimized trajectory could not be copied.");
    return;
  }
#endif

  bool success = true;
  success = removePathRecursively(dense_rgb_cache_dir) && success;
  if (!keyframe_output_dir.empty()) success = removePathRecursively(keyframe_output_dir) && success;

#ifdef FAST_LIVO_HAS_LOOP_BACKEND
  if (!online_loop_final_output_dir.empty())
  {
    // 默认最终稠密 PCD 已输出到 Log/pcd，因此该目录可整体清理。
    // 若用户把 output_path 指回该目录，则保护最终 PCD，仅删除其余中间结果。
    if (!isPathInsideDirectory(dense_rgb_final_output_path, online_loop_final_output_dir))
    {
      success = removePathRecursively(online_loop_final_output_dir) && success;
    }
    else
    {
      success = removePathRecursively(online_loop_final_output_dir + "/all_global_optimized.pcd") && success;
      success = removePathRecursively(online_loop_final_output_dir + "/loop_edges.csv") && success;
      success = removePathRecursively(online_loop_final_output_dir + "/report.yaml") && success;
      success = removePathRecursively(online_loop_final_output_dir + "/optimized_keyframe_poses_imu.txt") && success;
    }
  }
#endif

  // 保留 all_downsampled_points.pcd，与原 FAST-LIVO2 的结果组织保持一致。
  success = removePathRecursively(pcd_dir + "/lidar_poses.txt") && success;

  if (success)
  {
    ROS_INFO("[dense-rgb] C2 cleanup complete. Kept raw map, downsampled map, final dense RGB map, and optimized trajectory.");
  }
  else
  {
    ROS_WARN("[dense-rgb] C2 export succeeded, but some intermediate files could not be removed.");
  }
}

#ifdef FAST_LIVO_HAS_LOOP_BACKEND
void LIVMapper::initializeOnlineLoopBackend()
{
  if (!online_loop_enable) return;

  // 在线参数与离线工具使用同一组语义，便于先用阶段 A 验证，再平滑迁移到阶段 B。
  ros::NodeHandle nh;
  LoopBackendConfig config;
  nh.param<double>("loop_backend/candidate_radius_m", config.candidate_radius_m, config.candidate_radius_m);
  nh.param<double>("loop_backend/min_time_separation_s", config.min_time_separation_s,
                   config.min_time_separation_s);
  nh.param<int>("loop_backend/min_keyframe_index_gap", config.min_keyframe_index_gap,
                config.min_keyframe_index_gap);
  nh.param<int>("loop_backend/history_keyframes_each_side", config.history_keyframes_each_side,
                config.history_keyframes_each_side);
  nh.param<double>("loop_backend/icp_voxel_leaf_m", config.icp_voxel_leaf_m, config.icp_voxel_leaf_m);
  nh.param<double>("loop_backend/icp_max_correspondence_m", config.icp_max_correspondence_m,
                   config.icp_max_correspondence_m);
  nh.param<int>("loop_backend/icp_max_iterations", config.icp_max_iterations, config.icp_max_iterations);
  nh.param<double>("loop_backend/icp_fitness_threshold", config.icp_fitness_threshold,
                   config.icp_fitness_threshold);
  nh.param<int>("loop_backend/min_current_points", config.min_current_points, config.min_current_points);
  nh.param<int>("loop_backend/min_history_points", config.min_history_points, config.min_history_points);
  nh.param<double>("loop_backend/odom_rotation_sigma_rad", config.odom_rotation_sigma_rad,
                   config.odom_rotation_sigma_rad);
  nh.param<double>("loop_backend/odom_translation_sigma_m", config.odom_translation_sigma_m,
                   config.odom_translation_sigma_m);
  nh.param<double>("loop_backend/loop_rotation_sigma_rad", config.loop_rotation_sigma_rad,
                   config.loop_rotation_sigma_rad);
  nh.param<double>("loop_backend/loop_translation_sigma_min_m", config.loop_translation_sigma_min_m,
                   config.loop_translation_sigma_min_m);
  nh.param<double>("loop_backend/loop_translation_sigma_max_m", config.loop_translation_sigma_max_m,
                   config.loop_translation_sigma_max_m);
  nh.param<double>("loop_backend/global_map_leaf_m", config.global_map_leaf_m, config.global_map_leaf_m);

  if (online_loop_frequency_hz <= 0.0) online_loop_frequency_hz = 0.2;
  if (online_loop_min_new_keyframes < 1) online_loop_min_new_keyframes = 1;

  Eigen::Isometry3d T_imu_lidar = Eigen::Isometry3d::Identity();
  T_imu_lidar.linear() = extR;
  T_imu_lidar.translation() = extT;
  online_loop_backend.reset(new LoopBackend(config));
  online_loop_backend->setImuLidarExtrinsic(T_imu_lidar);
  ROS_INFO("Online loop backend enabled: %.2f Hz, at least %d new keyframes per optimization.",
           online_loop_frequency_hz, online_loop_min_new_keyframes);
}

void LIVMapper::startOnlineLoopBackend()
{
  if (!online_loop_enable || !online_loop_backend || online_loop_running.load()) return;

  online_loop_stop.store(false);
  online_loop_finalized = false;
  online_loop_final_export_succeeded = false;
  online_loop_final_output_dir.clear();
  online_loop_running.store(true);
  online_loop_thread = std::thread(&LIVMapper::onlineLoopBackendWorker, this);
  ROS_INFO("Online loop-backend worker started.");
}

void LIVMapper::stopOnlineLoopBackend()
{
  if (!online_loop_backend || online_loop_finalized) return;

  online_loop_stop.store(true);
  if (online_loop_thread.joinable()) online_loop_thread.join();
  online_loop_running.store(false);

  // Ctrl-C 后仍可能有不足 online_min_new_keyframes 的尾部数据。
  // 工作线程已停止，此处独占后端并将其纳入最终图优化，避免最终地图缺尾段。
  if (online_loop_backend->pendingOnlineKeyframeCount() > 0)
  {
    std::string final_optimize_error;
    if (!online_loop_backend->runOnce(&final_optimize_error))
    {
      ROS_ERROR("Final online loop optimization failed: %s", final_optimize_error.c_str());
    }
  }

  if (online_loop_export_on_shutdown)
  {
    std::string output_dir = online_loop_output_dir;
    if (output_dir.empty())
    {
      ROS_ERROR("loop_backend/output_dir is empty; skip final optimized-map export.");
    }
    else
    {
      if (output_dir.front() != '/') output_dir = std::string(ROOT_DIR) + output_dir;
      const std::string report_path = output_dir + "/report.yaml";
      if (!online_loop_output_overwrite && std::ifstream(report_path).good())
      {
        ROS_ERROR("Final loop output already exists: %s. Choose a new loop_backend/output_dir or set output_overwrite=true.",
                  report_path.c_str());
      }
      else
      {
        std::string export_error;
        if (online_loop_backend->exportLatestOnlineResult(output_dir, &export_error))
        {
          online_loop_final_export_succeeded = true;
          online_loop_final_output_dir = output_dir;
          ROS_INFO("Final online loop result exported to: %s", output_dir.c_str());
        }
        else
        {
          ROS_ERROR("Final optimized-map export failed: %s", export_error.c_str());
        }
      }
    }
  }
  online_loop_finalized = true;
}

void LIVMapper::onlineLoopBackendWorker()
{
  ros::Rate rate(online_loop_frequency_hz);
  while (ros::ok() && !online_loop_stop.load())
  {
    if (online_loop_backend &&
        static_cast<int>(online_loop_backend->pendingOnlineKeyframeCount()) >= online_loop_min_new_keyframes)
    {
      std::string error;
      if (online_loop_backend->runOnce(&error))
      {
        LoopBackendResult result;
        if (online_loop_backend->getLatestResult(&result)) publishOnlineLoopResult(result);
      }
      else if (!error.empty())
      {
        ROS_ERROR("Online loop-backend optimization failed: %s", error.c_str());
      }
    }
    rate.sleep();
  }
}

void LIVMapper::publishOnlineLoopResult(const LoopBackendResult &result)
{
  if (result.keyframes.empty() ||
      result.keyframes.size() != result.optimized_imu_poses.size() ||
      !result.global_keyframe_map)
  {
    ROS_WARN("Online loop-backend produced an incomplete result generation.");
    return;
  }

  const ros::Time stamp = ros::Time::now();
  nav_msgs::Path optimized_path;
  optimized_path.header.stamp = stamp;
  optimized_path.header.frame_id = "map";
  optimized_path.poses.reserve(result.keyframes.size());
  for (std::size_t index = 0; index < result.keyframes.size(); ++index)
  {
    geometry_msgs::PoseStamped pose;
    pose.header.stamp.fromSec(result.keyframes[index].timestamp);
    pose.header.frame_id = "map";
    const Eigen::Vector3d translation = result.optimized_imu_poses[index].translation();
    const Eigen::Quaterniond rotation(result.optimized_imu_poses[index].rotation());
    pose.pose.position.x = translation.x();
    pose.pose.position.y = translation.y();
    pose.pose.position.z = translation.z();
    pose.pose.orientation.x = rotation.x();
    pose.pose.orientation.y = rotation.y();
    pose.pose.orientation.z = rotation.z();
    pose.pose.orientation.w = rotation.w();
    optimized_path.poses.push_back(pose);
  }
  pubLoopOptimizedPath.publish(optimized_path);

  sensor_msgs::PointCloud2 map_message;
  pcl::toROSMsg(*result.global_keyframe_map, map_message);
  map_message.header.stamp = stamp;
  map_message.header.frame_id = "map";
  pubLoopKeyframeMap.publish(map_message);

  const std::size_t latest = result.keyframes.size() - 1;
  nav_msgs::Odometry optimized_odom;
  optimized_odom.header.stamp = stamp;
  optimized_odom.header.frame_id = "map";
  optimized_odom.child_frame_id = "aft_mapped_optimized";
  const Eigen::Vector3d latest_translation = result.optimized_imu_poses[latest].translation();
  const Eigen::Quaterniond latest_rotation(result.optimized_imu_poses[latest].rotation());
  optimized_odom.pose.pose.position.x = latest_translation.x();
  optimized_odom.pose.pose.position.y = latest_translation.y();
  optimized_odom.pose.pose.position.z = latest_translation.z();
  optimized_odom.pose.pose.orientation.x = latest_rotation.x();
  optimized_odom.pose.pose.orientation.y = latest_rotation.y();
  optimized_odom.pose.pose.orientation.z = latest_rotation.z();
  optimized_odom.pose.pose.orientation.w = latest_rotation.w();
  pubLoopOptimizedOdom.publish(optimized_odom);

  // 使用最新关键帧的校正量发布 map -> camera_init。
  // 前端仍在 camera_init(odom) 中工作；该 TF 仅服务全局可视化和当前优化位姿。
  const Eigen::Isometry3d T_map_odom = result.optimized_imu_poses[latest] *
                                       result.keyframes[latest].T_odom_imu.inverse();
  const Eigen::Quaterniond correction_rotation(T_map_odom.rotation());
  static tf::TransformBroadcaster broadcaster;
  tf::Transform correction;
  correction.setOrigin(tf::Vector3(T_map_odom.translation().x(), T_map_odom.translation().y(),
                                   T_map_odom.translation().z()));
  correction.setRotation(tf::Quaternion(correction_rotation.x(), correction_rotation.y(),
                                        correction_rotation.z(), correction_rotation.w()));
  broadcaster.sendTransform(tf::StampedTransform(correction, stamp, "map", "camera_init"));

  visualization_msgs::MarkerArray edge_markers;
  visualization_msgs::Marker edges;
  edges.header.stamp = stamp;
  edges.header.frame_id = "map";
  edges.ns = "loop_backend";
  edges.id = 0;
  edges.type = visualization_msgs::Marker::LINE_LIST;
  edges.action = visualization_msgs::Marker::ADD;
  edges.scale.x = 0.08;
  edges.color.r = 1.0;
  edges.color.g = 0.2;
  edges.color.b = 0.0;
  edges.color.a = 1.0;
  for (const LoopEdgeReport &edge : result.loop_edges)
  {
    std::size_t current_index = result.keyframes.size();
    std::size_t history_index = result.keyframes.size();
    for (std::size_t index = 0; index < result.keyframes.size(); ++index)
    {
      if (result.keyframes[index].id == edge.current_id) current_index = index;
      if (result.keyframes[index].id == edge.history_id) history_index = index;
    }
    if (current_index == result.keyframes.size() || history_index == result.keyframes.size()) continue;

    geometry_msgs::Point current_point;
    geometry_msgs::Point history_point;
    current_point.x = result.optimized_imu_poses[current_index].translation().x();
    current_point.y = result.optimized_imu_poses[current_index].translation().y();
    current_point.z = result.optimized_imu_poses[current_index].translation().z();
    history_point.x = result.optimized_imu_poses[history_index].translation().x();
    history_point.y = result.optimized_imu_poses[history_index].translation().y();
    history_point.z = result.optimized_imu_poses[history_index].translation().z();
    edges.points.push_back(current_point);
    edges.points.push_back(history_point);
  }
  edge_markers.markers.push_back(edges);
  pubLoopConstraintEdges.publish(edge_markers);

  ROS_INFO("[loop-backend] Published generation %llu: %zu keyframes, %zu loop edges.",
           static_cast<unsigned long long>(result.generation), result.keyframes.size(), result.loop_edges.size());
}
#endif

void LIVMapper::saveLoopClosureKeyframe()
{
  bool online_backend_ready = false;
#ifdef FAST_LIVO_HAS_LOOP_BACKEND
  online_backend_ready = online_loop_enable && static_cast<bool>(online_loop_backend);
#endif
  if ((!keyframe_save_en || !keyframe_output_ready) && !online_backend_ready) return;
  if (feats_down_body->empty()) return;

  const double timestamp = LidarMeasures.last_lio_update_time;
  if (timestamp - _first_lidar_time < keyframe_skip_initialization) return;

  bool should_save = !has_last_keyframe;
  if (has_last_keyframe)
  {
    const double elapsed = timestamp - last_keyframe_time;
    if (elapsed < keyframe_min_interval) return;

    const double translation = (_state.pos_end - last_keyframe_pos).norm();
    const Eigen::AngleAxisd rotation_delta(last_keyframe_rot.transpose() * _state.rot_end);
    const double rotation_deg = rotation_delta.angle() * 180.0 / std::acos(-1.0);
    should_save = translation >= keyframe_translation_thresh || rotation_deg >= keyframe_rotation_thresh_deg;
  }
  if (!should_save) return;

  if (keyframe_save_en && keyframe_output_ready)
  {
    std::ostringstream filename;
    filename << keyframe_output_dir << "/" << std::setfill('0') << std::setw(6) << keyframe_id << ".pcd";
    pcl::PCDWriter writer;
    if (writer.writeBinary(filename.str(), *feats_down_body) != 0)
    {
      ROS_ERROR("Failed to write loop-closure keyframe: %s", filename.str().c_str());
      return;
    }

    const Eigen::Quaterniond q(_state.rot_end);
    fout_keyframe_pose << std::fixed << std::setprecision(9)
                       << keyframe_id << " " << timestamp << " "
                       << _state.pos_end[0] << " " << _state.pos_end[1] << " " << _state.pos_end[2] << " "
                       << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
    fout_keyframe_pose.flush();
  }

#ifdef FAST_LIVO_HAS_LOOP_BACKEND
  if (online_backend_ready)
  {
    Eigen::Isometry3d T_odom_imu = Eigen::Isometry3d::Identity();
    T_odom_imu.linear() = _state.rot_end;
    T_odom_imu.translation() = _state.pos_end;
    std::string error;
    if (!online_loop_backend->enqueueOnlineKeyframe(keyframe_id, timestamp, feats_down_body, T_odom_imu, &error))
    {
      ROS_ERROR("Failed to enqueue online loop keyframe %d: %s", keyframe_id, error.c_str());
    }
  }
#endif

  last_keyframe_time = timestamp;
  last_keyframe_pos = _state.pos_end;
  last_keyframe_rot = _state.rot_end;
  has_last_keyframe = true;
  ++keyframe_id;
}

void LIVMapper::initializeSubscribersAndPublishers(ros::NodeHandle &nh, image_transport::ImageTransport &it) 
{
  sub_pcl = p_pre->lidar_type == AVIA ? 
            nh.subscribe(lid_topic, 200000, &LIVMapper::livox_pcl_cbk, this): 
            nh.subscribe(lid_topic, 200000, &LIVMapper::standard_pcl_cbk, this);
  sub_imu = nh.subscribe(imu_topic, 200000, &LIVMapper::imu_cbk, this);
  // 机器狗原始数据为 CompressedImage；通过参数选择订阅类型，避免影响原有 raw Image 数据集。
  sub_img = img_compressed ?
            nh.subscribe(img_topic, 200000, &LIVMapper::compressed_img_cbk, this):
            nh.subscribe(img_topic, 200000, &LIVMapper::img_cbk, this);
  
  pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100);
  pubNormal = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 100);
  pubSubVisualMap = nh.advertise<sensor_msgs::PointCloud2>("/cloud_visual_sub_map_before", 100);
  pubLaserCloudEffect = nh.advertise<sensor_msgs::PointCloud2>("/cloud_effected", 100);
  pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>("/Laser_map", 100);
  pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 10);
  pubPath = nh.advertise<nav_msgs::Path>("/path", 10);
  plane_pub = nh.advertise<visualization_msgs::Marker>("/planner_normal", 1);
  voxel_pub = nh.advertise<visualization_msgs::MarkerArray>("/voxels", 1);
  pubLaserCloudDyn = nh.advertise<sensor_msgs::PointCloud2>("/dyn_obj", 100);
  pubLaserCloudDynRmed = nh.advertise<sensor_msgs::PointCloud2>("/dyn_obj_removed", 100);
  pubLaserCloudDynDbg = nh.advertise<sensor_msgs::PointCloud2>("/dyn_obj_dbg_hist", 100);
  mavros_pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
  pubImage = it.advertise("/rgb_img", 1);
  pubImuPropOdom = nh.advertise<nav_msgs::Odometry>("/LIVO2/imu_propagate", 10000);
  imu_prop_timer = nh.createTimer(ros::Duration(0.004), &LIVMapper::imu_prop_callback, this);
  voxelmap_manager->voxel_map_pub_= nh.advertise<visualization_msgs::MarkerArray>("/planes", 10000);
#ifdef FAST_LIVO_HAS_LOOP_BACKEND
  pubLoopOptimizedPath = nh.advertise<nav_msgs::Path>("/loop_backend/optimized_path", 1, true);
  pubLoopKeyframeMap = nh.advertise<sensor_msgs::PointCloud2>("/loop_backend/global_keyframe_map", 1, true);
  pubLoopOptimizedOdom = nh.advertise<nav_msgs::Odometry>("/loop_backend/optimized_odom", 1);
  pubLoopConstraintEdges = nh.advertise<visualization_msgs::MarkerArray>("/loop_backend/loop_edges", 1, true);
  startOnlineLoopBackend();
#endif
}

void LIVMapper::handleFirstFrame() 
{
  if (!is_first_frame)
  {
    _first_lidar_time = LidarMeasures.last_lio_update_time;
    p_imu->first_lidar_time = _first_lidar_time; // Only for IMU data log
    is_first_frame = true;
    cout << "FIRST LIDAR FRAME!" << endl;
  }
}

void LIVMapper::gravityAlignment() 
{
  if (!p_imu->imu_need_init && !gravity_align_finished) 
  {
    std::cout << "Gravity Alignment Starts" << std::endl;
    V3D ez(0, 0, -1), gz(_state.gravity);
    Quaterniond G_q_I0 = Quaterniond::FromTwoVectors(gz, ez);
    M3D G_R_I0 = G_q_I0.toRotationMatrix();

    _state.pos_end = G_R_I0 * _state.pos_end;
    _state.rot_end = G_R_I0 * _state.rot_end;
    _state.vel_end = G_R_I0 * _state.vel_end;
    _state.gravity = G_R_I0 * _state.gravity;
    gravity_align_finished = true;
    std::cout << "Gravity Alignment Finished" << std::endl;
  }
}

void LIVMapper::processImu() 
{
  // double t0 = omp_get_wtime();

  p_imu->Process2(LidarMeasures, _state, feats_undistort);

  if (gravity_align_en) gravityAlignment();

  state_propagat = _state;
  voxelmap_manager->state_ = _state;
  voxelmap_manager->feats_undistort_ = feats_undistort;

  // double t_prop = omp_get_wtime();

  // std::cout << "[ Mapping ] feats_undistort: " << feats_undistort->size() << std::endl;
  // std::cout << "[ Mapping ] predict cov: " << _state.cov.diagonal().transpose() << std::endl;
  // std::cout << "[ Mapping ] predict sta: " << state_propagat.pos_end.transpose() << state_propagat.vel_end.transpose() << std::endl;
}

void LIVMapper::stateEstimationAndMapping() 
{
  switch (LidarMeasures.lio_vio_flg) 
  {
    case VIO:
      handleVIO();
      break;
    case LIO:
    case LO:
      handleLIO();
      break;
  }
}

void LIVMapper::handleVIO() 
{
  euler_cur = RotMtoEuler(_state.rot_end);
  fout_pre << std::setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
            << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
            << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << std::endl;
    
  if (pcl_w_wait_pub->empty() || (pcl_w_wait_pub == nullptr)) 
  {
    std::cout << "[ VIO ] No point!!!" << std::endl;
    return;
  }
    
  std::cout << "[ VIO ] Raw feature num: " << pcl_w_wait_pub->points.size() << std::endl;

  if (fabs((LidarMeasures.last_lio_update_time - _first_lidar_time) - plot_time) < (frame_cnt / 2 * 0.1)) 
  {
    vio_manager->plot_flag = true;
  } 
  else 
  {
    vio_manager->plot_flag = false;
  }

  vio_manager->processFrame(LidarMeasures.measures.back().img, _pv_list, voxelmap_manager->voxel_map_, LidarMeasures.last_lio_update_time - _first_lidar_time);

  if (imu_prop_enable) 
  {
    ekf_finish_once = true;
    latest_ekf_state = _state;
    latest_ekf_time = LidarMeasures.last_lio_update_time;
    state_update_flg = true;
  }

  // int size_sub_map = vio_manager->visual_sub_map_cur.size();
  // visual_sub_map->reserve(size_sub_map);
  // for (int i = 0; i < size_sub_map; i++) 
  // {
  //   PointType temp_map;
  //   temp_map.x = vio_manager->visual_sub_map_cur[i]->pos_[0];
  //   temp_map.y = vio_manager->visual_sub_map_cur[i]->pos_[1];
  //   temp_map.z = vio_manager->visual_sub_map_cur[i]->pos_[2];
  //   temp_map.intensity = 0.;
  //   visual_sub_map->push_back(temp_map);
  // }

  publish_frame_world(pubLaserCloudFullRes, vio_manager);
  publish_img_rgb(pubImage, vio_manager);

  euler_cur = RotMtoEuler(_state.rot_end);
  fout_out << std::setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
            << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
            << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << " " << feats_undistort->points.size() << std::endl;
}

void LIVMapper::handleLIO() 
{    
  euler_cur = RotMtoEuler(_state.rot_end);
  fout_pre << setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
           << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
           << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << endl;
           
  if (feats_undistort->empty() || (feats_undistort == nullptr)) 
  {
    std::cout << "[ LIO ]: No point!!!" << std::endl;
    return;
  }

  double t0 = omp_get_wtime();

  downSizeFilterSurf.setInputCloud(feats_undistort);
  downSizeFilterSurf.filter(*feats_down_body);
  
  double t_down = omp_get_wtime();

  feats_down_size = feats_down_body->points.size();
  voxelmap_manager->feats_down_body_ = feats_down_body;
  transformLidar(_state.rot_end, _state.pos_end, feats_down_body, feats_down_world);
  voxelmap_manager->feats_down_world_ = feats_down_world;
  voxelmap_manager->feats_down_size_ = feats_down_size;
  
  if (!lidar_map_inited) 
  {
    lidar_map_inited = true;
    voxelmap_manager->BuildVoxelMap();
  }

  double t1 = omp_get_wtime();

  voxelmap_manager->StateEstimation(state_propagat);
  _state = voxelmap_manager->state_;
  _pv_list = voxelmap_manager->pv_list_;
  saveLoopClosureKeyframe();

  double t2 = omp_get_wtime();

  if (imu_prop_enable) 
  {
    ekf_finish_once = true;
    latest_ekf_state = _state;
    latest_ekf_time = LidarMeasures.last_lio_update_time;
    state_update_flg = true;
  }

  if (pose_output_en) 
  {
    static bool pos_opend = false;
    static int ocount = 0;
    std::ofstream outFile, evoFile;
    if (!pos_opend) 
    {
      evoFile.open(std::string(ROOT_DIR) + "Log/result/" + seq_name + ".txt", std::ios::out);
      pos_opend = true;
      if (!evoFile.is_open()) ROS_ERROR("open fail\n");
    } 
    else 
    {
      evoFile.open(std::string(ROOT_DIR) + "Log/result/" + seq_name + ".txt", std::ios::app);
      if (!evoFile.is_open()) ROS_ERROR("open fail\n");
    }
    Eigen::Matrix4d outT;
    Eigen::Quaterniond q(_state.rot_end);
    evoFile << std::fixed;
    evoFile << LidarMeasures.last_lio_update_time << " " << _state.pos_end[0] << " " << _state.pos_end[1] << " " << _state.pos_end[2] << " "
            << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
  }
  
  euler_cur = RotMtoEuler(_state.rot_end);
  geoQuat = tf::createQuaternionMsgFromRollPitchYaw(euler_cur(0), euler_cur(1), euler_cur(2));
  publish_odometry(pubOdomAftMapped);

  double t3 = omp_get_wtime();

  PointCloudXYZI::Ptr world_lidar(new PointCloudXYZI());
  transformLidar(_state.rot_end, _state.pos_end, feats_down_body, world_lidar);
  for (size_t i = 0; i < world_lidar->points.size(); i++) 
  {
    voxelmap_manager->pv_list_[i].point_w << world_lidar->points[i].x, world_lidar->points[i].y, world_lidar->points[i].z;
    M3D point_crossmat = voxelmap_manager->cross_mat_list_[i];
    M3D var = voxelmap_manager->body_cov_list_[i];
    var = (_state.rot_end * extR) * var * (_state.rot_end * extR).transpose() +
          (-point_crossmat) * _state.cov.block<3, 3>(0, 0) * (-point_crossmat).transpose() + _state.cov.block<3, 3>(3, 3);
    voxelmap_manager->pv_list_[i].var = var;
  }
  voxelmap_manager->UpdateVoxelMap(voxelmap_manager->pv_list_);
  std::cout << "[ LIO ] Update Voxel Map" << std::endl;
  _pv_list = voxelmap_manager->pv_list_;
  
  double t4 = omp_get_wtime();

  if(voxelmap_manager->config_setting_.map_sliding_en)
  {
    voxelmap_manager->mapSliding();
  }
  
  PointCloudXYZI::Ptr laserCloudFullRes(dense_map_en ? feats_undistort : feats_down_body);
  int size = laserCloudFullRes->points.size();
  PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

  for (int i = 0; i < size; i++) 
  {
    RGBpointBodyToWorld(&laserCloudFullRes->points[i], &laserCloudWorld->points[i]);
  }
  *pcl_w_wait_pub = *laserCloudWorld;

  publish_frame_world(pubLaserCloudFullRes, vio_manager);
  if (pub_effect_point_en) publish_effect_world(pubLaserCloudEffect, voxelmap_manager->ptpl_list_);
  if (voxelmap_manager->config_setting_.is_pub_plane_map_) voxelmap_manager->pubVoxelMap();
  publish_path(pubPath);
  publish_mavros(mavros_pose_publisher);

  frame_num++;
  aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t4 - t0) / frame_num;

  // aver_time_icp = aver_time_icp * (frame_num - 1) / frame_num + (t2 - t1) / frame_num;
  // aver_time_map_inre = aver_time_map_inre * (frame_num - 1) / frame_num + (t4 - t3) / frame_num;
  // aver_time_solve = aver_time_solve * (frame_num - 1) / frame_num + (solve_time) / frame_num;
  // aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1) / frame_num + solve_const_H_time / frame_num;
  // printf("[ mapping time ]: per scan: propagation %0.6f downsample: %0.6f match: %0.6f solve: %0.6f  ICP: %0.6f  map incre: %0.6f total: %0.6f \n"
  //         "[ mapping time ]: average: icp: %0.6f construct H: %0.6f, total: %0.6f \n",
  //         t_prop - t0, t1 - t_prop, match_time, solve_time, t3 - t1, t5 - t3, t5 - t0, aver_time_icp, aver_time_const_H_time, aver_time_consu);

  // printf("\033[1;36m[ LIO mapping time ]: current scan: icp: %0.6f secs, map incre: %0.6f secs, total: %0.6f secs.\033[0m\n"
  //         "\033[1;36m[ LIO mapping time ]: average: icp: %0.6f secs, map incre: %0.6f secs, total: %0.6f secs.\033[0m\n",
  //         t2 - t1, t4 - t3, t4 - t0, aver_time_icp, aver_time_map_inre, aver_time_consu);
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;34m|                         LIO Mapping Time                    |\033[0m\n");
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;34m| %-29s | %-27s |\033[0m\n", "Algorithm Stage", "Time (secs)");
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "DownSample", t_down - t0);
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "ICP", t2 - t1);
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "updateVoxelMap", t4 - t3);
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "Current Total Time", t4 - t0);
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "Average Total Time", aver_time_consu);
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");

  euler_cur = RotMtoEuler(_state.rot_end);
  fout_out << std::setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
            << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
            << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << " " << feats_undistort->points.size() << std::endl;
}

void LIVMapper::savePCD() 
{
  if (pcd_save_en && (pcl_wait_save->points.size() > 0 || pcl_wait_save_intensity->points.size() > 0) && pcd_save_interval < 0) 
  {
    std::string raw_points_dir = std::string(ROOT_DIR) + "Log/pcd/all_raw_points.pcd";
    std::string downsampled_points_dir = std::string(ROOT_DIR) + "Log/pcd/all_downsampled_points.pcd";
    pcl::PCDWriter pcd_writer;

    if (img_en)
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
      voxel_filter.setInputCloud(pcl_wait_save);
      voxel_filter.setLeafSize(filter_size_pcd, filter_size_pcd, filter_size_pcd);
      voxel_filter.filter(*downsampled_cloud);
  
      pcd_writer.writeBinary(raw_points_dir, *pcl_wait_save); // Save the raw point cloud data
      std::cout << GREEN << "Raw point cloud data saved to: " << raw_points_dir 
                << " with point count: " << pcl_wait_save->points.size() << RESET << std::endl;
      
      pcd_writer.writeBinary(downsampled_points_dir, *downsampled_cloud); // Save the downsampled point cloud data
      std::cout << GREEN << "Downsampled point cloud data saved to: " << downsampled_points_dir 
                << " with point count after filtering: " << downsampled_cloud->points.size() << RESET << std::endl;

      if(colmap_output_en)
      {
        fout_points << "# 3D point list with one line of data per point\n";
        fout_points << "#  POINT_ID, X, Y, Z, R, G, B, ERROR\n";
        for (size_t i = 0; i < downsampled_cloud->size(); ++i) 
        {
            const auto& point = downsampled_cloud->points[i];
            fout_points << i << " "
                        << std::fixed << std::setprecision(6)
                        << point.x << " " << point.y << " " << point.z << " "
                        << static_cast<int>(point.r) << " "
                        << static_cast<int>(point.g) << " "
                        << static_cast<int>(point.b) << " "
                        << 0 << std::endl;
        }
      }
    }
    else
    {      
      pcd_writer.writeBinary(raw_points_dir, *pcl_wait_save_intensity);
      std::cout << GREEN << "Raw point cloud data saved to: " << raw_points_dir 
                << " with point count: " << pcl_wait_save_intensity->points.size() << RESET << std::endl;
    }
  }
}

void LIVMapper::run() 
{
  startDenseRgbCacheWriter();
  ros::Rate rate(5000);
  while (ros::ok()) 
  {
    ros::spinOnce();
    if (!sync_packages(LidarMeasures)) 
    {
      rate.sleep();
      continue;
    }
    handleFirstFrame();

    processImu();

    // if (!p_imu->imu_time_init) continue;

    stateEstimationAndMapping();
  }
#ifdef FAST_LIVO_HAS_LOOP_BACKEND
  stopOnlineLoopBackend();
#endif
  stopDenseRgbCacheWriter();
  exportDenseRgbMapOnShutdown();
  savePCD();
  cleanupC2IntermediateFiles();
}

void LIVMapper::prop_imu_once(StatesGroup &imu_prop_state, const double dt, V3D acc_avr, V3D angvel_avr)
{
  double mean_acc_norm = p_imu->IMU_mean_acc_norm;
  acc_avr = acc_avr * G_m_s2 / mean_acc_norm - imu_prop_state.bias_a;
  angvel_avr -= imu_prop_state.bias_g;

  M3D Exp_f = Exp(angvel_avr, dt);
  /* propogation of IMU attitude */
  imu_prop_state.rot_end = imu_prop_state.rot_end * Exp_f;

  /* Specific acceleration (global frame) of IMU */
  V3D acc_imu = imu_prop_state.rot_end * acc_avr + V3D(imu_prop_state.gravity[0], imu_prop_state.gravity[1], imu_prop_state.gravity[2]);

  /* propogation of IMU */
  imu_prop_state.pos_end = imu_prop_state.pos_end + imu_prop_state.vel_end * dt + 0.5 * acc_imu * dt * dt;

  /* velocity of IMU */
  imu_prop_state.vel_end = imu_prop_state.vel_end + acc_imu * dt;
}

void LIVMapper::imu_prop_callback(const ros::TimerEvent &e)
{
  if (p_imu->imu_need_init || !new_imu || !ekf_finish_once) { return; }
  mtx_buffer_imu_prop.lock();
  new_imu = false; // 控制propagate频率和IMU频率一致
  if (imu_prop_enable && !prop_imu_buffer.empty())
  {
    static double last_t_from_lidar_end_time = 0;
    if (state_update_flg)
    {
      imu_propagate = latest_ekf_state;
      // drop all useless imu pkg
      while ((!prop_imu_buffer.empty() && prop_imu_buffer.front().header.stamp.toSec() < latest_ekf_time))
      {
        prop_imu_buffer.pop_front();
      }
      last_t_from_lidar_end_time = 0;
      for (int i = 0; i < prop_imu_buffer.size(); i++)
      {
        double t_from_lidar_end_time = prop_imu_buffer[i].header.stamp.toSec() - latest_ekf_time;
        double dt = t_from_lidar_end_time - last_t_from_lidar_end_time;
        // cout << "prop dt" << dt << ", " << t_from_lidar_end_time << ", " << last_t_from_lidar_end_time << endl;
        V3D acc_imu(prop_imu_buffer[i].linear_acceleration.x, prop_imu_buffer[i].linear_acceleration.y, prop_imu_buffer[i].linear_acceleration.z);
        V3D omg_imu(prop_imu_buffer[i].angular_velocity.x, prop_imu_buffer[i].angular_velocity.y, prop_imu_buffer[i].angular_velocity.z);
        prop_imu_once(imu_propagate, dt, acc_imu, omg_imu);
        last_t_from_lidar_end_time = t_from_lidar_end_time;
      }
      state_update_flg = false;
    }
    else
    {
      V3D acc_imu(newest_imu.linear_acceleration.x, newest_imu.linear_acceleration.y, newest_imu.linear_acceleration.z);
      V3D omg_imu(newest_imu.angular_velocity.x, newest_imu.angular_velocity.y, newest_imu.angular_velocity.z);
      double t_from_lidar_end_time = newest_imu.header.stamp.toSec() - latest_ekf_time;
      double dt = t_from_lidar_end_time - last_t_from_lidar_end_time;
      prop_imu_once(imu_propagate, dt, acc_imu, omg_imu);
      last_t_from_lidar_end_time = t_from_lidar_end_time;
    }

    V3D posi, vel_i;
    Eigen::Quaterniond q;
    posi = imu_propagate.pos_end;
    vel_i = imu_propagate.vel_end;
    q = Eigen::Quaterniond(imu_propagate.rot_end);
    imu_prop_odom.header.frame_id = "world";
    imu_prop_odom.header.stamp = newest_imu.header.stamp;
    imu_prop_odom.pose.pose.position.x = posi.x();
    imu_prop_odom.pose.pose.position.y = posi.y();
    imu_prop_odom.pose.pose.position.z = posi.z();
    imu_prop_odom.pose.pose.orientation.w = q.w();
    imu_prop_odom.pose.pose.orientation.x = q.x();
    imu_prop_odom.pose.pose.orientation.y = q.y();
    imu_prop_odom.pose.pose.orientation.z = q.z();
    imu_prop_odom.twist.twist.linear.x = vel_i.x();
    imu_prop_odom.twist.twist.linear.y = vel_i.y();
    imu_prop_odom.twist.twist.linear.z = vel_i.z();
    pubImuPropOdom.publish(imu_prop_odom);
  }
  mtx_buffer_imu_prop.unlock();
}

void LIVMapper::transformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud, PointCloudXYZI::Ptr &trans_cloud)
{
  PointCloudXYZI().swap(*trans_cloud);
  trans_cloud->reserve(input_cloud->size());
  for (size_t i = 0; i < input_cloud->size(); i++)
  {
    pcl::PointXYZINormal p_c = input_cloud->points[i];
    Eigen::Vector3d p(p_c.x, p_c.y, p_c.z);
    p = (rot * (extR * p + extT) + t);
    PointType pi;
    pi.x = p(0);
    pi.y = p(1);
    pi.z = p(2);
    pi.intensity = p_c.intensity;
    trans_cloud->points.push_back(pi);
  }
}

void LIVMapper::pointBodyToWorld(const PointType &pi, PointType &po)
{
  V3D p_body(pi.x, pi.y, pi.z);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po.x = p_global(0);
  po.y = p_global(1);
  po.z = p_global(2);
  po.intensity = pi.intensity;
}

template <typename T> void LIVMapper::pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
  V3D p_body(pi[0], pi[1], pi[2]);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po[0] = p_global(0);
  po[1] = p_global(1);
  po[2] = p_global(2);
}

template <typename T> Matrix<T, 3, 1> LIVMapper::pointBodyToWorld(const Matrix<T, 3, 1> &pi)
{
  V3D p(pi[0], pi[1], pi[2]);
  p = (_state.rot_end * (extR * p + extT) + _state.pos_end);
  Matrix<T, 3, 1> po(p[0], p[1], p[2]);
  return po;
}

void LIVMapper::RGBpointBodyToWorld(PointType const *const pi, PointType *const po)
{
  V3D p_body(pi->x, pi->y, pi->z);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->intensity = pi->intensity;
}

void LIVMapper::RGBpointBodyLidarToIMU(PointType const *const pi, PointType *const po)
{
  V3D p_body_lidar(pi->x, pi->y, pi->z);
  V3D p_body_imu(extR * p_body_lidar + extT);

  po->x = p_body_imu(0);
  po->y = p_body_imu(1);
  po->z = p_body_imu(2);
  po->intensity = pi->intensity;
}

void LIVMapper::standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  if (!lidar_en) return;
  mtx_buffer.lock();

  double cur_head_time = msg->header.stamp.toSec() + lidar_time_offset;
  // cout<<"got feature"<<endl;
  if (cur_head_time < last_timestamp_lidar)
  {
    ROS_ERROR("lidar loop back, clear buffer");
    lid_raw_data_buffer.clear();
  }
  // ROS_INFO("get point cloud at time: %.6f", msg->header.stamp.toSec());
  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  p_pre->process(msg, ptr);
  lid_raw_data_buffer.push_back(ptr);
  lid_header_time_buffer.push_back(cur_head_time);
  last_timestamp_lidar = cur_head_time;

  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

void LIVMapper::livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg_in)
{
  if (!lidar_en) return;
  mtx_buffer.lock();
  livox_ros_driver::CustomMsg::Ptr msg(new livox_ros_driver::CustomMsg(*msg_in));
  // if ((abs(msg->header.stamp.toSec() - last_timestamp_lidar) > 0.2 && last_timestamp_lidar > 0) || sync_jump_flag)
  // {
  //   ROS_WARN("lidar jumps %.3f\n", msg->header.stamp.toSec() - last_timestamp_lidar);
  //   sync_jump_flag = true;
  //   msg->header.stamp = ros::Time().fromSec(last_timestamp_lidar + 0.1);
  // }
  if (abs(last_timestamp_imu - msg->header.stamp.toSec()) > 1.0 && !imu_buffer.empty())
  {
    double timediff_imu_wrt_lidar = last_timestamp_imu - msg->header.stamp.toSec();
    printf("\033[95mSelf sync IMU and LiDAR, HARD time lag is %.10lf \n\033[0m", timediff_imu_wrt_lidar - 0.100);
    // imu_time_offset = timediff_imu_wrt_lidar;
  }

  double cur_head_time = msg->header.stamp.toSec();
  ROS_INFO("Get LiDAR, its header time: %.6f", cur_head_time);
  if (cur_head_time < last_timestamp_lidar)
  {
    ROS_ERROR("lidar loop back, clear buffer");
    lid_raw_data_buffer.clear();
  }
  // ROS_INFO("get point cloud at time: %.6f", msg->header.stamp.toSec());
  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  p_pre->process(msg, ptr);

  if (!ptr || ptr->empty()) {
    ROS_ERROR("Received an empty point cloud");
    mtx_buffer.unlock();
    return;
  }

  lid_raw_data_buffer.push_back(ptr);
  lid_header_time_buffer.push_back(cur_head_time);
  last_timestamp_lidar = cur_head_time;

  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

void LIVMapper::imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in)
{
  if (!imu_en) return;

  if (last_timestamp_lidar < 0.0) return;
  // ROS_INFO("get imu at time: %.6f", msg_in->header.stamp.toSec());
  sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));
  msg->header.stamp = ros::Time().fromSec(msg->header.stamp.toSec() - imu_time_offset);
  double timestamp = msg->header.stamp.toSec();

  if (fabs(last_timestamp_lidar - timestamp) > 0.5 && (!ros_driver_fix_en))
  {
    ROS_WARN("IMU and LiDAR not synced! delta time: %lf .\n", last_timestamp_lidar - timestamp);
  }

  if (ros_driver_fix_en) timestamp += std::round(last_timestamp_lidar - timestamp);
  msg->header.stamp = ros::Time().fromSec(timestamp);

  mtx_buffer.lock();

  if (last_timestamp_imu > 0.0 && timestamp < last_timestamp_imu)
  {
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    ROS_ERROR("imu loop back, offset: %lf \n", last_timestamp_imu - timestamp);
    return;
  }

  // if (last_timestamp_imu > 0.0 && timestamp > last_timestamp_imu + 0.2)
  // {

  //   ROS_WARN("imu time stamp Jumps %0.4lf seconds \n", timestamp - last_timestamp_imu);
  //   mtx_buffer.unlock();
  //   sig_buffer.notify_all();
  //   return;
  // }

  last_timestamp_imu = timestamp;

  imu_buffer.push_back(msg);
  // cout<<"got imu: "<<timestamp<<" imu size "<<imu_buffer.size()<<endl;
  mtx_buffer.unlock();
  if (imu_prop_enable)
  {
    mtx_buffer_imu_prop.lock();
    if (imu_prop_enable && !p_imu->imu_need_init) { prop_imu_buffer.push_back(*msg); }
    newest_imu = *msg;
    new_imu = true;
    mtx_buffer_imu_prop.unlock();
  }
  sig_buffer.notify_all();
}

cv::Mat LIVMapper::getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
  cv::Mat img;
  img = cv_bridge::toCvCopy(img_msg, "bgr8")->image;
  return img;
}

cv::Mat LIVMapper::getImageFromCompressedMsg(const sensor_msgs::CompressedImageConstPtr &img_msg)
{
  // 压缩图像只在进入视觉前端前解码为 BGR，消息时间戳仍由回调中的 header.stamp 提供。
  if (img_msg->data.empty()) return cv::Mat();
  cv::Mat encoded(1, static_cast<int>(img_msg->data.size()), CV_8UC1, const_cast<uint8_t *>(img_msg->data.data()));
  return cv::imdecode(encoded, cv::IMREAD_COLOR);
}

void LIVMapper::img_cbk(const sensor_msgs::ImageConstPtr &msg_in)
{
  if (!img_en) return;
  sensor_msgs::Image::Ptr msg(new sensor_msgs::Image(*msg_in));
  // if ((abs(msg->header.stamp.toSec() - last_timestamp_img) > 0.2 && last_timestamp_img > 0) || sync_jump_flag)
  // {
  //   ROS_WARN("img jumps %.3f\n", msg->header.stamp.toSec() - last_timestamp_img);
  //   sync_jump_flag = true;
  //   msg->header.stamp = ros::Time().fromSec(last_timestamp_img + 0.1);
  // }

  // Hiliti2022 40Hz
  if (hilti_en)
  {
    static int frame_counter = 0;
    if (++frame_counter % 4 != 0) return;
  }
  // double msg_header_time =  msg->header.stamp.toSec();
  double msg_header_time = msg->header.stamp.toSec() + img_time_offset;
  if (abs(msg_header_time - last_timestamp_img) < 0.001) return;
  ROS_INFO("Get image, its header time: %.6f", msg_header_time);
  if (last_timestamp_lidar < 0) return;

  if (msg_header_time < last_timestamp_img)
  {
    ROS_ERROR("image loop back. \n");
    return;
  }

  mtx_buffer.lock();

  double img_time_correct = msg_header_time; // last_timestamp_lidar + 0.105;

  if (img_time_correct - last_timestamp_img < 0.02)
  {
    ROS_WARN("Image need Jumps: %.6f", img_time_correct);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    return;
  }

  cv::Mat img_cur = getImageFromMsg(msg);
  img_buffer.push_back(img_cur);
  img_time_buffer.push_back(img_time_correct);

  // ROS_INFO("Correct Image time: %.6f", img_time_correct);

  last_timestamp_img = img_time_correct;
  // cv::imshow("img", img);
  // cv::waitKey(1);
  // cout<<"last_timestamp_img:::"<<last_timestamp_img<<endl;
  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

void LIVMapper::compressed_img_cbk(const sensor_msgs::CompressedImageConstPtr &msg_in)
{
  if (!img_en) return;

  // Hiliti2022 40Hz
  if (hilti_en)
  {
    static int frame_counter = 0;
    if (++frame_counter % 4 != 0) return;
  }

  double msg_header_time = msg_in->header.stamp.toSec() + img_time_offset;
  if (abs(msg_header_time - last_timestamp_img) < 0.001) return;
  ROS_INFO("Get compressed image, its header time: %.6f", msg_header_time);
  if (last_timestamp_lidar < 0) return;

  if (msg_header_time < last_timestamp_img)
  {
    ROS_ERROR("compressed image loop back. \n");
    return;
  }

  double img_time_correct = msg_header_time;
  if (img_time_correct - last_timestamp_img < 0.02)
  {
    ROS_WARN("Compressed image need Jumps: %.6f", img_time_correct);
    sig_buffer.notify_all();
    return;
  }

  cv::Mat img_cur = getImageFromCompressedMsg(msg_in);
  if (img_cur.empty())
  {
    ROS_WARN("Compressed image decode failed, skip this frame.");
    return;
  }

  mtx_buffer.lock();
  img_buffer.push_back(img_cur);
  img_time_buffer.push_back(img_time_correct);
  last_timestamp_img = img_time_correct;
  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

bool LIVMapper::sync_packages(LidarMeasureGroup &meas)
{
  if (lid_raw_data_buffer.empty() && lidar_en) return false;
  if (img_buffer.empty() && img_en) return false;
  if (imu_buffer.empty() && imu_en) return false;

  switch (slam_mode_)
  {
  case ONLY_LIO:
  {
    if (meas.last_lio_update_time < 0.0) meas.last_lio_update_time = lid_header_time_buffer.front();
    if (!lidar_pushed)
    {
      // If not push the lidar into measurement data buffer
      meas.lidar = lid_raw_data_buffer.front(); // push the first lidar topic
      if (meas.lidar->points.size() <= 1) return false;

      meas.lidar_frame_beg_time = lid_header_time_buffer.front();                                                // generate lidar_frame_beg_time
      meas.lidar_frame_end_time = meas.lidar_frame_beg_time + meas.lidar->points.back().curvature / double(1000); // calc lidar scan end time
      meas.pcl_proc_cur = meas.lidar;
      lidar_pushed = true;                                                                                       // flag
    }

    if (imu_en && last_timestamp_imu < meas.lidar_frame_end_time)
    { // waiting imu message needs to be
      // larger than _lidar_frame_end_time,
      // make sure complete propagate.
      // ROS_ERROR("out sync");
      return false;
    }

    struct MeasureGroup m; // standard method to keep imu message.

    m.imu.clear();
    m.lio_time = meas.lidar_frame_end_time;
    mtx_buffer.lock();
    while (!imu_buffer.empty())
    {
      if (imu_buffer.front()->header.stamp.toSec() > meas.lidar_frame_end_time) break;
      m.imu.push_back(imu_buffer.front());
      imu_buffer.pop_front();
    }
    lid_raw_data_buffer.pop_front();
    lid_header_time_buffer.pop_front();
    mtx_buffer.unlock();
    sig_buffer.notify_all();

    meas.lio_vio_flg = LIO; // process lidar topic, so timestamp should be lidar scan end.
    meas.measures.push_back(m);
    // ROS_INFO("ONlY HAS LiDAR and IMU, NO IMAGE!");
    lidar_pushed = false; // sync one whole lidar scan.
    return true;

    break;
  }

  case LIVO:
  {
    /*** For LIVO mode, the time of LIO update is set to be the same as VIO, LIO
     * first than VIO imediatly ***/
    EKF_STATE last_lio_vio_flg = meas.lio_vio_flg;
    // double t0 = omp_get_wtime();
    switch (last_lio_vio_flg)
    {
    // double img_capture_time = meas.lidar_frame_beg_time + exposure_time_init;
    case WAIT:
    case VIO:
    {
      // printf("!!! meas.lio_vio_flg: %d \n", meas.lio_vio_flg);
      double img_capture_time = img_time_buffer.front() + exposure_time_init;
      /*** has img topic, but img topic timestamp larger than lidar end time,
       * process lidar topic. After LIO update, the meas.lidar_frame_end_time
       * will be refresh. ***/
      if (meas.last_lio_update_time < 0.0) meas.last_lio_update_time = lid_header_time_buffer.front();
      // printf("[ Data Cut ] wait \n");
      // printf("[ Data Cut ] last_lio_update_time: %lf \n",
      // meas.last_lio_update_time);

      double lid_newest_time = lid_header_time_buffer.back() + lid_raw_data_buffer.back()->points.back().curvature / double(1000);
      double imu_newest_time = imu_buffer.back()->header.stamp.toSec();

      if (img_capture_time < meas.last_lio_update_time + 0.00001)
      {
        img_buffer.pop_front();
        img_time_buffer.pop_front();
        ROS_ERROR("[ Data Cut ] Throw one image frame! \n");
        return false;
      }

      if (img_capture_time > lid_newest_time || img_capture_time > imu_newest_time)
      {
        // ROS_ERROR("lost first camera frame");
        // printf("img_capture_time, lid_newest_time, imu_newest_time: %lf , %lf
        // , %lf \n", img_capture_time, lid_newest_time, imu_newest_time);
        return false;
      }

      struct MeasureGroup m;

      // printf("[ Data Cut ] LIO \n");
      // printf("[ Data Cut ] img_capture_time: %lf \n", img_capture_time);
      m.imu.clear();
      m.lio_time = img_capture_time;
      mtx_buffer.lock();
      while (!imu_buffer.empty())
      {
        if (imu_buffer.front()->header.stamp.toSec() > m.lio_time) break;

        if (imu_buffer.front()->header.stamp.toSec() > meas.last_lio_update_time) m.imu.push_back(imu_buffer.front());

        imu_buffer.pop_front();
        // printf("[ Data Cut ] imu time: %lf \n",
        // imu_buffer.front()->header.stamp.toSec());
      }
      mtx_buffer.unlock();
      sig_buffer.notify_all();

      *(meas.pcl_proc_cur) = *(meas.pcl_proc_next);
      PointCloudXYZI().swap(*meas.pcl_proc_next);

      int lid_frame_num = lid_raw_data_buffer.size();
      int max_size = meas.pcl_proc_cur->size() + 24000 * lid_frame_num;
      meas.pcl_proc_cur->reserve(max_size);
      meas.pcl_proc_next->reserve(max_size);
      // deque<PointCloudXYZI::Ptr> lidar_buffer_tmp;

      while (!lid_raw_data_buffer.empty())
      {
        if (lid_header_time_buffer.front() > img_capture_time) break;
        auto pcl(lid_raw_data_buffer.front()->points);
        double frame_header_time(lid_header_time_buffer.front());
        float max_offs_time_ms = (m.lio_time - frame_header_time) * 1000.0f;

        for (int i = 0; i < pcl.size(); i++)
        {
          auto pt = pcl[i];
          if (pcl[i].curvature < max_offs_time_ms)
          {
            pt.curvature += (frame_header_time - meas.last_lio_update_time) * 1000.0f;
            meas.pcl_proc_cur->points.push_back(pt);
          }
          else
          {
            pt.curvature += (frame_header_time - m.lio_time) * 1000.0f;
            meas.pcl_proc_next->points.push_back(pt);
          }
        }
        lid_raw_data_buffer.pop_front();
        lid_header_time_buffer.pop_front();
      }

      meas.measures.push_back(m);
      meas.lio_vio_flg = LIO;
      // meas.last_lio_update_time = m.lio_time;
      // printf("!!! meas.lio_vio_flg: %d \n", meas.lio_vio_flg);
      // printf("[ Data Cut ] pcl_proc_cur number: %d \n", meas.pcl_proc_cur
      // ->points.size()); printf("[ Data Cut ] LIO process time: %lf \n",
      // omp_get_wtime() - t0);
      return true;
    }

    case LIO:
    {
      double img_capture_time = img_time_buffer.front() + exposure_time_init;
      meas.lio_vio_flg = VIO;
      // printf("[ Data Cut ] VIO \n");
      meas.measures.clear();
      double imu_time = imu_buffer.front()->header.stamp.toSec();

      struct MeasureGroup m;
      m.vio_time = img_capture_time;
      m.lio_time = meas.last_lio_update_time;
      m.img = img_buffer.front();
      mtx_buffer.lock();
      // while ((!imu_buffer.empty() && (imu_time < img_capture_time)))
      // {
      //   imu_time = imu_buffer.front()->header.stamp.toSec();
      //   if (imu_time > img_capture_time) break;
      //   m.imu.push_back(imu_buffer.front());
      //   imu_buffer.pop_front();
      //   printf("[ Data Cut ] imu time: %lf \n",
      //   imu_buffer.front()->header.stamp.toSec());
      // }
      img_buffer.pop_front();
      img_time_buffer.pop_front();
      mtx_buffer.unlock();
      sig_buffer.notify_all();
      meas.measures.push_back(m);
      lidar_pushed = false; // after VIO update, the _lidar_frame_end_time will be refresh.
      // printf("[ Data Cut ] VIO process time: %lf \n", omp_get_wtime() - t0);
      return true;
    }

    default:
    {
      // printf("!! WRONG EKF STATE !!");
      return false;
    }
      // return false;
    }
    break;
  }

  case ONLY_LO:
  {
    if (!lidar_pushed) 
    { 
      // If not in lidar scan, need to generate new meas
      if (lid_raw_data_buffer.empty())  return false;
      meas.lidar = lid_raw_data_buffer.front(); // push the first lidar topic
      meas.lidar_frame_beg_time = lid_header_time_buffer.front(); // generate lidar_beg_time
      meas.lidar_frame_end_time  = meas.lidar_frame_beg_time + meas.lidar->points.back().curvature / double(1000); // calc lidar scan end time
      lidar_pushed = true;             
    }
    struct MeasureGroup m; // standard method to keep imu message.
    m.lio_time = meas.lidar_frame_end_time;
    mtx_buffer.lock();
    lid_raw_data_buffer.pop_front();
    lid_header_time_buffer.pop_front();
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    lidar_pushed = false; // sync one whole lidar scan.
    meas.lio_vio_flg = LO; // process lidar topic, so timestamp should be lidar scan end.
    meas.measures.push_back(m);
    return true;
    break;
  }

  default:
  {
    printf("!! WRONG SLAM TYPE !!");
    return false;
  }
  }
  ROS_ERROR("out sync");
}

void LIVMapper::publish_img_rgb(const image_transport::Publisher &pubImage, VIOManagerPtr vio_manager)
{
  cv::Mat img_rgb = vio_manager->img_cp;
  cv_bridge::CvImage out_msg;
  out_msg.header.stamp = ros::Time::now();
  // out_msg.header.frame_id = "camera_init";
  out_msg.encoding = sensor_msgs::image_encodings::BGR8;
  out_msg.image = img_rgb;
  pubImage.publish(out_msg.toImageMsg());
}

// Provide output format for LiDAR-visual BA
void LIVMapper::publish_frame_world(const ros::Publisher &pubLaserCloudFullRes, VIOManagerPtr vio_manager)
{
  if (pcl_w_wait_pub->empty()) return;
  PointCloudXYZRGB::Ptr laserCloudWorldRGB(new PointCloudXYZRGB());
  static int pub_num = 1;
  pub_num++;

  if (LidarMeasures.lio_vio_flg == VIO)
  {
    *pcl_wait_pub += *pcl_w_wait_pub;
    if(pub_num >= pub_scan_num)
    {
      pub_num = 1;
      size_t size = pcl_wait_pub->points.size();
      laserCloudWorldRGB->reserve(size);
      // double inv_expo = _state.inv_expo_time;
      cv::Mat img_rgb = vio_manager->img_rgb;
      for (size_t i = 0; i < size; i++)
      {
        PointTypeRGB pointRGB;
        pointRGB.x = pcl_wait_pub->points[i].x;
        pointRGB.y = pcl_wait_pub->points[i].y;
        pointRGB.z = pcl_wait_pub->points[i].z;

        V3D p_w(pcl_wait_pub->points[i].x, pcl_wait_pub->points[i].y, pcl_wait_pub->points[i].z);
        V3D pf(vio_manager->new_frame_->w2f(p_w)); if (pf[2] < 0) continue;
        V2D pc(vio_manager->new_frame_->w2c(p_w));

        if (vio_manager->new_frame_->cam_->isInFrame(pc.cast<int>(), 3)) // 100
        {
          V3F pixel = vio_manager->getInterpolatedPixel(img_rgb, pc);
          pointRGB.r = pixel[2];
          pointRGB.g = pixel[1];
          pointRGB.b = pixel[0];
          // pointRGB.r = pixel[2] * inv_expo; pointRGB.g = pixel[1] * inv_expo; pointRGB.b = pixel[0] * inv_expo;
          // if (pointRGB.r > 255) pointRGB.r = 255; else if (pointRGB.r < 0) pointRGB.r = 0;
          // if (pointRGB.g > 255) pointRGB.g = 255; else if (pointRGB.g < 0) pointRGB.g = 0;
          // if (pointRGB.b > 255) pointRGB.b = 255; else if (pointRGB.b < 0) pointRGB.b = 0;
          if (pf.norm() > blind_rgb_points) laserCloudWorldRGB->push_back(pointRGB);
        }
      }
    }
  }

  // C1 records precisely the colored batches that are accumulated into the
  // raw RGB map.  They remain in the original camera_init frame; the offline
  // exporter later applies the time-interpolated map<-odom correction.
  if (dense_rgb_cache_enabled && LidarMeasures.lio_vio_flg == VIO && !laserCloudWorldRGB->empty())
  {
    enqueueDenseRgbCache(laserCloudWorldRGB, LidarMeasures.measures.back().vio_time);
  }

  /*** Publish Frame ***/
  const bool visual_update = slam_mode_ == LIVO && LidarMeasures.lio_vio_flg == VIO;
  const bool lidar_only_mode = slam_mode_ == ONLY_LIO || slam_mode_ == ONLY_LO;
  const bool has_publishable_cloud =
      (visual_update && !laserCloudWorldRGB->empty()) ||
      (lidar_only_mode && !pcl_w_wait_pub->empty());

  // VIO frames are accumulated according to pub_scan_num. Do not publish an
  // intermediate empty PointCloud2, because it clears RViz's current display.
  if (has_publishable_cloud && pubLaserCloudFullRes.getNumSubscribers() > 0)
  {
    sensor_msgs::PointCloud2 laserCloudmsg;
    if (visual_update)
    {
      pcl::toROSMsg(*laserCloudWorldRGB, laserCloudmsg);
    }
    else
    {
      pcl::toROSMsg(*pcl_w_wait_pub, laserCloudmsg);
    }
    const double sensor_timestamp = LidarMeasures.lio_vio_flg == VIO ?
        LidarMeasures.measures.back().vio_time : LidarMeasures.measures.back().lio_time;
    laserCloudmsg.header.stamp.fromSec(sensor_timestamp);
    laserCloudmsg.header.frame_id = "camera_init";
    pubLaserCloudFullRes.publish(laserCloudmsg);
  }

  /**************** save map ****************/
  /* 1. make sure you have enough memories
  /* 2. noted that pcd save will influence the real-time performences **/
  double update_time = 0.0;
  if (LidarMeasures.lio_vio_flg == VIO) {
    update_time = LidarMeasures.measures.back().vio_time;
  } else { // LIO / LO
    update_time = LidarMeasures.measures.back().lio_time;
  }
  std::stringstream ss_time;
  ss_time << std::fixed << std::setprecision(6) << update_time;

  if (pcd_save_en)
  {
    static int scan_wait_num = 0;

    switch (pcd_save_type)
    {
      case 0: /** world frame **/
        if (slam_mode_ == LIVO)
        {
          *pcl_wait_save += *laserCloudWorldRGB;
        }
        else
        {
          *pcl_wait_save_intensity += *pcl_w_wait_pub;
        }
        if(LidarMeasures.lio_vio_flg == LIO || LidarMeasures.lio_vio_flg == LO) scan_wait_num++;
        break;

      case 1: /** body frame **/
        if (LidarMeasures.lio_vio_flg == LIO || LidarMeasures.lio_vio_flg == LO)
        {
          int size = feats_undistort->points.size();
          PointCloudXYZI::Ptr laserCloudBody(new PointCloudXYZI(size, 1));
          for (int i = 0; i < size; i++)
          {
            RGBpointBodyLidarToIMU(&feats_undistort->points[i], &laserCloudBody->points[i]);
          }
          *pcl_wait_save_intensity += *laserCloudBody;
          scan_wait_num++;
          cout << "save body frame points: " << pcl_wait_save_intensity->points.size() << endl;
        }
        pcd_save_interval = 1;
        
        break;

      default:
        pcd_save_interval = 1;
        scan_wait_num++;
        break;
    }
    if ((pcl_wait_save->size() > 0 || pcl_wait_save_intensity->size() > 0) && pcd_save_interval > 0 && scan_wait_num >= pcd_save_interval)
    {
      string all_points_dir(string(string(ROOT_DIR) + "Log/pcd/") + ss_time.str() + string(".pcd"));

      pcl::PCDWriter pcd_writer;

      cout << "current scan saved to " << all_points_dir << endl;
      if (pcl_wait_save->points.size() > 0)
      {
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save); // pcl::io::savePCDFileASCII(all_points_dir, *pcl_wait_save);
        PointCloudXYZRGB().swap(*pcl_wait_save);
      }
      if(pcl_wait_save_intensity->points.size() > 0)
      {
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save_intensity);
        PointCloudXYZI().swap(*pcl_wait_save_intensity);
      }
      scan_wait_num = 0;
    }
    
    if(LidarMeasures.lio_vio_flg == LIO || LidarMeasures.lio_vio_flg == LO)
    {
      Eigen::Quaterniond q(_state.rot_end);
      fout_lidar_pos << std::fixed << std::setprecision(6);
      fout_lidar_pos <<  LidarMeasures.measures.back().lio_time << " " << _state.pos_end[0] << " " << _state.pos_end[1] << " " << _state.pos_end[2] << " " << q.x() << " " << q.y() << " " << q.z()
          << " " << q.w() << " " << endl;
    }
  }
  if (img_save_en && LidarMeasures.lio_vio_flg == VIO)
  {
    static int img_wait_num = 0;
    img_wait_num++;

    if (img_save_interval > 0 && img_wait_num >= img_save_interval)
    {
      imwrite(string(string(ROOT_DIR) + "Log/image/") + ss_time.str() + string(".png"), vio_manager->img_rgb);
      
      Eigen::Quaterniond q(_state.rot_end);
      fout_visual_pos << std::fixed << std::setprecision(6);
      fout_visual_pos << LidarMeasures.measures.back().vio_time << " " << _state.pos_end[0] << " " << _state.pos_end[1] << " " << _state.pos_end[2] << " "
            << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
      img_wait_num = 0;
    }
  }

  if(laserCloudWorldRGB->size() > 0)  PointCloudXYZI().swap(*pcl_wait_pub); 
  if(LidarMeasures.lio_vio_flg == VIO)  PointCloudXYZI().swap(*pcl_w_wait_pub);
}

void LIVMapper::publish_visual_sub_map(const ros::Publisher &pubSubVisualMap)
{
  PointCloudXYZI::Ptr laserCloudFullRes(visual_sub_map);
  int size = laserCloudFullRes->points.size(); if (size == 0) return;
  PointCloudXYZI::Ptr sub_pcl_visual_map_pub(new PointCloudXYZI());
  *sub_pcl_visual_map_pub = *laserCloudFullRes;
  if (1)
  {
    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*sub_pcl_visual_map_pub, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time::now();
    laserCloudmsg.header.frame_id = "camera_init";
    pubSubVisualMap.publish(laserCloudmsg);
  }
}

void LIVMapper::publish_effect_world(const ros::Publisher &pubLaserCloudEffect, const std::vector<PointToPlane> &ptpl_list)
{
  int effect_feat_num = ptpl_list.size();
  PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(effect_feat_num, 1));
  for (int i = 0; i < effect_feat_num; i++)
  {
    laserCloudWorld->points[i].x = ptpl_list[i].point_w_[0];
    laserCloudWorld->points[i].y = ptpl_list[i].point_w_[1];
    laserCloudWorld->points[i].z = ptpl_list[i].point_w_[2];
  }
  sensor_msgs::PointCloud2 laserCloudFullRes3;
  pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
  laserCloudFullRes3.header.stamp = ros::Time::now();
  laserCloudFullRes3.header.frame_id = "camera_init";
  pubLaserCloudEffect.publish(laserCloudFullRes3);
}

template <typename T> void LIVMapper::set_posestamp(T &out)
{
  out.position.x = _state.pos_end(0);
  out.position.y = _state.pos_end(1);
  out.position.z = _state.pos_end(2);
  out.orientation.x = geoQuat.x;
  out.orientation.y = geoQuat.y;
  out.orientation.z = geoQuat.z;
  out.orientation.w = geoQuat.w;
}

void LIVMapper::publish_odometry(const ros::Publisher &pubOdomAftMapped)
{
  odomAftMapped.header.frame_id = "camera_init";
  odomAftMapped.child_frame_id = "aft_mapped";
  odomAftMapped.header.stamp.fromSec(LidarMeasures.last_lio_update_time);
  set_posestamp(odomAftMapped.pose.pose);

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;
  transform.setOrigin(tf::Vector3(_state.pos_end(0), _state.pos_end(1), _state.pos_end(2)));
  q.setW(geoQuat.w);
  q.setX(geoQuat.x);
  q.setY(geoQuat.y);
  q.setZ(geoQuat.z);
  transform.setRotation(q);
  br.sendTransform( tf::StampedTransform(transform, odomAftMapped.header.stamp, "camera_init", "aft_mapped") );
  pubOdomAftMapped.publish(odomAftMapped);
}

void LIVMapper::publish_mavros(const ros::Publisher &mavros_pose_publisher)
{
  msg_body_pose.header.stamp = ros::Time::now();
  msg_body_pose.header.frame_id = "camera_init";
  set_posestamp(msg_body_pose.pose);
  mavros_pose_publisher.publish(msg_body_pose);
}

void LIVMapper::publish_path(const ros::Publisher pubPath)
{
  set_posestamp(msg_body_pose.pose);
  msg_body_pose.header.stamp = ros::Time::now();
  msg_body_pose.header.frame_id = "camera_init";
  path.poses.push_back(msg_body_pose);
  pubPath.publish(path);
}
