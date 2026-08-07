/****************************************************************************************
 *
 * Copyright (c) 2024, Shenyang Institute of Automation, Chinese Academy of Sciences
 *
 * Authors: Yanpeng Jia
 * Contact: jiayanpeng@sia.cn
 *
 ****************************************************************************************/

// PCL specific includes
#include <pcl/common/io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/transforms.h>
#include "pcl_ros/impl/transforms.hpp"

#include <ros/ros.h>
#include "center_pointpillars/centerpoint.h"

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <algorithm>
#include <ros/package.h>

#include <deque>
#include <fstream>
#include <numeric>  // std::accumulate
#include <cmath>    // std::cos, std::sin, std::atan2, std::fabs
#include <stdexcept>
#include <string>
#include <sstream>
#include <iomanip>

#include <mutex>
#include <thread>
#include <condition_variable>
#include <atomic>
#include <cstdint>
#include <limits>

#include <unordered_map>
#include <pcl/filters/filter.h>   // removeNaNFromPointCloud
#include "preprocess.h"

// CenterPoint only consumes XYZ and intensity.  Dynamic removal receives the
// RobotDog PointCloud2 layout, so this temporary PCL representation must match
// its available fields.  Adding unused Velodyne-only time/ring members makes
// pcl::fromROSMsg emit one missing-field warning per input frame.
namespace centerpp_ros {
struct EIGEN_ALIGN16 Point
{
    PCL_ADD_POINT4D;
    float intensity;
    std::uint8_t tag;
    std::uint8_t line;
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace centerpp_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(centerpp_ros::Point,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
    (std::uint8_t, tag, tag)(std::uint8_t, line, line)(double, timestamp, timestamp))

using PointT   = centerpp_ros::Point;
using CloudT   = pcl::PointCloud<PointT>;
using CloudTPtr = CloudT::Ptr;


#define GPU_CHECK(ans)                                                         \
    { GPUAssert((ans), __FILE__, __LINE__); }
inline void GPUAssert(cudaError_t code, const char *file, int line,
                      bool abort = true) {
    if (code != cudaSuccess) {
        fprintf(stderr, "GPUassert: %s %s %d\n", cudaGetErrorString(code), file,
                line);
        if (abort)
            exit(code);
    }
};

// Per-frame CUDA failures must be recoverable by the ordered worker.  Startup
// allocation/device failures may still be fatal, but a copy/synchronize error
// while handling one cloud becomes that cloud's same-frame passthrough.
inline void cudaCheckOrThrow(cudaError_t code, const char *operation)
{
    if (code != cudaSuccess) {
        throw std::runtime_error(std::string(operation) + ": " + cudaGetErrorString(code));
    }
}

std::vector<unsigned char> color;

std::vector<double> avg_centerpoint_time;

static inline double wall_now_ms()
{
    // 用 WallTime：不受 /use_sim_time 影响，适合做性能统计
    return ros::WallTime::now().toSec() * 1000.0; // ms
}

static inline size_t packCloudToPillarsInput(
        const CloudT& cloud,
        float* h_points, size_t max_points,
        float min_range, float max_range)
{
    const float min2 = min_range * min_range;
    const float max2 = max_range * max_range;

    size_t n = 0;
    for (const auto& p : cloud.points) {
        const float r2 = p.x*p.x + p.y*p.y + p.z*p.z;
        if (r2 < min2 || r2 > max2) continue;
        if (n >= max_points) break;

        float* dst = h_points + n * 5;
        dst[0] = p.x;
        dst[1] = p.y;
        dst[2] = p.z;
        dst[3] = 0.0f;
        dst[4] = 0.0f;
        ++n;
    }
    return n;
}


static inline pcl::PointCloud<pcl::PointXYZI> toXYZI(const CloudT& in)
{
    pcl::PointCloud<pcl::PointXYZI> out;
    out.points.reserve(in.points.size());
    for (const auto& p : in.points) {
        pcl::PointXYZI q;
        q.x = p.x; q.y = p.y; q.z = p.z;
        q.intensity = p.intensity;
        out.points.push_back(q);
    }
    out.width = (uint32_t)out.points.size();
    out.height = 1;
    out.is_dense = in.is_dense;
    return out;
}

void GetDeviceInfo()
{
    cudaDeviceProp prop;

    int count = 0;
    cudaGetDeviceCount(&count);
    printf("\nGPU has cuda devices: %d\n", count);
    for (int i = 0; i < count; ++i) {
        cudaGetDeviceProperties(&prop, i);
        printf("----device id: %d info----\n", i);
        printf("  GPU : %s \n", prop.name);
        printf("  Capbility: %d.%d\n", prop.major, prop.minor);
        printf("  Global memory: %luMB\n", prop.totalGlobalMem >> 20);
        printf("  Const memory: %luKB\n", prop.totalConstMem  >> 10);
        printf("  SM in a block: %luKB\n", prop.sharedMemPerBlock >> 10);
        printf("  warp size: %d\n", prop.warpSize);
        printf("  threads in a block: %d\n", prop.maxThreadsPerBlock);
        printf("  block dim: (%d,%d,%d)\n", prop.maxThreadsDim[0], prop.maxThreadsDim[1], prop.maxThreadsDim[2]);
        printf("  grid dim: (%d,%d,%d)\n", prop.maxGridSize[0], prop.maxGridSize[1], prop.maxGridSize[2]);
    }
    printf("\n");
}

void initDevice(int devNum) {
    int dev = devNum;
    cudaDeviceProp deviceProp;

    GPU_CHECK(cudaGetDeviceProperties(&deviceProp, dev));
    printf("Using device %d: %s\n", dev, deviceProp.name);
    GPU_CHECK(cudaSetDevice(dev));
}

namespace cpp {

class Center_PointPillars_ROS {
  public:
    Center_PointPillars_ROS(ros::NodeHandle nh);
    ~Center_PointPillars_ROS();

    void Process();
    void extractBBoxPointcloud(std::vector<Bndbox> filter_BBox,
                               const CloudTPtr& cloud_in,
                               CloudTPtr& cloud_out,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_cluster);

    struct FrameBoxes {
        ros::Time stamp;
        std::vector<Bndbox> boxes;
    };
    std::deque<FrameBoxes> box_hist_;
    struct Pose {
        ros::Time stamp;
        Eigen::Matrix4f T; // odom系位姿
    };
    std::deque<Pose> odom_hist_;

    double coast_time_sec_ = 0.8;     // 允许回放历史框的时间窗口
    int    coast_max_frames_ = 8;     // 最多回放的历史帧数
    double inflate_ratio_ = 0.12;     // 尺寸按比例膨胀
    double inflate_m_ = 0.25;         // 尺寸再加固定冗余（米）
    double merge_center_thresh_ = 1.0; // 历史框与当前框中心小于该距离则认为重复
    bool normalize_world_cloud_to_local_ = true;
    double max_input_lag_sec_ = 0.20;

    // Detection-only mode is deliberately true by default:
    // the node may publish boxes and diagnostic clouds, but it must never alter
    // the cloud that a future SLAM subscriber could consume.
    bool detection_only_ = true;
    // Dynamic removal never changes pointcloud_static; it creates diagnostic
    // candidates on dedicated topics after motion and ground checks succeed.
    bool dynamic_removal_enable_ = false;
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_pointcloud_;
    ros::Subscriber sub_odom_;
    ros::Publisher pub_pointcloud_static_;
    ros::Publisher pub_pointcloud_raw_;
    ros::Publisher pub_bbox_;
    ros::Publisher pub_dynamic_bbox_;
    ros::Publisher pub_pointcloud_cluster_;
    ros::Publisher pub_filtered_points_;
    ros::Publisher pub_static_candidate_points_;
    ros::Publisher pub_protected_ground_points_;
    // One small text message per semantic candidate.  This is intentionally
    // separate from point clouds so offline acceptance can inspect the exact
    // motion/ground decision without inferring it from an aggregated PCD.
    ros::Publisher pub_ground_diagnostics_;
    // One record for every input frame.  This is deliberately separate from
    // ground_diagnostics, which is per semantic candidate rather than per
    // LiDAR frame.
    ros::Publisher pub_frame_diagnostics_;
    ros::Publisher pub_text_vel_;
    ros::Publisher pub_center_points_;

    cudaEvent_t start_, stop_;
    cudaStream_t stream_ = NULL;

    Params params;

    std::string Model_File_Dir_;
    std::string odom_frame_;
    std::string child_frame_;
    bool use_input_frame_ = false;

    CloudTPtr original_scan_;

    double MINIMUM_RANGE;
    double MAXMUM_RANGE;
    bool crop_use_;
    double crop_size_;

    bool vf_use_;
    double vf_res_;

    pcl::CropBox<PointT> crop;
    pcl::VoxelGrid<pcl::PointXYZI> vf;
    visualization_msgs::MarkerArray center_points_array;

    pcl::PointCloud<pcl::PointXYZI>::Ptr global_static_map_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr  global_dynamic_mask_;


    std::string save_dir_;
    bool save_on_shutdown_;

    bool verbose = true;

    enum class MotionState { UNKNOWN, STATIC, MOVING };
    struct MotionEvidence {
        int observations = 0;
        double duration_sec = 0.0;
        double speed_mps = std::numeric_limits<double>::quiet_NaN();
    };
    struct TrackObservation {
        ros::Time stamp;
        Eigen::Vector3f center_world;
    };
    struct ObjectTrack {
        uint64_t id = 0;
        int class_id = -1;
        ros::Time last_stamp;
        std::deque<TrackObservation> observations;
    };
    struct BoxParam {
        float cx, cy, cz;
        float c, s;     // cos(yaw), sin(yaw)
        float hx, hy, hz;
        float r2;       // (hx,hy) 外接圆半径^2，用于快速门限
    };
    struct GroundPlane {
        enum class Status {
            NOT_EVALUATED,
            INSUFFICIENT_RING,
            RANSAC_FAILED,
            TILT_REJECTED,
            HEIGHT_REJECTED,
            VALID,
        };
        Eigen::Vector3f normal = Eigen::Vector3f::UnitZ();
        float d = 0.0f;
        int inliers = 0;
        float z_at_box_center = std::numeric_limits<float>::quiet_NaN();
        float box_bottom_z = std::numeric_limits<float>::quiet_NaN();
        float height_above_box_bottom = std::numeric_limits<float>::quiet_NaN();
        Status status = Status::NOT_EVALUATED;
        bool valid = false;
    };
    struct SafeMovingBox {
        BoxParam box;
        GroundPlane ground;
        // A valid plane must still protect only the lower part of the object
        // box.  This guard prevents a misfitted elevated plane from retaining
        // a pedestrian torso as alleged "ground".
        float max_protected_z = std::numeric_limits<float>::infinity();
    };
    struct CandidateDiagnostic {
        Bndbox box;
        MotionState motion = MotionState::UNKNOWN;
        MotionEvidence evidence;
        GroundPlane ground;
        bool accepted_for_removal = false;
    };

    std::vector<ObjectTrack> tracks_;
    uint64_t next_track_id_ = 1;
    double motion_window_sec_ = 1.5;
    double track_timeout_sec_ = 1.0;
    double association_distance_m_ = 2.0;
    double odom_max_time_delta_sec_ = 0.10;
    double odom_interpolation_max_gap_sec_ = 0.60;
    double odom_wait_wall_sec_ = 0.50;
    int motion_min_observations_ = 6;
    double motion_min_duration_sec_ = 0.6;
    double static_speed_max_mps_ = 0.35;
    double moving_speed_min_mps_ = 0.80;
    double ground_search_margin_m_ = 1.0;
    int ground_min_inliers_ = 50;
    double ground_ransac_distance_m_ = 0.12;
    double ground_keep_distance_m_ = 0.18;
    // A horizontal plane in the ring is accepted as road only when it is near
    // the detector box bottom.  It rejects roofs/hoods/steps that can also be
    // locally horizontal but are not the road underneath the moving object.
    double ground_max_height_above_box_bottom_m_ = 0.25;
    double ground_max_height_below_box_bottom_m_ = 0.50;
    double ground_protect_max_height_above_box_bottom_m_ = 0.35;
    // /aft_mapped_to_init is the IMU/body pose.  Candidate centers arrive in
    // the LiDAR frame, so motion association must apply this calibrated lever
    // arm before transforming them into camera_init.
    Eigen::Matrix3f lidar_to_imu_R_ = Eigen::Matrix3f::Identity();
    Eigen::Vector3f lidar_to_imu_t_ = Eigen::Vector3f::Zero();

    std::unique_ptr<CenterPoint> center_pointpillars_ptr_;

    void PointCloud_Callback(const sensor_msgs::PointCloud2ConstPtr& msg);
    void Odometry_Callback(const nav_msgs::OdometryPtr &odom);
    void publishCloud(std_msgs::Header header, const CloudTPtr& in_cloud_to_publish_ptr);
    void publishRawInput(const sensor_msgs::PointCloud2& cloud_msg);
    void publishPassthrough(const sensor_msgs::PointCloud2& cloud_msg);
    void publishObjectBoundingBox(std_msgs::Header in_msg_header, std::vector<Bndbox> filter_BBox);
    void publishDynamicBoundingBox(std_msgs::Header in_msg_header, std::vector<Bndbox> dynamic_BBox);
    void publishClusterCloud(std_msgs::Header header, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, std::vector<pcl::PointIndices> cluster_indices);

    void preprocessPoints(const CloudTPtr& cloud_in, float th1, float th2,
                          const std_msgs::Header &header);
    void removeClosedPointCloud(const CloudT &cloud_in, CloudT &cloud_out, float th1, float th2);
    void publishClusterRaw(std_msgs::Header header,const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in);
    void publishSelectedRecords(const sensor_msgs::PointCloud2& input,
                                const std::vector<uint8_t>& selected,
                                ros::Publisher& publisher) const;
    void publishDiagnosticCloud(const std_msgs::Header& header,
                                const CloudTPtr& cloud,
                                ros::Publisher& publisher) const;
    std::string outputFrame(const std_msgs::Header &header) const;
    // --- 历史框并集相关（私有） ---
    bool nearestPose_(ros::Time t, Pose& out);
    bool relativeT_(ros::Time ta, ros::Time tb, Eigen::Matrix4f& T_ba);
    Bndbox transformBox_(const Bndbox& b, const Eigen::Matrix4f& T);
    void augmentWithHistory_(std::vector<Bndbox>& boxes_now, ros::Time t_now);

    struct QueuedFrame {
        uint64_t seq = 0;
        sensor_msgs::PointCloud2ConstPtr msg;
        ros::WallTime received_wall;
        bool bypass = false;
        std::string bypass_reason;
        size_t queue_depth_on_receive = 0;
    };

    struct FrameResult {
        std::string outcome = "passthrough";
        std::string reason = "unknown";
        size_t removed_points = 0;
        double inference_ms = 0.0;
        bool has_odom = false;
    };

    // There is exactly one publishing worker.  It consumes this FIFO in
    // sequence order, so a later bypass can never overtake an earlier
    // inference result.  cloud_q_max_ is an inference-admission limit, not a
    // permission to drop PointCloud2 messages: excess frames are enqueued as
    // ordered passthrough results.
    std::mutex mtx_cloud_;
    std::condition_variable cv_cloud_;
    std::deque<QueuedFrame> cloud_q_;
    size_t cloud_q_max_ = 3;
    bool exit_worker_ = false;
    std::thread worker_;
    std::atomic<bool> worker_active_{false};

    int input_subscriber_queue_size_ = 8;
    int output_publisher_queue_size_ = 8;
    double inference_budget_ms_ = -1.0;
    int test_inject_delay_ms_ = 0;
    int test_inject_exception_seq_ = -1;
    bool test_force_odom_unavailable_ = false;
    bool test_force_empty_detections_ = false;

    // --- odom历史互斥（odom回调与worker会并发访问） ---
    std::mutex mtx_odom_;

    void processingLoop_();
    // quality: nearest time error, or bracketing odom interval.
    bool poseAt_(ros::Time t, Eigen::Matrix4f& T_odom_child,
                 double* quality_sec = nullptr, bool* interpolated = nullptr);
    FrameResult processOneCloud_(const sensor_msgs::PointCloud2ConstPtr& msg,
                                 const Eigen::Matrix4f& T_odom_child, bool has_odom,
                                 uint64_t seq);
    std::vector<BoxParam> makeBoxParams_(const std::vector<Bndbox>& boxes,
                                         bool padded) const;
    static bool pointInBox_(const PointT& point, const BoxParam& box);
    MotionState updateTrack_(const Bndbox& box, const ros::Time& stamp,
                             const Eigen::Matrix4f& T_odom_child, bool has_odom,
                             MotionEvidence* evidence = nullptr);
    bool estimateGroundPlane_(const Bndbox& box, const CloudTPtr& cloud,
                              GroundPlane& plane) const;
    static const char* motionStateName_(MotionState state);
    static const char* groundStatusName_(GroundPlane::Status status);
    void publishGroundDiagnostics_(const std_msgs::Header& header,
                                   const std::vector<CandidateDiagnostic>& diagnostics);
    size_t runDynamicRemoval_(const sensor_msgs::PointCloud2ConstPtr& msg,
                       const CloudTPtr& cloud, const std::vector<Bndbox>& candidates,
                       const Eigen::Matrix4f& T_odom_child, bool has_odom);
    void publishBypassFrame_(const sensor_msgs::PointCloud2ConstPtr& msg);
    void publishFrameDiagnostics_(const QueuedFrame& frame, const FrameResult& result,
                                  const ros::WallTime& processing_started,
                                  const ros::WallTime& published);

    float* d_points_ = nullptr;
    float* h_points_ = nullptr;  // pinned
    size_t max_points_ = MAX_POINTS_NUM;

    // ---- timing stats ----
    struct Stat {
        double ema_ms = 0.0;
        double last_ms = 0.0;
        bool inited = false;
        void add(double ms, double alpha=0.1) {
            last_ms = ms;
            if (!inited) { ema_ms = ms; inited = true; }
            else ema_ms = (1.0-alpha)*ema_ms + alpha*ms;
        }
    };

    std::atomic<uint64_t> recv_cnt_{0};
    std::atomic<uint64_t> proc_cnt_{0};
    std::atomic<uint64_t> dropq_cnt_{0};
    std::atomic<uint64_t> queue_bypass_cnt_{0};
    std::atomic<uint64_t> exception_cnt_{0};
    std::atomic<uint64_t> timeout_cnt_{0};
    std::atomic<uint64_t> max_queue_depth_{0};
    std::atomic<uint64_t> next_input_seq_{0};

    std::atomic<uint64_t> bypass_cnt_{0};  // lag 太大走 bypass 分支的帧数
    std::atomic<uint64_t> empty_cnt_{0};   // boxes_to_remove.empty() 早退的帧数

    std::atomic<int>      q_size_{0};      // 当前 cloud_q_ 队列长度（用于打印）

    Stat t_fromros_, t_norm_, t_pack_, t_h2d_, t_infer_;
    Stat t_hist_, t_extract_, t_pre_, t_pub_, t_total_;

    ros::WallTime last_report_wall_ = ros::WallTime(0);
    ros::Time last_proc_stamp_ = ros::Time(0);
};

Center_PointPillars_ROS::Center_PointPillars_ROS(ros::NodeHandle nh) : nh_(nh) {
    const std::string pkg_trlo = ros::package::getPath("fast_livo");
    std::string model_dir   = pkg_trlo + "/model/center_pointpillars/";
    // nh_ is private ("~"), matching the <param> placement in the launch
    // file.  This avoids silently falling back to the package directory when
    // a user supplies a model_dir launch argument.
    nh_.param<std::string>("Model_File_Dir", this->Model_File_Dir_, model_dir);
    if (!this->Model_File_Dir_.empty() && this->Model_File_Dir_.back() != '/') {
        this->Model_File_Dir_ += '/';
    }
    ros::param::param<std::string>("~center_pp/frame/odom_frame", this->odom_frame_, "robot/odom");
    ros::param::param<std::string>("~center_pp/frame/child_frame", this->child_frame_, "robot/base_link");
    ros::param::param<bool>("~center_pp/frame/use_input_frame", this->use_input_frame_, false);
    ros::param::param<double>("~center_pp/runtime/max_input_lag_sec", this->max_input_lag_sec_, 0.20);
    ros::param::param<int>("~center_pp/runtime/input_subscriber_queue_size",
                           this->input_subscriber_queue_size_, 8);
    ros::param::param<int>("~center_pp/runtime/output_publisher_queue_size",
                           this->output_publisher_queue_size_, 8);
    int inference_queue_capacity = 3;
    ros::param::param<int>("~center_pp/runtime/inference_queue_capacity",
                           inference_queue_capacity, 3);
    this->cloud_q_max_ = static_cast<size_t>(std::max(1, inference_queue_capacity));
    ros::param::param<double>("~center_pp/runtime/inference_budget_ms",
                              this->inference_budget_ms_, -1.0);
    ros::param::param<int>("~center_pp/runtime/test_inject_delay_ms",
                           this->test_inject_delay_ms_, 0);
    ros::param::param<int>("~center_pp/runtime/test_inject_exception_seq",
                           this->test_inject_exception_seq_, -1);
    ros::param::param<bool>("~center_pp/runtime/test_force_odom_unavailable",
                            this->test_force_odom_unavailable_, false);
    ros::param::param<bool>("~center_pp/runtime/test_force_empty_detections",
                            this->test_force_empty_detections_, false);
    if (input_subscriber_queue_size_ < 1) input_subscriber_queue_size_ = 1;
    if (output_publisher_queue_size_ < 1) output_publisher_queue_size_ = 1;
    if (test_inject_delay_ms_ < 0) test_inject_delay_ms_ = 0;
    ROS_INFO("[centerpp][runtime] subscriber_queue=%d publisher_queue=%d inference_capacity=%zu budget_ms=%.2f "
             "delay_ms=%d exception_seq=%d force_odom_unavailable=%s force_empty=%s",
             input_subscriber_queue_size_, output_publisher_queue_size_, cloud_q_max_, inference_budget_ms_,
             test_inject_delay_ms_, test_inject_exception_seq_,
             test_force_odom_unavailable_ ? "true" : "false",
             test_force_empty_detections_ ? "true" : "false");

    ros::param::param<double>("~center_pp/preprocessing/threshold/MINIMUM_RANGE", this->MINIMUM_RANGE, 0.5);
    ros::param::param<double>("~center_pp/preprocessing/threshold/MAXMUM_RANGE", this->MAXMUM_RANGE, 80);

    // Crop Box Filter
    ros::param::param<bool>("~center_pp/preprocessing/cropBoxFilter/use", this->crop_use_, false);
    ros::param::param<double>("~center_pp/preprocessing/cropBoxFilter/size", this->crop_size_, 1.0);

    // Voxel Grid Filter
    ros::param::param<bool>("~center_pp/preprocessing/voxelFilter/use", this->vf_use_, true);
    ros::param::param<double>("~center_pp/preprocessing/voxelFilter/res", this->vf_res_, 0.05);

    ros::param::param<double>("~center_pp/filter_history/coast_time_sec", this->coast_time_sec_, 0.8);
    ros::param::param<int>   ("~center_pp/filter_history/coast_max_frames", this->coast_max_frames_, 8);
    ros::param::param<double>("~center_pp/filter_history/inflate_ratio", this->inflate_ratio_, 0.12);
    ros::param::param<double>("~center_pp/filter_history/inflate_m", this->inflate_m_, 0.25);
    ros::param::param<double>("~center_pp/filter_history/merge_center_thresh", this->merge_center_thresh_, 0.5);
    ros::param::param<bool>("~center_pp/normalize_world_cloud_to_local",
                            this->normalize_world_cloud_to_local_, true);
    ros::param::param<bool>("~center_pp/mode/detect_only", this->detection_only_, true);
    ros::param::param<bool>("~center_pp/dynamic_removal/enable", this->dynamic_removal_enable_, false);
    ros::param::param<double>("~center_pp/dynamic_removal/motion_window_sec", this->motion_window_sec_, 1.5);
    ros::param::param<double>("~center_pp/dynamic_removal/track_timeout_sec", this->track_timeout_sec_, 1.0);
    ros::param::param<double>("~center_pp/dynamic_removal/association_distance_m", this->association_distance_m_, 2.0);
    ros::param::param<double>("~center_pp/dynamic_removal/odom_max_time_delta_sec", this->odom_max_time_delta_sec_, 0.10);
    ros::param::param<double>("~center_pp/dynamic_removal/odom_interpolation_max_gap_sec",
                              this->odom_interpolation_max_gap_sec_, 0.60);
    ros::param::param<double>("~center_pp/dynamic_removal/odom_wait_wall_sec", this->odom_wait_wall_sec_, 0.50);
    ros::param::param<int>("~center_pp/dynamic_removal/min_observations", this->motion_min_observations_, 6);
    ros::param::param<double>("~center_pp/dynamic_removal/min_duration_sec", this->motion_min_duration_sec_, 0.6);
    ros::param::param<double>("~center_pp/dynamic_removal/static_speed_max_mps", this->static_speed_max_mps_, 0.35);
    ros::param::param<double>("~center_pp/dynamic_removal/moving_speed_min_mps", this->moving_speed_min_mps_, 0.80);
    ros::param::param<double>("~center_pp/dynamic_removal/ground_search_margin_m", this->ground_search_margin_m_, 1.0);
    ros::param::param<int>("~center_pp/dynamic_removal/ground_min_inliers", this->ground_min_inliers_, 50);
    ros::param::param<double>("~center_pp/dynamic_removal/ground_ransac_distance_m", this->ground_ransac_distance_m_, 0.12);
    ros::param::param<double>("~center_pp/dynamic_removal/ground_keep_distance_m", this->ground_keep_distance_m_, 0.18);
    ros::param::param<double>("~center_pp/dynamic_removal/ground_max_height_above_box_bottom_m",
                              this->ground_max_height_above_box_bottom_m_, 0.25);
    ros::param::param<double>("~center_pp/dynamic_removal/ground_max_height_below_box_bottom_m",
                              this->ground_max_height_below_box_bottom_m_, 0.50);
    ros::param::param<double>("~center_pp/dynamic_removal/ground_protect_max_height_above_box_bottom_m",
                              this->ground_protect_max_height_above_box_bottom_m_, 0.35);

    if (this->dynamic_removal_enable_) {
        std::vector<double> extrinsic_t;
        std::vector<double> extrinsic_r;
        // Keep this calibration private to the dynamic-removal node.  Loading the
        // full RobotDog YAML here would overwrite /common/img_en in a running
        // mapper and make the evaluation launch unexpectedly re-enable VIO.
        if (nh_.getParam("center_pp/dynamic_removal/lidar_to_imu_T", extrinsic_t) &&
            nh_.getParam("center_pp/dynamic_removal/lidar_to_imu_R", extrinsic_r) &&
            extrinsic_t.size() == 3 && extrinsic_r.size() == 9) {
            lidar_to_imu_t_ = Eigen::Vector3f(static_cast<float>(extrinsic_t[0]),
                                               static_cast<float>(extrinsic_t[1]),
                                               static_cast<float>(extrinsic_t[2]));
            for (int row = 0; row < 3; ++row) {
                for (int col = 0; col < 3; ++col) {
                    lidar_to_imu_R_(row, col) = static_cast<float>(extrinsic_r[row * 3 + col]);
                }
            }
        } else {
            ROS_WARN("[centerpp][dynamic_removal] private lidar_to_imu_T/R is unavailable; "
                     "using identity LiDAR-to-IMU transform. This is safe only for evaluation, not final tuning.");
        }
    }
    ros::param::param<std::string>("~center_pp/save_dir", this->save_dir_, "/tmp");
    ros::param::param<bool>("~center_pp/save_on_shutdown", this->save_on_shutdown_, true);

    this->global_static_map_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    this->global_dynamic_mask_.reset(new pcl::PointCloud<pcl::PointXYZ>());



    if (!std::ifstream(this->Model_File_Dir_ + "rpn_centerhead_sim.plan").good() ||
        !std::ifstream(this->Model_File_Dir_ + "centerpoint.scn.onnx").good()) {
        throw std::runtime_error(
            "CenterPoint model files are missing. Expected rpn_centerhead_sim.plan and "
            "centerpoint.scn.onnx under: " + this->Model_File_Dir_);
    }

    if (this->dynamic_removal_enable_) {
        ROS_INFO("[centerpp] Dynamic-removal safety mode: semantic candidates require odometry-based "
                 "motion evidence, and a locally fitted ground plane protects road points. "
                 "static_points remains an exact input passthrough.");
    } else if (this->detection_only_) {
        ROS_INFO("[centerpp] Detection-only mode: static_points is an exact input passthrough; "
                 "no point removal, voxel filtering, CropBox filtering, history boxes, or global-map accumulation.");
    }

    checkCudaErrors(cudaEventCreate(&this->start_));
    checkCudaErrors(cudaEventCreate(&this->stop_));
    GPU_CHECK(cudaStreamCreate(&this->stream_));
    this->center_pointpillars_ptr_.reset(new CenterPoint(this->Model_File_Dir_, this->verbose)); // 外部定义调用不了cuda函数
    this->original_scan_.reset(new CloudT());
    this->crop.setNegative(true);
    this->crop.setMin(Eigen::Vector4f(-this->crop_size_, -this->crop_size_, -this->crop_size_, 1.0));
    this->crop.setMax(Eigen::Vector4f(this->crop_size_, this->crop_size_, this->crop_size_, 1.0));

    center_pointpillars_ptr_->prepare();
    setlocale(LC_ALL,"");

    worker_ = std::thread(&Center_PointPillars_ROS::processingLoop_, this);

    cudaMalloc(&d_points_, max_points_ * 5 * sizeof(float));
    cudaHostAlloc(&h_points_, max_points_ * 5 * sizeof(float), cudaHostAllocDefault); // pinned
}

Center_PointPillars_ROS::~Center_PointPillars_ROS(){
    {
        std::lock_guard<std::mutex> lk(mtx_cloud_);
        exit_worker_ = true;
    }
    cv_cloud_.notify_all();
    if (worker_.joinable()) worker_.join();

    checkCudaErrors(cudaEventDestroy(this->start_));
    checkCudaErrors(cudaEventDestroy(this->stop_));
    checkCudaErrors(cudaStreamDestroy(this->stream_));

    if (d_points_) cudaFree(d_points_);
    if (h_points_) cudaFreeHost(h_points_);

    if (save_on_shutdown_ && !detection_only_) {
        // ---- 1. 直接保存全局静态地图：PointXYZI ----
        if (!global_static_map_->empty()) {
            global_static_map_->width  = global_static_map_->points.size();
            global_static_map_->height = 1;
            global_static_map_->is_dense = true;

            std::string static_path = save_dir_ + "/all_static_point_cloud_map.pcd";
            pcl::PCDWriter pcd_writer;
            pcd_writer.writeBinary(static_path, *global_static_map_); // pcl::io::savePCDFileASCII(all_points_dir, *pcl_wait_save);
//            PointCloudXYZI().swap(*global_static_map_);
//            // 二进制保存，体积更小；如果你想 ASCII，就换成 savePCDFileASCII
//            pcl::io::savePCDFileBinary(static_path, *global_static_map_);
            ROS_INFO_STREAM("[centerpp] Saved global static map to " << static_path);
        }

        // ---- 2. 直接保存全局动态 mask：PointXYZ ----
        if (!global_dynamic_mask_->empty()) {
            global_dynamic_mask_->width  = global_dynamic_mask_->points.size();
            global_dynamic_mask_->height = 1;
            global_dynamic_mask_->is_dense = true;

            std::string dyn_path = save_dir_ + "/all_dynamic_mask.pcd";
            pcl::PCDWriter pcd_writer;
            pcd_writer.writeBinary(dyn_path , *global_dynamic_mask_); // pcl::io::savePCDFileASCII(all_points_dir, *pcl_wait_save);
//            PointCloudXYZ().swap(*global_dynamic_mask_);
//            pcl::io::savePCDFileBinary(dyn_path, *global_dynamic_mask_);
            ROS_INFO_STREAM("[centerpp] Saved global dynamic mask to " << dyn_path);
        }
    }
}

void Center_PointPillars_ROS::Process() {
    std::cout << "Ready to receive point cloud topic!" << std::endl;
    this->sub_odom_ = nh_.subscribe ("odom", 160, &Center_PointPillars_ROS::Odometry_Callback, this, ros::TransportHints().tcpNoDelay());
    this->sub_pointcloud_ = nh_.subscribe("pointcloud", input_subscriber_queue_size_,
                                           &Center_PointPillars_ROS::PointCloud_Callback, this,
                                           ros::TransportHints().tcpNoDelay());
//    this->sub_odom_ = nh_.subscribe ("odom", 160, &Center_PointPillars_ROS::Odometry_Callback, this);
//    this->pub_pointcloud_static_ = nh_.advertise<sensor_msgs::PointCloud2>("pointcloud_static", 10);
//    this->pub_pointcloud_raw_ = nh_.advertise<sensor_msgs::PointCloud2>("pointcloud_raw", 10);
    this->pub_pointcloud_static_ = nh_.advertise<sensor_msgs::PointCloud2>("pointcloud_static", output_publisher_queue_size_);
    this->pub_pointcloud_raw_ = nh_.advertise<sensor_msgs::PointCloud2>("pointcloud_raw", output_publisher_queue_size_);
    this->pub_bbox_ = nh_.advertise<jsk_recognition_msgs::BoundingBoxArray>("box", 10, true);
    this->pub_dynamic_bbox_ = nh_.advertise<jsk_recognition_msgs::BoundingBoxArray>("dynamic_box", 10, true);
    this->pub_pointcloud_cluster_ = nh_.advertise<sensor_msgs::PointCloud2>("pointcloud_cluster", output_publisher_queue_size_);
    this->pub_filtered_points_ = nh_.advertise<sensor_msgs::PointCloud2>("filtered_points", output_publisher_queue_size_);
    this->pub_static_candidate_points_ = nh_.advertise<sensor_msgs::PointCloud2>("static_candidate_points", output_publisher_queue_size_);
    this->pub_protected_ground_points_ = nh_.advertise<sensor_msgs::PointCloud2>("protected_ground_points", output_publisher_queue_size_);
    this->pub_ground_diagnostics_ = nh_.advertise<std_msgs::String>("ground_diagnostics", 100);
    this->pub_frame_diagnostics_ = nh_.advertise<std_msgs::String>("frame_diagnostics", 100);
    //    this->pub_pointcloud_cluster_ = nh_.advertise<sensor_msgs::PointCloud2>("pointcloud_cluster", 10);
    this->pub_text_vel_ = nh_.advertise<visualization_msgs::MarkerArray>("centerpoint_vel", 10);
    this->pub_center_points_ = nh_.advertise<visualization_msgs::MarkerArray> ("center_markers", 10);
    ros::spin();
}

void Center_PointPillars_ROS::PointCloud_Callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    recv_cnt_.fetch_add(1, std::memory_order_relaxed);
    QueuedFrame frame;
    frame.seq = next_input_seq_.fetch_add(1, std::memory_order_relaxed);
    frame.msg = msg;
    frame.received_wall = ros::WallTime::now();

    {
        std::lock_guard<std::mutex> lk(mtx_cloud_);
        // Do not erase a queued point cloud.  Once the bounded inference
        // admission window is full, this frame becomes an ordered raw
        // passthrough result and remains behind every earlier sequence.
        const size_t active = worker_active_.load(std::memory_order_relaxed) ? 1 : 0;
        const size_t depth = cloud_q_.size() + active;
        frame.queue_depth_on_receive = depth;
        if (depth >= cloud_q_max_) {
            frame.bypass = true;
            frame.bypass_reason = "queue_busy";
            queue_bypass_cnt_.fetch_add(1, std::memory_order_relaxed);
        }
        cloud_q_.push_back(frame);
        const uint64_t new_depth = static_cast<uint64_t>(cloud_q_.size() + active);
        uint64_t observed = max_queue_depth_.load(std::memory_order_relaxed);
        while (new_depth > observed &&
               !max_queue_depth_.compare_exchange_weak(observed, new_depth,
                                                        std::memory_order_relaxed)) {}
        q_size_.store(static_cast<int>(cloud_q_.size()), std::memory_order_relaxed);
    }
    cv_cloud_.notify_one();
}

void Center_PointPillars_ROS::Odometry_Callback(const nav_msgs::OdometryPtr &odom)
{
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    Eigen::Quaternionf q(odom->pose.pose.orientation.w,
                         odom->pose.pose.orientation.x,
                         odom->pose.pose.orientation.y,
                         odom->pose.pose.orientation.z);
    T.block<3,3>(0,0) = q.toRotationMatrix();
    T.block<3,1>(0,3) = Eigen::Vector3f(odom->pose.pose.position.x,
                                        odom->pose.pose.position.y,
                                        odom->pose.pose.position.z);

    {
        std::lock_guard<std::mutex> lk(mtx_odom_);
        odom_hist_.push_back({odom->header.stamp, T});
        while (!odom_hist_.empty() &&
               (odom_hist_.back().stamp - odom_hist_.front().stamp).toSec() > 5.0) {
            odom_hist_.pop_front();
        }
    }
}

void Center_PointPillars_ROS::processingLoop_()
{
    while (ros::ok()) {
        QueuedFrame frame;
        {
            std::unique_lock<std::mutex> lk(mtx_cloud_);
            cv_cloud_.wait(lk, [&]{ return exit_worker_ || !cloud_q_.empty(); });
            if (exit_worker_) break;

            frame = cloud_q_.front();
            cloud_q_.pop_front();
            worker_active_.store(true, std::memory_order_relaxed);
            q_size_.store((int)cloud_q_.size(), std::memory_order_relaxed);
        }

        const sensor_msgs::PointCloud2ConstPtr& msg = frame.msg;
        const ros::WallTime processing_started = ros::WallTime::now();

        // “弹出=开始处理” 计数（不受 early return 影响）
        proc_cnt_.fetch_add(1, std::memory_order_relaxed);

        // Publishing occurs only in this FIFO worker.  In particular, a
        // queue_busy bypass never publishes directly from the subscriber
        // callback and therefore cannot overtake an earlier frame.
        publishRawInput(*msg);
        FrameResult result;
        result.has_odom = false;
        try {
            if (frame.bypass) {
                bypass_cnt_.fetch_add(1, std::memory_order_relaxed);
                result.outcome = "passthrough";
                result.reason = frame.bypass_reason;
                publishBypassFrame_(msg);
            } else {
                Eigen::Matrix4f T_odom_child = Eigen::Matrix4f::Identity();
                double odom_quality_sec = std::numeric_limits<double>::infinity();
                bool odom_interpolated = false;
                bool has_odom = poseAt_(msg->header.stamp, T_odom_child,
                                        &odom_quality_sec, &odom_interpolated);
                if (dynamic_removal_enable_) {
                    const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(odom_wait_wall_sec_);
                    while (ros::ok()) {
                        const bool quality_ok = has_odom &&
                            (odom_interpolated
                                ? odom_quality_sec <= odom_interpolation_max_gap_sec_
                                : odom_quality_sec <= odom_max_time_delta_sec_);
                        if (quality_ok || ros::WallTime::now() >= deadline) break;
                        ros::WallDuration(0.005).sleep();
                        odom_quality_sec = std::numeric_limits<double>::infinity();
                        odom_interpolated = false;
                        has_odom = poseAt_(msg->header.stamp, T_odom_child,
                                           &odom_quality_sec, &odom_interpolated);
                    }
                    const bool quality_ok = has_odom &&
                        (odom_interpolated
                            ? odom_quality_sec <= odom_interpolation_max_gap_sec_
                            : odom_quality_sec <= odom_max_time_delta_sec_);
                    if (!quality_ok || test_force_odom_unavailable_) has_odom = false;
                }
                result = processOneCloud_(msg, T_odom_child, has_odom, frame.seq);
            }
        } catch (const std::exception& error) {
            exception_cnt_.fetch_add(1, std::memory_order_relaxed);
            bypass_cnt_.fetch_add(1, std::memory_order_relaxed);
            result.outcome = "passthrough";
            result.reason = "exception";
            result.has_odom = false;
            ROS_ERROR("[centerpp][runtime] seq=%lu exception: %s; publishing same-frame passthrough.",
                      static_cast<unsigned long>(frame.seq), error.what());
            publishBypassFrame_(msg);
        } catch (...) {
            exception_cnt_.fetch_add(1, std::memory_order_relaxed);
            bypass_cnt_.fetch_add(1, std::memory_order_relaxed);
            result.outcome = "passthrough";
            result.reason = "unknown_exception";
            result.has_odom = false;
            ROS_ERROR("[centerpp][runtime] seq=%lu unknown exception; publishing same-frame passthrough.",
                      static_cast<unsigned long>(frame.seq));
            publishBypassFrame_(msg);
        }

        const ros::WallTime published = ros::WallTime::now();
        publishFrameDiagnostics_(frame, result, processing_started, published);
        worker_active_.store(false, std::memory_order_relaxed);
        cv_cloud_.notify_one();
    }
}

bool Center_PointPillars_ROS::poseAt_(ros::Time t, Eigen::Matrix4f& T_odom_child,
                                      double* quality_sec, bool* interpolated)
{
    std::lock_guard<std::mutex> lk(mtx_odom_);
    if (odom_hist_.empty()) return false;

    if (interpolated) *interpolated = false;

    size_t best = 0;
    double bestdt = 1e9;
    for (size_t i = 0; i < odom_hist_.size(); ++i) {
        double dt = std::fabs((odom_hist_[i].stamp - t).toSec());
        if (dt < bestdt) { bestdt = dt; best = i; }
    }
    // Prefer exact/near-exact odometry.  This retains the original fast path
    // when the mapper publishes once per cloud.
    if (bestdt <= 1e-4) {
        T_odom_child = odom_hist_[best].T;
        if (quality_sec) *quality_sec = bestdt;
        return true;
    }

    // For a slower mapper, linearly interpolate translation and slerp rotation
    // only when the requested timestamp is bracketed by two odometry samples.
    // The caller rejects overly wide brackets, so sparse odometry can never
    // silently become authority for deleting a point.
    for (size_t index = 1; index < odom_hist_.size(); ++index) {
        const Pose& before = odom_hist_[index - 1];
        const Pose& after = odom_hist_[index];
        if (before.stamp > t || after.stamp < t) continue;
        const double interval = (after.stamp - before.stamp).toSec();
        if (interval <= 1e-6) continue;
        const double alpha = (t - before.stamp).toSec() / interval;
        if (alpha < 0.0 || alpha > 1.0) continue;

        const Eigen::Quaternionf q_before(before.T.block<3,3>(0, 0));
        const Eigen::Quaternionf q_after(after.T.block<3,3>(0, 0));
        const Eigen::Quaternionf q = q_before.slerp(static_cast<float>(alpha), q_after).normalized();
        T_odom_child = Eigen::Matrix4f::Identity();
        T_odom_child.block<3,3>(0, 0) = q.toRotationMatrix();
        T_odom_child.block<3,1>(0, 3) =
                (1.0f - static_cast<float>(alpha)) * before.T.block<3,1>(0, 3) +
                static_cast<float>(alpha) * after.T.block<3,1>(0, 3);
        if (quality_sec) *quality_sec = interval;
        if (interpolated) *interpolated = true;
        return true;
    }

    T_odom_child = odom_hist_[best].T;
    if (quality_sec) *quality_sec = bestdt;
    return true;
}

Center_PointPillars_ROS::FrameResult Center_PointPillars_ROS::processOneCloud_(
        const sensor_msgs::PointCloud2ConstPtr& msg, const Eigen::Matrix4f& T_odom_child,
        bool has_odom, uint64_t seq)
{
    FrameResult result;
    result.has_odom = has_odom;
    double T_total0 = wall_now_ms();

    // 如果你在用 /use_sim_time，这里 ros::Time::now() 是仿真时间，同样成立
    const double lag = (ros::Time::now() - msg->header.stamp).toSec();

    if (max_input_lag_sec_ >= 0.0 && lag > max_input_lag_sec_) {
        bypass_cnt_.fetch_add(1, std::memory_order_relaxed);
        result.reason = "input_lag";
        publishBypassFrame_(msg);

        ROS_WARN_THROTTLE(1.0, "[centerpp] lag=%.3f > %.3f, bypass centerpoint for sync",
                          lag, max_input_lag_sec_);
        return result;
    }

    double t0 = wall_now_ms();
    CloudTPtr in_cloud_ptr(new CloudT());
    pcl::fromROSMsg(*msg, *in_cloud_ptr);
    t_fromros_.add(wall_now_ms() - t0);

    double tN0 = wall_now_ms();
    // ---- 如果输入是世界系（camera_init/map/world），先拉回到本地(child_frame_) ----
    auto in_frame = msg->header.frame_id;
    bool is_world_frame =
            (in_frame == "camera_init" || in_frame == "map" || in_frame == "world");

    if (this->normalize_world_cloud_to_local_ && is_world_frame) {
        // 取最接近当前点云时间戳的里程计（你已经有 nearestPose_）
        Pose P;
        if (this->nearestPose_(msg->header.stamp, P)) {
            // P.T = T_wc = 世界->child_frame_ 的位姿（来自 Odometry）
            const Eigen::Matrix4f& T_wc = P.T;

            // 求 child<-world 变换：T_cw = inv(T_wc)
            Eigen::Matrix4f T_cw = Eigen::Matrix4f::Identity();
            T_cw.block<3,3>(0,0) = T_wc.block<3,3>(0,0).transpose();
            T_cw.block<3,1>(0,3) = -T_cw.block<3,3>(0,0) * T_wc.block<3,1>(0,3);

            // 就地把点云从世界系拉回到 child_frame_ 局部系
            pcl::transformPointCloud(*in_cloud_ptr, *in_cloud_ptr, T_cw);

            // 让下游表意一致（可选）：标注为 child_frame_
            in_cloud_ptr->header.frame_id = this->child_frame_;
        } else {
            ROS_WARN_THROTTLE(1.0,
                              "[centerpp] no odom near stamp=%.3f, skip world->local normalization",
                              msg->header.stamp.toSec());
        }
    }

    t_norm_.add(wall_now_ms() - tN0);

    double tp0 = wall_now_ms();
    // 1) pack to pinned host (no disk)
    size_t points_num = packCloudToPillarsInput(
            *in_cloud_ptr, h_points_, max_points_,
            (float)MINIMUM_RANGE, (float)MAXMUM_RANGE);
    t_pack_.add(wall_now_ms() - tp0);

    double th0 = wall_now_ms();
    // 2) H2D async copy on your stream
    cudaCheckOrThrow(cudaMemcpyAsync(
            d_points_, h_points_,
            points_num * 5 * sizeof(float),
            cudaMemcpyHostToDevice,
            stream_), "cudaMemcpyAsync");
    cudaCheckOrThrow(cudaStreamSynchronize(stream_), "cudaStreamSynchronize before inference");
    t_h2d_.add(wall_now_ms() - th0);

    if (test_inject_exception_seq_ >= 0 &&
        seq == static_cast<uint64_t>(test_inject_exception_seq_)) {
        throw std::runtime_error("runtime injected inference exception");
    }
    const ros::WallTime inference_started = ros::WallTime::now();
    if (test_inject_delay_ms_ > 0) {
        ros::WallDuration(static_cast<double>(test_inject_delay_ms_) / 1000.0).sleep();
    }
    double ti0 = wall_now_ms();
    // 3) inference (runs on stream)
    cudaCheckOrThrow(cudaEventRecord(start_, stream_), "cudaEventRecord start");
    double t1 = ros::Time::now().toSec();
    center_pointpillars_ptr_->doinfer((void*)d_points_, points_num, stream_);
    cudaCheckOrThrow(cudaStreamSynchronize(stream_), "cudaStreamSynchronize after inference");
    double t2 = ros::Time::now().toSec();
    cudaCheckOrThrow(cudaEventRecord(stop_, stream_), "cudaEventRecord stop");

    t_infer_.add(wall_now_ms() - ti0);
    result.inference_ms = (ros::WallTime::now() - inference_started).toSec() * 1000.0;

    // timing（可选：建议不要每帧都同步 event）
    avg_centerpoint_time.push_back((t2 - t1) * 1000);


    if (inference_budget_ms_ >= 0.0 && result.inference_ms > inference_budget_ms_) {
        timeout_cnt_.fetch_add(1, std::memory_order_relaxed);
        bypass_cnt_.fetch_add(1, std::memory_order_relaxed);
        result.reason = "inference_timeout";
        publishBypassFrame_(msg);
        ROS_WARN("[centerpp][runtime] seq=%lu inference %.2f ms exceeded budget %.2f ms; passthrough.",
                 static_cast<unsigned long>(seq), result.inference_ms, inference_budget_ms_);
        return result;
    }

    std::vector<Bndbox> filter_BBox;
    for (auto box : this->center_pointpillars_ptr_->nms_pred_) {
        // car(id=0)/pedestrain(id=8)/cyclists(id=6)/truck(id=3)
        if (   (box.id == 0 && box.score > 0.5)    // car
               || (box.id == 6 && box.score > 0.75)   // cyclist
               || (box.id == 8 && box.score > 0.30)   // pedestrian
                )
        {
            filter_BBox.push_back(box);   // 都当作需要剔除的“动态目标”
        }
    }

    if (test_force_empty_detections_) filter_BBox.clear();

    std::vector<Bndbox> dynamic_BBox = filter_BBox;
    std::vector<Bndbox> boxes_to_remove = filter_BBox;

    if (dynamic_removal_enable_) {
        result.removed_points = runDynamicRemoval_(msg, in_cloud_ptr, filter_BBox, T_odom_child, has_odom);
        if (filter_BBox.empty()) {
            result.reason = "empty_detection";
        } else if (!has_odom) {
            result.reason = "odom_unavailable";
        } else if (result.removed_points == 0) {
            result.reason = "no_safe_moving";
        } else {
            result.outcome = "filtered";
            result.reason = "filtered";
        }
        return result;
    }

    if (detection_only_) {
        // Detection-only mode deliberately does not feed a filtered cloud to SLAM.  We
        // still extract box-contained XYZ points for offline inspection, while
        // static_points remains the original PointCloud2 message.
        CloudTPtr ignored_static(new CloudT());
        pcl::PointCloud<pcl::PointXYZ>::Ptr dynamic_points(new pcl::PointCloud<pcl::PointXYZ>());
        if (!boxes_to_remove.empty()) {
            this->extractBBoxPointcloud(boxes_to_remove, in_cloud_ptr, ignored_static, dynamic_points);
        }

        dynamic_points->width = static_cast<std::uint32_t>(dynamic_points->size());
        dynamic_points->height = 1;
        dynamic_points->is_dense = true;
        this->publishPassthrough(*msg);
        this->publishObjectBoundingBox(msg->header, filter_BBox);
        this->publishDynamicBoundingBox(msg->header, dynamic_BBox);
        this->publishClusterRaw(msg->header, dynamic_points);
        result.reason = "detection_only";
        return result;
    }


    if (boxes_to_remove.empty()) {
        empty_cnt_.fetch_add(1, std::memory_order_relaxed);
        // 1) 预处理 + 发布：这里 in_cloud_ptr 还是 CloudTPtr（含time/ring），不会丢字段
        this->preprocessPoints(in_cloud_ptr, this->MINIMUM_RANGE, this->MAXMUM_RANGE, msg->header);
        this->publishCloud(msg->header, in_cloud_ptr);

        this->publishObjectBoundingBox(msg->header, filter_BBox);
        this->publishDynamicBoundingBox(msg->header, dynamic_BBox);

        // 2) 累积全局地图：global_static_map_ 是 PointXYZI，所以要先把 CloudT 转成 XYZI
        pcl::PointCloud<pcl::PointXYZI> in_xyzi = toXYZI(*in_cloud_ptr);

        cpp::Center_PointPillars_ROS::Pose P_world;
        if (this->nearestPose_(msg->header.stamp, P_world)) {
            const Eigen::Matrix4f& T_odom_child = P_world.T;

            pcl::PointCloud<pcl::PointXYZI> in_world;
            pcl::transformPointCloud(in_xyzi, in_world, T_odom_child);
            *this->global_static_map_ += in_world;
        } else {
            *this->global_static_map_ += in_xyzi;
        }

        result.reason = "empty_detection";
        return result;
    }


    CloudTPtr out_cloud_ptr(new CloudT());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>());

    double thist0 = wall_now_ms();
// 用历史补洞：把 boxes_to_remove 在过去若干帧的并集考虑进来
    std::vector<Bndbox> active = boxes_to_remove;
    if (has_odom) this->augmentWithHistory_(active, msg->header.stamp);
    t_hist_.add(wall_now_ms() - thist0);

    double tex0 = wall_now_ms();
// 从 active 这些区域里抠除点云
    this->extractBBoxPointcloud(active, in_cloud_ptr, out_cloud_ptr, cloud_cluster);
    t_extract_.add(wall_now_ms() - tex0);

    double avg_centerpoint_totaltime = std::accumulate(avg_centerpoint_time.begin(), avg_centerpoint_time.end(), 0.0) / avg_centerpoint_time.size();
    std::cout << "CenterPoint Time :: " << std::setfill(' ') << std::setw(6) << avg_centerpoint_time.back() << " ms    // Avg: " << std::setw(5) << avg_centerpoint_totaltime << std::endl;

    double tpre0 = wall_now_ms();
    this->preprocessPoints(out_cloud_ptr, this->MINIMUM_RANGE, this->MAXMUM_RANGE, msg->header);
    t_pre_.add(wall_now_ms() - tpre0);

    double tpub0 = wall_now_ms();
    this->publishCloud(msg->header, out_cloud_ptr);
    this->publishObjectBoundingBox(msg->header, filter_BBox);
    this->publishDynamicBoundingBox(msg->header, dynamic_BBox);
//    this->publishClusterCloud(msg->header, cloud_cluster, cluster_indices);
    cloud_cluster->width  = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    this->publishClusterRaw(msg->header, cloud_cluster);
    t_pub_.add(wall_now_ms() - tpub0);
    // 记录本帧检测到的原始框（不膨胀）
    this->box_hist_.push_back({msg->header.stamp, filter_BBox});
    // 历史上限（例：2 秒）
    while (!this->box_hist_.empty() &&
           (msg->header.stamp - this->box_hist_.front().stamp).toSec() > 2.0) {
        this->box_hist_.pop_front();
    }

    // ---- 利用最近的里程计位姿，把当前帧点云变到世界系(odom_frame_)后再叠加 ----
    cpp::Center_PointPillars_ROS::Pose P_world;

    // 先把 CloudT(velodyne_ros::Point) 转成 PointXYZI
    pcl::PointCloud<pcl::PointXYZI> out_xyzi = toXYZI(*out_cloud_ptr);

    if (this->nearestPose_(msg->header.stamp, P_world)) {
        const Eigen::Matrix4f& T_odom_child = P_world.T;  // child_frame_ -> odom_frame_

        // 静态点云：XYZI
        pcl::PointCloud<pcl::PointXYZI> out_world;
        pcl::transformPointCloud(out_xyzi, out_world, T_odom_child);
        *this->global_static_map_ += out_world;

        // 动态点 mask：XYZ
        pcl::PointCloud<pcl::PointXYZ> cluster_world;
        pcl::transformPointCloud(*cloud_cluster, cluster_world, T_odom_child);
        *this->global_dynamic_mask_ += cluster_world;
    } else {
        ROS_WARN_THROTTLE(1.0,
                          "[centerpp] no odom for stamp %.3f, accumulate in local frame",
                          msg->header.stamp.toSec());

        // 找不到里程计：直接叠加本地系的 XYZI（注意不是 out_cloud_ptr）
        *this->global_static_map_ += out_xyzi;
        *this->global_dynamic_mask_ += *cloud_cluster;
    }

    t_total_.add(wall_now_ms() - T_total0);

    ros::WallTime noww = ros::WallTime::now();
    if ((noww - last_report_wall_).toSec() > 1.0) {
        last_report_wall_ = noww;

        uint64_t recv   = recv_cnt_.load(std::memory_order_relaxed);
        uint64_t proc   = proc_cnt_.load(std::memory_order_relaxed);
        uint64_t dropQ  = dropq_cnt_.load(std::memory_order_relaxed);
        uint64_t byp    = bypass_cnt_.load(std::memory_order_relaxed);
        uint64_t emp    = empty_cnt_.load(std::memory_order_relaxed);
        int qsz         = q_size_.load(std::memory_order_relaxed);

        long diff = (long)(recv - proc);

        double stamp_gap = 0.0;
        if (!last_proc_stamp_.isZero()) {
            stamp_gap = (msg->header.stamp - last_proc_stamp_).toSec();
        }
        last_proc_stamp_ = msg->header.stamp;

        ROS_WARN("[centerpp][timing][EMA ms] fromROS=%.2f norm=%.2f pack=%.2f h2d=%.2f infer=%.2f hist=%.2f extract=%.2f pre=%.2f pub=%.2f TOTAL=%.2f | recv=%lu proc=%lu dropQ=%lu diff=%ld q=%d | bypass=%lu empty=%lu | stamp_gap=%.3f",
                 t_fromros_.ema_ms, t_norm_.ema_ms, t_pack_.ema_ms, t_h2d_.ema_ms, t_infer_.ema_ms,
                 t_hist_.ema_ms, t_extract_.ema_ms, t_pre_.ema_ms, t_pub_.ema_ms,
                 t_total_.ema_ms,
                 (unsigned long)recv, (unsigned long)proc, (unsigned long)dropQ, diff, qsz,
                 (unsigned long)byp, (unsigned long)emp,
                 stamp_gap);
    }
    result.outcome = "filtered";
    result.reason = "legacy_filtered";
    return result;
}



std::vector<Center_PointPillars_ROS::BoxParam>
Center_PointPillars_ROS::makeBoxParams_(const std::vector<Bndbox>& boxes, bool padded) const
{
    const bool yaw_flip = true;
    const float scale_xy = padded ? 1.2f : 1.0f;
    const float pad_xy = padded ? 0.35f : 0.0f;
    const float pad_z = padded ? 0.40f : 0.0f;
    const float scale_z = padded ? 1.05f : 1.0f;

    std::vector<BoxParam> result;
    result.reserve(boxes.size());
    for (const auto& b : boxes) {
        float yaw = static_cast<float>(b.rt);
        if (yaw_flip) yaw = -yaw - static_cast<float>(M_PI / 2.0);
        const float c = std::cos(yaw);
        const float s = std::sin(yaw);
        const float hx = 0.5f * static_cast<float>(b.l) * scale_xy + pad_xy;
        const float hy = 0.5f * static_cast<float>(b.w) * scale_xy + pad_xy;
        const float hz = 0.5f * static_cast<float>(b.h) * scale_z + pad_z;
        result.push_back({static_cast<float>(b.x), static_cast<float>(b.y),
                          static_cast<float>(b.z), c, s, hx, hy, hz,
                          hx * hx + hy * hy});
    }
    return result;
}

bool Center_PointPillars_ROS::pointInBox_(const PointT& point, const BoxParam& box)
{
    const float dx = point.x - box.cx;
    const float dy = point.y - box.cy;
    if (dx * dx + dy * dy > box.r2 || std::fabs(point.z - box.cz) > box.hz) return false;

    const float lx = dx * box.c + dy * box.s;
    const float ly = -dx * box.s + dy * box.c;
    return std::fabs(lx) <= box.hx && std::fabs(ly) <= box.hy;
}

Center_PointPillars_ROS::MotionState Center_PointPillars_ROS::updateTrack_(
        const Bndbox& box, const ros::Time& stamp, const Eigen::Matrix4f& T_odom_child,
        bool has_odom, MotionEvidence* evidence)
{
    if (evidence) *evidence = MotionEvidence();
    if (!has_odom) return MotionState::UNKNOWN;

    const Eigen::Vector3f center_imu = lidar_to_imu_R_ * Eigen::Vector3f(box.x, box.y, box.z)
                                      + lidar_to_imu_t_;
    const Eigen::Vector3f center_world =
            (T_odom_child * Eigen::Vector4f(center_imu.x(), center_imu.y(), center_imu.z(), 1.0f)).head<3>();

    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                  [&](const ObjectTrack& track) {
                      return (stamp - track.last_stamp).toSec() > track_timeout_sec_;
                  }), tracks_.end());

    ObjectTrack* matched = nullptr;
    float best_distance = static_cast<float>(association_distance_m_);
    for (auto& track : tracks_) {
        if (track.class_id != box.id || track.observations.empty()) continue;
        const float distance = (center_world - track.observations.back().center_world).head<2>().norm();
        if (distance < best_distance) {
            best_distance = distance;
            matched = &track;
        }
    }

    if (!matched) {
        ObjectTrack track;
        track.id = next_track_id_++;
        track.class_id = box.id;
        track.last_stamp = stamp;
        track.observations.push_back({stamp, center_world});
        tracks_.push_back(track);
        if (evidence) evidence->observations = 1;
        return MotionState::UNKNOWN;
    }

    matched->last_stamp = stamp;
    matched->observations.push_back({stamp, center_world});
    while (matched->observations.size() > 1 &&
           (stamp - matched->observations.front().stamp).toSec() > motion_window_sec_) {
        matched->observations.pop_front();
    }

    const TrackObservation& first = matched->observations.front();
    const TrackObservation& last = matched->observations.back();
    const double duration = (last.stamp - first.stamp).toSec();
    if (evidence) {
        evidence->observations = static_cast<int>(matched->observations.size());
        evidence->duration_sec = duration;
    }
    if (static_cast<int>(matched->observations.size()) < motion_min_observations_ ||
        duration < motion_min_duration_sec_) {
        return MotionState::UNKNOWN;
    }

    const double speed_mps = (last.center_world - first.center_world).head<2>().norm() / duration;
    if (evidence) evidence->speed_mps = speed_mps;
    if (speed_mps <= static_speed_max_mps_) return MotionState::STATIC;
    if (speed_mps >= moving_speed_min_mps_) return MotionState::MOVING;
    return MotionState::UNKNOWN;
}

bool Center_PointPillars_ROS::estimateGroundPlane_(const Bndbox& box, const CloudTPtr& cloud,
                                                    GroundPlane& plane) const
{
    plane = GroundPlane();
    const std::vector<Bndbox> one_box(1, box);
    const BoxParam core = makeBoxParams_(one_box, false).front();
    BoxParam outer = core;
    outer.hx += static_cast<float>(ground_search_margin_m_);
    outer.hy += static_cast<float>(ground_search_margin_m_);
    outer.r2 = outer.hx * outer.hx + outer.hy * outer.hy;

    pcl::PointCloud<pcl::PointXYZ>::Ptr ring(new pcl::PointCloud<pcl::PointXYZ>());
    ring->points.reserve(400);
    for (const auto& point : cloud->points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) continue;
        if (pointInBox_(point, outer) && !pointInBox_(point, core)) {
            ring->points.push_back(pcl::PointXYZ(point.x, point.y, point.z));
        }
    }
    if (ring->points.size() < static_cast<size_t>(ground_min_inliers_)) {
        plane.status = GroundPlane::Status::INSUFFICIENT_RING;
        return false;
    }
    ring->width = static_cast<uint32_t>(ring->points.size());
    ring->height = 1;

    pcl::SACSegmentation<pcl::PointXYZ> segmentation;
    segmentation.setOptimizeCoefficients(true);
    segmentation.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setAxis(Eigen::Vector3f::UnitZ());
    segmentation.setEpsAngle(static_cast<float>(25.0 * M_PI / 180.0));
    segmentation.setMaxIterations(100);
    segmentation.setDistanceThreshold(ground_ransac_distance_m_);
    segmentation.setInputCloud(ring);

    pcl::PointIndices inliers;
    pcl::ModelCoefficients coefficients;
    segmentation.segment(inliers, coefficients);
    plane.inliers = static_cast<int>(inliers.indices.size());
    if (inliers.indices.size() < static_cast<size_t>(ground_min_inliers_) || coefficients.values.size() != 4) {
        plane.status = GroundPlane::Status::RANSAC_FAILED;
        return false;
    }

    Eigen::Vector3f normal(coefficients.values[0], coefficients.values[1], coefficients.values[2]);
    const float norm = normal.norm();
    if (norm < 1e-4f || std::fabs(normal.z() / norm) < std::cos(static_cast<float>(25.0 * M_PI / 180.0))) {
        plane.status = GroundPlane::Status::TILT_REJECTED;
        return false;
    }
    normal /= norm;
    float d = coefficients.values[3] / norm;
    if (normal.z() < 0.0f) { normal = -normal; d = -d; }
    // Keep the fitted coefficients in diagnostics even when the subsequent
    // physical-height check rejects this plane.
    plane.normal = normal;
    plane.d = d;
    plane.z_at_box_center = -(normal.x() * box.x + normal.y() * box.y + d) / normal.z();
    plane.box_bottom_z = box.z - 0.5f * box.h;
    plane.height_above_box_bottom = plane.z_at_box_center - plane.box_bottom_z;
    // A road plane must be near the object contact height.  The earlier
    // implementation only constrained the normal direction, so a car roof or
    // elevated sidewalk surface in the outer ring could be mistaken for ground
    // and preserve a moving person's upper-body points.
    if (plane.height_above_box_bottom > ground_max_height_above_box_bottom_m_ ||
        plane.height_above_box_bottom < -ground_max_height_below_box_bottom_m_) {
        plane.status = GroundPlane::Status::HEIGHT_REJECTED;
        return false;
    }
    plane.status = GroundPlane::Status::VALID;
    plane.valid = true;
    return true;
}

const char* Center_PointPillars_ROS::motionStateName_(MotionState state)
{
    switch (state) {
    case MotionState::UNKNOWN: return "unknown";
    case MotionState::STATIC: return "static";
    case MotionState::MOVING: return "moving";
    }
    return "unknown";
}

const char* Center_PointPillars_ROS::groundStatusName_(GroundPlane::Status status)
{
    switch (status) {
    case GroundPlane::Status::NOT_EVALUATED: return "not_evaluated";
    case GroundPlane::Status::INSUFFICIENT_RING: return "insufficient_ring";
    case GroundPlane::Status::RANSAC_FAILED: return "ransac_failed";
    case GroundPlane::Status::TILT_REJECTED: return "tilt_rejected";
    case GroundPlane::Status::HEIGHT_REJECTED: return "height_rejected";
    case GroundPlane::Status::VALID: return "valid";
    }
    return "not_evaluated";
}

void Center_PointPillars_ROS::publishGroundDiagnostics_(
        const std_msgs::Header& header, const std::vector<CandidateDiagnostic>& diagnostics)
{
    // Publish one compact key=value record per semantic candidate.  This keeps
    // diagnostics human-readable in rosbag while retaining the exact source
    // stamp, motion evidence and physical ground-plane decision.
    for (const auto& diagnostic : diagnostics) {
        std_msgs::String message;
        std::ostringstream output;
        output << std::setprecision(9)
               << "stamp_ns=" << header.stamp.toNSec()
               << ";label=" << diagnostic.box.id
               << ";state=" << motionStateName_(diagnostic.motion)
               << ";accepted=" << (diagnostic.accepted_for_removal ? 1 : 0)
               << ";observations=" << diagnostic.evidence.observations
               << ";duration_sec=" << diagnostic.evidence.duration_sec
               << ";speed_mps=" << diagnostic.evidence.speed_mps
               << ";ground_status=" << groundStatusName_(diagnostic.ground.status)
               << ";ground_inliers=" << diagnostic.ground.inliers
               << ";ground_nx=" << diagnostic.ground.normal.x()
               << ";ground_ny=" << diagnostic.ground.normal.y()
               << ";ground_nz=" << diagnostic.ground.normal.z()
               << ";ground_d=" << diagnostic.ground.d
               << ";ground_z_at_box_center=" << diagnostic.ground.z_at_box_center
               << ";box_bottom_z=" << diagnostic.ground.box_bottom_z
               << ";ground_height_above_box_bottom="
               << diagnostic.ground.height_above_box_bottom;
        message.data = output.str();
        pub_ground_diagnostics_.publish(message);
    }
}

size_t Center_PointPillars_ROS::runDynamicRemoval_(
        const sensor_msgs::PointCloud2ConstPtr& msg, const CloudTPtr& cloud,
        const std::vector<Bndbox>& candidates, const Eigen::Matrix4f& T_odom_child, bool has_odom)
{
    // static_points is always the established byte-identical safety baseline.
    // filtered_points is the separately launched, controlled dynamic mapper
    // input and is either a binary-record subset or this exact input frame.
    publishPassthrough(*msg);
    publishObjectBoundingBox(msg->header, candidates);

    const size_t input_records = static_cast<size_t>(msg->width) * msg->height;
    if (msg->point_step == 0 || input_records != cloud->points.size()) {
        ROS_WARN_THROTTLE(1.0, "[centerpp][dynamic_removal] PointCloud2/PCL point count mismatch; preserve the entire frame.");
        const CloudTPtr empty(new CloudT());
        publishDiagnosticCloud(msg->header, empty, pub_pointcloud_cluster_);
        publishDiagnosticCloud(msg->header, empty, pub_static_candidate_points_);
        publishDiagnosticCloud(msg->header, empty, pub_protected_ground_points_);
        pub_filtered_points_.publish(*msg);
        publishDynamicBoundingBox(msg->header, std::vector<Bndbox>());
        return 0;
    }

    std::vector<Bndbox> preserved_candidate_boxes;
    std::vector<Bndbox> safe_moving_boxes;
    std::vector<SafeMovingBox> safe_moving;
    std::vector<CandidateDiagnostic> diagnostics;
    diagnostics.reserve(candidates.size());
    for (const auto& candidate : candidates) {
        CandidateDiagnostic diagnostic;
        diagnostic.box = candidate;
        diagnostic.motion = updateTrack_(candidate, msg->header.stamp, T_odom_child,
                                         has_odom, &diagnostic.evidence);
        if (diagnostic.motion != MotionState::MOVING) {
            // STATIC and UNKNOWN are both safe defaults: do not remove points.
            preserved_candidate_boxes.push_back(candidate);
            diagnostics.push_back(diagnostic);
            continue;
        }

        GroundPlane ground;
        if (!estimateGroundPlane_(candidate, cloud, ground)) {
            // A moving classification without a reliable local ground model is
            // still not sufficient authority to delete a road point.
            ROS_WARN_THROTTLE(1.0,
                              "[centerpp][dynamic_removal] moving candidate has no reliable ground plane; preserve it.");
            preserved_candidate_boxes.push_back(candidate);
            diagnostic.ground = ground;
            diagnostics.push_back(diagnostic);
            continue;
        }
        const std::vector<Bndbox> one_box(1, candidate);
        const BoxParam padded_box = makeBoxParams_(one_box, true).front();
        safe_moving.push_back({
            padded_box,
            ground,
            // The padded box is only the conservative XY/Z inclusion region
            // for an object.  Its bottom is deliberately below the detector
            // box bottom (Z padding is 0.40 m), so using it here would put the
            // protection ceiling below the physical road in many frames.
            // Keep the ground-neighbourhood gate below, but define its height
            // relative to the original detector box bottom as configured.
            candidate.z - 0.5f * candidate.h +
                static_cast<float>(ground_protect_max_height_above_box_bottom_m_),
        });
        safe_moving_boxes.push_back(candidate);
        diagnostic.ground = ground;
        diagnostic.accepted_for_removal = true;
        diagnostics.push_back(diagnostic);
    }
    publishGroundDiagnostics_(msg->header, diagnostics);

    const std::vector<BoxParam> preserved_params = makeBoxParams_(preserved_candidate_boxes, true);
    std::vector<uint8_t> removed(cloud->points.size(), 0);
    CloudTPtr moving_points(new CloudT());
    CloudTPtr protected_points(new CloudT());
    CloudTPtr candidate_points(new CloudT());
    moving_points->points.reserve(1024);
    protected_points->points.reserve(1024);
    candidate_points->points.reserve(1024);

    for (size_t index = 0; index < cloud->points.size(); ++index) {
        const PointT& point = cloud->points[index];
        bool preserve = false;
        for (const auto& box : preserved_params) {
            if (pointInBox_(point, box)) { preserve = true; break; }
        }
        if (preserve) {
            candidate_points->points.push_back(point);
            continue;
        }

        for (const auto& moving : safe_moving) {
            if (!pointInBox_(point, moving.box)) continue;
            const float plane_distance = std::fabs(moving.ground.normal.dot(
                    Eigen::Vector3f(point.x, point.y, point.z)) + moving.ground.d);
            if (plane_distance <= ground_keep_distance_m_ &&
                point.z <= moving.max_protected_z) {
                protected_points->points.push_back(point);
            } else {
                removed[index] = 1;
                moving_points->points.push_back(point);
            }
            break;
        }
    }

    size_t removed_count = 0;
    std::vector<uint8_t> keep(removed.size(), 1);
    for (size_t index = 0; index < removed.size(); ++index) {
        if (removed[index]) { keep[index] = 0; ++removed_count; }
    }

    publishDiagnosticCloud(msg->header, moving_points, pub_pointcloud_cluster_);
    publishDiagnosticCloud(msg->header, candidate_points, pub_static_candidate_points_);
    publishDiagnosticCloud(msg->header, protected_points, pub_protected_ground_points_);
    publishDynamicBoundingBox(msg->header, safe_moving_boxes);
    if (removed_count == 0) pub_filtered_points_.publish(*msg);
    else publishSelectedRecords(*msg, keep, pub_filtered_points_);

    ROS_INFO_THROTTLE(1.0,
                      "[centerpp][dynamic_removal] candidates=%zu safe_moving=%zu removed=%zu protected_ground=%zu odom=%s",
                      candidates.size(), safe_moving.size(), removed_count,
                      protected_points->points.size(), has_odom ? "yes" : "no");
    return removed_count;
}


void Center_PointPillars_ROS::extractBBoxPointcloud(
        std::vector<Bndbox> filter_BBox,
        const CloudTPtr& cloud_in,
        CloudTPtr& cloud_out,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_cluster)
{
    if (filter_BBox.empty()) { *cloud_out = *cloud_in; return; }

    const bool  YAW_FLIP = true;
    const float SCALE_XY = 1.2f;
    const float PAD_X = 0.35f, PAD_Y = 0.35f, PAD_Z = 0.40f;

    std::vector<BoxParam> B; B.reserve(filter_BBox.size());
    for (const auto& b : filter_BBox) {
        float yaw = (float)b.rt;
        if (YAW_FLIP) { yaw = -yaw - (float)M_PI/2.0f; }
        float c = std::cos(yaw), s = std::sin(yaw);

        float hx = 0.5f*(float)b.l*SCALE_XY + PAD_X;
        float hy = 0.5f*(float)b.w*SCALE_XY + PAD_Y;
        float hz = 0.5f*(float)b.h*1.05f    + PAD_Z;
        float r2 = (hx*hx + hy*hy); // 外接圆半径^2（保守一点也行）

        B.push_back({(float)b.x,(float)b.y,(float)b.z, c,s, hx,hy,hz, r2});
    }

    const int N = (int)cloud_in->size();
    std::vector<uint8_t> rm(N, 0);

    // 可选：OpenMP 并行（有的话速度提升很明显）
    // #pragma omp parallel for schedule(static)
    for (int i = 0; i < N; ++i) {
        const auto& pi = cloud_in->points[i];
        const float px = pi.x, py = pi.y, pz = pi.z;

        for (const auto& bp : B) {
            float dx = px - bp.cx, dy = py - bp.cy;
            if (dx*dx + dy*dy > bp.r2) continue;  // 快速圆门限

            float dz = pz - bp.cz;
            if (std::fabs(dz) > bp.hz) continue;

            float lx =  dx*bp.c + dy*bp.s;
            float ly = -dx*bp.s + dy*bp.c;
            if (std::fabs(lx) <= bp.hx && std::fabs(ly) <= bp.hy) {
                rm[i] = 1;
                break;
            }
        }
    }

    cloud_out->points.clear();
    cloud_cluster->points.clear();
    cloud_out->points.reserve(cloud_in->size());
    cloud_cluster->points.reserve(std::min<size_t>(cloud_in->size(), 100000));

    for (int i = 0; i < N; ++i) {
        const auto& pi = cloud_in->points[i];
        if (rm[i]) cloud_cluster->points.push_back(pcl::PointXYZ{pi.x,pi.y,pi.z});
        else       cloud_out->points.push_back(pi); // Preserve all point fields.
    }

    cloud_out->width  = (uint32_t)cloud_out->points.size();
    cloud_out->height = 1;
    cloud_out->is_dense = true;
}



// 取时间 t 最近的里程计
bool Center_PointPillars_ROS::nearestPose_(ros::Time t, Pose& out) {
    std::lock_guard<std::mutex> lk(mtx_odom_);
    if (odom_hist_.empty()) return false;
    size_t best = 0; double bestdt = 1e9;
    for (size_t i = 0; i < odom_hist_.size(); ++i) {
        double dt = std::fabs((odom_hist_[i].stamp - t).toSec());
        if (dt < bestdt) { bestdt = dt; best = i; }
    }
    out = odom_hist_[best];
    return true;
}

// 计算 T_ba = T_b * inv(T_a)
bool Center_PointPillars_ROS::relativeT_(ros::Time ta, ros::Time tb, Eigen::Matrix4f& T_ba) {
    Pose A, B;
    if (!nearestPose_(ta, A) || !nearestPose_(tb, B)) return false;
    Eigen::Matrix4f Ainv = Eigen::Matrix4f::Identity();
    Ainv.block<3,3>(0,0) = A.T.block<3,3>(0,0).transpose();
    Ainv.block<3,1>(0,3) = -Ainv.block<3,3>(0,0) * A.T.block<3,1>(0,3);
    T_ba = B.T * Ainv;
    return true;
}

// 把框按 4x4 变换到当前时刻，并做尺寸膨胀
Bndbox Center_PointPillars_ROS::transformBox_(const Bndbox& b, const Eigen::Matrix4f& T) {
    Eigen::Vector4f c(b.x, b.y, b.z, 1.f);
    Eigen::Vector4f c_new = T * c;
    // XY 平面旋转近似
    float yaw_delta = std::atan2(T(1,0), T(0,0));

    Bndbox o = b;
    o.x = c_new.x(); o.y = c_new.y(); o.z = c_new.z();
    o.rt = b.rt + yaw_delta;
    o.l  = b.l * (1.0 + inflate_ratio_) + 2.0 * inflate_m_;
    o.w  = b.w * (1.0 + inflate_ratio_) + 2.0 * inflate_m_;
    o.h  = b.h * (1.0 + inflate_ratio_) + 2.0 * inflate_m_;
    return o;
}

// 把历史框并入当帧
void Center_PointPillars_ROS::augmentWithHistory_(std::vector<Bndbox>& boxes_now, ros::Time t_now) {
    int used = 0;
    for (auto it = box_hist_.rbegin(); it != box_hist_.rend(); ++it) {
        if ((t_now - it->stamp).toSec() > coast_time_sec_) break;
        if (used >= coast_max_frames_) break;

        Eigen::Matrix4f T; // T_now_prev: prev -> now
        if (!relativeT_(it->stamp, t_now, T)) { ++used; continue; }

        for (const auto& b : it->boxes) {
            Bndbox bb = transformBox_(b, T);

            // 与当前已有框做一个简单合并（中心距离）
            bool dup = false;
            for (const auto& cur : boxes_now) {
                float dx = bb.x - cur.x, dy = bb.y - cur.y;
                if (dx*dx + dy*dy < static_cast<float>(merge_center_thresh_ * merge_center_thresh_)) {
                    dup = true; break;
                }
            }
            if (!dup) boxes_now.push_back(bb);
        }
        ++used;
    }
}

void Center_PointPillars_ROS::publishCloud(std_msgs::Header header,
                                           const CloudTPtr& in_cloud_to_publish_ptr)
{
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);

    static bool printed=false;
    if(!printed){
        std::ostringstream ss;
        ss << "[centerpp] OUT fields: ";
        for(const auto& f: cloud_msg.fields) ss << f.name << " ";
        ROS_WARN_STREAM(ss.str());
        printed=true;
    }

    cloud_msg.header = header;
    cloud_msg.header.frame_id = outputFrame(header);
    this->pub_pointcloud_static_.publish(cloud_msg);
}

void Center_PointPillars_ROS::publishRawInput(const sensor_msgs::PointCloud2& cloud_msg)
{
    // Do not deserialize and rebuild this message: dynamic-removal validation
    // compares this topic with static_points to prove that detection-only mode
    // cannot change the point layout, timestamp, or point-wise time offsets.
    this->pub_pointcloud_raw_.publish(cloud_msg);
}

void Center_PointPillars_ROS::publishBypassFrame_(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    // The worker has already published raw_points.  Publish every output that
    // might otherwise be missing on an error/timeout path, and keep the only
    // mapper-facing cloud byte-identical to this input frame.
    publishPassthrough(*msg);
    pub_filtered_points_.publish(*msg);
    const CloudTPtr empty(new CloudT());
    publishDiagnosticCloud(msg->header, empty, pub_pointcloud_cluster_);
    publishDiagnosticCloud(msg->header, empty, pub_static_candidate_points_);
    publishDiagnosticCloud(msg->header, empty, pub_protected_ground_points_);
    publishObjectBoundingBox(msg->header, std::vector<Bndbox>());
    publishDynamicBoundingBox(msg->header, std::vector<Bndbox>());
}

void Center_PointPillars_ROS::publishFrameDiagnostics_(
        const QueuedFrame& frame, const FrameResult& result,
        const ros::WallTime& processing_started, const ros::WallTime& published)
{
    std_msgs::String message;
    const double queue_delay_ms = (processing_started - frame.received_wall).toSec() * 1000.0;
    const double end_to_end_ms = (published - frame.received_wall).toSec() * 1000.0;
    std::ostringstream output;
    output << std::fixed << std::setprecision(3)
           << "seq=" << frame.seq
           << ";stamp_ns=" << frame.msg->header.stamp.toNSec()
           << ";outcome=" << result.outcome
           << ";reason=" << result.reason
           << ";queue_delay_ms=" << queue_delay_ms
           << ";end_to_end_ms=" << end_to_end_ms
           << ";inference_ms=" << result.inference_ms
           << ";queue_depth_on_receive=" << frame.queue_depth_on_receive
           << ";queue_depth_on_publish=" << q_size_.load(std::memory_order_relaxed)
           << ";max_queue_depth=" << max_queue_depth_.load(std::memory_order_relaxed)
           << ";removed_points=" << result.removed_points
           << ";odom=" << (result.has_odom ? "available" : "unavailable")
           << ";dropQ=" << dropq_cnt_.load(std::memory_order_relaxed)
           << ";queue_bypass=" << queue_bypass_cnt_.load(std::memory_order_relaxed)
           << ";timeouts=" << timeout_cnt_.load(std::memory_order_relaxed)
           << ";exceptions=" << exception_cnt_.load(std::memory_order_relaxed);
    message.data = output.str();
    pub_frame_diagnostics_.publish(message);
}

void Center_PointPillars_ROS::publishPassthrough(const sensor_msgs::PointCloud2& cloud_msg)
{
    // The static topic is intentionally a direct copy of the input.
    // This function is also the safe fallback for timeout/error handling in
    // later phases; never replace the original header with ros::Time::now().
    this->pub_pointcloud_static_.publish(cloud_msg);
}

void Center_PointPillars_ROS::publishSelectedRecords(
        const sensor_msgs::PointCloud2& input, const std::vector<uint8_t>& selected,
        ros::Publisher& publisher) const
{
    const size_t input_records = static_cast<size_t>(input.width) * input.height;
    if (input.point_step == 0 || selected.size() != input_records) {
        ROS_ERROR("[centerpp][dynamic_removal] invalid PointCloud2 record selection; publishing input unchanged.");
        publisher.publish(input);
        return;
    }

    size_t kept = 0;
    for (const uint8_t value : selected) kept += value ? 1 : 0;

    // Keep the exact binary record for every retained point.  Only height and
    // width change because an arbitrary subset of a PointCloud2 is no longer
    // organized; all fields, point_step and point-wise timestamps survive.
    sensor_msgs::PointCloud2 output = input;
    output.height = 1;
    output.width = static_cast<uint32_t>(kept);
    output.row_step = output.width * output.point_step;
    output.data.resize(output.row_step);

    size_t write_offset = 0;
    for (size_t index = 0; index < input_records; ++index) {
        if (!selected[index]) continue;
        const size_t row = index / input.width;
        const size_t column = index % input.width;
        const size_t read_offset = row * input.row_step + column * input.point_step;
        if (read_offset + input.point_step > input.data.size() ||
            write_offset + input.point_step > output.data.size()) {
            ROS_ERROR("[centerpp][dynamic_removal] PointCloud2 data bounds check failed; publishing input unchanged.");
            publisher.publish(input);
            return;
        }
        std::copy(input.data.begin() + read_offset,
                  input.data.begin() + read_offset + input.point_step,
                  output.data.begin() + write_offset);
        write_offset += input.point_step;
    }
    publisher.publish(output);
}

void Center_PointPillars_ROS::publishDiagnosticCloud(
        const std_msgs::Header& header, const CloudTPtr& cloud, ros::Publisher& publisher) const
{
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header = header;
    output.header.frame_id = outputFrame(header);
    publisher.publish(output);
}

std::string Center_PointPillars_ROS::outputFrame(const std_msgs::Header &header) const
{
    if (use_input_frame_ && !header.frame_id.empty()) return header.frame_id;
    return child_frame_;
}



void Center_PointPillars_ROS::publishObjectBoundingBox(std_msgs::Header in_msg_header, std::vector<Bndbox> filter_BBox) {

    jsk_recognition_msgs::BoundingBoxArray arr_bbox;
    int i = 0;

    for (const auto box : filter_BBox) {
        jsk_recognition_msgs::BoundingBox bbox;

        bbox.header = in_msg_header;
        bbox.header.frame_id = outputFrame(in_msg_header);
        bbox.pose.position.x =  box.x;
        bbox.pose.position.y =  box.y;
        bbox.pose.position.z = box.z;
        bbox.dimensions.x = box.w;  // width
        bbox.dimensions.y = box.l;  // length
        bbox.dimensions.z = box.h;  // height
        // Using tf::Quaternion for quaternion from roll, pitch, yaw
        tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, -box.rt);
        bbox.pose.orientation.x = q.x();
        bbox.pose.orientation.y = q.y();
        bbox.pose.orientation.z = q.z();
        bbox.pose.orientation.w = q.w();
        bbox.value = box.score;
        bbox.label = box.id;
        arr_bbox.boxes.push_back(bbox);
        // if(box.score>0.5){
        // arr_bbox.boxes.push_back(bbox);
        // }

    }
    // std::cout<<"find bbox Num:"<<arr_bbox.boxes.size()<<std::endl;
    arr_bbox.header = in_msg_header;
    arr_bbox.header.frame_id = outputFrame(in_msg_header);

    this->pub_bbox_.publish(arr_bbox);
}


void Center_PointPillars_ROS::publishDynamicBoundingBox(std_msgs::Header in_msg_header, std::vector<Bndbox> dynamic_BBox) {

    this->center_points_array.markers.clear();
    this->center_points_array.markers.reserve(dynamic_BBox.size());

    jsk_recognition_msgs::BoundingBoxArray arr_bbox;
    visualization_msgs::MarkerArray text_vel_array;
    visualization_msgs::Marker text_vel, center_points;
    int id = 0;
    center_points.lifetime = ros::Duration();
    center_points.header = in_msg_header;
    center_points.header.frame_id = outputFrame(in_msg_header);
    center_points.ns = "center_points";
    center_points.action = visualization_msgs::Marker::ADD;
    center_points.type = visualization_msgs::Marker::POINTS;
    center_points.scale.x = 0.7;
    center_points.scale.y = 0.7;
    center_points.scale.z = 0.7;

    for (const auto box : dynamic_BBox) {
        jsk_recognition_msgs::BoundingBox bbox;

        bbox.header = in_msg_header;
        bbox.header.frame_id = outputFrame(in_msg_header);
        bbox.pose.position.x =  box.x;
        bbox.pose.position.y =  box.y;
        bbox.pose.position.z = box.z;
        bbox.dimensions.x = box.w;  // width
        bbox.dimensions.y = box.l;  // length
        bbox.dimensions.z = box.h;  // height
        // Using tf::Quaternion for quaternion from roll, pitch, yaw
        tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, -box.rt);
        bbox.pose.orientation.x = q.x();
        bbox.pose.orientation.y = q.y();
        bbox.pose.orientation.z = q.z();
        bbox.pose.orientation.w = q.w();
        bbox.value = box.score;
        bbox.label = box.id;
        arr_bbox.boxes.push_back(bbox);
        // if(box.score>0.5){
        // arr_bbox.boxes.push_back(bbox);
        // }

        text_vel.header = in_msg_header;
        text_vel.header.frame_id = outputFrame(in_msg_header);
        text_vel.ns = "dynamic_vel";
        text_vel.action = visualization_msgs::Marker::ADD;
        text_vel.id = id;
        text_vel.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_vel.scale.z = 1;
        text_vel.color.r = text_vel.color.b = text_vel.color.g =  1;
        text_vel.color.a = 1;
        float vel = box.vy;
        std::ostringstream oss;
        oss << std::setprecision(2) << vel;
        text_vel.text = oss.str() + "m/s";
        text_vel.pose.orientation.x = q.x();
        text_vel.pose.orientation.y = q.y();
        text_vel.pose.orientation.z = q.z();
        text_vel.pose.orientation.w = q.w();
        text_vel.pose.position.x = box.x;
        text_vel.pose.position.y = box.y;
        text_vel.pose.position.z = box.z + box.h / 2 + 0.1;
        text_vel_array.markers.push_back(text_vel);

        center_points.id = id;
        center_points.type = visualization_msgs::Marker::POINTS;
        int ci = (id % 100) * 3;
        center_points.color.r = color[ci];
        center_points.color.g = color[ci + 1];
        center_points.color.b = color[ci + 2];
        center_points.color.a = 0.7;
        center_points.pose.orientation.w = 1;
        geometry_msgs::Point p;
        p.x = box.x;
        p.y = box.y;
        p.z = box.z;
        center_points.points.push_back(p);

        this->center_points_array.markers.push_back(center_points);

        ++id;
    }
    // std::cout<<"find bbox Num:"<<arr_bbox.boxes.size()<<std::endl;
    arr_bbox.header = in_msg_header;
    arr_bbox.header.frame_id = outputFrame(in_msg_header);

    this->pub_dynamic_bbox_.publish(arr_bbox);
    this->pub_text_vel_.publish(text_vel_array);
    this->pub_center_points_.publish(this->center_points_array);
}



void Center_PointPillars_ROS::publishClusterCloud(std_msgs::Header header, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, std::vector<pcl::PointIndices> cluster_indices) {
    int color_index = 0;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_point(new pcl::PointCloud<pcl::PointXYZRGB>());
    int clusterSize = cluster_indices.size();
    for (int i = 0; i < clusterSize; i++) {
        int clusterindixSize = cluster_indices[i].indices.size();
        for (int j = 0; j < clusterindixSize; j++) {
            pcl::PointXYZRGB point;
            point.x = cloud_in->points[cluster_indices[i].indices[j]].x;
            point.y = cloud_in->points[cluster_indices[i].indices[j]].y;
            point.z = cloud_in->points[cluster_indices[i].indices[j]].z;
            point.r = color[int(3) * color_index];
            point.g = color[int(3) * color_index + 1];
            point.b = color[int(3) * color_index + 2];
            color_point->push_back(point);
        }
        color_index++;
    }

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*color_point, cloud_msg);
    cloud_msg.header = header;
    cloud_msg.header.frame_id = outputFrame(header);
    this->pub_pointcloud_cluster_.publish(cloud_msg);
}

void Center_PointPillars_ROS::publishClusterRaw(std_msgs::Header header,
                                                const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in) {
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud_in, msg);
    msg.header = header;
    msg.header.frame_id = outputFrame(header);
    this->pub_pointcloud_cluster_.publish(msg);
}

static inline void voxelDownsampleKeepFirst_(CloudT& cloud, float leaf)
{
    if (cloud.empty() || leaf <= 1e-6f) return;

    struct Key { int ix, iy, iz; };
    struct KeyHash {
        size_t operator()(const Key& k) const noexcept {
            size_t h = 1469598103934665603ULL;
            auto mix = [&](int v){
                h ^= (size_t)v + 0x9e3779b97f4a7c15ULL + (h<<6) + (h>>2);
            };
            mix(k.ix); mix(k.iy); mix(k.iz);
            return h;
        }
    };
    struct KeyEq {
        bool operator()(const Key& a, const Key& b) const noexcept {
            return a.ix==b.ix && a.iy==b.iy && a.iz==b.iz;
        }
    };

    std::unordered_map<Key, int, KeyHash, KeyEq> seen;
    seen.reserve(cloud.size()/4);

    CloudT out;
    out.header = cloud.header;
    out.is_dense = cloud.is_dense;
    out.points.reserve(cloud.points.size()/4);

    for (const auto& p : cloud.points) {
        Key k{
                (int)std::floor(p.x / leaf),
                (int)std::floor(p.y / leaf),
                (int)std::floor(p.z / leaf)
        };
        if (seen.emplace(k, 1).second) {
            out.points.push_back(p); // Preserve all point fields.
        }
    }
    out.width  = (uint32_t)out.points.size();
    out.height = 1;
    cloud.swap(out);
}

void Center_PointPillars_ROS::preprocessPoints(const CloudTPtr& cloud_in, float th1, float th2,
                                               const std_msgs::Header &header)
{
    // raw 发布（可选）
    *this->original_scan_ = *cloud_in;
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*this->original_scan_, cloud_msg);
    cloud_msg.header = header;
    cloud_msg.header.frame_id = outputFrame(header);
    this->pub_pointcloud_raw_.publish(cloud_msg);

    // Remove NaNs
    std::vector<int> idx;
    cloud_in->is_dense = false;
    pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, idx);

    // Range filter（保留字段）
    this->removeClosedPointCloud(*cloud_in, *cloud_in, th1, th2);

    // Crop（不会改字段）
    if (this->crop_use_) {
        this->crop.setInputCloud(cloud_in);
        this->crop.filter(*cloud_in);
    }

    // Voxel downsampling keeps the first complete point in each voxel.
    if (this->vf_use_) {
        voxelDownsampleKeepFirst_(*cloud_in, (float)this->vf_res_);
    }
}

void Center_PointPillars_ROS::removeClosedPointCloud(const CloudT &cloud_in,
                                                     CloudT &cloud_out,
                                                     float th1, float th2)
{
    if (&cloud_in != &cloud_out) {
        cloud_out.header = cloud_in.header;
        cloud_out.points.resize(cloud_in.points.size());
    }

    size_t j = 0;
    for (size_t i = 0; i < cloud_in.points.size(); ++i) {
        const float x = cloud_in.points[i].x;
        const float y = cloud_in.points[i].y;
        const float z = cloud_in.points[i].z;
        float dis = x*x + y*y + z*z;
        if (dis < th1 * th1) continue;
        if (dis > th2 * th2) continue;
        cloud_out.points[j++] = cloud_in.points[i]; // Copy the complete point.
    }

    cloud_out.points.resize(j);
    cloud_out.height = 1;
    cloud_out.width  = (uint32_t)j;
    cloud_out.is_dense = true;
}




} // namespace cpp

int main(int argc, char **argv) {
    ros::init(argc, argv, "centerpp_node");
    ros::NodeHandle nh("~");

    int device_count = 0;
    const cudaError_t device_status = cudaGetDeviceCount(&device_count);
    if (device_status != cudaSuccess || device_count < 1) {
        ROS_FATAL("[centerpp] No CUDA device is available. CenterPoint dynamic removal requires CUDA, TensorRT, "
                  "spconv, and the two model files; the base FAST-LIVO2 mapping node is unaffected.");
        return EXIT_FAILURE;
    }

    color.clear();
    for (size_t i_segment = 0; i_segment < 100; i_segment++)
    {
        color.push_back(static_cast<unsigned char>(rand() % 256));
        color.push_back(static_cast<unsigned char>(rand() % 256));
        color.push_back(static_cast<unsigned char>(rand() % 256));
    }

    // GetDeviceInfo();
    initDevice(0);

    cpp::Center_PointPillars_ROS center_pintPillars_ros(nh);
    center_pintPillars_ros.Process();

    return 0;
}
