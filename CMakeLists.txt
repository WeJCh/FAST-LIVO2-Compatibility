cmake_minimum_required(VERSION 2.8.3)
project(fast_livo)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set common compile options
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")

# Specific settings for Debug build
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g")

# Detect CPU architecture
message(STATUS "Current CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")

# Specific settings for Release build
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm|aarch64|ARM|AARCH64)")
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    # 64-bit ARM optimizations (e.g., RK3588 and Jetson Orin NX)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -mcpu=native -mtune=native -ffast-math")
    message(STATUS "Using 64-bit ARM optimizations: -O3 -mcpu=native -mtune=native -ffast-math")
  else()
    # 32-bit ARM optimizations with NEON support
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -mcpu=native -mtune=native -mfpu=neon -ffast-math")
    message(STATUS "Using 32-bit ARM optimizations: -O3 -mcpu=native -mtune=native -mfpu=neon -ffast-math")
  endif()
  add_definitions(-DARM_ARCH)
else()
  # x86-64 (Intel/AMD) optimizations
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=native -mtune=native -funroll-loops") #-flto
  message(STATUS "Using general x86 optimizations: -O3 -march=native -mtune=native -funroll-loops") 
  add_definitions(-DX86_ARCH)
endif()

# Define project root directory
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

# Detect CPU core count for potential multithreading optimization
include(ProcessorCount)
ProcessorCount(N)
message(STATUS "Processor count: ${N}")

# Set the number of cores for multithreading
if(N GREATER 4)
  math(EXPR PROC_NUM "4")
  add_definitions(-DMP_EN -DMP_PROC_NUM=${PROC_NUM})
  message(STATUS "Multithreading enabled. Cores: ${PROC_NUM}")
elseif(N GREATER 1)
  math(EXPR PROC_NUM "${N}")
  add_definitions(-DMP_EN -DMP_PROC_NUM=${PROC_NUM})
  message(STATUS "Multithreading enabled. Cores: ${PROC_NUM}")
else()
  add_definitions(-DMP_PROC_NUM=1)
  message(STATUS "Single core detected. Multithreading disabled.")
endif()

# Check for OpenMP support
find_package(OpenMP QUIET)
if(OpenMP_CXX_FOUND)
  message(STATUS "OpenMP found")
  add_compile_options(${OpenMP_CXX_FLAGS})
else()
  message(STATUS "OpenMP not found, proceeding without it")
endif()

# Check for mimalloc support
find_package(mimalloc QUIET)
if(mimalloc_FOUND)
  message(STATUS "mimalloc found")
else()
  message(STATUS "mimalloc not found, proceeding without it")
endif()

# Find catkin and required dependencies
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  pcl_ros
  tf
  message_generation
  eigen_conversions
  vikit_common
  vikit_ros
  cv_bridge
  image_transport
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

option(BUILD_LOOP_BACKEND "Build the GTSAM LiDAR loop-closure backend" ON)
if(BUILD_LOOP_BACKEND)
  find_package(GTSAMCMakeTools QUIET)
  find_package(GTSAM QUIET)
  if(NOT GTSAM_FOUND)
    message(WARNING "GTSAM was not found: LiDAR loop closure will be disabled. Install/source GTSAM, then configure again with -DBUILD_LOOP_BACKEND=ON.")
  endif()
endif()

# Dynamic-object removal is an independent CUDA executable.
# Keep it disabled by default so the original FAST-LIVO2 build remains usable
# on CPU-only machines and does not acquire TensorRT/spconv as a base dependency.
option(BUILD_DYNAMIC_REMOVAL "Build the CenterPoint dynamic-removal node" OFF)

# Offline conversion of loop-optimized FAST-LIVO2 keyframes into an OctoMap
# and a ROS1 static 2D navigation map.  It is off by default so developers
# without OctoMap headers can still build the mapper itself.
option(BUILD_NAV_MAP_BUILDER "Build the offline 3D-to-2D navigation-map builder" OFF)

set(Sophus_LIBRARIES libSophus.so)

# Define the catkin package
catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime cv_bridge vikit_common vikit_ros image_transport
  DEPENDS EIGEN3 PCL OpenCV Sophus
)

# Include directories for dependencies
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  include
)

# Add libraries
add_library(vio src/vio.cpp src/frame.cpp src/visual_point.cpp)
add_library(lio src/voxel_map.cpp)
add_library(pre src/preprocess.cpp)
add_library(imu_proc src/IMU_Processing.cpp)
add_library(laser_mapping src/LIVMapper.cpp)

# Add the main executable
add_executable(fastlivo_mapping src/main.cpp)

# 稠密 RGB 全局地图重投影库。建图结束后由 LIVMapper 自动调用。
add_library(dense_rgb_reprojector src/dense_rgb_reprojector.cpp)
target_link_libraries(laser_mapping dense_rgb_reprojector)

# 离线道路高度裁剪工具：从完成建图后的彩色 PCD 中估计局部地面参考高度，
# 输出仅保留指定离地高度以内点的道路视图。它不参与在线 LIO/VIO。
add_executable(fast_livo_road_height_filter src/road_height_filter_main.cpp)
target_link_libraries(fast_livo_road_height_filter
  ${PCL_LIBRARIES}
)

# Link libraries to the executable
target_link_libraries(fastlivo_mapping
  laser_mapping
  vio
  lio
  pre
  imu_proc
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Sophus_LIBRARIES}
  ${Boost_LIBRARIES}
)

# Link mimalloc if found
if(mimalloc_FOUND)
  target_link_libraries(fastlivo_mapping mimalloc)
endif()

if(BUILD_LOOP_BACKEND AND GTSAM_FOUND)
  add_library(loop_backend
    src/loop_backend.cpp
  )
  target_link_libraries(loop_backend
    ${PCL_LIBRARIES}
    gtsam
  )

  # LIVMapper 仅依赖 LoopBackend 的快照接口，不直接依赖 GTSAM 状态。
  target_link_libraries(laser_mapping
    loop_backend
  )
  target_compile_definitions(laser_mapping PRIVATE FAST_LIVO_HAS_LOOP_BACKEND)
  target_compile_definitions(fastlivo_mapping PRIVATE FAST_LIVO_HAS_LOOP_BACKEND)
endif()

if(BUILD_NAV_MAP_BUILDER)
  find_package(octomap REQUIRED)

  add_executable(fast_livo_nav_map_builder
    src/nav_map_builder_main.cpp
  )
  target_include_directories(fast_livo_nav_map_builder PRIVATE
    ${OCTOMAP_INCLUDE_DIRS}
  )
  target_link_libraries(fast_livo_nav_map_builder
    ${PCL_LIBRARIES}
    ${OCTOMAP_LIBRARIES}
    gomp
  )
  add_dependencies(fast_livo_nav_map_builder ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
endif()

if(BUILD_DYNAMIC_REMOVAL)
  find_package(CUDA REQUIRED)
  find_package(jsk_recognition_msgs REQUIRED)

  if(DEFINED ENV{TENSORRT_ROOT})
    set(TENSORRT_ROOT "$ENV{TENSORRT_ROOT}")
  else()
    set(TENSORRT_ROOT "/opt/TensorRT")
  endif()

  if(DEFINED ENV{SPCONV_ROOT})
    set(SPCONV_ROOT "$ENV{SPCONV_ROOT}")
  else()
    set(SPCONV_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/libspconv")
  endif()

  if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|ARM|arm64")
    set(SPCONV_LIB_DIR "${SPCONV_ROOT}/lib/aarch64")
  else()
    set(SPCONV_LIB_DIR "${SPCONV_ROOT}/lib/x86_64")
  endif()

  foreach(REQUIRED_FILE
      "${TENSORRT_ROOT}/include/NvInfer.h"
      "${TENSORRT_ROOT}/lib/libnvinfer.so"
      "${SPCONV_ROOT}/include/spconv/engine.hpp"
      "${SPCONV_LIB_DIR}/libspconv.so")
    if(NOT EXISTS "${REQUIRED_FILE}")
      message(FATAL_ERROR
        "BUILD_DYNAMIC_REMOVAL=ON requires ${REQUIRED_FILE}. "
        "Set TENSORRT_ROOT/SPCONV_ROOT to valid installations, or build with -DBUILD_DYNAMIC_REMOVAL=OFF.")
    endif()
  endforeach()

  include_directories(
    ${CUDA_INCLUDE_DIRS}
    ${jsk_recognition_msgs_INCLUDE_DIRS}
    ${TENSORRT_ROOT}/include
    ${SPCONV_ROOT}/include
  )
  link_directories(
    ${TENSORRT_ROOT}/lib
    ${SPCONV_LIB_DIR}
  )

  file(GLOB CENTERPOINT_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/center_pointpillars/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/center_pointpillars/*.cu")

  cuda_add_executable(centerpp_node
    src/centerpp_node/centerpp_node.cpp
    ${CENTERPOINT_SOURCES}
  )
  target_compile_definitions(centerpp_node PRIVATE PCL_NO_PRECOMPILE)
  target_compile_options(centerpp_node PRIVATE ${OpenMP_CXX_FLAGS})
  set_target_properties(centerpp_node PROPERTIES
    CUDA_SEPARABLE_COMPILATION OFF)
  set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -Xcompiler=-fopenmp")
  target_link_libraries(centerpp_node
    ${catkin_LIBRARIES}
    ${jsk_recognition_msgs_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
    pthread
    gomp
    nvinfer
    nvinfer_plugin
    nvonnxparser
    spconv
  )
  target_include_directories(centerpp_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  add_dependencies(centerpp_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  install(TARGETS centerpp_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endif()
