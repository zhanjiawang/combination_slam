English | [中文](README_zh.md)

# combination_slam

`combination_slam` is a SLAM framework composed of three independent but integrable modules: front-end odometry, back-end optimization, and recurrent localization. Its features include:

- **Front-end Odometry (lio_front_end)**: Currently integrates fast_lio and dlio (direct_lidar_inertial_odometry). Both odometry modules are optimized for memory usage, preventing memory growth over time and making them suitable for long-term operation.
- **Back-end Optimization (sam_back_end)**: Derived from lio_sam's back-end, supporting loop closure detection (scan context, icp), GNSS input, GTSAM optimization, and data saving.
- **Localization Module (recurrent_localization)**: Currently integrates context_localization and plane_localization. context_localization performs relocalization based on scan context and recursive localization using octree NDT. plane_localization performs relocalization based on plane features and recursive localization using octree NDT.

## Dependencies

### System & ROS

Recommended environment:

- Ubuntu 20.04
- ROS noetic

### ROS / C++ Dependencies

According to `combination_slam`'s `CMakeLists.txt` / `package.xml`, the following are required:

- ROS Packages
  - `roscpp`
  - `rospy`
  - `rosbag`
  - `rostest`
  - `message_generation`
  - `message_filters`
  - `std_msgs`
  - `sensor_msgs`
  - `nav_msgs`
  - `geometry_msgs`
  - `tf`
  - `pcl_ros`

- C++ / Third-party Libraries
  - C++17
  - OpenMP
  - Boost
  - OpenCV (bundled with ROS)
  - PCL (bundled with ROS)
  - Eigen3 (system default)
  - GTSAM (4.2.0 recommended)
  - GeographicLib

## Quick Installation

### 1. Install ROS Dependencies (optional; the Desktop Full installation suffices for a fresh environment)

### 2. Install C++ Dependencies (optional; reference for a fresh environment)

```bash
git clone https://github.com/geographiclib/geographiclib.git
cd geographiclib
mkdir build && cd build
cmake ..
sudo make install

wget -O gtsam.zip https://github.com/borglab/gtsam/archive/refs/tags/4.2.0.zip
unzip gtsam.zip
cd gtsam-4.2.0/
mkdir build && cd build
cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON ..
sudo make install -j16
```

### 3. Build

```bash
git clone https://github.com/zhanjiawang/combination_slam.git
cd combination_slam
catkin_make / catkin_make install
```

## Running the Program

### Mapping
![Mapping](docs/readme_assets/mapping.png)
Mapping uses the script start_mapping.sh. You need to modify the PROGRAM_PATH parameter in the script according to your installation path.

Notes:
- **Front-end Odometry**: Different front-end odometry modules can be switched in the start_mapping.sh script. Overall, dlio offers higher accuracy but also higher CPU usage.
- **Front-end Odometry**: Both Livox CustomMsg and ROS PointCloud2 are supported. For fast_lio, use lio_livox.launch and lio_point.launch to distinguish between them. For dlio, configure the subscribed point cloud topics (pointcloud_topic and livox_topic) in dlio_odom.launch.
- **Back-end Optimization**: The main configuration parameters are in sam_back_end/config/params.yaml. Modify savePCDDirectory to specify the storage location for mapping data (including keyframe point clouds → /pcd, keyframe poses → pose.json, scan context features → sc.bin, point cloud map → FullMap.pcd, mapping trajectory → trajectory.pcd, etc.)

### Reconstruction
![Reconstruction](docs/readme_assets/construct.png)
Reconstruction uses the script start_construct.sh. You need to modify the PROGRAM_PATH parameter in the script according to your installation path.

Notes:
- **context_localization**: The main function is to reconstruct an octree map from the keyframe point clouds and keyframe poses obtained during mapping, and compute the NDT distribution characteristics of octree map nodes for subsequent NDT recursive localization. Specify relative_path in params_construct.yaml to determine the storage location of the data (octree distribution → octree_distribution.bin).
- **plane_localization**: Similar to the reconstruction in context_localization, but additionally extracts global plane features for global map relocalization. Specify relative_path in params_construct.yaml to determine the storage location of the data (octree occupancy → octree_occupy.bin and global plane features → octree_planes.bin).

### Localization
![Localization](docs/readme_assets/localization.png)
Localization uses the script start_localization.sh. You need to modify the PROGRAM_PATH parameter in the script according to your installation path.

Notes:
- **Obtaining Reconstruction Data**: For context_localization, specify relative_path in params_construct.yaml to determine the storage location of the data (octree distribution → octree_distribution.bin). For plane_localization, specify relative_path in params_construct.yaml to determine the storage location of the data (octree occupancy → octree_occupy.bin and global plane features → octree_planes.bin).
- **Recursive Localizer**: Different recursive localizers can be switched in the start_localization.sh script. Overall, context_localization is recommended (based on the open-source scan context, it is more robust and should adapt better to unstructured scenes). plane_localization provides global plane features, offering good potential for optimization and extension.
- **Localization Data Input**: Localization supports two localization_mode inputs: pure_point_cloud mode and pose_point_cloud (odometry) mode. In pure_point_cloud mode, only point clouds are needed as input for recursive localization. In pose_point_cloud mode, synchronized point clouds and odometry (provided by lio_front_end or custom sources) are required. Compared to pure_point_cloud, this mode better ensures localization success under aggressive motion.
- **Three Relocalization Modes**:
  - Via context_localization's `/context_global_localization` topic: when set to true, scan context is used for global localization (with octree-based multi-result verification); when false, the mapping origin is used as the initial localization value.
  - Via plane_localization's `/plane_global_localization` topic: when set to true, plane features are used for global localization (also with octree-based multi-result verification); when false, the mapping origin is used as the initial localization value.
  - Via the `/initialpose` topic: manually provide an initial localization value.
