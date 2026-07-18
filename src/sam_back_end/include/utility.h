#pragma once
#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_
#define PCL_NO_PRECOMPILE

// 修复：FLANN 序列化缺少 unordered_map 支持（flann::lsh::LshTable
// 内部使用了它）
#include <unordered_map>
namespace flann {
namespace serialization {
// 前向声明，Serializer 主模板定义在 flann 的 serialization.h 中
template <typename T>
struct Serializer;

template <typename K, typename V>
struct Serializer<std::unordered_map<K, V>> {
  template <typename InputArchive>
  static inline void load(InputArchive &ar, std::unordered_map<K, V> &map_val) {
    size_t size;
    ar &size;
    map_val.clear();
    map_val.reserve(size);
    for (size_t i = 0; i < size; ++i) {
      K key;
      ar &key;
      V value;
      ar &value;
      map_val.emplace(std::move(key), std::move(value));
    }
  }
  template <typename OutputArchive>
  static inline void save(OutputArchive &ar,
                          const std::unordered_map<K, V> &map_val) {
    ar &map_val.size();
    for (const auto &pair : map_val) {
      ar &pair.first;
      ar &pair.second;
    }
  }
};
}  // namespace serialization
}  // namespace flann

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/console/time.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/package.h>  // get package_path
#include <ros/ros.h>
#include <rosbag/bag.h>  // save map
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Header.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <ctime>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <pcl/search/impl/search.hpp>
#include <queue>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "GeographicLib/LocalCartesian.hpp"
#include "geometry_msgs/QuaternionStamped.h"
#include "sensor_msgs/NavSatFix.h"

using namespace std;

typedef pcl::PointXYZI PointType;

typedef message_filters::sync_policies::ApproximateTime<
    nav_msgs::Odometry, sensor_msgs::PointCloud2>
    syncOdomCloudPol;
typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::NavSatFix, geometry_msgs::QuaternionStamped>
    syncFixHeadingPol;

enum class SensorType { VELODYNE, OUSTER, LIVOX };

class ParamServer {
 public:
  ros::NodeHandle nh;

  bool gnssInited = false;
  GeographicLib::LocalCartesian gnssGeo;
  Eigen::Quaternionf axisTransform;

  std::string robot_id;

  // Topics
  string pointCloudTopic;
  string imuTopic;
  string odomTopic;
  string gpsTopic;

  // Frames
  string lidarFrame;
  string baselinkFrame;
  string odometryFrame;
  string mapFrame;

  // GPS Settings
  bool useImuHeadingInitialization;
  bool useGpsElevation;
  float gpsCovThreshold;
  float poseCovThreshold;

  // Save pcd
  bool savePCD;
  string savePCDDirectory;

  SensorType sensor;

  // IMU
  float imuAccNoise;
  float imuGyrNoise;
  float imuAccBiasN;
  float imuGyrBiasN;
  float imuGravity;
  float imuRPYWeight;
  vector<double> extRotV;
  vector<double> extRPYV;
  vector<double> extTransV;
  Eigen::Matrix3d extRot;
  Eigen::Matrix3d extRPY;
  Eigen::Vector3d extTrans;
  Eigen::Quaterniond extQRPY;

  // voxel filter paprams
  float odometrySurfLeafSize;
  float mappingCornerLeafSize;
  float mappingSurfLeafSize;

  float z_tollerance;
  float rotation_tollerance;

  // CPU Params
  int numberOfCores;
  double mappingProcessInterval;

  // Surrounding map
  float surroundingkeyframeAddingDistThreshold;
  float surroundingkeyframeAddingAngleThreshold;
  float surroundingKeyframeDensity;
  float surroundingKeyframeSearchRadius;

  // Loop closure
  bool loopClosureEnableFlag;
  float loopClosureFrequency;
  int surroundingKeyframeSize;
  float historyKeyframeSearchRadius;
  float historyKeyframeSearchTimeDiff;
  int historyKeyframeSearchNum;
  float historyKeyframeFitnessScore;

  // global map visualization radius
  float globalMapVisualizationSearchRadius;
  float globalMapVisualizationPoseDensity;
  float globalMapVisualizationLeafSize;

  ParamServer() {
    nh.param<std::string>("/robot_id", robot_id, "robot");

    nh.param<std::string>("sam_back_end/pointCloudTopic", pointCloudTopic,
                          "points_raw");
    nh.param<std::string>("sam_back_end/imuTopic", imuTopic, "imu_correct");
    nh.param<std::string>("sam_back_end/odomTopic", odomTopic, "odometry/imu");
    nh.param<std::string>("sam_back_end/gpsTopic", gpsTopic, "odometry/gps");

    nh.param<std::string>("sam_back_end/lidarFrame", lidarFrame, "base_link");
    nh.param<std::string>("sam_back_end/baselinkFrame", baselinkFrame,
                          "base_link");
    nh.param<std::string>("sam_back_end/odometryFrame", odometryFrame, "odom");
    nh.param<std::string>("sam_back_end/mapFrame", mapFrame, "map");

    nh.param<bool>("sam_back_end/useImuHeadingInitialization",
                   useImuHeadingInitialization, false);
    nh.param<bool>("sam_back_end/useGpsElevation", useGpsElevation, false);
    nh.param<float>("sam_back_end/gpsCovThreshold", gpsCovThreshold, 2.0);
    nh.param<float>("sam_back_end/poseCovThreshold", poseCovThreshold, 25.0);

    nh.param<bool>("sam_back_end/savePCD", savePCD, false);
    nh.param<std::string>("sam_back_end/savePCDDirectory", savePCDDirectory,
                          "scene");

    std::string sensorStr;
    nh.param<std::string>("sam_back_end/sensor", sensorStr, "");
    if (sensorStr == "velodyne") {
      sensor = SensorType::VELODYNE;
    } else if (sensorStr == "ouster") {
      sensor = SensorType::OUSTER;
    } else if (sensorStr == "livox") {
      sensor = SensorType::LIVOX;
    } else {
      ROS_ERROR_STREAM(
          "Invalid sensor type (must be either 'velodyne' or 'ouster' or "
          "'livox'): "
          << sensorStr);
      ros::shutdown();
    }

    nh.param<float>("sam_back_end/imuAccNoise", imuAccNoise, 0.01);
    nh.param<float>("sam_back_end/imuGyrNoise", imuGyrNoise, 0.001);
    nh.param<float>("sam_back_end/imuAccBiasN", imuAccBiasN, 0.0002);
    nh.param<float>("sam_back_end/imuGyrBiasN", imuGyrBiasN, 0.00003);
    nh.param<float>("sam_back_end/imuGravity", imuGravity, 9.80511);
    nh.param<float>("sam_back_end/imuRPYWeight", imuRPYWeight, 0.01);
    nh.param<vector<double>>("sam_back_end/extrinsicRot", extRotV,
                             vector<double>());
    nh.param<vector<double>>("sam_back_end/extrinsicRPY", extRPYV,
                             vector<double>());
    nh.param<vector<double>>("sam_back_end/extrinsicTrans", extTransV,
                             vector<double>());
    extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(
        extRotV.data(), 3, 3);
    extRPY = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(
        extRPYV.data(), 3, 3);
    extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(
        extTransV.data(), 3, 1);
    extQRPY = Eigen::Quaterniond(extRPY).inverse();

    nh.param<float>("sam_back_end/odometrySurfLeafSize", odometrySurfLeafSize,
                    0.2);
    nh.param<float>("sam_back_end/mappingCornerLeafSize", mappingCornerLeafSize,
                    0.2);
    nh.param<float>("sam_back_end/mappingSurfLeafSize", mappingSurfLeafSize,
                    0.2);

    nh.param<float>("sam_back_end/z_tollerance", z_tollerance, FLT_MAX);
    nh.param<float>("sam_back_end/rotation_tollerance", rotation_tollerance,
                    FLT_MAX);

    nh.param<int>("sam_back_end/numberOfCores", numberOfCores, 2);
    nh.param<double>("sam_back_end/mappingProcessInterval",
                     mappingProcessInterval, 0.15);

    nh.param<float>("sam_back_end/surroundingkeyframeAddingDistThreshold",
                    surroundingkeyframeAddingDistThreshold, 1.0);
    nh.param<float>("sam_back_end/surroundingkeyframeAddingAngleThreshold",
                    surroundingkeyframeAddingAngleThreshold, 0.2);
    nh.param<float>("sam_back_end/surroundingKeyframeDensity",
                    surroundingKeyframeDensity, 1.0);
    nh.param<float>("sam_back_end/surroundingKeyframeSearchRadius",
                    surroundingKeyframeSearchRadius, 50.0);

    nh.param<bool>("sam_back_end/loopClosureEnableFlag", loopClosureEnableFlag,
                   false);
    nh.param<float>("sam_back_end/loopClosureFrequency", loopClosureFrequency,
                    1.0);
    nh.param<int>("sam_back_end/surroundingKeyframeSize",
                  surroundingKeyframeSize, 50);
    nh.param<float>("sam_back_end/historyKeyframeSearchRadius",
                    historyKeyframeSearchRadius, 10.0);
    nh.param<float>("sam_back_end/historyKeyframeSearchTimeDiff",
                    historyKeyframeSearchTimeDiff, 30.0);
    nh.param<int>("sam_back_end/historyKeyframeSearchNum",
                  historyKeyframeSearchNum, 25);
    nh.param<float>("sam_back_end/historyKeyframeFitnessScore",
                    historyKeyframeFitnessScore, 0.3);

    nh.param<float>("sam_back_end/globalMapVisualizationSearchRadius",
                    globalMapVisualizationSearchRadius, 1e3);
    nh.param<float>("sam_back_end/globalMapVisualizationPoseDensity",
                    globalMapVisualizationPoseDensity, 10.0);
    nh.param<float>("sam_back_end/globalMapVisualizationLeafSize",
                    globalMapVisualizationLeafSize, 1.0);

    usleep(100);
  }

  sensor_msgs::Imu imuConverter(const sensor_msgs::Imu &imu_in) {
    sensor_msgs::Imu imu_out = imu_in;
    // rotate acceleration
    Eigen::Vector3d acc(imu_in.linear_acceleration.x,
                        imu_in.linear_acceleration.y,
                        imu_in.linear_acceleration.z);
    acc = extRot * acc;
    imu_out.linear_acceleration.x = acc.x();
    imu_out.linear_acceleration.y = acc.y();
    imu_out.linear_acceleration.z = acc.z();
    // rotate gyroscope
    Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y,
                        imu_in.angular_velocity.z);
    gyr = extRot * gyr;
    imu_out.angular_velocity.x = gyr.x();
    imu_out.angular_velocity.y = gyr.y();
    imu_out.angular_velocity.z = gyr.z();
    // rotate roll pitch yaw
    Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x,
                              imu_in.orientation.y, imu_in.orientation.z);
    Eigen::Quaterniond q_final = q_from * extQRPY;
    imu_out.orientation.x = q_final.x();
    imu_out.orientation.y = q_final.y();
    imu_out.orientation.z = q_final.z();
    imu_out.orientation.w = q_final.w();

    if (sqrt(q_final.x() * q_final.x() + q_final.y() * q_final.y() +
             q_final.z() * q_final.z() + q_final.w() * q_final.w()) < 0.1) {
      ROS_ERROR("Invalid quaternion, please use a 9-axis IMU!");
      ros::shutdown();
    }

    return imu_out;
  }
};

template <typename T>
sensor_msgs::PointCloud2 publishCloud(const ros::Publisher &thisPub,
                                      const T &thisCloud, ros::Time thisStamp,
                                      std::string thisFrame) {
  sensor_msgs::PointCloud2 tempCloud;
  pcl::toROSMsg(*thisCloud, tempCloud);
  tempCloud.header.stamp = thisStamp;
  tempCloud.header.frame_id = thisFrame;
  if (thisPub.getNumSubscribers() != 0) thisPub.publish(tempCloud);
  return tempCloud;
}

template <typename T>
double ROS_TIME(T msg) {
  return msg->header.stamp.toSec();
}

template <typename T>
void imuAngular2rosAngular(sensor_msgs::Imu *thisImuMsg, T *angular_x,
                           T *angular_y, T *angular_z) {
  *angular_x = thisImuMsg->angular_velocity.x;
  *angular_y = thisImuMsg->angular_velocity.y;
  *angular_z = thisImuMsg->angular_velocity.z;
}

template <typename T>
void imuAccel2rosAccel(sensor_msgs::Imu *thisImuMsg, T *acc_x, T *acc_y,
                       T *acc_z) {
  *acc_x = thisImuMsg->linear_acceleration.x;
  *acc_y = thisImuMsg->linear_acceleration.y;
  *acc_z = thisImuMsg->linear_acceleration.z;
}

template <typename T>
void imuRPY2rosRPY(sensor_msgs::Imu *thisImuMsg, T *rosRoll, T *rosPitch,
                   T *rosYaw) {
  double imuRoll, imuPitch, imuYaw;
  tf::Quaternion orientation;
  tf::quaternionMsgToTF(thisImuMsg->orientation, orientation);
  tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);

  *rosRoll = imuRoll;
  *rosPitch = imuPitch;
  *rosYaw = imuYaw;
}

float pointDistance(PointType p) {
  return sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

float pointDistance(PointType p1, PointType p2) {
  return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) +
              (p1.z - p2.z) * (p1.z - p2.z));
}

#endif
