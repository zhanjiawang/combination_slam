#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

#include <atomic>
#include <deque>
#include <fstream>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>

#include "CustomMsg.h"
#include "Scancontext.h"
#include "distribution_octree.hpp"
#include "se2.hpp"
#include "se3.hpp"

typedef struct DistYawIndex {
  int index;
  double dist;
  float yaw;
} DistYawIndex;

typedef struct TimedPointCloud {
  double time;
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud;
} TimedPointCloud;

typedef struct TimedPosedPointCloud {
  double time;
  Eigen::Matrix4f pose;
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud;
} TimedPosedPointCloud;

typedef message_filters::sync_policies::ApproximateTime<
    nav_msgs::Odometry, sensor_msgs::PointCloud2>
    SyncPosedPointCloudPol;

std::string work_mode_;
std::string program_path_;
std::string localization_mode_;
int cache_size_;
int localization_fps_;
std::string relative_path_;
float lidar_max_range_;
int ndt_max_iteration_;
int verify_number_;
float ndt_residual_outlier_threshold_;
int ndt_constraint_number_threshold_;
float ndt_convergence_condition_threshold_;

std::shared_ptr<message_filters::Synchronizer<SyncPosedPointCloudPol>>
    sync_posed_point_cloud_ = nullptr;
std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>>
    sub_front_end_odom_ = nullptr;
std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>>
    sub_front_end_cloud_ = nullptr;

ros::Subscriber sub_lidar_cloud_;
ros::Subscriber sub_context_global_localization_;
ros::Subscriber sub_initialpose_;

ros::Publisher pub_map_points_;
ros::Publisher pub_scan_points_;
ros::Publisher pub_point_cloud_occupy_map_;
ros::Publisher pub_transformation_matrix_points_;
ros::Publisher pub_localization_pose_;

std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

std::atomic<bool> init_localization_(false);
std::atomic<bool> stop_localization_(false);
std::mutex timed_point_clouds_mutex_;
std::mutex timed_posed_point_clouds_mutex_;
std::deque<TimedPointCloud> timed_point_clouds_;
std::deque<TimedPosedPointCloud> timed_posed_point_clouds_;

double previous_process_time_ = 0;
Eigen::Matrix4f guess_localization_pose_ = Eigen::Matrix4f::Identity();
Eigen::Matrix4f previous_odom_pose_ = Eigen::Matrix4f::Identity();

std::vector<Eigen::Matrix4f> pose_matrix_vector_;

int distribution_octree_occupy_map_depth_;
std::shared_ptr<DistributionOctree> distribution_octree_occupy_map_;

std::unique_ptr<SCManager> sc_manager_;

//读取参数
void ReadParam(ros::NodeHandle &nh) {
  nh.param<std::string>("context_localization/work_mode", work_mode_, "");
  std::cout << "work_mode: " << work_mode_ << std::endl;
  nh.param<std::string>("context_localization/relative_path", relative_path_,
                        "");
  std::cout << "relative_path: " << relative_path_ << std::endl;
  nh.param<std::string>("context_localization/localization_mode",
                        localization_mode_, "");
  std::cout << "localization_mode: " << localization_mode_ << std::endl;
  nh.param<int>("context_localization/cache_size", cache_size_, 10);
  std::cout << "cache_size: " << cache_size_ << std::endl;
  nh.param<int>("context_localization/localization_fps", localization_fps_, 10);
  std::cout << "localization_fps: " << localization_fps_ << std::endl;
  nh.param<float>("context_localization/lidar_max_range", lidar_max_range_,
                  50.0);
  std::cout << "lidar_max_range: " << lidar_max_range_ << std::endl;
  nh.param<int>("context_localization/verify_number", verify_number_, 10);
  std::cout << "verify_number: " << verify_number_ << std::endl;
  nh.param<int>("context_localization/ndt_max_iteration", ndt_max_iteration_,
                10);
  std::cout << "ndt_max_iteration: " << ndt_max_iteration_ << std::endl;
  nh.param<float>("context_localization/ndt_residual_outlier_threshold",
                  ndt_residual_outlier_threshold_, 20.0);
  std::cout << "ndt_residual_outlier_threshold: "
            << ndt_residual_outlier_threshold_ << std::endl;
  nh.param<int>("context_localization/ndt_constraint_number_threshold",
                ndt_constraint_number_threshold_, 30);
  std::cout << "ndt_constraint_number_threshold: "
            << ndt_constraint_number_threshold_ << std::endl;
  nh.param<float>("context_localization/ndt_convergence_condition_threshold",
                  ndt_convergence_condition_threshold_, 0.01);
  std::cout << "ndt_convergence_condition_threshold: "
            << ndt_convergence_condition_threshold_ << std::endl;
}

//读取构成地图的关键帧的位置，并估计八叉树地图的跨度
void ReadPose(std::string &path, float &range) {
  std::ifstream pose_file(path);
  if (pose_file.is_open()) {
    std::string line;
    while (getline(pose_file, line)) {
      if (line.length() > 6) {
        float x;
        float y;
        float z;
        float qw;
        float qx;
        float qy;
        float qz;
        std::stringstream string_stream(line);
        string_stream >> x >> y >> z >> qw >> qx >> qy >> qz;
        Eigen::Quaternionf rotation_quaternion;
        rotation_quaternion.w() = qw;
        rotation_quaternion.x() = qx;
        rotation_quaternion.y() = qy;
        rotation_quaternion.z() = qz;
        Eigen::Matrix3f rotation_matrix =
            rotation_quaternion.toRotationMatrix();
        Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();
        transformation_matrix(0, 0) = rotation_matrix(0, 0);
        transformation_matrix(0, 1) = rotation_matrix(0, 1);
        transformation_matrix(0, 2) = rotation_matrix(0, 2);
        transformation_matrix(1, 0) = rotation_matrix(1, 0);
        transformation_matrix(1, 1) = rotation_matrix(1, 1);
        transformation_matrix(1, 2) = rotation_matrix(1, 2);
        transformation_matrix(2, 0) = rotation_matrix(2, 0);
        transformation_matrix(2, 1) = rotation_matrix(2, 1);
        transformation_matrix(2, 2) = rotation_matrix(2, 2);
        transformation_matrix(0, 3) = x;
        transformation_matrix(1, 3) = y;
        transformation_matrix(2, 3) = z;
        pose_matrix_vector_.push_back(transformation_matrix);
      }
    }
    std::cout << "read pose file successful" << std::endl;
  } else {
    std::cout << "read pose file failed" << std::endl;
  }
  pose_file.close();

  float min_x = FLT_MAX;
  float max_x = -FLT_MAX;
  float min_y = FLT_MAX;
  float max_y = -FLT_MAX;
  float min_z = FLT_MAX;
  float max_z = -FLT_MAX;
  for (int i = 0; i < pose_matrix_vector_.size(); i++) {
    if (pose_matrix_vector_[i](0, 3) < min_x) {
      min_x = pose_matrix_vector_[i](0, 3);
    }
    if (pose_matrix_vector_[i](0, 3) > max_x) {
      max_x = pose_matrix_vector_[i](0, 3);
    }
    if (pose_matrix_vector_[i](1, 3) < min_y) {
      min_y = pose_matrix_vector_[i](1, 3);
    }
    if (pose_matrix_vector_[i](1, 3) > max_y) {
      max_y = pose_matrix_vector_[i](1, 3);
    }
    if (pose_matrix_vector_[i](2, 3) < min_z) {
      min_z = pose_matrix_vector_[i](2, 3);
    }
    if (pose_matrix_vector_[i](2, 3) > max_z) {
      max_z = pose_matrix_vector_[i](2, 3);
    }
  }
  float length = fabs(min_x) > fabs(max_x) ? fabs(min_x) : fabs(max_x);
  float width = fabs(min_y) > fabs(max_y) ? fabs(min_y) : fabs(max_y);
  float height = fabs(min_z) > fabs(max_z) ? fabs(min_z) : fabs(max_z);
  float length_width_max = length > width ? length : width;
  float length_width_height_max =
      length_width_max > height ? length_width_max : height;

  range = length_width_height_max + lidar_max_range_;
}

//四元数转欧拉角
Eigen::Vector3f Quaternion2Euler(const Eigen::Quaternionf &q) {
  Eigen::Vector3f euler;
  double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
  double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
  euler(0) = std::atan2(sinr_cosp, cosr_cosp);

  double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
  if (std::abs(sinp) >= 1)
    euler(1) = std::copysign(M_PI / 2, sinp);  // use 90 degrees if out of range
  else
    euler(1) = std::asin(sinp);

  double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
  double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
  euler(2) = std::atan2(siny_cosp, cosy_cosp);
  return euler;

  // 第二种方法
  // Eigen::Vector3f euler =
  //     q.toRotationMatrix().eulerAngles(2, 1, 0); // ZYX顺序
}

//欧拉角和平移向量转变化矩阵
void EulerTranslation2Matrix(float roll, float pitch, float yaw, float tx,
                             float ty, float tz,
                             Eigen::Matrix4f &transformation_matrix) {
  Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());

  Eigen::Matrix3f rotation =
      (yawAngle * pitchAngle * rollAngle).toRotationMatrix();

  transformation_matrix(0, 0) = rotation(0, 0);
  transformation_matrix(0, 1) = rotation(0, 1);
  transformation_matrix(0, 2) = rotation(0, 2);
  transformation_matrix(0, 3) = tx;
  transformation_matrix(1, 0) = rotation(1, 0);
  transformation_matrix(1, 1) = rotation(1, 1);
  transformation_matrix(1, 2) = rotation(1, 2);
  transformation_matrix(1, 3) = ty;
  transformation_matrix(2, 0) = rotation(2, 0);
  transformation_matrix(2, 1) = rotation(2, 1);
  transformation_matrix(2, 2) = rotation(2, 2);
  transformation_matrix(2, 3) = tz;
}

//点云地图可视化
void ViewMapPoints() {
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_map(
      new pcl::PointCloud<pcl::PointXYZ>);

  for (int i = 0; i < pose_matrix_vector_.size(); i++) {
    std::string scan_name = program_path_ + "/data/" + relative_path_ +
                            "/pcd/" + std::to_string(i) + ".pcd";
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(scan_name, *point_cloud) == -1) {
      PCL_ERROR("Couldn't read file \n");
      return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_filtered(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(point_cloud);
    voxel_grid.setLeafSize(0.5, 0.5, 0.5);
    voxel_grid.filter(*point_cloud_filtered);

    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_transformed(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*point_cloud_filtered, *point_cloud_transformed,
                             pose_matrix_vector_[i]);
    (*point_cloud_map) += (*point_cloud_transformed);
  }

  std::cout << "map size: " << (*point_cloud_map).size() << std::endl;

  sensor_msgs::PointCloud2 point_cloud_map_msg;
  point_cloud_map_msg.header.stamp = ros::Time::now();
  pcl::toROSMsg(*point_cloud_map, point_cloud_map_msg);
  point_cloud_map_msg.header.frame_id = "map";
  pub_map_points_.publish(point_cloud_map_msg);
}

void LidarLocalization() {
  float localization_period_s = 1.0 / localization_fps_;
  int localization_period_us = localization_period_s * 1000000;
  while (!stop_localization_) {
    //初始化成功才执行定位
    if (init_localization_) {
      Eigen::Matrix4f odom_pose = Eigen::Matrix4f::Identity();
      pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(
          new pcl::PointCloud<pcl::PointXYZ>);

      if (localization_mode_ == "pure_point_cloud") {
        std::unique_lock<std::mutex> lock(timed_point_clouds_mutex_);
        if (timed_point_clouds_.size() > 0) {
          //不处理已经处理过的
          if (timed_point_clouds_.front().time <= previous_process_time_) {
            lock.unlock();
            usleep(localization_period_us);
            continue;
          }
          previous_process_time_ = timed_point_clouds_.front().time;
          //深拷贝点云
          pcl::copyPointCloud(*timed_point_clouds_.front().point_cloud,
                              *point_cloud);
          lock.unlock();
        } else {
          lock.unlock();
          usleep(localization_period_us);
          continue;
        }
      } else if (localization_mode_ == "pose_point_cloud") {
        std::unique_lock<std::mutex> lock(timed_posed_point_clouds_mutex_);
        if (timed_posed_point_clouds_.size() > 0) {
          //不处理已经处理过的
          if (timed_posed_point_clouds_.front().time <=
              previous_process_time_) {
            lock.unlock();
            usleep(localization_period_us);
            continue;
          }
          previous_process_time_ = timed_posed_point_clouds_.front().time;
          odom_pose = timed_posed_point_clouds_.front().pose;
          //深拷贝点云
          pcl::copyPointCloud(*timed_posed_point_clouds_.front().point_cloud,
                              *point_cloud);
          lock.unlock();
        } else {
          lock.unlock();
          usleep(localization_period_us);
          continue;
        }
      } else {
        std::cout << "no support localization mode, return!" << std::endl;
        return;
      }

      ros::Time ros_stamp_time;
      ros_stamp_time.fromSec(previous_process_time_);

      if (localization_mode_ == "pose_point_cloud") {
        Eigen::Matrix4f relative_odom_pose =
            previous_odom_pose_.inverse() * odom_pose;
        guess_localization_pose_ =
            guess_localization_pose_ * relative_odom_pose;
        previous_odom_pose_ = odom_pose;
      }

      clock_t start_ndt_localization = clock();

      //执行NDT定位
      pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_filtered(
          new pcl::PointCloud<pcl::PointXYZ>);
      pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
      voxel_grid.setInputCloud(point_cloud);
      voxel_grid.setLeafSize(0.5, 0.5, 0.5);
      voxel_grid.filter(*point_cloud_filtered);

      //根据猜测位姿进行NDT位姿优化
      Sophus::SE3f iter_pose(guess_localization_pose_);
      Eigen::Vector3f q;
      int ndt_constraint_number = 0;
      for (int iter = 0; iter < ndt_max_iteration_; iter++) {
        Eigen::Matrix<float, 6, 6> H = Eigen::Matrix<float, 6, 6>::Zero();
        Eigen::Matrix<float, 6, 1> err = Eigen::Matrix<float, 6, 1>::Zero();

        for (auto &point : point_cloud_filtered->points) {
          q[0] = point.x;
          q[1] = point.y;
          q[2] = point.z;
          Eigen::Vector3f qs = iter_pose * q;

          Eigen::Vector3f query_point = Eigen::Vector3f(qs[0], qs[1], qs[2]);
          Eigen::Vector3f query_mean;
          Eigen::Matrix3f query_covariance;
          bool is_distribution =
              distribution_octree_occupy_map_->QueryDistribution(
                  query_point, distribution_octree_occupy_map_depth_ - 1,
                  query_mean, query_covariance);

          if (is_distribution) {
            Eigen::JacobiSVD<Eigen::MatrixXf> svd(
                query_covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
            Eigen::Vector3f lambda = svd.singularValues();
            if (lambda[1] < lambda[0] * 1e-3) {
              lambda[1] = lambda[0] * 1e-3;
            }
            if (lambda[2] < lambda[0] * 1e-3) {
              lambda[2] = lambda[0] * 1e-3;
            }
            Eigen::Matrix3f inv_lambda =
                Eigen::Vector3f(1.0 / lambda[0], 1.0 / lambda[1],
                                1.0 / lambda[2])
                    .asDiagonal();
            //信息矩阵是协方差矩阵的逆
            Eigen::Matrix3f information =
                svd.matrixV() * inv_lambda * svd.matrixU().transpose();

            Eigen::Vector3f e = qs - query_mean;

            float res = e.transpose() * information * e;
            if (std::isnan(res) || res > ndt_residual_outlier_threshold_) {
              continue;
            }

            // build residual
            Eigen::Matrix<float, 3, 6> J;
            J.block<3, 3>(0, 0) =
                -iter_pose.so3().matrix() * Sophus::SO3f::hat(q);
            J.block<3, 3>(0, 3) = Eigen::Matrix3f::Identity();

            H += (J.transpose() * information * J);
            err += (-J.transpose() * information * e);

            ndt_constraint_number++;
          }
        }

        if (ndt_constraint_number < ndt_constraint_number_threshold_) {
          break;
        }

        Eigen::Matrix<float, 6, 1> dx = H.inverse() * err;
        iter_pose.so3() = iter_pose.so3() * Sophus::SO3f::exp(dx.head<3>());
        iter_pose.translation() += dx.tail<3>();

        if (dx.norm() < ndt_convergence_condition_threshold_) {
          break;
        }
      }

      guess_localization_pose_ = iter_pose.matrix();
      pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_transformed(
          new pcl::PointCloud<pcl::PointXYZ>);
      pcl::transformPointCloud(*point_cloud_filtered, *point_cloud_transformed,
                               guess_localization_pose_);

      sensor_msgs::PointCloud2 point_cloud_scan_msg;
      point_cloud_scan_msg.header.stamp = ros_stamp_time;
      pcl::toROSMsg(*point_cloud_transformed, point_cloud_scan_msg);
      point_cloud_scan_msg.header.frame_id = "map";
      pub_scan_points_.publish(point_cloud_scan_msg);

      Eigen::Matrix3f guess_localization_pose_rotation =
          guess_localization_pose_.block<3, 3>(0, 0);
      Eigen::Vector3f guess_localization_pose_translation =
          guess_localization_pose_.block<3, 1>(0, 3);
      Eigen::Quaternionf guess_localization_pose_orientation(
          guess_localization_pose_rotation);

      geometry_msgs::TransformStamped localization_pose_pose_msg;
      localization_pose_pose_msg.header.stamp = ros_stamp_time;
      localization_pose_pose_msg.header.frame_id = "map";
      localization_pose_pose_msg.child_frame_id = "localization_base";
      localization_pose_pose_msg.transform.translation.x =
          guess_localization_pose_translation[0];
      localization_pose_pose_msg.transform.translation.y =
          guess_localization_pose_translation[1];
      localization_pose_pose_msg.transform.translation.z =
          guess_localization_pose_translation[2];
      localization_pose_pose_msg.transform.rotation.w =
          guess_localization_pose_orientation.w();
      localization_pose_pose_msg.transform.rotation.x =
          guess_localization_pose_orientation.x();
      localization_pose_pose_msg.transform.rotation.y =
          guess_localization_pose_orientation.y();
      localization_pose_pose_msg.transform.rotation.z =
          guess_localization_pose_orientation.z();
      tf_broadcaster_->sendTransform(localization_pose_pose_msg);

      nav_msgs::Odometry localization_pose_odom_msg;
      localization_pose_odom_msg.header.stamp = ros_stamp_time;
      localization_pose_odom_msg.header.frame_id = "map";
      localization_pose_odom_msg.child_frame_id = "localization_base";
      localization_pose_odom_msg.pose.pose.position.x =
          guess_localization_pose_translation[0];
      localization_pose_odom_msg.pose.pose.position.y =
          guess_localization_pose_translation[1];
      localization_pose_odom_msg.pose.pose.position.z =
          guess_localization_pose_translation[2];
      localization_pose_odom_msg.pose.pose.orientation.w =
          guess_localization_pose_orientation.w();
      localization_pose_odom_msg.pose.pose.orientation.x =
          guess_localization_pose_orientation.x();
      localization_pose_odom_msg.pose.pose.orientation.y =
          guess_localization_pose_orientation.y();
      localization_pose_odom_msg.pose.pose.orientation.z =
          guess_localization_pose_orientation.z();
      pub_localization_pose_.publish(localization_pose_odom_msg);

      clock_t end_ndt_localization = clock();
      double ndt_localization_time =
          ((double)(end_ndt_localization - start_ndt_localization) /
           (double)CLOCKS_PER_SEC);
      std::cout << "ndt_localization_time: " << ndt_localization_time
                << std::endl;

      if (ndt_localization_time < localization_period_s) {
        float rest_time_s = localization_period_s - ndt_localization_time;
        int rest_time_us = rest_time_s * 1000000;
        usleep(rest_time_us);
      }
    } else {
      usleep(localization_period_us);
    }
  }
}

//原始点云回调函数
void LidarCallback(const livox_ros_driver::CustomMsgConstPtr msg) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  for (unsigned int i = 0; i < msg->point_num; ++i) {
    pcl::PointXYZ point_xyz;
    point_xyz.x = msg->points[i].x;
    point_xyz.y = msg->points[i].y;
    point_xyz.z = msg->points[i].z;
    (*point_cloud).push_back(point_xyz);
  }

  TimedPointCloud timed_point_cloud;
  timed_point_cloud.time = msg->header.stamp.toSec();
  timed_point_cloud.point_cloud = point_cloud;

  {
    std::unique_lock<std::mutex> lock(timed_point_clouds_mutex_);
    if (timed_point_clouds_.size() < cache_size_) {
      timed_point_clouds_.push_front(timed_point_cloud);
    } else {
      timed_point_clouds_.pop_back();
      timed_point_clouds_.push_front(timed_point_cloud);
    }
    lock.unlock();
  }
}

//里程计的位姿点云回调函数
void PosedPointCloudCallback(
    const nav_msgs::Odometry::ConstPtr &odom_msg,
    const sensor_msgs::PointCloud2::ConstPtr &point_cloud_msg) {
  Eigen::Matrix4f odom_pose = Eigen::Matrix4f::Identity();
  Eigen::Vector3f odom_translation = Eigen::Vector3f(
      odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y,
      odom_msg->pose.pose.position.z);
  Eigen::Quaternionf odom_orientation(
      odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x,
      odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z);
  Eigen::Matrix3f odom_rotation = odom_orientation.toRotationMatrix();
  odom_pose.block<3, 3>(0, 0) = odom_rotation;
  odom_pose.block<3, 1>(0, 3) = odom_translation;
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*point_cloud_msg, *point_cloud);

  TimedPosedPointCloud timed_posed_point_cloud;
  timed_posed_point_cloud.time = point_cloud_msg->header.stamp.toSec();
  timed_posed_point_cloud.pose = odom_pose;
  timed_posed_point_cloud.point_cloud = point_cloud;

  {
    std::unique_lock<std::mutex> lock(timed_posed_point_clouds_mutex_);
    if (timed_posed_point_clouds_.size() < cache_size_) {
      timed_posed_point_clouds_.push_front(timed_posed_point_cloud);
    } else {
      timed_posed_point_clouds_.pop_back();
      timed_posed_point_clouds_.push_front(timed_posed_point_cloud);
    }
    lock.unlock();
  }
}

//平面重定位回调函数
void ContextGlobalLocalizationCallback(std_msgs::Bool::ConstPtr msg) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);

  if (localization_mode_ == "pure_point_cloud") {
    std::unique_lock<std::mutex> lock(timed_point_clouds_mutex_);
    if (timed_point_clouds_.size() > 0) {
      previous_process_time_ = timed_point_clouds_.front().time;
      //深拷贝点云
      pcl::copyPointCloud(*timed_point_clouds_.front().point_cloud,
                          *point_cloud);
      lock.unlock();
    } else {
      lock.unlock();
      std::cout << "no input point cloud, return!" << std::endl;
      return;
    }
  } else if (localization_mode_ == "pose_point_cloud") {
    std::unique_lock<std::mutex> lock(timed_posed_point_clouds_mutex_);
    if (timed_posed_point_clouds_.size() > 0) {
      previous_process_time_ = timed_posed_point_clouds_.front().time;
      previous_odom_pose_ = timed_posed_point_clouds_.front().pose;
      //深拷贝点云
      pcl::copyPointCloud(*timed_posed_point_clouds_.front().point_cloud,
                          *point_cloud);
      lock.unlock();
    } else {
      lock.unlock();
      std::cout << "no input point cloud, return!" << std::endl;
      return;
    }
  } else {
    std::cout << "no support localization mode, return!" << std::endl;
    return;
  }

  clock_t start_localization = clock();

  //如果为真使用scancontext进行重定位，如果为假使用建图原点默认值作为重定位值
  if (msg->data == true) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_filtered(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(point_cloud);
    voxel_grid.setLeafSize(0.4, 0.4, 0.4);
    voxel_grid.filter(*point_cloud_filtered);

    pcl::PointCloud<pcl::PointXYZI>::Ptr pointi_cloud_filtered(
        new pcl::PointCloud<pcl::PointXYZI>);
    pcl::copyPointCloud(*point_cloud_filtered, *pointi_cloud_filtered);
    sc_manager_->makeAndSaveScancontextAndKeys(*pointi_cloud_filtered);

    std::vector<DistYawIndex> dist_yaw_indexs;
    for (int i = 0; i < pose_matrix_vector_.size(); i++) {
      auto comapre_result = sc_manager_->compareCurrentAndIndexed(i);
      double candidate_dist = comapre_result.first;
      float candidate_yaw_diff_rad = comapre_result.second;
      DistYawIndex dist_yaw_index;
      dist_yaw_index.index = i;
      dist_yaw_index.dist = candidate_dist;
      dist_yaw_index.yaw = candidate_yaw_diff_rad;
      dist_yaw_indexs.push_back(dist_yaw_index);
    }

    //按得分由小到大进行排序
    std::sort(dist_yaw_indexs.begin(), dist_yaw_indexs.end(),
              [](const DistYawIndex &a, const DistYawIndex &b) {
                return a.dist < b.dist;
              });

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformation_matrix_points(
        new pcl::PointCloud<pcl::PointXYZRGB>);

    //精细验证，基于占用八叉树地图计算变换矩阵得分，选取最佳变换矩阵
    int best_transformation_index = -1;
    int best_transformation_score = 0;
    Eigen::Matrix4f best_transformation_matrix = Eigen::Matrix4f::Identity();
    verify_number_ = verify_number_ > (dist_yaw_indexs.size() - 1)
                         ? (dist_yaw_indexs.size() - 1)
                         : verify_number_;
    for (int i = 0; i < verify_number_; i++) {
      Eigen::Matrix4f candidate_localization_pose =
          pose_matrix_vector_[dist_yaw_indexs[i].index];
      Eigen::Matrix4f scancontext_yaw_transformation =
          Eigen::Matrix4f::Identity();
      EulerTranslation2Matrix(0.0, 0.0, dist_yaw_indexs[i].yaw, 0.0, 0.0, 0.0,
                              scancontext_yaw_transformation);
      candidate_localization_pose =
          candidate_localization_pose * scancontext_yaw_transformation;

      int r = rand() % 255;
      int g = rand() % 255;
      int b = rand() % 255;
      pcl::PointXYZRGB point_xyzrgb;
      point_xyzrgb.r = r;
      point_xyzrgb.g = g;
      point_xyzrgb.b = b;
      point_xyzrgb.x = candidate_localization_pose(0, 3);
      point_xyzrgb.y = candidate_localization_pose(1, 3);
      point_xyzrgb.z = candidate_localization_pose(2, 3);
      (*transformation_matrix_points).push_back(point_xyzrgb);

      pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_transformed(
          new pcl::PointCloud<pcl::PointXYZ>);
      pcl::transformPointCloud(*point_cloud_filtered, *point_cloud_transformed,
                               candidate_localization_pose);
      int transformation_score =
          distribution_octree_occupy_map_->QueryOccupancys(
              point_cloud_transformed,
              distribution_octree_occupy_map_depth_ + 1);
      if (transformation_score > best_transformation_score) {
        best_transformation_index = dist_yaw_indexs[i].index;
        best_transformation_score = transformation_score;
        best_transformation_matrix = candidate_localization_pose;
      }
    }

    sensor_msgs::PointCloud2 transformation_matrix_points_msg;
    transformation_matrix_points_msg.header.stamp = ros::Time::now();
    pcl::toROSMsg(*transformation_matrix_points,
                  transformation_matrix_points_msg);
    transformation_matrix_points_msg.header.frame_id = "map";
    pub_transformation_matrix_points_.publish(transformation_matrix_points_msg);

    if (best_transformation_index >= 0) {
      guess_localization_pose_ = best_transformation_matrix;
      init_localization_ = true;
    }
  } else {
    Eigen::Matrix4f best_transformation_matrix = Eigen::Matrix4f::Identity();
    guess_localization_pose_ = best_transformation_matrix;
    init_localization_ = true;
  }

  clock_t end_localization = clock();
  double localization_time = ((double)(end_localization - start_localization) /
                              (double)CLOCKS_PER_SEC);
  std::cout << "localization_time: " << localization_time << std::endl;
}

//外部初始定位回调函数
void InitPoseCallback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg) {
  if (localization_mode_ == "pure_point_cloud") {
    std::unique_lock<std::mutex> lock(timed_point_clouds_mutex_);
    if (timed_point_clouds_.size() > 0) {
      previous_process_time_ = timed_point_clouds_.front().time;
      lock.unlock();
    } else {
      lock.unlock();
      std::cout << "no input point cloud, return!" << std::endl;
      return;
    }
  } else if (localization_mode_ == "pose_point_cloud") {
    std::unique_lock<std::mutex> lock(timed_posed_point_clouds_mutex_);
    if (timed_posed_point_clouds_.size() > 0) {
      previous_process_time_ = timed_posed_point_clouds_.front().time;
      previous_odom_pose_ = timed_posed_point_clouds_.front().pose;
      lock.unlock();
    } else {
      lock.unlock();
      std::cout << "no input point cloud, return!" << std::endl;
      return;
    }
  } else {
    std::cout << "no support localization mode, return!" << std::endl;
    return;
  }

  Eigen::Matrix4f init_transformation_matrix = Eigen::Matrix4f::Identity();
  Eigen::Vector3f init_translation =
      Eigen::Vector3f(msg->pose.pose.position.x, msg->pose.pose.position.y,
                      msg->pose.pose.position.z);
  Eigen::Quaternionf init_orientation(
      msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
  Eigen::Matrix3f init_rotation = init_orientation.toRotationMatrix();
  init_transformation_matrix.block<3, 3>(0, 0) = init_rotation;
  init_transformation_matrix.block<3, 1>(0, 3) = init_translation;
  guess_localization_pose_ = init_transformation_matrix;
  init_localization_ = true;
}

//定义ROS订阅和发布
void RegisterSubscribeAndPublish(ros::NodeHandle &nh) {
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>();

  if (localization_mode_ == "pure_point_cloud") {
    //订阅点云话题，进行定位
    sub_lidar_cloud_ = nh.subscribe<livox_ros_driver::CustomMsg>(
        "/livox/lidar", 10, LidarCallback);
  } else if (localization_mode_ == "pose_point_cloud") {
    //订阅里程计的位姿点云话题，进行定位
    sub_front_end_odom_ =
        std::make_shared<message_filters::Subscriber<nav_msgs::Odometry>>(
            nh, "/front_end_odom", 10);
    sub_front_end_cloud_ =
        std::make_shared<message_filters::Subscriber<sensor_msgs::PointCloud2>>(
            nh, "/front_end_cloud", 10);
    sync_posed_point_cloud_ =
        std::make_shared<message_filters::Synchronizer<SyncPosedPointCloudPol>>(
            SyncPosedPointCloudPol(100), *sub_front_end_odom_,
            *sub_front_end_cloud_);
    sync_posed_point_cloud_->registerCallback(
        std::bind(&PosedPointCloudCallback, std::placeholders::_1,
                  std::placeholders::_2));
  }

  //订阅外部初始位姿重定位
  sub_initialpose_ = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
      "/initialpose", 10, InitPoseCallback);

  //订阅开启重定位的命令
  sub_context_global_localization_ = nh.subscribe<std_msgs::Bool>(
      "/context_global_localization", 10, ContextGlobalLocalizationCallback);

  pub_transformation_matrix_points_ = nh.advertise<sensor_msgs::PointCloud2>(
      "/transformation_matrix_points", 1, true);
  pub_point_cloud_occupy_map_ =
      nh.advertise<sensor_msgs::PointCloud2>("/point_cloud_occupy", 1, true);
  pub_map_points_ =
      nh.advertise<sensor_msgs::PointCloud2>("/map_points", 1, true);
  pub_scan_points_ =
      nh.advertise<sensor_msgs::PointCloud2>("/scan_points", 1, true);
  pub_localization_pose_ =
      nh.advertise<nav_msgs::Odometry>("/ndt_localization_pose", 1, true);
}

//使用八叉树构建地图
void ConstructOctreeMap(float map_range) {
  clock_t start_create_octree = clock();

  //八叉树占用地图，用于变化矩阵的验证。
  //构造函数的参数分别为:
  //八叉树地图的跨度、八叉树地图的分辨率、判断叶子节点为平面的最少点数量、判断叶子节点为平面的最小特征和最大特征值的比值、
  //判断叶子节点为平面的次大特征和最大特征值比值、平面融合时的法向量角度阈值、平面融合时一平面中心到另一平面的距离
  distribution_octree_occupy_map_ =
      std::make_shared<DistributionOctree>(map_range, 0.5, 20);

  for (int i = 0; i < pose_matrix_vector_.size(); i++) {
    std::string scan_name = program_path_ + "/data/" + relative_path_ +
                            "/pcd/" + std::to_string(i) + ".pcd";
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(scan_name, *point_cloud) == -1) {
      PCL_ERROR("Couldn't read file \n");
      return;
    }

    //构造八叉树占用地图
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_transformed(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*point_cloud, *point_cloud_transformed,
                             pose_matrix_vector_[i]);
    distribution_octree_occupy_map_->OctreeDistributionAddPointCloud(
        point_cloud_transformed);
  }
  //融合分布特性
  distribution_octree_occupy_map_->OctreeDistributionExtraction();

  //将八叉树占用地图转为二进制，存入硬盘
  std::string octree_occupy_binary_name =
      program_path_ + "/data/" + relative_path_ + "/octree_distribution.bin";
  std::vector<char> octree_occupy_binary_out_buffer;
  distribution_octree_occupy_map_->OctreeOccupyToBinary(
      distribution_octree_occupy_map_->point_cloud_octree_data_,
      octree_occupy_binary_out_buffer);
  // 将缓冲区写入文件
  std::ofstream octree_occupy_binary_out(octree_occupy_binary_name,
                                         std::ios::binary);
  if (octree_occupy_binary_out) {
    // 直接写入vector的连续内存块
    octree_occupy_binary_out.write(
        reinterpret_cast<char *>(octree_occupy_binary_out_buffer.data()),
        static_cast<std::streamsize>(octree_occupy_binary_out_buffer.size()));

    // 检查是否写入成功
    if (!octree_occupy_binary_out.good()) {
      std::cerr << "unable to write file!" << std::endl;
      return;
    }
    octree_occupy_binary_out.close();
  } else {
    std::cerr << "unable to open file!" << std::endl;
    return;
  }

  //打印耗时
  clock_t end_create_octree = clock();
  double create_octree_time =
      ((double)(end_create_octree - start_create_octree) /
       (double)CLOCKS_PER_SEC);
  std::cout << "create_octree_time: " << create_octree_time << std::endl;

  //八叉树占用地图可视化，发布ros话题
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_occupy_map(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  distribution_octree_occupy_map_depth_ =
      distribution_octree_occupy_map_->GetOctreeDepth();
  distribution_octree_occupy_map_->OctreeOccupyView(
      distribution_octree_occupy_map_->point_cloud_octree_data_,
      point_cloud_occupy_map, distribution_octree_occupy_map_depth_ + 1);

  sensor_msgs::PointCloud2 point_cloud_occupy_map_msg;
  point_cloud_occupy_map_msg.header.stamp = ros::Time::now();
  pcl::toROSMsg(*point_cloud_occupy_map, point_cloud_occupy_map_msg);
  point_cloud_occupy_map_msg.header.frame_id = "map";
  pub_point_cloud_occupy_map_.publish(point_cloud_occupy_map_msg);
}

//读取存储在硬盘的二进制的已经构建好的八叉树地图
void ReadOctreeMap(float map_range) {
  clock_t start_read_octree = clock();

  distribution_octree_occupy_map_ =
      std::make_shared<DistributionOctree>(map_range, 0.5, 20);

  //读取存储在硬盘的二进制占用八叉树地图
  std::string octree_occupy_binary_name =
      program_path_ + "/data/" + relative_path_ + "/octree_distribution.bin";
  // 打开二进制文件
  std::ifstream octree_occupy_binary_in(octree_occupy_binary_name,
                                        std::ios::binary | std::ios::ate);
  if (!octree_occupy_binary_in.is_open()) {
    std::cerr << "unable to open file!" << std::endl;
    return;
  }
  // 获取文件大小并分配缓冲区
  std::streamsize octree_occupy_binary_in_size =
      octree_occupy_binary_in.tellg();
  octree_occupy_binary_in.seekg(0, std::ios::beg);
  // 使用 vector 作为动态缓冲区
  std::vector<char> octree_occupy_binary_in_buffer(
      octree_occupy_binary_in_size);
  // 读取整个文件到缓冲区
  if (!octree_occupy_binary_in.read(octree_occupy_binary_in_buffer.data(),
                                    octree_occupy_binary_in_size)) {
    std::cerr << "failed to read file!" << std::endl;
    return;
  }
  octree_occupy_binary_in.close();
  char *octree_occupy_binary_ptr = octree_occupy_binary_in_buffer.data();
  char *octree_occupy_binary_end_ptr =
      octree_occupy_binary_in_buffer.data() + octree_occupy_binary_in_size;
  octree_occupy_binary_ptr += sizeof(bool);
  distribution_octree_occupy_map_->SetOctreeOccupyBinaryPtr(
      octree_occupy_binary_ptr, octree_occupy_binary_end_ptr);
  distribution_octree_occupy_map_->BinaryToOctreeOccupy(
      distribution_octree_occupy_map_->point_cloud_octree_data_);

  //打印耗时
  clock_t end_read_octree = clock();
  double read_octree_time =
      ((double)(end_read_octree - start_read_octree) / (double)CLOCKS_PER_SEC);
  std::cout << "read_octree_time: " << read_octree_time << std::endl;

  //八叉树占用地图可视化，发布ros话题
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_occupy_map(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  distribution_octree_occupy_map_depth_ =
      distribution_octree_occupy_map_->GetOctreeDepth();
  distribution_octree_occupy_map_->OctreeOccupyView(
      distribution_octree_occupy_map_->point_cloud_octree_data_,
      point_cloud_occupy_map, distribution_octree_occupy_map_depth_ + 1);

  sensor_msgs::PointCloud2 point_cloud_occupy_map_msg;
  point_cloud_occupy_map_msg.header.stamp = ros::Time::now();
  pcl::toROSMsg(*point_cloud_occupy_map, point_cloud_occupy_map_msg);
  point_cloud_occupy_map_msg.header.frame_id = "map";
  pub_point_cloud_occupy_map_.publish(point_cloud_occupy_map_msg);
}

//读取存储在硬盘的二进制的scancontext描述符
void ReadScanContext() {
  std::string sc_name = program_path_ + "/data/" + relative_path_ + "/sc.bin";
  bool sc_read = sc_manager_->loadPolarContexts(sc_name);
  if (!sc_read) {
    std::cout << "Reading scancontex failed" << std::endl;
  }
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "context_localization");
  ros::NodeHandle nh;

  //确定工作空间
  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "-program_path" && i + 1 < argc) {
      program_path_ = std::string(argv[i + 1]);
      program_path_ = program_path_.substr(0, program_path_.rfind('/'));
      program_path_ = program_path_.substr(0, program_path_.rfind('/'));
      program_path_ = program_path_.substr(0, program_path_.rfind('/'));
      break;
    }
  }
  std::cout << "program_path: " << program_path_ << std::endl;

  //读参数
  ReadParam(nh);

  //定义ROS订阅和发布
  RegisterSubscribeAndPublish(nh);

  //读位姿数据，同时估计八叉树地图的跨度
  std::string pose_name =
      program_path_ + "/data/" + relative_path_ + "/pose.json";
  std::cout << "pose_name: " << pose_name << std::endl;
  float map_range;
  ReadPose(pose_name, map_range);
  std::cout << "map_range: " << map_range << std::endl;

  //点云地图可视化
  ViewMapPoints();

  if (work_mode_ == "construct") {
    //八叉树地图只需要构建一次，之后直接读取即可
    //构建八叉树地图
    ConstructOctreeMap(map_range);
  } else if (work_mode_ == "read") {
    //读取已经构建好的八叉树地图
    ReadOctreeMap(map_range);
    //读取scancontext描述符
    sc_manager_ = std::make_unique<SCManager>();
    ReadScanContext();
  } else {
    return -1;
  }

  std::thread lidar_localization_thread(LidarLocalization);

  ros::spin();

  stop_localization_ = true;
  lidar_localization_thread.join();

  return 0;
}
