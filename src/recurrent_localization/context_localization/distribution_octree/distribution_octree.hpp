#ifndef DISTRIBUTION_OCTREE_HPP_
#define DISTRIBUTION_OCTREE_HPP_

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
    ColorHandlerT;

typedef struct OctreePoint {
  float x;
  float y;
  float z;
} OctreePoint;

typedef struct OctreePointCloud {
  int r;
  int g;
  int b;
  std::vector<OctreePoint> octree_point_cloud;
} OctreePointCloud;

typedef struct OctreeNode {
  int depth = 1;
  int size = 0;
  bool occupy = false;
  Eigen::Vector3f center = Eigen::Vector3f(0, 0, 0);
  Eigen::Vector3f mean = Eigen::Vector3f(0, 0, 0);
  Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
  std::shared_ptr<OctreePointCloud> data = nullptr;
  std::vector<std::shared_ptr<OctreeNode>> octree_ptr =
      std::vector<std::shared_ptr<OctreeNode>>(8, nullptr);
} OctreeNode;

class DistributionOctree {
 public:
  DistributionOctree(float octree_range, float octree_resolution,
                     int vaild_leaf_node_size) {
    direction_vector_ = std::vector<std::vector<std::vector<int>>>(
        3, std::vector<std::vector<int>>(3, std::vector<int>(3, -1)));
    direction_vector_[0][2][2] = 0;
    direction_vector_[2][2][2] = 1;
    direction_vector_[0][0][2] = 2;
    direction_vector_[2][0][2] = 3;
    direction_vector_[0][2][0] = 4;
    direction_vector_[2][2][0] = 5;
    direction_vector_[0][0][0] = 6;
    direction_vector_[2][0][0] = 7;

    octree_range_ = octree_range;
    octree_resolution_ = octree_resolution;
    vaild_leaf_node_size_ = vaild_leaf_node_size;

    //数据初始化
    point_cloud_octree_data_ = std::make_shared<OctreeNode>();

    //确定八叉树所需要的深度
    octree_resolution_ = octree_resolution_ / 2.0;
    while ((octree_resolution_ * pow(2, octree_depth_)) < octree_range_) {
      octree_depth_++;
    }
    std::cout << "------------------------------------------" << std::endl;
    std::cout << "octree_range_: " << octree_range_ << std::endl;
    std::cout << "octree_resolution: " << octree_resolution_ << std::endl;
    std::cout << "vaild_leaf_node_size: " << vaild_leaf_node_size_ << std::endl;
    std::cout << "octree_depth: " << octree_depth_ << std::endl;
  }

  ~DistributionOctree() {}

  void ResetDistributionOctree() {
    //数据初始化
    point_cloud_octree_data_ = std::make_shared<OctreeNode>();
  }

  int GetOctreeDepth() { return octree_depth_; }

  bool QueryOccupancy(Eigen::Vector3f query_point, int depth) {
    std::shared_ptr<OctreeNode> point_cloud_octree_ptr =
        point_cloud_octree_data_;

    if (fabs(query_point[0]) >= octree_range_ ||
        fabs(query_point[1]) >= octree_range_ ||
        fabs(query_point[2]) >= octree_range_) {
      return false;
    }

    while (point_cloud_octree_ptr->depth < depth) {
      int direction_x =
          query_point[0] > point_cloud_octree_ptr->center[0] ? 1 : -1;
      int direction_y =
          query_point[1] > point_cloud_octree_ptr->center[1] ? 1 : -1;
      int direction_z =
          query_point[2] > point_cloud_octree_ptr->center[2] ? 1 : -1;
      int direction_index =
          direction_vector_[direction_x + 1][direction_y + 1][direction_z + 1];
      if (point_cloud_octree_ptr->octree_ptr[direction_index] != nullptr) {
        if (point_cloud_octree_ptr->octree_ptr[direction_index]->depth ==
            depth) {
          if (point_cloud_octree_ptr->octree_ptr[direction_index]->occupy ==
              true) {
            return true;
          } else {
            return false;
          }
        }
        //继续到更深的节点去
        point_cloud_octree_ptr =
            point_cloud_octree_ptr->octree_ptr[direction_index];
      } else {
        return false;
      }
    }
  }

  bool QueryDistribution(Eigen::Vector3f query_point, int depth,
                         Eigen::Vector3f &mean, Eigen::Matrix3f &covariance) {
    std::shared_ptr<OctreeNode> point_cloud_octree_ptr =
        point_cloud_octree_data_;

    if (fabs(query_point[0]) >= octree_range_ ||
        fabs(query_point[1]) >= octree_range_ ||
        fabs(query_point[2]) >= octree_range_) {
      return false;
    }

    while (point_cloud_octree_ptr->depth < depth) {
      int direction_x =
          query_point[0] > point_cloud_octree_ptr->center[0] ? 1 : -1;
      int direction_y =
          query_point[1] > point_cloud_octree_ptr->center[1] ? 1 : -1;
      int direction_z =
          query_point[2] > point_cloud_octree_ptr->center[2] ? 1 : -1;
      int direction_index =
          direction_vector_[direction_x + 1][direction_y + 1][direction_z + 1];
      if (point_cloud_octree_ptr->octree_ptr[direction_index] != nullptr) {
        if (point_cloud_octree_ptr->octree_ptr[direction_index]->depth ==
            depth) {
          if (point_cloud_octree_ptr->octree_ptr[direction_index]->size >
              vaild_leaf_node_size_) {
            mean = point_cloud_octree_ptr->octree_ptr[direction_index]->mean;
            covariance =
                point_cloud_octree_ptr->octree_ptr[direction_index]->covariance;
            return true;
          } else {
            return false;
          }
        }
        //继续到更深的节点去
        point_cloud_octree_ptr =
            point_cloud_octree_ptr->octree_ptr[direction_index];
      } else {
        return false;
      }
    }
  }

  int QueryOccupancys(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
                      int depth) {
    std::shared_ptr<OctreeNode> point_cloud_octree_ptr =
        point_cloud_octree_data_;

    int count_occupancy = 0;
    for (auto &point : *point_cloud) {
      if (fabs(point.x) >= octree_range_ || fabs(point.y) >= octree_range_ ||
          fabs(point.z) >= octree_range_) {
        continue;
      }
      point_cloud_octree_ptr = point_cloud_octree_data_;
      while (point_cloud_octree_ptr->depth < depth) {
        int direction_x = point.x > point_cloud_octree_ptr->center[0] ? 1 : -1;
        int direction_y = point.y > point_cloud_octree_ptr->center[1] ? 1 : -1;
        int direction_z = point.z > point_cloud_octree_ptr->center[2] ? 1 : -1;
        int direction_index =
            direction_vector_[direction_x + 1][direction_y + 1]
                             [direction_z + 1];
        if (point_cloud_octree_ptr->octree_ptr[direction_index] != nullptr) {
          if (point_cloud_octree_ptr->octree_ptr[direction_index]->depth ==
              depth) {
            if (point_cloud_octree_ptr->octree_ptr[direction_index]->occupy ==
                true) {
              count_occupancy++;
            }
          }
          //继续到更深的节点去
          point_cloud_octree_ptr =
              point_cloud_octree_ptr->octree_ptr[direction_index];
        } else {
          break;
        }
      }
    }
    return count_occupancy;
  }

  //添加点云到八叉树
  void OctreeDistributionAddPointCloud(
      pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud) {
    std::shared_ptr<OctreeNode> point_cloud_octree_ptr =
        point_cloud_octree_data_;

    //把每一个点加入八叉树的同时构建八叉树
    for (auto &point : *point_cloud) {
      if (fabs(point.x) >= octree_range_ || fabs(point.y) >= octree_range_ ||
          fabs(point.z) >= octree_range_) {
        continue;
      }

      point_cloud_octree_ptr = point_cloud_octree_data_;
      while (point_cloud_octree_ptr->depth <= octree_depth_) {
        int direction_x = point.x > point_cloud_octree_ptr->center[0] ? 1 : -1;
        int direction_y = point.y > point_cloud_octree_ptr->center[1] ? 1 : -1;
        int direction_z = point.z > point_cloud_octree_ptr->center[2] ? 1 : -1;
        int direction_index =
            direction_vector_[direction_x + 1][direction_y + 1]
                             [direction_z + 1];
        //如果还没有构建此节点那么就构建此节点
        if (point_cloud_octree_ptr->octree_ptr[direction_index] == nullptr) {
          point_cloud_octree_ptr->octree_ptr[direction_index] =
              std::make_shared<OctreeNode>();
          point_cloud_octree_ptr->octree_ptr[direction_index]->occupy = true;

          point_cloud_octree_ptr->octree_ptr[direction_index]->depth =
              point_cloud_octree_ptr->depth + 1;
          point_cloud_octree_ptr->octree_ptr[direction_index]->center[0] =
              point_cloud_octree_ptr->center[0] +
              (direction_x * octree_resolution_ *
               pow(2, octree_depth_ - point_cloud_octree_ptr->depth));
          point_cloud_octree_ptr->octree_ptr[direction_index]->center[1] =
              point_cloud_octree_ptr->center[1] +
              (direction_y * octree_resolution_ *
               pow(2, octree_depth_ - point_cloud_octree_ptr->depth));
          point_cloud_octree_ptr->octree_ptr[direction_index]->center[2] =
              point_cloud_octree_ptr->center[2] +
              (direction_z * octree_resolution_ *
               pow(2, octree_depth_ - point_cloud_octree_ptr->depth));

          //如果到了最深的深度也就是叶子节点那么就把点数据放入叶子节点
          if (point_cloud_octree_ptr->octree_ptr[direction_index]->depth ==
              (octree_depth_ + 1)) {
            //在这个深度上它的八个子节点是nullptr但是data是有数据的
            if (point_cloud_octree_ptr->octree_ptr[direction_index]->data ==
                nullptr) {
              point_cloud_octree_ptr->octree_ptr[direction_index]->data =
                  std::make_shared<OctreePointCloud>();
              point_cloud_octree_ptr->octree_ptr[direction_index]->data->r =
                  rand() % 255;
              point_cloud_octree_ptr->octree_ptr[direction_index]->data->g =
                  rand() % 255;
              point_cloud_octree_ptr->octree_ptr[direction_index]->data->b =
                  rand() % 255;
              OctreePoint odtree_point;
              odtree_point.x = point.x;
              odtree_point.y = point.y;
              odtree_point.z = point.z;
              point_cloud_octree_ptr->octree_ptr[direction_index]
                  ->data->octree_point_cloud.push_back(odtree_point);
            } else {
              //已经构造过叶子节点了，直接放入数据
              OctreePoint odtree_point;
              odtree_point.x = point.x;
              odtree_point.y = point.y;
              odtree_point.z = point.z;
              point_cloud_octree_ptr->octree_ptr[direction_index]
                  ->data->octree_point_cloud.push_back(odtree_point);
            }
          }

          //继续到更深的节点去
          point_cloud_octree_ptr =
              point_cloud_octree_ptr->octree_ptr[direction_index];
        } else {
          //已经构建过此节点了，判断它是不是叶子节点，是的话直接放入数据
          if (point_cloud_octree_ptr->octree_ptr[direction_index]->depth ==
              (octree_depth_ + 1)) {
            OctreePoint odtree_point;
            odtree_point.x = point.x;
            odtree_point.y = point.y;
            odtree_point.z = point.z;
            point_cloud_octree_ptr->octree_ptr[direction_index]
                ->data->octree_point_cloud.push_back(odtree_point);
          }

          //继续到更深的节点去
          point_cloud_octree_ptr =
              point_cloud_octree_ptr->octree_ptr[direction_index];
        }
      }
    }
  }

  //可视化八叉树的叶子节点占用情况
  void OctreeOccupyView(
      std::shared_ptr<OctreeNode> point_cloud_octree_ptr,
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_occupy, int depth) {
    if (point_cloud_octree_ptr->depth == depth &&
        point_cloud_octree_ptr->occupy == true) {
      pcl::PointXYZRGB point_xyzrgb;
      point_xyzrgb.x = point_cloud_octree_ptr->center[0];
      point_xyzrgb.y = point_cloud_octree_ptr->center[1];
      point_xyzrgb.z = point_cloud_octree_ptr->center[2];
      point_xyzrgb.r = rand() % 255;
      point_xyzrgb.g = rand() % 255;
      point_xyzrgb.b = rand() % 255;
      (*point_cloud_occupy).push_back(point_xyzrgb);
    }
    for (int i = 0; i < 8; i++) {
      if (point_cloud_octree_ptr->octree_ptr[i] != nullptr) {
        OctreeOccupyView(point_cloud_octree_ptr->octree_ptr[i],
                         point_cloud_occupy, depth);
      }
    }
  }

  //将占用八叉树存储为二进制文件
  void OctreeOccupyToBinary(std::shared_ptr<OctreeNode> point_cloud_octree_ptr,
                            std::vector<char> &octree_occupy_binary_buffer) {
    bool ptr_not_nullptr = true;
    char *ptr_not_nullptr_ptr = reinterpret_cast<char *>(&ptr_not_nullptr);
    octree_occupy_binary_buffer.insert(octree_occupy_binary_buffer.end(),
                                       ptr_not_nullptr_ptr,
                                       ptr_not_nullptr_ptr + sizeof(bool));
    int depth = point_cloud_octree_ptr->depth;
    char *depth_ptr = reinterpret_cast<char *>(&depth);
    octree_occupy_binary_buffer.insert(octree_occupy_binary_buffer.end(),
                                       depth_ptr, depth_ptr + sizeof(int));
    int size = point_cloud_octree_ptr->size;
    char *size_ptr = reinterpret_cast<char *>(&size);
    octree_occupy_binary_buffer.insert(octree_occupy_binary_buffer.end(),
                                       size_ptr, size_ptr + sizeof(int));
    bool occupy = point_cloud_octree_ptr->occupy;
    char *occupy_ptr = reinterpret_cast<char *>(&occupy);
    octree_occupy_binary_buffer.insert(octree_occupy_binary_buffer.end(),
                                       occupy_ptr, occupy_ptr + sizeof(bool));
    float center_x = point_cloud_octree_ptr->center[0];
    char *center_x_ptr = reinterpret_cast<char *>(&center_x);
    octree_occupy_binary_buffer.insert(octree_occupy_binary_buffer.end(),
                                       center_x_ptr,
                                       center_x_ptr + sizeof(float));
    float center_y = point_cloud_octree_ptr->center[1];
    char *center_y_ptr = reinterpret_cast<char *>(&center_y);
    octree_occupy_binary_buffer.insert(octree_occupy_binary_buffer.end(),
                                       center_y_ptr,
                                       center_y_ptr + sizeof(float));
    float center_z = point_cloud_octree_ptr->center[2];
    char *center_z_ptr = reinterpret_cast<char *>(&center_z);
    octree_occupy_binary_buffer.insert(octree_occupy_binary_buffer.end(),
                                       center_z_ptr,
                                       center_z_ptr + sizeof(float));
    float mean_x = point_cloud_octree_ptr->mean[0];
    char *mean_x_ptr = reinterpret_cast<char *>(&mean_x);
    octree_occupy_binary_buffer.insert(octree_occupy_binary_buffer.end(),
                                       mean_x_ptr, mean_x_ptr + sizeof(float));
    float mean_y = point_cloud_octree_ptr->mean[1];
    char *mean_y_ptr = reinterpret_cast<char *>(&mean_y);
    octree_occupy_binary_buffer.insert(octree_occupy_binary_buffer.end(),
                                       mean_y_ptr, mean_y_ptr + sizeof(float));
    float mean_z = point_cloud_octree_ptr->mean[2];
    char *mean_z_ptr = reinterpret_cast<char *>(&mean_z);
    octree_occupy_binary_buffer.insert(octree_occupy_binary_buffer.end(),
                                       mean_z_ptr, mean_z_ptr + sizeof(float));
    float covariance_xx = point_cloud_octree_ptr->covariance(0, 0);
    char *covariance_xx_ptr = reinterpret_cast<char *>(&covariance_xx);
    octree_occupy_binary_buffer.insert(octree_occupy_binary_buffer.end(),
                                       covariance_xx_ptr,
                                       covariance_xx_ptr + sizeof(float));
    float covariance_xy = point_cloud_octree_ptr->covariance(0, 1);
    char *covariance_xy_ptr = reinterpret_cast<char *>(&covariance_xy);
    octree_occupy_binary_buffer.insert(octree_occupy_binary_buffer.end(),
                                       covariance_xy_ptr,
                                       covariance_xy_ptr + sizeof(float));
    float covariance_xz = point_cloud_octree_ptr->covariance(0, 2);
    char *covariance_xz_ptr = reinterpret_cast<char *>(&covariance_xz);
    octree_occupy_binary_buffer.insert(octree_occupy_binary_buffer.end(),
                                       covariance_xz_ptr,
                                       covariance_xz_ptr + sizeof(float));
    float covariance_yx = point_cloud_octree_ptr->covariance(1, 0);
    char *covariance_yx_ptr = reinterpret_cast<char *>(&covariance_yx);
    octree_occupy_binary_buffer.insert(octree_occupy_binary_buffer.end(),
                                       covariance_yx_ptr,
                                       covariance_yx_ptr + sizeof(float));
    float covariance_yy = point_cloud_octree_ptr->covariance(1, 1);
    char *covariance_yy_ptr = reinterpret_cast<char *>(&covariance_yy);
    octree_occupy_binary_buffer.insert(octree_occupy_binary_buffer.end(),
                                       covariance_yy_ptr,
                                       covariance_yy_ptr + sizeof(float));
    float covariance_yz = point_cloud_octree_ptr->covariance(1, 2);
    char *covariance_yz_ptr = reinterpret_cast<char *>(&covariance_yz);
    octree_occupy_binary_buffer.insert(octree_occupy_binary_buffer.end(),
                                       covariance_yz_ptr,
                                       covariance_yz_ptr + sizeof(float));
    float covariance_zx = point_cloud_octree_ptr->covariance(2, 0);
    char *covariance_zx_ptr = reinterpret_cast<char *>(&covariance_zx);
    octree_occupy_binary_buffer.insert(octree_occupy_binary_buffer.end(),
                                       covariance_zx_ptr,
                                       covariance_zx_ptr + sizeof(float));
    float covariance_zy = point_cloud_octree_ptr->covariance(2, 1);
    char *covariance_zy_ptr = reinterpret_cast<char *>(&covariance_zy);
    octree_occupy_binary_buffer.insert(octree_occupy_binary_buffer.end(),
                                       covariance_zy_ptr,
                                       covariance_zy_ptr + sizeof(float));
    float covariance_zz = point_cloud_octree_ptr->covariance(2, 2);
    char *covariance_zz_ptr = reinterpret_cast<char *>(&covariance_zz);
    octree_occupy_binary_buffer.insert(octree_occupy_binary_buffer.end(),
                                       covariance_zz_ptr,
                                       covariance_zz_ptr + sizeof(float));

    for (int i = 0; i < 8; i++) {
      if (point_cloud_octree_ptr->octree_ptr[i] != nullptr) {
        OctreeOccupyToBinary(point_cloud_octree_ptr->octree_ptr[i],
                             octree_occupy_binary_buffer);
      } else {
        bool child_ptr_not_nullptr = false;
        char *child_ptr_not_nullptr_ptr =
            reinterpret_cast<char *>(&child_ptr_not_nullptr);
        octree_occupy_binary_buffer.insert(
            octree_occupy_binary_buffer.end(), child_ptr_not_nullptr_ptr,
            child_ptr_not_nullptr_ptr + sizeof(bool));
      }
    }
  }

  void SetOctreeOccupyBinaryPtr(char *octree_occupy_binary_ptr,
                                char *octree_occupy_binary_end_ptr) {
    octree_occupy_binary_ptr_ = octree_occupy_binary_ptr;
    octree_occupy_binary_end_ptr_ = octree_occupy_binary_end_ptr;
  }

  //将二进制文件解析为占用八叉树
  void BinaryToOctreeOccupy(
      std::shared_ptr<OctreeNode> point_cloud_octree_ptr) {
    if (octree_occupy_binary_ptr_ < octree_occupy_binary_end_ptr_) {
      int depth = *reinterpret_cast<int *>(octree_occupy_binary_ptr_);
      point_cloud_octree_ptr->depth = depth;
      octree_occupy_binary_ptr_ += sizeof(int);
      int size = *reinterpret_cast<int *>(octree_occupy_binary_ptr_);
      point_cloud_octree_ptr->size = size;
      octree_occupy_binary_ptr_ += sizeof(int);
      bool occupy = *reinterpret_cast<bool *>(octree_occupy_binary_ptr_);
      point_cloud_octree_ptr->occupy = occupy;
      octree_occupy_binary_ptr_ += sizeof(bool);
      float center_x = *reinterpret_cast<float *>(octree_occupy_binary_ptr_);
      point_cloud_octree_ptr->center[0] = center_x;
      octree_occupy_binary_ptr_ += sizeof(float);
      float center_y = *reinterpret_cast<float *>(octree_occupy_binary_ptr_);
      point_cloud_octree_ptr->center[1] = center_y;
      octree_occupy_binary_ptr_ += sizeof(float);
      float center_z = *reinterpret_cast<float *>(octree_occupy_binary_ptr_);
      point_cloud_octree_ptr->center[2] = center_z;
      octree_occupy_binary_ptr_ += sizeof(float);
      float mean_x = *reinterpret_cast<float *>(octree_occupy_binary_ptr_);
      point_cloud_octree_ptr->mean[0] = mean_x;
      octree_occupy_binary_ptr_ += sizeof(float);
      float mean_y = *reinterpret_cast<float *>(octree_occupy_binary_ptr_);
      point_cloud_octree_ptr->mean[1] = mean_y;
      octree_occupy_binary_ptr_ += sizeof(float);
      float mean_z = *reinterpret_cast<float *>(octree_occupy_binary_ptr_);
      point_cloud_octree_ptr->mean[2] = mean_z;
      octree_occupy_binary_ptr_ += sizeof(float);
      float covariance_xx =
          *reinterpret_cast<float *>(octree_occupy_binary_ptr_);
      point_cloud_octree_ptr->covariance(0, 0) = covariance_xx;
      octree_occupy_binary_ptr_ += sizeof(float);
      float covariance_xy =
          *reinterpret_cast<float *>(octree_occupy_binary_ptr_);
      point_cloud_octree_ptr->covariance(0, 1) = covariance_xy;
      octree_occupy_binary_ptr_ += sizeof(float);
      float covariance_xz =
          *reinterpret_cast<float *>(octree_occupy_binary_ptr_);
      point_cloud_octree_ptr->covariance(0, 2) = covariance_xz;
      octree_occupy_binary_ptr_ += sizeof(float);
      float covariance_yx =
          *reinterpret_cast<float *>(octree_occupy_binary_ptr_);
      point_cloud_octree_ptr->covariance(1, 0) = covariance_yx;
      octree_occupy_binary_ptr_ += sizeof(float);
      float covariance_yy =
          *reinterpret_cast<float *>(octree_occupy_binary_ptr_);
      point_cloud_octree_ptr->covariance(1, 1) = covariance_yy;
      octree_occupy_binary_ptr_ += sizeof(float);
      float covariance_yz =
          *reinterpret_cast<float *>(octree_occupy_binary_ptr_);
      point_cloud_octree_ptr->covariance(1, 2) = covariance_yz;
      octree_occupy_binary_ptr_ += sizeof(float);
      float covariance_zx =
          *reinterpret_cast<float *>(octree_occupy_binary_ptr_);
      point_cloud_octree_ptr->covariance(2, 0) = covariance_zx;
      octree_occupy_binary_ptr_ += sizeof(float);
      float covariance_zy =
          *reinterpret_cast<float *>(octree_occupy_binary_ptr_);
      point_cloud_octree_ptr->covariance(2, 1) = covariance_zy;
      octree_occupy_binary_ptr_ += sizeof(float);
      float covariance_zz =
          *reinterpret_cast<float *>(octree_occupy_binary_ptr_);
      point_cloud_octree_ptr->covariance(2, 2) = covariance_zz;
      octree_occupy_binary_ptr_ += sizeof(float);

      for (int i = 0; i < 8; ++i) {
        if (octree_occupy_binary_ptr_ < octree_occupy_binary_end_ptr_) {
          bool not_nullptr =
              *reinterpret_cast<bool *>(octree_occupy_binary_ptr_);
          octree_occupy_binary_ptr_ += sizeof(bool);
          if (not_nullptr) {
            point_cloud_octree_ptr->octree_ptr[i] =
                std::make_shared<OctreeNode>();
            BinaryToOctreeOccupy(point_cloud_octree_ptr->octree_ptr[i]);
          }
        }
      }
    }
  }

  //判断八叉树的叶子节点是否是平面
  void OctreeDistributionJudgment(
      std::shared_ptr<OctreeNode> point_cloud_octree_ptr) {
    if (point_cloud_octree_ptr->data != nullptr) {
      int left_node_point_cloud_size =
          point_cloud_octree_ptr->data->octree_point_cloud.size();
      if (left_node_point_cloud_size > vaild_leaf_node_size_) {
        Eigen::Vector3f centroid(0, 0, 0);
        for (int i = 0; i < left_node_point_cloud_size; i++) {
          Eigen::Vector3f point(
              point_cloud_octree_ptr->data->octree_point_cloud[i].x,
              point_cloud_octree_ptr->data->octree_point_cloud[i].y,
              point_cloud_octree_ptr->data->octree_point_cloud[i].z);
          centroid += point;
        }
        centroid /= left_node_point_cloud_size;

        Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
        for (int i = 0; i < left_node_point_cloud_size; i++) {
          Eigen::Vector3f point(
              point_cloud_octree_ptr->data->octree_point_cloud[i].x,
              point_cloud_octree_ptr->data->octree_point_cloud[i].y,
              point_cloud_octree_ptr->data->octree_point_cloud[i].z);
          Eigen::Vector3f centered = point - centroid;
          covariance += centered * centered.transpose();
        }
        covariance /= left_node_point_cloud_size;

        point_cloud_octree_ptr->size = left_node_point_cloud_size;
        point_cloud_octree_ptr->mean = centroid;
        point_cloud_octree_ptr->covariance = covariance;
      }
    }
    for (int i = 0; i < 8; i++) {
      if (point_cloud_octree_ptr->octree_ptr[i] != nullptr) {
        OctreeDistributionJudgment(point_cloud_octree_ptr->octree_ptr[i]);
      }
    }
  }

  //融合八叉树不同深度上的分布特性
  void OctreeDistributionFusion(
      std::shared_ptr<OctreeNode> point_cloud_octree_ptr, int fusion_depth) {
    if (point_cloud_octree_ptr->depth == fusion_depth) {
      int merged_size = 0;
      Eigen::Vector3f before_centered = Eigen::Vector3f(0, 0, 0);
      Eigen::Vector3f current_centered = Eigen::Vector3f(0, 0, 0);
      Eigen::Vector3f merged_mean = Eigen::Vector3f(0, 0, 0);
      Eigen::Matrix3f merged_covariance = Eigen::Matrix3f::Zero();
      for (int i = 0; i < 8; i++) {
        if (point_cloud_octree_ptr->octree_ptr[i] != nullptr) {
          //合并子节点的均值和方差
          if (point_cloud_octree_ptr->octree_ptr[i]->size > 0) {
            if (point_cloud_octree_ptr->size == 0) {
              point_cloud_octree_ptr->size =
                  point_cloud_octree_ptr->octree_ptr[i]->size;
              point_cloud_octree_ptr->mean =
                  point_cloud_octree_ptr->octree_ptr[i]->mean;
              point_cloud_octree_ptr->covariance =
                  point_cloud_octree_ptr->octree_ptr[i]->covariance;
            } else {
              merged_size = point_cloud_octree_ptr->size +
                            point_cloud_octree_ptr->octree_ptr[i]->size;
              merged_mean =
                  (point_cloud_octree_ptr->size * point_cloud_octree_ptr->mean +
                   point_cloud_octree_ptr->octree_ptr[i]->size *
                       point_cloud_octree_ptr->octree_ptr[i]->mean) /
                  merged_size;
              before_centered = point_cloud_octree_ptr->mean - merged_mean;
              current_centered =
                  point_cloud_octree_ptr->octree_ptr[i]->mean - merged_mean;
              merged_covariance =
                  (point_cloud_octree_ptr->size *
                       (point_cloud_octree_ptr->covariance +
                        before_centered * before_centered.transpose()) +
                   point_cloud_octree_ptr->octree_ptr[i]->size *
                       (point_cloud_octree_ptr->octree_ptr[i]->covariance +
                        current_centered * current_centered.transpose())) /
                  merged_size;
              point_cloud_octree_ptr->size = merged_size;
              point_cloud_octree_ptr->mean = merged_mean;
              point_cloud_octree_ptr->covariance = merged_covariance;
            }
          }
        }
      }
    }
    for (int i = 0; i < 8; i++) {
      if (point_cloud_octree_ptr->depth < fusion_depth &&
          point_cloud_octree_ptr->octree_ptr[i] != nullptr) {
        OctreeDistributionFusion(point_cloud_octree_ptr->octree_ptr[i],
                                 fusion_depth);
      }
    }
  }

  //提取八叉树的分布特性
  void OctreeDistributionExtraction() {
    std::shared_ptr<OctreeNode> point_cloud_octree_ptr =
        point_cloud_octree_data_;

    OctreeDistributionJudgment(point_cloud_octree_ptr);

    //在八叉树不同深度上进行分布特性的融合
    for (int depth = octree_depth_; depth > 0; depth--) {
      point_cloud_octree_ptr = point_cloud_octree_data_;
      OctreeDistributionFusion(point_cloud_octree_ptr, depth);
    }
  }

 public:
  std::shared_ptr<OctreeNode> point_cloud_octree_data_;

 private:
  //八叉树八个节点的索引
  std::vector<std::vector<std::vector<int>>> direction_vector_;

  char *octree_occupy_binary_ptr_;
  char *octree_occupy_binary_end_ptr_;

  float octree_range_ = 50.0;
  float octree_resolution_ = 0.5;  //八叉树的分辨率
  int octree_depth_ = 1;  //八叉树的初始深度，后续根据输入点云的跨度重新计算
  int vaild_leaf_node_size_ = 20;  //有效的叶子节点的点数量
};

#endif