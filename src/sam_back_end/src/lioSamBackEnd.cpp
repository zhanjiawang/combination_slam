#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include "Scancontext.h"
#include "sam_back_end/save_map.h"
#include "utility.h"

std::string m_package_path;
std::string program_path;

float normal_radius = 2.0;
float feature_radius = 5.0;
float noise_bound = 0.2;
float voxel_grid_size = 0.5;

using namespace gtsam;

using symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::G;  // GPS pose
using symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

/*
 * A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is
 * time stamp)
 */
struct PointXYZIRPYT {
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY;  // preferred way of adding a XYZ+padding
  float roll;
  float pitch;
  float yaw;
  double time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
} EIGEN_ALIGN16;  // enforce SSE padding for correct memory alignment

// giseop
enum class SCInputType { SINGLE_SCAN_FULL, SINGLE_SCAN_FEAT, MULTI_SCAN_FEAT };

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZIRPYT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
        float, roll, roll)(float, pitch, pitch)(float, yaw, yaw)(double, time,
                                                                 time))

typedef PointXYZIRPYT PointTypePose;

class mapOptimization : public ParamServer {
 public:
  // gtsam
  NonlinearFactorGraph gtSAMgraph;
  Values initialEstimate;
  Values optimizedEstimate;
  ISAM2 *isam;
  Values isamCurrentEstimate;
  Eigen::MatrixXd poseCovariance;

  ros::Publisher pubLaserCloudSurround;
  ros::Publisher pubLaserOdometryGlobal;
  ros::Publisher pubKeyPoses;
  ros::Publisher pubPath;
  ros::Publisher pubGnssPose;

  ros::Publisher pubHistoryKeyFrames;
  ros::Publisher pubIcpKeyFrames;
  ros::Publisher pubRecentKeyFrame;
  ros::Publisher pubLoopConstraintEdge;

  shared_ptr<message_filters::Synchronizer<syncOdomCloudPol>> syncSubOdomCloud =
      nullptr;
  shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> subOdom = nullptr;
  shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> subCloud =
      nullptr;

  shared_ptr<message_filters::Synchronizer<syncFixHeadingPol>>
      syncSubFixHeading = nullptr;
  shared_ptr<message_filters::Subscriber<sensor_msgs::NavSatFix>> subFix =
      nullptr;
  shared_ptr<message_filters::Subscriber<geometry_msgs::QuaternionStamped>>
      subHeading = nullptr;

  ros::Subscriber subGPS;
  ros::Subscriber subLoop;

  ros::ServiceServer srvSaveMap;

  std::deque<nav_msgs::Odometry> gpsQueue;

  vector<pcl::PointCloud<PointType>::Ptr> fullCloudKeyFrames;

  vector<pcl::PointCloud<PointType>::Ptr> rawCloudKeyFrames;

  pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
  pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;
  pcl::PointCloud<PointTypePose>::Ptr cloudKeyOdom6D;
  pcl::PointCloud<PointType>::Ptr copy_cloudKeyPoses3D;
  pcl::PointCloud<PointTypePose>::Ptr copy_cloudKeyPoses6D;

  pcl::PointCloud<PointType>::Ptr laserCloudFullLast;
  pcl::PointCloud<PointType>::Ptr laserCloudFullLastDS;

  pcl::PointCloud<PointType>::Ptr laserCloudOri;
  pcl::PointCloud<PointType>::Ptr coeffSel;

  pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudGnssPose;

  map<int, pair<pcl::PointCloud<PointType>, pcl::PointCloud<PointType>>>
      laserCloudMapContainer;

  pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingKeyPoses;
  pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses;

  pcl::VoxelGrid<PointType> downSizeFilterFull;
  pcl::VoxelGrid<PointType> downSizeFilterICP;
  pcl::VoxelGrid<PointType>
      downSizeFilterSurroundingKeyPoses;  // for surrounding key poses of
                                          // scan-to-map optimization

  ros::Time timeLaserInfoStamp;
  double timeLaserInfoCur;

  float transformTobeMapped[6];

  std::mutex mtx;
  std::mutex mtxLoopInfo;

  bool isDegenerate = false;
  cv::Mat matP;

  int laserCloudFullLastDSNum = 0;

  bool aLoopIsClosed = false;
  map<int, int> loopIndexContainer;  // from new to old
  vector<pair<int, int>> loopIndexQueue;
  vector<gtsam::Pose3> loopPoseQueue;
  vector<gtsam::noiseModel::Diagonal::shared_ptr> loopNoiseQueue;
  deque<std_msgs::Float64MultiArray> loopInfoVec;

  nav_msgs::Path globalPath;

  Eigen::Affine3f transPointAssociateToMap;
  Eigen::Affine3f incrementalOdometryAffineFront;
  Eigen::Affine3f incrementalOdometryAffineBack;

  // // loop detector
  SCManager scManager;

  mapOptimization() {
    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.1;
    parameters.relinearizeSkip = 1;
    isam = new ISAM2(parameters);

    pubKeyPoses = nh.advertise<sensor_msgs::PointCloud2>(
        "sam_back_end/mapping/trajectory", 1);
    pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>(
        "sam_back_end/mapping/map_global", 1);
    pubLaserOdometryGlobal =
        nh.advertise<nav_msgs::Odometry>("sam_back_end/mapping/odometry", 1);
    pubPath = nh.advertise<nav_msgs::Path>("sam_back_end/mapping/path", 1);
    pubGnssPose =
        nh.advertise<sensor_msgs::PointCloud2>("sam_back_end/gnss_pose", 1);

    subOdom = std::make_shared<message_filters::Subscriber<nav_msgs::Odometry>>(
        nh, odomTopic, 10);
    subCloud =
        std::make_shared<message_filters::Subscriber<sensor_msgs::PointCloud2>>(
            nh, pointCloudTopic, 10);
    syncSubOdomCloud =
        std::make_shared<message_filters::Synchronizer<syncOdomCloudPol>>(
            syncOdomCloudPol(10), *subOdom, *subCloud);
    syncSubOdomCloud->registerCallback(
        boost::bind(&mapOptimization::laserCloudInfoHandler, this, _1, _2));

    subFix =
        std::make_shared<message_filters::Subscriber<sensor_msgs::NavSatFix>>(
            nh, "/fix", 10);
    subHeading = std::make_shared<
        message_filters::Subscriber<geometry_msgs::QuaternionStamped>>(
        nh, "/heading", 10);
    syncSubFixHeading =
        std::make_shared<message_filters::Synchronizer<syncFixHeadingPol>>(
            syncFixHeadingPol(10), *subFix, *subHeading);
    syncSubFixHeading->registerCallback(
        boost::bind(&mapOptimization::gpsHandler, this, _1, _2));

    subLoop = nh.subscribe<std_msgs::Float64MultiArray>(
        "sam_back_end/loop_closure_detection", 1,
        &mapOptimization::loopInfoHandler, this,
        ros::TransportHints().tcpNoDelay());

    srvSaveMap = nh.advertiseService("sam_back_end/save_map",
                                     &mapOptimization::saveMapService, this);

    pubHistoryKeyFrames = nh.advertise<sensor_msgs::PointCloud2>(
        "sam_back_end/mapping/icp_loop_closure_history_cloud", 1);
    pubIcpKeyFrames = nh.advertise<sensor_msgs::PointCloud2>(
        "sam_back_end/mapping/icp_loop_closure_corrected_cloud", 1);
    pubLoopConstraintEdge = nh.advertise<visualization_msgs::MarkerArray>(
        "/sam_back_end/mapping/loop_closure_constraints", 1);

    pubRecentKeyFrame = nh.advertise<sensor_msgs::PointCloud2>(
        "sam_back_end/mapping/cloud_registered", 1);

    downSizeFilterFull.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize,
                                   mappingSurfLeafSize);
    downSizeFilterICP.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize,
                                  mappingSurfLeafSize);
    downSizeFilterSurroundingKeyPoses.setLeafSize(
        surroundingKeyframeDensity, surroundingKeyframeDensity,
        surroundingKeyframeDensity);  // for surrounding key poses of
                                      // scan-to-map optimization

    allocateMemory();
    // std::string pcd_path = program_path + "/data/" + savePCDDirectory +
    // "/pcd/"; createDirectoryIfNotExists(pcd_path);
  }

  void allocateMemory() {
    cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
    cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
    cloudKeyOdom6D.reset(new pcl::PointCloud<PointTypePose>());
    copy_cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
    copy_cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
    laserCloudGnssPose.reset(new pcl::PointCloud<pcl::PointXYZ>());

    kdtreeSurroundingKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());
    kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());

    laserCloudFullLast.reset(new pcl::PointCloud<PointType>());
    laserCloudFullLastDS.reset(new pcl::PointCloud<PointType>());

    laserCloudOri.reset(new pcl::PointCloud<PointType>());
    coeffSel.reset(new pcl::PointCloud<PointType>());

    for (int i = 0; i < 6; ++i) {
      transformTobeMapped[i] = 0;
    }

    matP = cv::Mat(6, 6, CV_32F, cv::Scalar::all(0));
  }

  void laserCloudInfoHandler(const nav_msgs::OdometryConstPtr &odom_msg,
                             const sensor_msgs::PointCloud2ConstPtr &pcd_msg) {
    // std::cout<<" laserCloudInfoHandler Callback!!!!! "<<std::endl;
    // extract time stamp
    timeLaserInfoStamp = pcd_msg->header.stamp;
    timeLaserInfoCur =
        ros::Time::now().toSec();  // pcd_msg->header.stamp.toSec();

    // extract info and feature cloud
    // cloudInfo = *msgIn;
    pcl::fromROSMsg(*pcd_msg, *laserCloudFullLast);

    std::lock_guard<std::mutex> lock(mtx);

    static double timeLastProcessing = -1;
    if (timeLaserInfoCur - timeLastProcessing >= mappingProcessInterval) {
      timeLastProcessing = timeLaserInfoCur;

      // updateInitialGuess();

      // extractSurroundingKeyFrames();

      downsampleCurrentScan();

      // scan2MapOptimization();

      tf::Quaternion quaternionTobeMapped;
      tf::quaternionMsgToTF(odom_msg->pose.pose.orientation,
                            quaternionTobeMapped);
      double quaternionTobeMappedRoll, quaternionTobeMappedPitch,
          quaternionTobeMappedYaw;
      tf::Matrix3x3(quaternionTobeMapped)
          .getRPY(quaternionTobeMappedRoll, quaternionTobeMappedPitch,
                  quaternionTobeMappedYaw);
      transformTobeMapped[0] = quaternionTobeMappedRoll;
      transformTobeMapped[1] = quaternionTobeMappedPitch;
      transformTobeMapped[2] = quaternionTobeMappedYaw;
      transformTobeMapped[3] = odom_msg->pose.pose.position.x;
      transformTobeMapped[4] = odom_msg->pose.pose.position.y;
      transformTobeMapped[5] = odom_msg->pose.pose.position.z;

      saveKeyFramesAndFactor();

      correctPoses();

      publishOdometry();

      publishFrames();
    }
  }

  void gpsHandler(
      const sensor_msgs::NavSatFix::ConstPtr &fix_msg,
      const geometry_msgs::QuaternionStamped::ConstPtr &heading_msg) {
    nav_msgs::Odometry msgOdom;
    msgOdom.header.frame_id = mapFrame;
    msgOdom.header.stamp = ros::Time::now();
    msgOdom.pose.covariance[0] = fix_msg->position_covariance[0];
    msgOdom.pose.covariance[7] = fix_msg->position_covariance[4];
    msgOdom.pose.covariance[14] = fix_msg->position_covariance[8];
    if (!gnssInited) {
      gnssGeo.Reset(fix_msg->latitude, fix_msg->longitude, fix_msg->altitude);
      Eigen::Quaternionf initHeading;
      initHeading.w() = heading_msg->quaternion.w;
      initHeading.x() = heading_msg->quaternion.x;
      initHeading.y() = heading_msg->quaternion.y;
      initHeading.z() = heading_msg->quaternion.z;
      axisTransform = initHeading.inverse();
      msgOdom.pose.pose.position.x = 0;
      msgOdom.pose.pose.position.y = 0;
      msgOdom.pose.pose.position.z = 0;
      gnssInited = true;
    } else {
      double x, y, z;
      gnssGeo.Forward(fix_msg->latitude, fix_msg->longitude, fix_msg->altitude,
                      x, y, z);
      Eigen::Vector3f positionBeforeAxisTransform = Eigen::Vector3f(x, y, z);
      Eigen::Vector3f positionafterAxisTransform =
          axisTransform * positionBeforeAxisTransform;
      msgOdom.pose.pose.position.x = positionBeforeAxisTransform[0];
      msgOdom.pose.pose.position.y = positionBeforeAxisTransform[1];
      msgOdom.pose.pose.position.z = positionBeforeAxisTransform[2];
    }
    pcl::PointXYZ pointxyz;
    pointxyz.x = msgOdom.pose.pose.position.x;
    pointxyz.y = msgOdom.pose.pose.position.y;
    pointxyz.z = msgOdom.pose.pose.position.z;
    (*laserCloudGnssPose).push_back(pointxyz);

    sensor_msgs::PointCloud2 laserCloudGnssPoseMsg;
    laserCloudGnssPoseMsg.header.stamp = ros::Time::now();
    pcl::toROSMsg(*laserCloudGnssPose, laserCloudGnssPoseMsg);
    laserCloudGnssPoseMsg.header.frame_id = mapFrame;
    pubGnssPose.publish(laserCloudGnssPoseMsg);

    gpsQueue.push_back(msgOdom);
  }

  pcl::PointCloud<PointType>::Ptr transformPointCloud(
      pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose *transformIn) {
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    Eigen::Affine3f transCur = pcl::getTransformation(
        transformIn->x, transformIn->y, transformIn->z, transformIn->roll,
        transformIn->pitch, transformIn->yaw);

#pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i) {
      const auto &pointFrom = cloudIn->points[i];
      cloudOut->points[i].x = transCur(0, 0) * pointFrom.x +
                              transCur(0, 1) * pointFrom.y +
                              transCur(0, 2) * pointFrom.z + transCur(0, 3);
      cloudOut->points[i].y = transCur(1, 0) * pointFrom.x +
                              transCur(1, 1) * pointFrom.y +
                              transCur(1, 2) * pointFrom.z + transCur(1, 3);
      cloudOut->points[i].z = transCur(2, 0) * pointFrom.x +
                              transCur(2, 1) * pointFrom.y +
                              transCur(2, 2) * pointFrom.z + transCur(2, 3);
      cloudOut->points[i].intensity = pointFrom.intensity;
    }
    return cloudOut;
  }

  gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint) {
    return gtsam::Pose3(
        gtsam::Rot3::RzRyRx(double(thisPoint.roll), double(thisPoint.pitch),
                            double(thisPoint.yaw)),
        gtsam::Point3(double(thisPoint.x), double(thisPoint.y),
                      double(thisPoint.z)));
  }

  gtsam::Pose3 trans2gtsamPose(float transformIn[]) {
    return gtsam::Pose3(
        gtsam::Rot3::RzRyRx(transformIn[0], transformIn[1], transformIn[2]),
        gtsam::Point3(transformIn[3], transformIn[4], transformIn[5]));
  }

  Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint) {
    return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z,
                                  thisPoint.roll, thisPoint.pitch,
                                  thisPoint.yaw);
  }

  Eigen::Affine3f trans2Affine3f(float transformIn[]) {
    return pcl::getTransformation(transformIn[3], transformIn[4],
                                  transformIn[5], transformIn[0],
                                  transformIn[1], transformIn[2]);
  }

  PointTypePose trans2PointTypePose(float transformIn[]) {
    PointTypePose thisPose6D;
    thisPose6D.x = transformIn[3];
    thisPose6D.y = transformIn[4];
    thisPose6D.z = transformIn[5];
    thisPose6D.roll = transformIn[0];
    thisPose6D.pitch = transformIn[1];
    thisPose6D.yaw = transformIn[2];
    return thisPose6D;
  }

  bool saveMapService(sam_back_end::save_mapRequest &req,
                      sam_back_end::save_mapResponse &res) {
    string saveMapDirectory;

    cout << "****************************************************" << endl;
    cout << "Saving map to pcd files ..." << endl;
    if (req.destination.empty())
      saveMapDirectory = program_path + "/data/" + savePCDDirectory;
    else
      saveMapDirectory = std::getenv("HOME") + req.destination;
    cout << "Save destination: " << saveMapDirectory << endl;
    // create directory and remove old files;
    int unused =
        system((std::string("exec rm -rf ") + saveMapDirectory).c_str());
    unused =
        system((std::string("mkdir -p ") + saveMapDirectory + "/pcd/").c_str());
    // save key frame transformations
    pcl::io::savePCDFileBinary(saveMapDirectory + "/trajectory.pcd",
                               *cloudKeyPoses3D);
    pcl::io::savePCDFileBinary(saveMapDirectory + "/transformations.pcd",
                               *cloudKeyPoses6D);
    // extract global point cloud map
    pcl::PointCloud<PointType>::Ptr globalFullCloud(
        new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalFullCloudDS(
        new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalMapCloud(
        new pcl::PointCloud<PointType>());

    std::string poseName = saveMapDirectory + "/pose.json";
    std::ofstream poseFile(poseName);
    if (!poseFile.is_open()) {
      std::cout << "create pose.json failed" << std::endl;
      return false;
    }
    for (int i = 0; i < (int)cloudKeyPoses3D->size(); i++) {
      tf::Quaternion quaternion_tf = tf::createQuaternionFromRPY(
          cloudKeyPoses6D->points[i].roll, cloudKeyPoses6D->points[i].pitch,
          cloudKeyPoses6D->points[i].yaw);
      poseFile << std::to_string(cloudKeyPoses6D->points[i].x) << " "
               << std::to_string(cloudKeyPoses6D->points[i].y) << " "
               << std::to_string(cloudKeyPoses6D->points[i].z) << " "
               << std::to_string(quaternion_tf.w()) << " "
               << std::to_string(quaternion_tf.x()) << " "
               << std::to_string(quaternion_tf.y()) << " "
               << std::to_string(quaternion_tf.z()) << std::endl;
      std::string pcdName =
          saveMapDirectory + "/pcd/" + std::to_string(i) + ".pcd";
      pcl::io::savePCDFileBinary(pcdName, *rawCloudKeyFrames[i]);
      *globalFullCloud += *transformPointCloud(fullCloudKeyFrames[i],
                                               &cloudKeyPoses6D->points[i]);
      cout << "\r" << std::flush << "Processing feature cloud " << i << " of "
           << cloudKeyPoses6D->size() << " ...";
    }
    poseFile.close();

    if (req.resolution != 0) {
      cout << "\n\nSave resolution: " << req.resolution << endl;
      downSizeFilterFull.setInputCloud(globalFullCloud);
      downSizeFilterFull.setLeafSize(req.resolution, req.resolution,
                                     req.resolution);
      downSizeFilterFull.filter(*globalFullCloudDS);
      pcl::io::savePCDFileBinary(saveMapDirectory + "/FullMap.pcd",
                                 *globalFullCloudDS);
    } else {
      pcl::io::savePCDFileBinary(saveMapDirectory + "/FullMap.pcd",
                                 *globalFullCloud);
    }

    // save global point cloud map
    *globalMapCloud += *globalFullCloud;

    int ret = pcl::io::savePCDFileBinary(saveMapDirectory + "/GlobalMap.pcd",
                                         *globalMapCloud);
    res.success = ret == 0;

    downSizeFilterFull.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize,
                                   mappingSurfLeafSize);

    std::string scName = saveMapDirectory + "/sc.bin";
    bool sc_save = scManager.savePolarContexts(scName);
    if (!sc_save) {
      cout << "Saving scancontex failed\n" << endl;
      return false;
    }

    cout << "****************************************************" << endl;
    cout << "Saving map to pcd files completed\n" << endl;

    return true;
  }

  void visualizeGlobalMapThread() {
    ros::Rate rate(0.2);
    while (ros::ok()) {
      rate.sleep();
      publishGlobalMap();
    }

    if (savePCD == false) return;

    sam_back_end::save_mapRequest req;
    sam_back_end::save_mapResponse res;

    if (!saveMapService(req, res)) {
      cout << "Fail to save map" << endl;
    }
  }

  void publishGlobalMap() {
    if (pubLaserCloudSurround.getNumSubscribers() == 0) return;

    if (cloudKeyPoses3D->points.empty() == true) return;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMap(
        new pcl::KdTreeFLANN<PointType>());
    ;
    pcl::PointCloud<PointType>::Ptr globalMapKeyPoses(
        new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalMapKeyPosesDS(
        new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalMapKeyFrames(
        new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr globalMapKeyFramesDS(
        new pcl::PointCloud<PointType>());

    // kd-tree to find near key frames to visualize
    std::vector<int> pointSearchIndGlobalMap;
    std::vector<float> pointSearchSqDisGlobalMap;
    // search near key frames to visualize
    mtx.lock();
    kdtreeGlobalMap->setInputCloud(cloudKeyPoses3D);
    kdtreeGlobalMap->radiusSearch(
        cloudKeyPoses3D->back(), globalMapVisualizationSearchRadius,
        pointSearchIndGlobalMap, pointSearchSqDisGlobalMap, 0);
    mtx.unlock();

    for (int i = 0; i < (int)pointSearchIndGlobalMap.size(); ++i)
      globalMapKeyPoses->push_back(
          cloudKeyPoses3D->points[pointSearchIndGlobalMap[i]]);
    // downsample near selected key frames
    pcl::VoxelGrid<PointType>
        downSizeFilterGlobalMapKeyPoses;  // for global map visualization
    downSizeFilterGlobalMapKeyPoses.setLeafSize(
        globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity,
        globalMapVisualizationPoseDensity);  // for global map visualization
    downSizeFilterGlobalMapKeyPoses.setInputCloud(globalMapKeyPoses);
    downSizeFilterGlobalMapKeyPoses.filter(*globalMapKeyPosesDS);
    for (auto &pt : globalMapKeyPosesDS->points) {
      kdtreeGlobalMap->nearestKSearch(pt, 1, pointSearchIndGlobalMap,
                                      pointSearchSqDisGlobalMap);
      pt.intensity =
          cloudKeyPoses3D->points[pointSearchIndGlobalMap[0]].intensity;
    }

    // extract visualized and downsampled key frames
    for (int i = 0; i < (int)globalMapKeyPosesDS->size(); ++i) {
      if (pointDistance(globalMapKeyPosesDS->points[i],
                        cloudKeyPoses3D->back()) >
          globalMapVisualizationSearchRadius)
        continue;
      int thisKeyInd = (int)globalMapKeyPosesDS->points[i].intensity;
      *globalMapKeyFrames += *transformPointCloud(
          fullCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
    }
    // downsample visualized points
    pcl::VoxelGrid<PointType>
        downSizeFilterGlobalMapKeyFrames;  // for global map visualization
    downSizeFilterGlobalMapKeyFrames.setLeafSize(
        globalMapVisualizationLeafSize, globalMapVisualizationLeafSize,
        globalMapVisualizationLeafSize);  // for global map visualization
    downSizeFilterGlobalMapKeyFrames.setInputCloud(globalMapKeyFrames);
    downSizeFilterGlobalMapKeyFrames.filter(*globalMapKeyFramesDS);
    publishCloud(pubLaserCloudSurround, globalMapKeyFramesDS,
                 timeLaserInfoStamp, mapFrame);
  }

  void voxelFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out,
                   float gridsize) {
    pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
    vox_grid.setLeafSize(gridsize, gridsize, gridsize);
    vox_grid.setInputCloud(cloud_in);
    vox_grid.filter(*cloud_out);
  }

  void loopClosureThread() {
    if (loopClosureEnableFlag == false) return;

    ros::Rate rate(loopClosureFrequency);
    while (ros::ok()) {
      rate.sleep();
      performLoopClosure();
      visualizeLoopClosure();
    }
  }

  void loopInfoHandler(const std_msgs::Float64MultiArray::ConstPtr &loopMsg) {
    std::lock_guard<std::mutex> lock(mtxLoopInfo);
    if (loopMsg->data.size() != 2) return;

    loopInfoVec.push_back(*loopMsg);

    while (loopInfoVec.size() > 5) loopInfoVec.pop_front();
  }

  void performLoopClosure() {
    if (cloudKeyPoses3D->points.empty() == true) return;

    mtx.lock();
    *copy_cloudKeyPoses3D = *cloudKeyPoses3D;
    *copy_cloudKeyPoses6D = *cloudKeyPoses6D;
    mtx.unlock();

    clock_t start_loop_time = clock();
    std::vector<int> loopCandidateIndex;
    if (detectLoopClosureDistanceCandidate(loopCandidateIndex) == false) {
      return;
    }
    int loopKeyCur = copy_cloudKeyPoses3D->size() - 1;
    int loopKeyPre = -1;
    double min_dist = 10000000;  // init with somthing large
    for (int i = 0; i < loopCandidateIndex.size(); i++) {
      auto comapreResult =
          scManager.compareCurrentAndIndexed(loopCandidateIndex[i]);
      double candidate_dist = comapreResult.first;
      float candidate_yaw_diff_rad = comapreResult.second;
      if (candidate_dist < min_dist) {
        loopKeyPre = loopCandidateIndex[i];
        min_dist = candidate_dist;
      }
    }

    if (loopKeyPre == -1) /* No loop found */
      return;

    float loopCurPoseX = cloudKeyPoses3D->points[loopKeyCur].x;
    float loopCurPoseY = cloudKeyPoses3D->points[loopKeyCur].y;
    float loopCurPoseZ = cloudKeyPoses3D->points[loopKeyCur].z;
    float loopPrePoseX = cloudKeyPoses3D->points[loopKeyPre].x;
    float loopPrePoseY = cloudKeyPoses3D->points[loopKeyPre].y;
    float loopPrePoseZ = cloudKeyPoses3D->points[loopKeyPre].z;
    float distanceLoop = sqrt(pow(loopCurPoseX - loopPrePoseX, 2) +
                              pow(loopCurPoseY - loopPrePoseY, 2) +
                              pow(loopCurPoseZ - loopPrePoseZ, 2));
    if (distanceLoop > historyKeyframeSearchRadius) {
      return;
    }

    std::cout << "scan context find loop: " << loopKeyCur << " " << loopKeyPre
              << std::endl;

    // extract cloud
    pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(
        new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(
        new pcl::PointCloud<PointType>());
    {
      // loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, 0);
      loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur,
                            historyKeyframeSearchNum, false);
      loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre,
                            historyKeyframeSearchNum, true);
      if (cureKeyframeCloud->size() < 500 || prevKeyframeCloud->size() < 1000)
        return;
      if (pubHistoryKeyFrames.getNumSubscribers() != 0)
        publishCloud(pubHistoryKeyFrames, prevKeyframeCloud, timeLaserInfoStamp,
                     mapFrame);
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_src_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < (*cureKeyframeCloud).size(); ++i) {
      pcl::PointXYZ pointxyz;
      pointxyz.x = (*cureKeyframeCloud)[i].x;
      pointxyz.y = (*cureKeyframeCloud)[i].y;
      pointxyz.z = (*cureKeyframeCloud)[i].z;
      (*pcl_src_cloud).push_back(pointxyz);
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_tgt_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < (*prevKeyframeCloud).size(); ++i) {
      pcl::PointXYZ pointxyz;
      pointxyz.x = (*prevKeyframeCloud)[i].x;
      pointxyz.y = (*prevKeyframeCloud)[i].y;
      pointxyz.z = (*prevKeyframeCloud)[i].z;
      (*pcl_tgt_cloud).push_back(pointxyz);
    }
    vector<int> indices_nan_pcl_src_cloud;
    vector<int> indices_nan_pcl_tgt_cloud;
    pcl::removeNaNFromPointCloud(*pcl_src_cloud, *pcl_src_cloud,
                                 indices_nan_pcl_src_cloud);
    pcl::removeNaNFromPointCloud(*pcl_tgt_cloud, *pcl_tgt_cloud,
                                 indices_nan_pcl_tgt_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_src_cloud_downsample(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_tgt_cloud_downsample(
        new pcl::PointCloud<pcl::PointXYZ>);
    voxelFilter(pcl_src_cloud, pcl_src_cloud_downsample, voxel_grid_size);
    voxelFilter(pcl_tgt_cloud, pcl_tgt_cloud_downsample, voxel_grid_size);

    // ICP Settings
    static pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(historyKeyframeSearchRadius * 2);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);

    // Align clouds
    icp.setInputSource(cureKeyframeCloud);
    icp.setInputTarget(prevKeyframeCloud);
    pcl::PointCloud<PointType>::Ptr unused_result(
        new pcl::PointCloud<PointType>());
    icp.align(*unused_result);

    clock_t end_loop_time = clock();
    std::cout << "loop time: "
              << (double)(end_loop_time - start_loop_time) /
                     (double)CLOCKS_PER_SEC
              << " s" << std::endl;

    std::cout << "loop: " << loopKeyCur << ' ' << loopKeyPre << ' '
              << icp.getFitnessScore() << std::endl;

    if (icp.hasConverged() == false ||
        icp.getFitnessScore() > historyKeyframeFitnessScore)
      return;

    // publish corrected cloud
    if (pubIcpKeyFrames.getNumSubscribers() != 0) {
      pcl::PointCloud<PointType>::Ptr closed_cloud(
          new pcl::PointCloud<PointType>());
      pcl::transformPointCloud(*cureKeyframeCloud, *closed_cloud,
                               icp.getFinalTransformation());
      publishCloud(pubIcpKeyFrames, closed_cloud, timeLaserInfoStamp, mapFrame);
    }

    pcl::PointCloud<PointType>::Ptr KeyframeCloudSave(
        new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*cureKeyframeCloud, *KeyframeCloudSave,
                             icp.getFinalTransformation());
    (*KeyframeCloudSave) += (*prevKeyframeCloud);

    // std::string pcdName =
    //     "/home/lenovo/CodeFiles/58Robot/srcMapping3D/src/sam_back_end/"
    //     "pcd/" +
    //     std::to_string(loopKeyCur) + "_" + std::to_string(loopKeyPre) +
    //     ".pcd";
    // pcl::io::savePCDFileBinary(pcdName, *KeyframeCloudSave);

    // Get pose transformation
    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f correctionLidarFrame;
    correctionLidarFrame = icp.getFinalTransformation();
    // transform from world origin to wrong pose
    Eigen::Affine3f tWrong =
        pclPointToAffine3f(copy_cloudKeyPoses6D->points[loopKeyCur]);
    // transform from world origin to corrected pose
    Eigen::Affine3f tCorrect =
        correctionLidarFrame *
        tWrong;  // pre-multiplying -> successive rotation about a fixed frame
    pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);
    gtsam::Pose3 poseFrom =
        Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
    gtsam::Pose3 poseTo =
        pclPointTogtsamPose3(copy_cloudKeyPoses6D->points[loopKeyPre]);
    gtsam::Vector Vector6(6);
    float noiseScore = icp.getFitnessScore();
    Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore,
        noiseScore;
    noiseModel::Diagonal::shared_ptr constraintNoise =
        noiseModel::Diagonal::Variances(Vector6);

    // Add pose constraint
    mtx.lock();
    loopIndexQueue.push_back(make_pair(loopKeyCur, loopKeyPre));
    loopPoseQueue.push_back(poseFrom.between(poseTo));
    loopNoiseQueue.push_back(constraintNoise);
    mtx.unlock();

    // add loop constriant
    loopIndexContainer[loopKeyCur] = loopKeyPre;
  }

  bool detectLoopClosureDistanceCandidate(
      std::vector<int> &loopCandidateIndex) {
    int loopKeyCur = copy_cloudKeyPoses3D->size() - 1;

    // check loop constraint added before
    auto it = loopIndexContainer.find(loopKeyCur);
    if (it != loopIndexContainer.end()) return false;

    // find the closest history key frame
    std::vector<int> pointSearchIndLoop;
    std::vector<float> pointSearchSqDisLoop;
    kdtreeHistoryKeyPoses->setInputCloud(copy_cloudKeyPoses3D);
    kdtreeHistoryKeyPoses->radiusSearch(
        copy_cloudKeyPoses3D->back(), historyKeyframeSearchRadius,
        pointSearchIndLoop, pointSearchSqDisLoop, 0);

    for (int i = 0; i < (int)pointSearchIndLoop.size(); ++i) {
      int id = pointSearchIndLoop[i];
      if (abs(copy_cloudKeyPoses6D->points[id].time - timeLaserInfoCur) >
          historyKeyframeSearchTimeDiff) {
        loopCandidateIndex.push_back(id);
      }
    }

    if (loopCandidateIndex.size() == 0) return false;

    return true;
  }

  bool detectLoopClosureDistance(int *latestID, int *closestID) {
    int loopKeyCur = copy_cloudKeyPoses3D->size() - 1;
    int loopKeyPre = -1;

    // check loop constraint added before
    auto it = loopIndexContainer.find(loopKeyCur);
    if (it != loopIndexContainer.end()) return false;

    // find the closest history key frame
    std::vector<int> pointSearchIndLoop;
    std::vector<float> pointSearchSqDisLoop;
    kdtreeHistoryKeyPoses->setInputCloud(copy_cloudKeyPoses3D);
    kdtreeHistoryKeyPoses->radiusSearch(
        copy_cloudKeyPoses3D->back(), historyKeyframeSearchRadius,
        pointSearchIndLoop, pointSearchSqDisLoop, 0);

    for (int i = 0; i < (int)pointSearchIndLoop.size(); ++i) {
      int id = pointSearchIndLoop[i];
      if (abs(copy_cloudKeyPoses6D->points[id].time - timeLaserInfoCur) >
          historyKeyframeSearchTimeDiff) {
        loopKeyPre = id;
        break;
      }
    }

    if (loopKeyPre == -1 || loopKeyCur == loopKeyPre) return false;

    *latestID = loopKeyCur;
    *closestID = loopKeyPre;

    return true;
  }

  bool detectLoopClosureExternal(int *latestID, int *closestID) {
    // this function is not used yet, please ignore it
    int loopKeyCur = -1;
    int loopKeyPre = -1;

    std::lock_guard<std::mutex> lock(mtxLoopInfo);
    if (loopInfoVec.empty()) return false;

    //上一次回环检测信息
    double loopTimeCur = loopInfoVec.front().data[0];
    double loopTimePre = loopInfoVec.front().data[1];
    loopInfoVec.pop_front();

    if (abs(loopTimeCur - loopTimePre) < historyKeyframeSearchTimeDiff)
      return false;

    int cloudSize = copy_cloudKeyPoses6D->size();
    if (cloudSize < 2) return false;

    // latest key
    loopKeyCur = cloudSize - 1;
    for (int i = cloudSize - 1; i >= 0; --i) {
      if (copy_cloudKeyPoses6D->points[i].time >= loopTimeCur)
        loopKeyCur = round(copy_cloudKeyPoses6D->points[i].intensity);
      else
        break;
    }

    // previous key
    loopKeyPre = 0;
    for (int i = 0; i < cloudSize; ++i) {
      if (copy_cloudKeyPoses6D->points[i].time <= loopTimePre)
        loopKeyPre = round(copy_cloudKeyPoses6D->points[i].intensity);
      else
        break;
    }

    if (loopKeyCur == loopKeyPre) return false;

    auto it = loopIndexContainer.find(loopKeyCur);
    if (it != loopIndexContainer.end()) return false;

    *latestID = loopKeyCur;
    *closestID = loopKeyPre;

    return true;
  }

  void loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr &nearKeyframes,
                             const int &key, const int &searchNum, bool type) {
    // extract near keyframes
    nearKeyframes->clear();
    int cloudSize = copy_cloudKeyPoses6D->size();
    if (type == true) {
      for (int i = -searchNum; i <= searchNum; ++i) {
        int keyNear = key + i;
        if (keyNear < 0 || keyNear >= cloudSize) continue;
        *nearKeyframes +=
            *transformPointCloud(fullCloudKeyFrames[keyNear],
                                 &copy_cloudKeyPoses6D->points[keyNear]);
      }
    } else {
      for (int i = -searchNum; i <= 0; ++i) {
        int keyNear = key + i;
        if (keyNear < 0 || keyNear >= cloudSize) continue;
        *nearKeyframes +=
            *transformPointCloud(fullCloudKeyFrames[keyNear],
                                 &copy_cloudKeyPoses6D->points[keyNear]);
      }
    }

    if (nearKeyframes->empty()) return;

    // downsample near keyframes
    pcl::PointCloud<PointType>::Ptr cloud_temp(
        new pcl::PointCloud<PointType>());
    downSizeFilterICP.setInputCloud(nearKeyframes);
    downSizeFilterICP.filter(*cloud_temp);
    *nearKeyframes = *cloud_temp;
  }

  void visualizeLoopClosure() {
    if (loopIndexContainer.empty()) return;

    visualization_msgs::MarkerArray markerArray;
    // loop nodes
    visualization_msgs::Marker markerNode;
    markerNode.header.frame_id = mapFrame;
    markerNode.header.stamp = timeLaserInfoStamp;
    markerNode.action = visualization_msgs::Marker::ADD;
    markerNode.type = visualization_msgs::Marker::SPHERE_LIST;
    markerNode.ns = "loop_nodes";
    markerNode.id = 0;
    markerNode.pose.orientation.w = 1;
    markerNode.scale.x = 0.3;
    markerNode.scale.y = 0.3;
    markerNode.scale.z = 0.3;
    markerNode.color.r = 0;
    markerNode.color.g = 0.8;
    markerNode.color.b = 1;
    markerNode.color.a = 1;
    // loop edges
    visualization_msgs::Marker markerEdge;
    markerEdge.header.frame_id = mapFrame;
    markerEdge.header.stamp = timeLaserInfoStamp;
    markerEdge.action = visualization_msgs::Marker::ADD;
    markerEdge.type = visualization_msgs::Marker::LINE_LIST;
    markerEdge.ns = "loop_edges";
    markerEdge.id = 1;
    markerEdge.pose.orientation.w = 1;
    markerEdge.scale.x = 0.1;
    markerEdge.color.r = 0.9;
    markerEdge.color.g = 0.9;
    markerEdge.color.b = 0;
    markerEdge.color.a = 1;

    for (auto it = loopIndexContainer.begin(); it != loopIndexContainer.end();
         ++it) {
      int key_cur = it->first;
      int key_pre = it->second;
      geometry_msgs::Point p;
      p.x = copy_cloudKeyPoses6D->points[key_cur].x;
      p.y = copy_cloudKeyPoses6D->points[key_cur].y;
      p.z = copy_cloudKeyPoses6D->points[key_cur].z;
      markerNode.points.push_back(p);
      markerEdge.points.push_back(p);
      p.x = copy_cloudKeyPoses6D->points[key_pre].x;
      p.y = copy_cloudKeyPoses6D->points[key_pre].y;
      p.z = copy_cloudKeyPoses6D->points[key_pre].z;
      markerNode.points.push_back(p);
      markerEdge.points.push_back(p);
    }

    markerArray.markers.push_back(markerNode);
    markerArray.markers.push_back(markerEdge);
    pubLoopConstraintEdge.publish(markerArray);
  }

  void downsampleCurrentScan() {
    // Downsample cloud from current scan
    laserCloudFullLastDS->clear();
    downSizeFilterFull.setInputCloud(laserCloudFullLast);
    downSizeFilterFull.filter(*laserCloudFullLastDS);
    laserCloudFullLastDSNum = laserCloudFullLastDS->size();
  }

  bool saveFrame() {
    if (cloudKeyPoses3D->points.empty()) return true;

    if (sensor == SensorType::LIVOX) {
      if (timeLaserInfoCur - cloudKeyPoses6D->back().time > 1.0) return true;
    }

    Eigen::Affine3f transStart = pclPointToAffine3f(cloudKeyPoses6D->back());
    Eigen::Affine3f transFinal = pcl::getTransformation(
        transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5],
        transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
    Eigen::Affine3f transBetween = transStart.inverse() * transFinal;
    float x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw);

    if (abs(roll) < surroundingkeyframeAddingAngleThreshold &&
        abs(pitch) < surroundingkeyframeAddingAngleThreshold &&
        abs(yaw) < surroundingkeyframeAddingAngleThreshold &&
        sqrt(x * x + y * y + z * z) < surroundingkeyframeAddingDistThreshold)
      return false;

    return true;
  }

  void addOdomFactor() {
    if (cloudKeyPoses3D->points.empty()) {
      noiseModel::Diagonal::shared_ptr priorNoise =
          noiseModel::Diagonal::Variances(
              (gtsam::Vector(6) << 1e-2, 1e-2, M_PI * M_PI, 1e8, 1e8, 1e8)
                  .finished());  // rad*rad, meter*meter
      gtSAMgraph.add(PriorFactor<Pose3>(0, trans2gtsamPose(transformTobeMapped),
                                        priorNoise));
      initialEstimate.insert(0, trans2gtsamPose(transformTobeMapped));
    } else {
      noiseModel::Diagonal::shared_ptr odometryNoise =
          noiseModel::Diagonal::Variances(
              (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4)
                  .finished());
      gtsam::Pose3 poseFrom =
          pclPointTogtsamPose3(cloudKeyOdom6D->points.back());
      gtsam::Pose3 poseTo = trans2gtsamPose(transformTobeMapped);
      gtSAMgraph.add(BetweenFactor<Pose3>(
          cloudKeyPoses3D->size() - 1, cloudKeyPoses3D->size(),
          poseFrom.between(poseTo), odometryNoise));
      initialEstimate.insert(cloudKeyPoses3D->size(), poseTo);
    }
  }

  void addGPSFactor() {
    if (gpsQueue.empty()) return;

    // wait for system initialized and settles down
    if (cloudKeyPoses3D->points.empty())
      return;
    else {
      if (pointDistance(cloudKeyPoses3D->front(), cloudKeyPoses3D->back()) <
          5.0)
        return;
    }

    // pose covariance small, no need to correct
    if (poseCovariance(3, 3) < poseCovThreshold &&
        poseCovariance(4, 4) < poseCovThreshold)
      return;

    // last gps position
    static PointType lastGPSPoint;

    while (!gpsQueue.empty()) {
      if (gpsQueue.front().header.stamp.toSec() < timeLaserInfoCur - 0.2) {
        // message too old
        gpsQueue.pop_front();
      } else if (gpsQueue.front().header.stamp.toSec() >
                 timeLaserInfoCur + 0.2) {
        // message too new
        break;
      } else {
        nav_msgs::Odometry thisGPS = gpsQueue.front();
        gpsQueue.pop_front();

        // GPS too noisy, skip
        float noise_x = thisGPS.pose.covariance[0];
        float noise_y = thisGPS.pose.covariance[7];
        float noise_z = thisGPS.pose.covariance[14];
        if (noise_x > gpsCovThreshold || noise_y > gpsCovThreshold) continue;

        float gps_x = thisGPS.pose.pose.position.x;
        float gps_y = thisGPS.pose.pose.position.y;
        float gps_z = thisGPS.pose.pose.position.z;
        if (!useGpsElevation) {
          gps_z = transformTobeMapped[5];
          noise_z = 0.01;
        }

        // GPS not properly initialized (0,0,0)
        if (abs(gps_x) < 1e-6 && abs(gps_y) < 1e-6) continue;

        // Add GPS every a few meters
        PointType curGPSPoint;
        curGPSPoint.x = gps_x;
        curGPSPoint.y = gps_y;
        curGPSPoint.z = gps_z;
        if (pointDistance(curGPSPoint, lastGPSPoint) < 5.0)
          continue;
        else
          lastGPSPoint = curGPSPoint;

        gtsam::Vector Vector3(3);
        Vector3 << max(noise_x, 1.0f), max(noise_y, 1.0f), max(noise_z, 1.0f);
        noiseModel::Diagonal::shared_ptr gps_noise =
            noiseModel::Diagonal::Variances(Vector3);
        gtsam::GPSFactor gps_factor(cloudKeyPoses3D->size(),
                                    gtsam::Point3(gps_x, gps_y, gps_z),
                                    gps_noise);
        gtSAMgraph.add(gps_factor);

        // std::cout<<"add gps factor done"<<std::endl;

        aLoopIsClosed = true;
        break;
      }
    }
  }

  void addLoopFactor() {
    if (loopIndexQueue.empty()) return;

    for (int i = 0; i < (int)loopIndexQueue.size(); ++i) {
      int indexFrom = loopIndexQueue[i].first;
      int indexTo = loopIndexQueue[i].second;
      gtsam::Pose3 poseBetween = loopPoseQueue[i];
      gtsam::noiseModel::Diagonal::shared_ptr noiseBetween = loopNoiseQueue[i];
      gtSAMgraph.add(
          BetweenFactor<Pose3>(indexFrom, indexTo, poseBetween, noiseBetween));
    }

    loopIndexQueue.clear();
    loopPoseQueue.clear();
    loopNoiseQueue.clear();
    aLoopIsClosed = true;
  }

  void saveKeyFramesAndFactor() {
    if (saveFrame() == false) return;

    // odom factor
    addOdomFactor();

    PointTypePose thisOdom6D;
    thisOdom6D.x = transformTobeMapped[3];
    thisOdom6D.y = transformTobeMapped[4];
    thisOdom6D.z = transformTobeMapped[5];
    thisOdom6D.intensity = cloudKeyOdom6D->size();  // this can be used as index
    thisOdom6D.roll = transformTobeMapped[0];
    thisOdom6D.pitch = transformTobeMapped[1];
    thisOdom6D.yaw = transformTobeMapped[2];
    thisOdom6D.time = timeLaserInfoCur;
    cloudKeyOdom6D->push_back(thisOdom6D);

    // gps factor
    addGPSFactor();

    // loop factor
    addLoopFactor();

    // cout << "****************************************************" << endl;
    // gtSAMgraph.print("GTSAM Graph:\n");

    // update iSAM
    isam->update(gtSAMgraph, initialEstimate);
    isam->update();

    if (aLoopIsClosed == true) {
      isam->update();
      isam->update();
      isam->update();
      isam->update();
      isam->update();
    }

    gtSAMgraph.resize(0);
    initialEstimate.clear();

    // save key poses
    PointType thisPose3D;
    PointTypePose thisPose6D;
    Pose3 latestEstimate;

    isamCurrentEstimate = isam->calculateEstimate();
    latestEstimate =
        isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size() - 1);
    // cout << "****************************************************" << endl;
    // isamCurrentEstimate.print("Current estimate: ");

    thisPose3D.x = latestEstimate.translation().x();
    thisPose3D.y = latestEstimate.translation().y();
    thisPose3D.z = latestEstimate.translation().z();
    thisPose3D.intensity =
        cloudKeyPoses3D->size();  // this can be used as index
    cloudKeyPoses3D->push_back(thisPose3D);

    thisPose6D.x = thisPose3D.x;
    thisPose6D.y = thisPose3D.y;
    thisPose6D.z = thisPose3D.z;
    thisPose6D.intensity = thisPose3D.intensity;  // this can be used as index
    thisPose6D.roll = latestEstimate.rotation().roll();
    thisPose6D.pitch = latestEstimate.rotation().pitch();
    thisPose6D.yaw = latestEstimate.rotation().yaw();
    thisPose6D.time = timeLaserInfoCur;
    cloudKeyPoses6D->push_back(thisPose6D);

    // cout << "****************************************************" << endl;
    // cout << "Pose covariance:" << endl;
    // cout << isam->marginalCovariance(isamCurrentEstimate.size()-1) << endl <<
    // endl;
    poseCovariance = isam->marginalCovariance(isamCurrentEstimate.size() - 1);

    // save updated transform
    transformTobeMapped[0] = latestEstimate.rotation().roll();
    transformTobeMapped[1] = latestEstimate.rotation().pitch();
    transformTobeMapped[2] = latestEstimate.rotation().yaw();
    transformTobeMapped[3] = latestEstimate.translation().x();
    transformTobeMapped[4] = latestEstimate.translation().y();
    transformTobeMapped[5] = latestEstimate.translation().z();

    // save all the received edge and surf points
    pcl::PointCloud<PointType>::Ptr thisFullKeyFrame(
        new pcl::PointCloud<PointType>());
    pcl::copyPointCloud(*laserCloudFullLastDS, *thisFullKeyFrame);

    // save key frame cloud
    fullCloudKeyFrames.push_back(thisFullKeyFrame);

    pcl::PointCloud<PointType>::Ptr thisRawKeyFrame(
        new pcl::PointCloud<PointType>());
    pcl::copyPointCloud(*laserCloudFullLast, *thisRawKeyFrame);

    rawCloudKeyFrames.push_back(thisRawKeyFrame);

    const SCInputType sc_input_type =
        SCInputType::SINGLE_SCAN_FULL;  // change this

    if (sc_input_type == SCInputType::SINGLE_SCAN_FULL) {
      pcl::PointCloud<PointType>::Ptr thisRawCloudKeyFrame(
          new pcl::PointCloud<PointType>());
      pcl::copyPointCloud(*thisFullKeyFrame, *thisRawCloudKeyFrame);
      scManager.makeAndSaveScancontextAndKeys(*thisRawCloudKeyFrame);
    } else if (sc_input_type == SCInputType::SINGLE_SCAN_FEAT) {
      scManager.makeAndSaveScancontextAndKeys(*thisFullKeyFrame);
    } else if (sc_input_type == SCInputType::MULTI_SCAN_FEAT) {
      pcl::PointCloud<PointType>::Ptr multiKeyFrameFeatureCloud(
          new pcl::PointCloud<PointType>());
      loopFindNearKeyframes(multiKeyFrameFeatureCloud,
                            cloudKeyPoses6D->size() - 1,
                            historyKeyframeSearchNum, false);
      scManager.makeAndSaveScancontextAndKeys(*multiKeyFrameFeatureCloud);
    }

    // save path for visualization
    updatePath(thisPose6D);
  }

  void correctPoses() {
    if (cloudKeyPoses3D->points.empty()) return;

    if (aLoopIsClosed == true) {
      // clear map cache
      laserCloudMapContainer.clear();
      // clear path
      globalPath.poses.clear();
      // update key poses
      int numPoses = isamCurrentEstimate.size();
      for (int i = 0; i < numPoses; ++i) {
        cloudKeyPoses3D->points[i].x =
            isamCurrentEstimate.at<Pose3>(i).translation().x();
        cloudKeyPoses3D->points[i].y =
            isamCurrentEstimate.at<Pose3>(i).translation().y();
        cloudKeyPoses3D->points[i].z =
            isamCurrentEstimate.at<Pose3>(i).translation().z();

        cloudKeyPoses6D->points[i].x = cloudKeyPoses3D->points[i].x;
        cloudKeyPoses6D->points[i].y = cloudKeyPoses3D->points[i].y;
        cloudKeyPoses6D->points[i].z = cloudKeyPoses3D->points[i].z;
        cloudKeyPoses6D->points[i].roll =
            isamCurrentEstimate.at<Pose3>(i).rotation().roll();
        cloudKeyPoses6D->points[i].pitch =
            isamCurrentEstimate.at<Pose3>(i).rotation().pitch();
        cloudKeyPoses6D->points[i].yaw =
            isamCurrentEstimate.at<Pose3>(i).rotation().yaw();

        updatePath(cloudKeyPoses6D->points[i]);
      }

      aLoopIsClosed = false;
    }
  }

  void updatePath(const PointTypePose &pose_in) {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time().fromSec(pose_in.time);
    pose_stamped.header.frame_id = mapFrame;
    pose_stamped.pose.position.x = pose_in.x;
    pose_stamped.pose.position.y = pose_in.y;
    pose_stamped.pose.position.z = pose_in.z;
    tf::Quaternion q =
        tf::createQuaternionFromRPY(pose_in.roll, pose_in.pitch, pose_in.yaw);
    pose_stamped.pose.orientation.x = q.x();
    pose_stamped.pose.orientation.y = q.y();
    pose_stamped.pose.orientation.z = q.z();
    pose_stamped.pose.orientation.w = q.w();

    globalPath.poses.push_back(pose_stamped);
  }

  void publishOdometry() {
    // Publish odometry for ROS (global)
    nav_msgs::Odometry laserOdometryROS;
    laserOdometryROS.header.stamp = timeLaserInfoStamp;
    laserOdometryROS.header.frame_id = mapFrame;
    laserOdometryROS.pose.pose.position.x = transformTobeMapped[3];
    laserOdometryROS.pose.pose.position.y = transformTobeMapped[4];
    laserOdometryROS.pose.pose.position.z = transformTobeMapped[5];
    laserOdometryROS.pose.pose.orientation =
        tf::createQuaternionMsgFromRollPitchYaw(transformTobeMapped[0],
                                                transformTobeMapped[1],
                                                transformTobeMapped[2]);
    pubLaserOdometryGlobal.publish(laserOdometryROS);
  }

  void publishFrames() {
    if (cloudKeyPoses3D->points.empty()) return;
    // publish key poses
    publishCloud(pubKeyPoses, cloudKeyPoses3D, timeLaserInfoStamp, mapFrame);
    // publish registered key frame
    if (pubRecentKeyFrame.getNumSubscribers() != 0) {
      pcl::PointCloud<PointType>::Ptr cloudOut(
          new pcl::PointCloud<PointType>());
      PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
      *cloudOut += *transformPointCloud(laserCloudFullLastDS, &thisPose6D);
      publishCloud(pubRecentKeyFrame, cloudOut, timeLaserInfoStamp, mapFrame);
    }
    // publish path
    if (pubPath.getNumSubscribers() != 0) {
      globalPath.header.stamp = timeLaserInfoStamp;
      globalPath.header.frame_id = mapFrame;
      pubPath.publish(globalPath);
    }
  }

  bool createDirectoryIfNotExists(const std::string &dir_path) {
    struct stat st;

    // 检查目录是否已存在
    if (stat(dir_path.c_str(), &st) == 0) {
      return S_ISDIR(st.st_mode);
    }

    // 迭代创建目录
    std::string path = dir_path;
    size_t pos = 0;

    // 处理绝对路径
    if (path[0] == '/') {
      pos = 1;
    }

    while ((pos = path.find('/', pos)) != std::string::npos) {
      std::string sub_path = path.substr(0, pos);
      if (mkdir(sub_path.c_str(), 0755) != 0 && errno != EEXIST) {
        std::cerr << "failed to create subdirectories: " << sub_path
                  << std::endl;
        return false;
      }
      pos++;
    }

    // 创建最后一层目录
    if (mkdir(dir_path.c_str(), 0755) != 0 && errno != EEXIST) {
      std::cerr << "failed to create directory: " << dir_path << std::endl;
      return false;
    }

    std::cout << "successfully created directory: " << dir_path << std::endl;
    return true;
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "sam_back_end");
  m_package_path = ros::package::getPath("sam_back_end");

  //确定工作空间
  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "-program_path" && i + 1 < argc) {
      program_path = std::string(argv[i + 1]);
      program_path = program_path.substr(0, program_path.rfind('/'));
      program_path = program_path.substr(0, program_path.rfind('/'));
      break;
    }
  }
  std::cout << "program_path: " << program_path << std::endl;

  mapOptimization MO;

  ROS_INFO("\033[1;32m----> Map Optimization Started.\033[0m");

  std::thread loopthread(&mapOptimization::loopClosureThread, &MO);
  std::thread visualizeMapThread(&mapOptimization::visualizeGlobalMapThread,
                                 &MO);

  ros::spin();

  loopthread.join();
  visualizeMapThread.join();

  return 0;
}
