// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

//c++ lib
#include <cmath>
#include <vector>
#include <mutex>
// #include <queue>
#include <deque>
#include <thread>
#include <chrono>

//ros lib
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
// #include <tf/transform_datatypes.h>
// #include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
//pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//local lib
#include "lidar.h"
#include "odomEstimation.h"
#include "floam_utils.h"

class OdomEstimationROS{
  private:
  std::mutex* mtx_; //mutex

  // node handler
  ros::NodeHandle* nh_;
  ros::NodeHandle* private_nh_;

  // sub & pub
  ros::Subscriber subEdgeLaserCloud_;
  ros::Subscriber subSurfLaserCloud_;
  ros::Publisher pubLaserOdometry;

  //
  std::deque<CloudFilteredObj>* cloudFilteredObjBuffer_;
  std::deque<OdomAndCloudFilteredObj>* odomAndCloudFilteredObjBuf_;
  std::thread process_thread_; //process thread
  lidar::Lidar lidar_param_;
  OdomEstimationClass odomEstimation_;

  // param
  int scan_line_;
  double scan_period_; // 1/scan rate (Hz)
  double vertical_angle_; // this variable seems like not used
  double max_dis_;
  double min_dis_;
  std::string base_frame_;
  std::string odom_frame_;
  double map_resolution_;
  bool publish_tf_;

  // misc
  bool is_odom_inited_ = false;
  double total_time_ = 0;
  int total_frame_ = 0;
  const size_t maxQueueSize = 5;

  public:
  OdomEstimationROS(std::mutex* mtx,ros::NodeHandle* nh, ros::NodeHandle* private_nh, std::deque<CloudFilteredObj>* cloudFilteredObjBuffer,std::deque<OdomAndCloudFilteredObj>* OdomAndCloudFilteredObjBuf);
  void cloud_edge_cb(const sensor_msgs::PointCloud2ConstPtr &msg);
  void cloud_surf_cb(const sensor_msgs::PointCloud2ConstPtr &msg);
  void initParam();
  void process();
};
