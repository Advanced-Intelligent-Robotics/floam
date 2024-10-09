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
#include <memory>

//ros lib
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>

//pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>

//local lib
#include "lidar.h"
#include "laserProcessing.h"
#include "utils.h"
#include <pcl/console/print.h>


class LaserProcessingROS{
  private:
  std::mutex* mtx_; //mutex

  // node handler
  ros::NodeHandle* nh_;
  ros::NodeHandle* private_nh_;

  // sub & pub
  ros::Publisher pubEdgePoints_;
  ros::Publisher pubSurfPoints_;
  ros::Publisher pubLaserCloudFiltered_;
  ros::Subscriber subLaserCloud_;

  // tf2
  tf2_ros::Buffer* tfBuffer_;
  // std::unique_ptr<tf2_ros::Buffer> tfBuffer_; // tf buffer - use for lookup transform between frames
  // std::shared_ptr<tf2_ros::TransformListener> tf_listener_; // helper to subscribe to tf tree

  //
  std::deque<sensor_msgs::PointCloud2> pointCloudBuf_; // PointCloud2 Buffer
  std::deque<CloudFilteredObj>* cloudFilteredObjBuffer_;
  std::thread process_thread_; //process thread
  lidar::Lidar lidar_param_;
  LaserProcessingClass laserProcessing;

  // param
  int scan_line_;
  double scan_period_; // 1/scan rate (Hz)
  double vertical_angle_; // this variable seems like not used
  double max_dis_;
  double min_dis_;
  std::string base_frame_;
  std::string odom_frame_;

  // misc
  double total_time_ = 0;
  int total_frame_ = 0;
  const size_t maxQueueSize = 5;

  public:
  LaserProcessingROS(std::mutex* mtx,ros::NodeHandle* nh, ros::NodeHandle* private_nh, tf2_ros::Buffer* tfBuffer,std::deque<CloudFilteredObj>* cloudFilteredObjBuffer );

  void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &msg);
  void initParam();
  void process();
  bool doTransformPCL(const std::string& target_frame,const pcl::PointCloud<pcl::PointXYZI> &cloud_in,pcl::PointCloud<pcl::PointXYZI> &cloud_out);
};



