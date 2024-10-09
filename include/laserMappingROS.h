// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

//c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>

//ros lib
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
// #include <tf/transform_datatypes.h>
// #include <tf/transform_broadcaster.h>

//pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//local lib
#include "laserMapping.h"
#include "lidar.h"
#include "floam_utils.h"


class LaserMappingROS{
  private:
  std::mutex* mtx_; //mutex

  // node handler
  ros::NodeHandle* nh_;
  ros::NodeHandle* private_nh_;

  // sub & pub
  ros::Subscriber subLaserCloud_;
  ros::Subscriber subOdometry_;
  ros::Publisher map_pub_;

  //
  std::deque<OdomAndCloudFilteredObj>* odomAndCloudFilteredObjBuf_;
  std::thread process_thread_; //process thread
  lidar::Lidar lidar_param_;
  LaserMappingClass laserMapping_;

  // param
  int scan_line_;
  double scan_period_; // 1/scan rate (Hz)
  double vertical_angle_; // this variable seems like not used
  double max_dis_;
  double min_dis_;
  std::string base_frame_;
  std::string odom_frame_;
  double map_resolution_;
  double map_pub_rate_;

  // misc
  double map_pub_period_;
  ros::Time prev_map_pub_time_ = ros::Time(0);

  public:
  LaserMappingROS(std::mutex* mtx,ros::NodeHandle* nh, ros::NodeHandle* private_nh, std::deque<OdomAndCloudFilteredObj>* odomAndCloudFilteredObjBuf);

  void initParam();

  void process();
};
