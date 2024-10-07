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
#include "laserMappingClass.h"
#include "lidar.h"


class MappingNode{
  private:
  std::mutex mtx; //mutex

  // node handler
  ros::NodeHandle* nh_;
  ros::NodeHandle* private_nh_;

  // sub & pub
  ros::Subscriber subLaserCloud_;
  ros::Subscriber subOdometry_;
  ros::Publisher map_pub_;

  //
  std::queue<nav_msgs::OdometryConstPtr> odometryBuf_;
  std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudBuf_;
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
  MappingNode(ros::NodeHandle* nh, ros::NodeHandle* private_nh)
  {
    nh_ = nh; //store nh
    private_nh_ = private_nh; //store private_nh

    // 1. initialize param
    initParam();
    map_pub_period_ = 1.0/map_pub_rate_;
    laserMapping_.init(map_resolution_);

    // 2. setup subscriber
    subLaserCloud_ = nh_->subscribe<sensor_msgs::PointCloud2>("cloud_filtered", 100, &MappingNode::cloud_cb,this);
    subOdometry_ = nh_->subscribe<nav_msgs::Odometry>("odom", 100, &MappingNode::odom_cb,this);

    // 3. setup publisher
    map_pub_ = nh_->advertise<sensor_msgs::PointCloud2>("loam_map", 100);

    // 4. start process thread
    process_thread_ = std::thread(&MappingNode::process,this);
  }

  void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &msg)
  {
    mtx.lock();
    pointCloudBuf_.push(msg);
    mtx.unlock();   
  }

  void odom_cb(const nav_msgs::OdometryConstPtr &msg)
  {
    mtx.lock();
    odometryBuf_.push(msg);
    mtx.unlock();   
  }

  void initParam()
  {
    scan_line_ = private_nh_->param("scan_line",16);
    scan_period_ = private_nh_->param("scan_period",0.1); // 1/scan rate (Hz)
    vertical_angle_ = private_nh_->param("vertical_angle",2.0); // this variable seems like not used
    max_dis_ = private_nh_->param("max_dis",60.0);
    min_dis_ = private_nh_->param("min_dis", 2.0);
    base_frame_ = private_nh_->param<std::string>("base_frame","base_footprint");
    odom_frame_ = private_nh_->param<std::string>("odom_frame","odom");
    map_resolution_ = private_nh_->param("map_resolution",0.4);
    map_pub_rate_ = private_nh_->param("map_pub_rate",1.0);
    
    lidar_param_.setScanPeriod(scan_period_);
    lidar_param_.setVerticalAngle(vertical_angle_);
    lidar_param_.setLines(scan_line_);
    lidar_param_.setMaxDistance(max_dis_);
    lidar_param_.setMinDistance(min_dis_);

    ROS_INFO_STREAM("[MappingNode] scan line is set to: "<< scan_line_);
    ROS_INFO_STREAM("[MappingNode] scan period is set to: "<< scan_period_);
    ROS_INFO_STREAM("[MappingNode] vertical angle is set to: "<< vertical_angle_);
    ROS_INFO_STREAM("[MappingNode] max distance is set to: "<< max_dis_);
    ROS_INFO_STREAM("[MappingNode] min distance is set to: "<< min_dis_);
    ROS_INFO_STREAM("[MappingNode] base_frame is set to: "<< base_frame_);
    ROS_INFO_STREAM("[MappingNode] odom_frame is set to: "<< odom_frame_);
    ROS_INFO_STREAM("[MappingNode] map_resolution is set to: "<< map_resolution_);
    ROS_INFO_STREAM("[MappingNode] map_pub_rate is set to: "<< map_pub_rate_);
  }

  void process()
  {
    while(true){
      if(!odometryBuf_.empty() && !pointCloudBuf_.empty())
      {
        //read data
        mtx.lock();
        if(!pointCloudBuf_.empty() && pointCloudBuf_.front()->header.stamp.toSec()<odometryBuf_.front()->header.stamp.toSec()-0.5*lidar_param_.scan_period){
            ROS_WARN("time stamp unaligned error and pointcloud discarded, pls check your data --> laser mapping node"); 
            pointCloudBuf_.pop();
            mtx.unlock();
            continue;              
        }

        if(!odometryBuf_.empty() && odometryBuf_.front()->header.stamp.toSec() < pointCloudBuf_.front()->header.stamp.toSec()-0.5*lidar_param_.scan_period){
            odometryBuf_.pop();
            ROS_INFO("time stamp unaligned with path final, pls check your data --> laser mapping node");
            mtx.unlock();
            continue;  
        }

        //if time aligned 
        pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*pointCloudBuf_.front(), *pointcloud_in);
        ros::Time pointcloud_time = (pointCloudBuf_.front())->header.stamp;

        Eigen::Isometry3d current_pose = Eigen::Isometry3d::Identity();
        current_pose.rotate(Eigen::Quaterniond(odometryBuf_.front()->pose.pose.orientation.w,odometryBuf_.front()->pose.pose.orientation.x,odometryBuf_.front()->pose.pose.orientation.y,odometryBuf_.front()->pose.pose.orientation.z));  
        current_pose.pretranslate(Eigen::Vector3d(odometryBuf_.front()->pose.pose.position.x,odometryBuf_.front()->pose.pose.position.y,odometryBuf_.front()->pose.pose.position.z));
        pointCloudBuf_.pop();
        odometryBuf_.pop();
        mtx.unlock();
        
        laserMapping_.updateCurrentPointsToMap(pointcloud_in,current_pose);

        if((ros::Time::now() - prev_map_pub_time_).toSec() > map_pub_period_)
        {
          pcl::PointCloud<pcl::PointXYZI>::Ptr pc_map = laserMapping_.getMap();
          sensor_msgs::PointCloud2 PointsMsg;
          pcl::toROSMsg(*pc_map, PointsMsg);
          PointsMsg.header.stamp = pointcloud_time;
          PointsMsg.header.frame_id = odom_frame_;
          map_pub_.publish(PointsMsg);
          prev_map_pub_time_ = ros::Time::now(); 
        }
    }
      //sleep 2 ms every time
      std::chrono::milliseconds dura(2);
      std::this_thread::sleep_for(dura);
    }
  }
};

int main (int argc, char **argv)
{
  ros::init(argc, argv, "floam_mapping_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  MappingNode node(&nh,&private_nh);
  ros::spin();
}