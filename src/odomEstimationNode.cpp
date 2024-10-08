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
#include "odomEstimationClass.h"

class OdomEstimationNode{
  private:
  std::mutex mtx; //mutex

  // node handler
  ros::NodeHandle* nh_;
  ros::NodeHandle* private_nh_;

  // sub & pub
  ros::Subscriber subEdgeLaserCloud_;
  ros::Subscriber subSurfLaserCloud_;
  ros::Publisher pubLaserOdometry;

  //
  std::deque<sensor_msgs::PointCloud2ConstPtr> pointCloudEdgeBuf_;
  std::deque<sensor_msgs::PointCloud2ConstPtr> pointCloudSurfBuf_;
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
  OdomEstimationNode(ros::NodeHandle* nh, ros::NodeHandle* private_nh)
  {
    nh_ = nh; //store nh
    private_nh_ = private_nh; //store private_nh

    // 1. initialize param
    initParam();
    odomEstimation_.init(lidar_param_, map_resolution_);

    // 2. setup subscriber
    subEdgeLaserCloud_ = nh_->subscribe<sensor_msgs::PointCloud2>("cloud_edge", 2, &OdomEstimationNode::cloud_edge_cb,this);
    subSurfLaserCloud_ = nh_->subscribe<sensor_msgs::PointCloud2>("cloud_surf", 2, &OdomEstimationNode::cloud_surf_cb,this);
    
    // 3. setup publisher
    pubLaserOdometry = nh_->advertise<nav_msgs::Odometry>("odom", 10);

    // 4. start process thread
    process_thread_ = std::thread(&OdomEstimationNode::process,this);
  }
  void cloud_edge_cb(const sensor_msgs::PointCloud2ConstPtr &msg)
  {
    mtx.lock();
    while ( pointCloudEdgeBuf_.size() > maxQueueSize ) pointCloudEdgeBuf_.pop_front();
    pointCloudEdgeBuf_.push_back(msg);
    mtx.unlock();   
  }
  void cloud_surf_cb(const sensor_msgs::PointCloud2ConstPtr &msg)
  {
    mtx.lock();
    while (pointCloudSurfBuf_.size() > maxQueueSize ) pointCloudSurfBuf_.pop_front();
    pointCloudSurfBuf_.push_back(msg);
    mtx.unlock();   
  }
  void initParam()
  {
    scan_line_ = private_nh_->param("scan_line",16);
    scan_period_ = private_nh_->param("scan_period",0.1); // 1/scan rate (Hz)
    vertical_angle_ = private_nh_->param("vertical_angle",2.0); // this variable seems like not used
    max_dis_ = private_nh_->param("max_dis",60.0);
    min_dis_ = private_nh_->param("min_dis", 2.0);
    base_frame_ = private_nh_->param("base_frame",(std::string)"base_footprint");
    odom_frame_ = private_nh_->param("odom_frame",(std::string)"odom");
    map_resolution_ = private_nh_->param("map_resolution",0.4);
    publish_tf_ = private_nh_->param("publish_tf",false);
    
    lidar_param_.setScanPeriod(scan_period_);
    lidar_param_.setVerticalAngle(vertical_angle_);
    lidar_param_.setLines(scan_line_);
    lidar_param_.setMaxDistance(max_dis_);
    lidar_param_.setMinDistance(min_dis_);

    ROS_INFO_STREAM("[OdomEstimationNode] scan line is set to: "<< scan_line_);
    ROS_INFO_STREAM("[OdomEstimationNode] scan period is set to: "<< scan_period_);
    ROS_INFO_STREAM("[OdomEstimationNode] vertical angle is set to: "<< vertical_angle_);
    ROS_INFO_STREAM("[OdomEstimationNode] max distance is set to: "<< max_dis_);
    ROS_INFO_STREAM("[OdomEstimationNode] min distance is set to: "<< min_dis_);
    ROS_INFO_STREAM("[OdomEstimationNode] base_frame is set to: "<< base_frame_);
    ROS_INFO_STREAM("[OdomEstimationNode] odom_frame is set to: "<< odom_frame_);
    ROS_INFO_STREAM("[OdomEstimationNode] map_resolution is set to: "<< map_resolution_);
    ROS_INFO_STREAM("[OdomEstimationNode] publish_tf is set to: "<< publish_tf_);
  }

  void process()
  {
    while(true){
      if(!pointCloudEdgeBuf_.empty() && !pointCloudSurfBuf_.empty())
      {
        //read data
        mtx.lock();
        if(!pointCloudSurfBuf_.empty() && (pointCloudSurfBuf_.front()->header.stamp.toSec()<pointCloudEdgeBuf_.front()->header.stamp.toSec()-0.5*lidar_param_.scan_period)){
          pointCloudSurfBuf_.pop_front();
          ROS_WARN_ONCE("time stamp unaligned with extra point cloud, pls check your data --> odom correction");
          mtx.unlock();
          continue;  
        }
        if(!pointCloudEdgeBuf_.empty() && (pointCloudEdgeBuf_.front()->header.stamp.toSec()<pointCloudSurfBuf_.front()->header.stamp.toSec()-0.5*lidar_param_.scan_period)){
          pointCloudEdgeBuf_.pop_front();
          ROS_WARN_ONCE("time stamp unaligned with extra point cloud, pls check your data --> odom correction");
          mtx.unlock();
          continue;  
        }

        //if time aligned 
        pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf_in(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge_in(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*pointCloudEdgeBuf_.front(), *pointcloud_edge_in);
        pcl::fromROSMsg(*pointCloudSurfBuf_.front(), *pointcloud_surf_in);
        ros::Time pointcloud_time = (pointCloudSurfBuf_.front())->header.stamp;
        pointCloudEdgeBuf_.pop_front();
        pointCloudSurfBuf_.pop_front();
        mtx.unlock();

        if(is_odom_inited_ == false){
          odomEstimation_.initMapWithPoints(pointcloud_edge_in, pointcloud_surf_in);
          is_odom_inited_ = true;
          ROS_INFO("odom inited");
        }else{
          std::chrono::time_point<std::chrono::system_clock> start, end;
          start = std::chrono::system_clock::now();
          odomEstimation_.updatePointsToMap(pointcloud_edge_in, pointcloud_surf_in);
          end = std::chrono::system_clock::now();
          std::chrono::duration<float> elapsed_seconds = end - start;
          total_frame_++;
          float time_temp = elapsed_seconds.count() * 1000;
          total_time_+=time_temp;
          ROS_INFO("average odom estimation time %f ms \n \n", total_time_/total_frame_);
        }

        Eigen::Quaterniond q_current(odomEstimation_.odom.rotation());
        //q_current.normalize();
        Eigen::Vector3d t_current = odomEstimation_.odom.translation();

        if(publish_tf_){
          static tf2_ros::TransformBroadcaster br;
          geometry_msgs::TransformStamped transformStamped;
          transformStamped.header.stamp = ros::Time::now();
          transformStamped.header.frame_id = odom_frame_;
          transformStamped.child_frame_id = base_frame_;
          transformStamped.transform.translation.x = t_current.x();
          transformStamped.transform.translation.y = t_current.y();
          transformStamped.transform.translation.z = t_current.z();
          tf2::Quaternion q(q_current.x(),q_current.y(),q_current.z(),q_current.w());
          transformStamped.transform.rotation.x = q.x();
          transformStamped.transform.rotation.y = q.y();
          transformStamped.transform.rotation.z = q.z();
          transformStamped.transform.rotation.w = q.w();
          br.sendTransform(transformStamped);
        }

        // publish odometry
        nav_msgs::Odometry laserOdometry;
        laserOdometry.header.frame_id = odom_frame_;
        laserOdometry.child_frame_id = base_frame_;
        laserOdometry.header.stamp = pointcloud_time;
        laserOdometry.pose.pose.orientation.x = q_current.x();
        laserOdometry.pose.pose.orientation.y = q_current.y();
        laserOdometry.pose.pose.orientation.z = q_current.z();
        laserOdometry.pose.pose.orientation.w = q_current.w();
        laserOdometry.pose.pose.position.x = t_current.x();
        laserOdometry.pose.pose.position.y = t_current.y();
        laserOdometry.pose.pose.position.z = t_current.z();
        pubLaserOdometry.publish(laserOdometry);
      }
      
      //sleep 2 ms every time
      std::chrono::milliseconds dura(2);
      std::this_thread::sleep_for(dura);
    }
  }
};

int main (int argc, char **argv)
{
  ros::init(argc, argv, "floam_odom_estimation_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  OdomEstimationNode node(&nh,&private_nh);
  ros::spin();
}