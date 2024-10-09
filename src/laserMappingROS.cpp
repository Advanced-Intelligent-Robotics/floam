// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#include "laserMappingROS.h"


LaserMappingROS::LaserMappingROS(std::mutex* mtx,ros::NodeHandle* nh, ros::NodeHandle* private_nh, std::deque<OdomAndCloudFilteredObj>* odomAndCloudFilteredObjBuf)
{
  mtx_ = mtx;
  nh_ = nh; //store nh
  private_nh_ = private_nh; //store private_nh
  odomAndCloudFilteredObjBuf_ = odomAndCloudFilteredObjBuf;

  // 1. initialize param
  initParam();
  map_pub_period_ = 1.0/map_pub_rate_;
  laserMapping_.init(map_resolution_);

  // 2. setup publisher
  map_pub_ = nh_->advertise<sensor_msgs::PointCloud2>("floam_map", 10);

  // 3. start process thread
  process_thread_ = std::thread(&LaserMappingROS::process,this);
}

void LaserMappingROS::initParam()
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

  ROS_INFO_STREAM("[MappingThread] scan line is set to: "<< scan_line_);
  ROS_INFO_STREAM("[MappingThread] scan period is set to: "<< scan_period_);
  ROS_INFO_STREAM("[MappingThread] vertical angle is set to: "<< vertical_angle_);
  ROS_INFO_STREAM("[MappingThread] max distance is set to: "<< max_dis_);
  ROS_INFO_STREAM("[MappingThread] min distance is set to: "<< min_dis_);
  ROS_INFO_STREAM("[MappingThread] base_frame is set to: "<< base_frame_);
  ROS_INFO_STREAM("[MappingThread] odom_frame is set to: "<< odom_frame_);
  ROS_INFO_STREAM("[MappingThread] map_resolution is set to: "<< map_resolution_);
  ROS_INFO_STREAM("[MappingThread] map_pub_rate is set to: "<< map_pub_rate_);
}

void LaserMappingROS::process()
{
  while(true){
    if(!odomAndCloudFilteredObjBuf_->empty())
    {
      //read data from buffer
      mtx_->lock();
      pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZI>());
      pointcloud_in = odomAndCloudFilteredObjBuf_->front().pointcloud_filtered;
      ros::Time pointcloud_time = odomAndCloudFilteredObjBuf_->front().time;

      Eigen::Isometry3d current_pose = Eigen::Isometry3d::Identity();
      current_pose.rotate(Eigen::Quaterniond(odomAndCloudFilteredObjBuf_->front().odom.pose.pose.orientation.w,
                                             odomAndCloudFilteredObjBuf_->front().odom.pose.pose.orientation.x,
                                             odomAndCloudFilteredObjBuf_->front().odom.pose.pose.orientation.y,
                                             odomAndCloudFilteredObjBuf_->front().odom.pose.pose.orientation.z));

      current_pose.pretranslate(Eigen::Vector3d(odomAndCloudFilteredObjBuf_->front().odom.pose.pose.position.x,
                                                odomAndCloudFilteredObjBuf_->front().odom.pose.pose.position.y,
                                                odomAndCloudFilteredObjBuf_->front().odom.pose.pose.position.z));
      odomAndCloudFilteredObjBuf_->pop_front();
      mtx_->unlock();
      
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
