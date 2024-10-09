// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro


#include "odomEstimation.h"
#include "odomEstimationROS.h"


OdomEstimationROS::OdomEstimationROS(std::mutex* mtx,ros::NodeHandle* nh, ros::NodeHandle* private_nh, std::deque<CloudFilteredObj>* cloudFilteredObjBuffer,std::deque<OdomAndCloudFilteredObj>* odomAndCloudFilteredObjBuf)
{
  mtx_ = mtx;
  nh_ = nh; //store nh
  private_nh_ = private_nh; //store private_nh
  cloudFilteredObjBuffer_ = cloudFilteredObjBuffer;
  odomAndCloudFilteredObjBuf_ = odomAndCloudFilteredObjBuf;
  // 1. initialize param
  initParam();
  odomEstimation_.init(lidar_param_, map_resolution_);

  // 3. setup publisher
  pubLaserOdometry = nh_->advertise<nav_msgs::Odometry>("odom", 10);

  // 4. start process thread
  process_thread_ = std::thread(&OdomEstimationROS::process,this);
}

void OdomEstimationROS::initParam()
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

  ROS_INFO_STREAM("[OdomEstimationThread] scan line is set to: "<< scan_line_);
  ROS_INFO_STREAM("[OdomEstimationThread] scan period is set to: "<< scan_period_);
  ROS_INFO_STREAM("[OdomEstimationThread] vertical angle is set to: "<< vertical_angle_);
  ROS_INFO_STREAM("[OdomEstimationThread] max distance is set to: "<< max_dis_);
  ROS_INFO_STREAM("[OdomEstimationThread] min distance is set to: "<< min_dis_);
  ROS_INFO_STREAM("[OdomEstimationThread] base_frame is set to: "<< base_frame_);
  ROS_INFO_STREAM("[OdomEstimationThread] odom_frame is set to: "<< odom_frame_);
  ROS_INFO_STREAM("[OdomEstimationThread] map_resolution is set to: "<< map_resolution_);
  ROS_INFO_STREAM("[OdomEstimationThread] publish_tf is set to: "<< publish_tf_);
}

void OdomEstimationROS::process()
{
  while(true){
    if(!cloudFilteredObjBuffer_->empty())
    {
      //read data
      mtx_->lock();
      pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf_in(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge_in(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_filtered_in(new pcl::PointCloud<pcl::PointXYZI>());
      ros::Time pointcloud_time = cloudFilteredObjBuffer_->front().time;
      pointcloud_surf_in = cloudFilteredObjBuffer_->front().pointcloud_surf;
      pointcloud_edge_in = cloudFilteredObjBuffer_->front().pointcloud_edge;
      pointcloud_filtered_in = cloudFilteredObjBuffer_->front().pointcloud_filtered;
      cloudFilteredObjBuffer_->pop_front();
      mtx_->unlock();

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
        // ROS_INFO("[OdomEstimationThread] odom estimation time %f ms \n \n", elapsed_seconds.count());
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

      OdomAndCloudFilteredObj odomAndCloudFilteredObj;
      odomAndCloudFilteredObj.odom = laserOdometry;
      odomAndCloudFilteredObj.pointcloud_filtered = pointcloud_filtered_in;
      odomAndCloudFilteredObj.time = pointcloud_time;
      mtx_->lock();
      while(odomAndCloudFilteredObjBuf_->size() > maxQueueSize) odomAndCloudFilteredObjBuf_->pop_front();
      odomAndCloudFilteredObjBuf_->push_back(odomAndCloudFilteredObj);
      mtx_->unlock(); 
      // ROS_INFO_STREAM("[OdomEstimationThread] OdomAndCloud buffer size: "<< odomAndCloudFilteredObjBuf_->size());

      pubLaserOdometry.publish(laserOdometry);
    }
    
    //sleep 2 ms every time
    std::chrono::milliseconds dura(2);
    std::this_thread::sleep_for(dura);
  }
}
