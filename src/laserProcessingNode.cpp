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
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
// #include <tf/transform_datatypes.h>
// #include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>

//pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>

//local lib
#include "lidar.h"
#include "laserProcessingClass.h"


LaserProcessingClass laserProcessing;
std::mutex mutex_lock;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudBuf;
lidar::Lidar lidar_param;

ros::Publisher pubEdgePoints;
ros::Publisher pubSurfPoints;
ros::Publisher pubLaserCloudFiltered;

boost::shared_ptr<tf2_ros::Buffer> tfBuffer_; // tf buffer - use for lookup transform between frames
boost::shared_ptr<tf2_ros::TransformListener> tf_listener_; // helper to subscribe to tf tree
std::string base_frame_ = "base_footprint";

void velodyneHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
  mutex_lock.lock();
  pointCloudBuf.push(laserCloudMsg); // push incoming pointcloud into buffer to be processed later
  mutex_lock.unlock();   
}

bool doTransformPCL(const std::string& target_frame,const pcl::PointCloud<pcl::PointXYZI> &cloud_in,pcl::PointCloud<pcl::PointXYZI> &cloud_out)
{
  geometry_msgs::TransformStamped transform;
  try{
    //  Obtain the transform from lidar frame to base_frame
    transform = tfBuffer_->lookupTransform(target_frame,cloud_in.header.frame_id,ros::Time(0));
    pcl_ros::transformPointCloud(cloud_in,cloud_out,transform.transform);
    return true;
  }catch (tf2::TransformException & e){
    ROS_ERROR_STREAM("Lookup transform failed: "<<e.what());
    return false;
  }
}

double total_time =0;
int total_frame=0;

void laser_processing(){
  while(1){
    // check if there is data in buffer
    if(!pointCloudBuf.empty()){
      //1. take out the front data in buffer, and process it
      pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_raw(new pcl::PointCloud<pcl::PointXYZI>());
      mutex_lock.lock(); //acquire lock
      pcl::fromROSMsg(*pointCloudBuf.front(), *pointcloud_raw);
      ros::Time pointcloud_time = (pointCloudBuf.front())->header.stamp;
      pointCloudBuf.pop();
      mutex_lock.unlock(); //release lock

      // 2. transform the pointcloud to base_frame
      pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZI>());
      if(!doTransformPCL(base_frame_,*pointcloud_raw,*pointcloud_in))
      {
        ROS_ERROR("Failed To Transform PointCloud to Base Frame");
        continue;;
      }

      pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge(new pcl::PointCloud<pcl::PointXYZI>());          
      pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf(new pcl::PointCloud<pcl::PointXYZI>());

      std::chrono::time_point<std::chrono::system_clock> start, end;
      start = std::chrono::system_clock::now();
      laserProcessing.featureExtraction(pointcloud_in,pointcloud_edge,pointcloud_surf);
      end = std::chrono::system_clock::now();
      std::chrono::duration<float> elapsed_seconds = end - start;
      total_frame++;
      float time_temp = elapsed_seconds.count() * 1000;
      total_time+=time_temp;
      //ROS_INFO("average laser processing time %f ms \n \n", total_time/total_frame);

      sensor_msgs::PointCloud2 laserCloudFilteredMsg;
      pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());  
      *pointcloud_filtered+=*pointcloud_edge;
      *pointcloud_filtered+=*pointcloud_surf;
      pcl::toROSMsg(*pointcloud_filtered, laserCloudFilteredMsg);
      laserCloudFilteredMsg.header.stamp = pointcloud_time;
      laserCloudFilteredMsg.header.frame_id = base_frame_;
      pubLaserCloudFiltered.publish(laserCloudFilteredMsg);

      sensor_msgs::PointCloud2 edgePointsMsg;
      pcl::toROSMsg(*pointcloud_edge, edgePointsMsg);
      edgePointsMsg.header.stamp = pointcloud_time;
      edgePointsMsg.header.frame_id = base_frame_;
      pubEdgePoints.publish(edgePointsMsg);


      sensor_msgs::PointCloud2 surfPointsMsg;
      pcl::toROSMsg(*pointcloud_surf, surfPointsMsg);
      surfPointsMsg.header.stamp = pointcloud_time;
      surfPointsMsg.header.frame_id = base_frame_;
      pubSurfPoints.publish(surfPointsMsg);

    }
    //sleep 2 ms every time
    std::chrono::milliseconds dura(2);
    std::this_thread::sleep_for(dura);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "main");
  ros::NodeHandle nh;

  // initialize tf_listener
  tfBuffer_ = boost::make_shared<tf2_ros::Buffer>();
  tf_listener_ = boost::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

  int scan_line = 64;
  double vertical_angle = 2.0;
  double scan_period= 0.1;
  double max_dis = 60.0;
  double min_dis = 2.0;
  

  nh.getParam("/scan_period", scan_period); 
  nh.getParam("/vertical_angle", vertical_angle); 
  nh.getParam("/max_dis", max_dis);
  nh.getParam("/min_dis", min_dis);
  nh.getParam("/scan_line", scan_line);

  lidar_param.setScanPeriod(scan_period);
  lidar_param.setVerticalAngle(vertical_angle);
  lidar_param.setLines(scan_line);
  lidar_param.setMaxDistance(max_dis);
  lidar_param.setMinDistance(min_dis);

  laserProcessing.init(lidar_param);

  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 100, velodyneHandler);

  pubLaserCloudFiltered = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points_filtered", 100);

  pubEdgePoints = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_edge", 100);

  pubSurfPoints = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf", 100); 

  std::thread laser_processing_process{laser_processing};

  ros::spin();

  return 0;
}

