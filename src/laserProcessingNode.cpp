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
#include "laserProcessingClass.h"

#include <pcl/console/print.h>


class LaserProcessingNode{
  private:
  std::mutex mtx; //mutex

  // node handler
  ros::NodeHandle* nh_;
  ros::NodeHandle* private_nh_;

  // sub & pub
  ros::Publisher pubEdgePoints_;
  ros::Publisher pubSurfPoints_;
  ros::Publisher pubLaserCloudFiltered_;
  ros::Subscriber subLaserCloud_;

  // tf2
  std::unique_ptr<tf2_ros::Buffer> tfBuffer_; // tf buffer - use for lookup transform between frames
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_; // helper to subscribe to tf tree

  //
  std::queue<sensor_msgs::PointCloud2> pointCloudBuf_; // PointCloud2 Buffer
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

  public:
  LaserProcessingNode(ros::NodeHandle* nh, ros::NodeHandle* private_nh)
  {
    nh_ = nh; //store nh
    private_nh_ = private_nh; //store private_nh
    // initialize tf_listener
    tfBuffer_ = std::make_unique<tf2_ros::Buffer>();
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

    // 1. initialize param
    initParam();

    // 2. setup subscriber
    subLaserCloud_ = nh_->subscribe<sensor_msgs::PointCloud2>("cloud", 100, &LaserProcessingNode::cloud_cb,this);
    
    // 3. setup publisher
    pubLaserCloudFiltered_ = nh_->advertise<sensor_msgs::PointCloud2>("cloud_filtered", 100);
    pubEdgePoints_ = nh_->advertise<sensor_msgs::PointCloud2>("cloud_edge", 100);
    pubSurfPoints_ = nh_->advertise<sensor_msgs::PointCloud2>("cloud_surf", 100); 

    // 4. start process thread
    process_thread_ = std::thread(&LaserProcessingNode::process,this);
  }
  void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &msg)
  {
    // just work around because multiscan intensity is "i" instead of "intensity"
    sensor_msgs::PointCloud2 cloud_copy = *msg;  
    for (auto &field : cloud_copy.fields) {
      if (field.name == "i") {
          field.name = "intensity";  // Remap 'i' to 'intensity'
      }
    }
    mtx.lock();
    pointCloudBuf_.push(cloud_copy); // push incoming pointcloud into buffer to be processed later
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
    
    lidar_param_.setScanPeriod(scan_period_);
    lidar_param_.setVerticalAngle(vertical_angle_);
    lidar_param_.setLines(scan_line_);
    lidar_param_.setMaxDistance(max_dis_);
    lidar_param_.setMinDistance(min_dis_);

    laserProcessing.init(lidar_param_);

    ROS_INFO_STREAM("[LaserProcessingNode] scan line is set to: "<< scan_line_);
    ROS_INFO_STREAM("[LaserProcessingNode] scan period is set to: "<< scan_period_);
    ROS_INFO_STREAM("[LaserProcessingNode] vertical angle is set to: "<< vertical_angle_);
    ROS_INFO_STREAM("[LaserProcessingNode] max distance is set to: "<< max_dis_);
    ROS_INFO_STREAM("[LaserProcessingNode] min distance is set to: "<< min_dis_);
    ROS_INFO_STREAM("[LaserProcessingNode] base_frame is set to: "<< base_frame_);
    ROS_INFO_STREAM("[LaserProcessingNode] odom_frame is set to: "<< odom_frame_);

  }

  void process()
  {
    while(true){
      if(!pointCloudBuf_.empty())
      {
        //1. take out the front data in buffer, and process it
        pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_raw(new pcl::PointCloud<pcl::PointXYZI>());
        mtx.lock(); //acquire lock
        pcl::fromROSMsg(pointCloudBuf_.front(), *pointcloud_raw);
        ros::Time pointcloud_time = (pointCloudBuf_.front()).header.stamp;
        pointCloudBuf_.pop();
        mtx.unlock(); //release lock

        // 2. transform the pointcloud to base_frame
        pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZI>());
        if(!doTransformPCL(base_frame_,*pointcloud_raw,*pointcloud_in))
        {
          ROS_ERROR("Failed To Transform PointCloud to Base Frame");
          continue;;
        }
        // 3. extract Feature in pointcloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge(new pcl::PointCloud<pcl::PointXYZI>());          
        pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf(new pcl::PointCloud<pcl::PointXYZI>());
        std::chrono::time_point<std::chrono::system_clock> start, end;
        start = std::chrono::system_clock::now();
        laserProcessing.featureExtraction(pointcloud_in,pointcloud_edge,pointcloud_surf);
        end = std::chrono::system_clock::now();
        std::chrono::duration<float> elapsed_seconds = end - start;
        total_frame_++;
        float time_temp = elapsed_seconds.count() * 1000;
        total_time_+=time_temp;
        // ROS_INFO("average laser processing time %f ms \n \n", total_time_/total_frame_);

        // 4. Combine both feature cloud and publish
        sensor_msgs::PointCloud2 laserCloudFilteredMsg;
        pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());  
        *pointcloud_filtered+=*pointcloud_edge;
        *pointcloud_filtered+=*pointcloud_surf;
        pcl::toROSMsg(*pointcloud_filtered, laserCloudFilteredMsg);
        laserCloudFilteredMsg.header.stamp = pointcloud_time;
        laserCloudFilteredMsg.header.frame_id = base_frame_;
        pubLaserCloudFiltered_.publish(laserCloudFilteredMsg);

        // 5. publish edge feature cloud
        sensor_msgs::PointCloud2 edgePointsMsg;
        pcl::toROSMsg(*pointcloud_edge, edgePointsMsg);
        edgePointsMsg.header.stamp = pointcloud_time;
        edgePointsMsg.header.frame_id = base_frame_;
        pubEdgePoints_.publish(edgePointsMsg);

        // 6. publish surface feature cloud
        sensor_msgs::PointCloud2 surfPointsMsg;
        pcl::toROSMsg(*pointcloud_surf, surfPointsMsg);
        surfPointsMsg.header.stamp = pointcloud_time;
        surfPointsMsg.header.frame_id = base_frame_;
        pubSurfPoints_.publish(surfPointsMsg);
      }
      //sleep 2 ms every time
      std::chrono::milliseconds dura(2);
      std::this_thread::sleep_for(dura);
    }
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
};

int main (int argc, char **argv)
{
  ros::init(argc, argv, "floam_laser_processing_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  LaserProcessingNode node(&nh,&private_nh);
  ros::spin();
}



