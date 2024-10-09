// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#include "laserProcessing.h"
#include "laserProcessingROS.h"



LaserProcessingROS::LaserProcessingROS(std::mutex* mtx,ros::NodeHandle* nh, ros::NodeHandle* private_nh, tf2_ros::Buffer* tfBuffer, std::deque<CloudFilteredObj>* cloudFilteredObjBuffer )
{
  mtx_ = mtx;
  nh_ = nh; //store nh
  private_nh_ = private_nh; //store private_nh
  tfBuffer_ = tfBuffer;
  cloudFilteredObjBuffer_ = cloudFilteredObjBuffer;

  // 1. initialize param
  initParam();

  // 2. setup subscriber
  subLaserCloud_ = nh_->subscribe<sensor_msgs::PointCloud2>("cloud", 10, &LaserProcessingROS::cloud_cb,this);
  
  // 3. setup publisher
  pubLaserCloudFiltered_ = nh_->advertise<sensor_msgs::PointCloud2>("cloud_filtered", 10);
  pubEdgePoints_ = nh_->advertise<sensor_msgs::PointCloud2>("cloud_edge", 10);
  pubSurfPoints_ = nh_->advertise<sensor_msgs::PointCloud2>("cloud_surf", 10); 

  // 4. start process thread
  process_thread_ = std::thread(&LaserProcessingROS::process,this);
}
void LaserProcessingROS::cloud_cb(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  // ROS_INFO_STREAM("[LaserProcessingThread] Receive new Cloud");
  // just work around because multiscan intensity is "i" instead of "intensity"
  sensor_msgs::PointCloud2 cloud_copy = *msg;  
  for (auto &field : cloud_copy.fields) {
    if (field.name == "i") {
        field.name = "intensity";  // Remap 'i' to 'intensity'
    }
    else if (field.name == "distances") {
        field.name = "intensity";  // Remap 'i' to 'intensity'
    }
  }
  mtx_->lock();
  while(pointCloudBuf_.size() > maxQueueSize) pointCloudBuf_.pop_front();
  pointCloudBuf_.push_back(cloud_copy);
  mtx_->unlock();   
}
void LaserProcessingROS::initParam()
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

  ROS_INFO_STREAM("[LaserProcessingThread] scan line is set to: "<< scan_line_);
  ROS_INFO_STREAM("[LaserProcessingThread] scan period is set to: "<< scan_period_);
  ROS_INFO_STREAM("[LaserProcessingThread] vertical angle is set to: "<< vertical_angle_);
  ROS_INFO_STREAM("[LaserProcessingThread] max distance is set to: "<< max_dis_);
  ROS_INFO_STREAM("[LaserProcessingThread] min distance is set to: "<< min_dis_);
  ROS_INFO_STREAM("[LaserProcessingThread] base_frame is set to: "<< base_frame_);
  ROS_INFO_STREAM("[LaserProcessingThread] odom_frame is set to: "<< odom_frame_);

}

void LaserProcessingROS::process()
{
  while(true){
    // ROS_INFO("Laser processing thread");
    if(!pointCloudBuf_.empty())
    {
      //1. take out the front data in buffer, and process it
      pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_raw(new pcl::PointCloud<pcl::PointXYZI>());
      mtx_->lock(); //acquire lock
      pcl::fromROSMsg(pointCloudBuf_.front(), *pointcloud_raw);
      ros::Time pointcloud_time = (pointCloudBuf_.front()).header.stamp;
      pointCloudBuf_.pop_front();
      mtx_->unlock(); //release lock

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
      pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());  
      *pointcloud_filtered+=*pointcloud_edge;
      *pointcloud_filtered+=*pointcloud_surf;

      pointcloud_edge->header.stamp = pointcloud_time.toSec();
      pointcloud_surf->header.stamp = pointcloud_time.toSec();
      pointcloud_filtered->header.stamp = pointcloud_time.toSec();
 
      CloudFilteredObj cloudFilteredObj;
      cloudFilteredObj.pointcloud_edge = pointcloud_edge;
      cloudFilteredObj.pointcloud_surf = pointcloud_surf;
      cloudFilteredObj.pointcloud_filtered = pointcloud_filtered;
      cloudFilteredObj.time = pointcloud_time;
      mtx_->lock();
      while(cloudFilteredObjBuffer_->size() > maxQueueSize) cloudFilteredObjBuffer_->pop_front();
      cloudFilteredObjBuffer_->push_back(cloudFilteredObj);
      mtx_->unlock();

      // publish 
      // 4. Combine both feature cloud and publish
      sensor_msgs::PointCloud2 laserCloudFilteredMsg;
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

      // ROS_INFO_STREAM("[LaserProcessingThread] CloudFilteredBuf size: "<< cloudFilteredObjBuffer_->size());
    }
    //sleep 2 ms every time
    std::chrono::milliseconds dura(2);
    std::this_thread::sleep_for(dura);
  }
}

bool LaserProcessingROS::doTransformPCL(const std::string& target_frame,const pcl::PointCloud<pcl::PointXYZI> &cloud_in,pcl::PointCloud<pcl::PointXYZI> &cloud_out)
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

