#include "laserProcessingROS.h"
#include "odomEstimationROS.h"
#include "laserMappingROS.h"
#include "utils.h"

int main (int argc, char** argv)
{
  ros::init(argc, argv, "floam");
  std::mutex mtx; //mutex

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  std::deque<CloudFilteredObj> cloudFilteredObjBuf; // PointCloud2 Buffer
  std::deque<OdomAndCloudFilteredObj> odomAndCloudFilteredObjBuf; // PointCloud2 Buffer

	LaserProcessingROS laserProcessing(&mtx,&nh,&nh_private,&tfBuffer, &cloudFilteredObjBuf);
  OdomEstimationROS odomEstimation(&mtx,&nh,&nh_private,&cloudFilteredObjBuf,&odomAndCloudFilteredObjBuf);
  LaserMappingROS laserMapping(&mtx,&nh,&nh_private,&odomAndCloudFilteredObjBuf);

	ros::spin();

	return 0;
}