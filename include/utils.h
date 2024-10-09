#pragma once
#include <ros/ros.h>
// #include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <nav_msgs/Odometry.h>

// #include <pcl_ros/transforms.h>

struct CloudFilteredObj
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge;
  pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf;
  pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_filtered;
  ros::Time time;
};
struct OdomAndCloudFilteredObj
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_filtered;
  nav_msgs::Odometry odom;
  ros::Time time;
};