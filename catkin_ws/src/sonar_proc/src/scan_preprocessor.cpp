
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include "sonar_proc/scan_preprocessor.h"

ScanPreprocessor::ScanPreprocessor(ros::NodeHandle& nh) :
  nh_(nh)
{
  full_scan_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/full_scan", 10, &ScanPreprocessor::fullScanCallback, this);
  filtered_scan_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("pcl_filtered", 1);

  updater_ = nh_.createTimer(ros::Duration(0.2), &ScanPreprocessor::updateCallback, this);
}

void ScanPreprocessor::fullScanCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  // Convert
  pcl_conversions::toPCL(*msg, *cloud);

  // Create the filtering object
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud(cloudPtr);
  sor.setLeafSize(0.01, 0.01, 0.01);
  sor.filter(cloud_filtered);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(cloud_filtered, output);

  // Publish the data
  filtered_scan_pub_.publish(output);
}

void ScanPreprocessor::updateCallback(const ros::TimerEvent& e)
{
  filtered_scan_pub_.publish(cloud_);
}
