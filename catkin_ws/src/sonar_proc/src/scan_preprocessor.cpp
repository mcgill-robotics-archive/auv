
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include "sonar_proc/scan_preprocessor.h"

ScanPreprocessor::ScanPreprocessor(ros::NodeHandle& nh)
{
  full_scan_sub_ = nh.subscribe<sensor_msgs::PointCloud>("/full_scan", 10, &ScanPreprocessor::fullScanCallback, this);
  filtered_scan_pub_ = nh.advertise<sensor_msgs::PointCloud>("pcl_filtered", 1);
}

void ScanPreprocessor::fullScanCallback(const sensor_msgs::PointCloud::ConstPtr& msg)
{
  // Convert to PointCloud2.
  sensor_msgs::PointCloud2 input;
  sensor_msgs::convertPointCloudToPointCloud2(*msg, input);

  // Container for original & filtered data.
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCLPointCloud2.
  pcl_conversions::toPCL(input, *cloud);

  // Create the filtering object
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud(cloudPtr);
  sor.setLeafSize(0.01, 0.01, 0.01);
  sor.filter(cloud_filtered);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output_pcl;
  pcl_conversions::fromPCL(cloud_filtered, output_pcl);

  // Convert back to PointCloud for backwards compatibility.
  sensor_msgs::PointCloud output;
  sensor_msgs::convertPointCloud2ToPointCloud(output_pcl, output);

  // Publish the data
  filtered_scan_pub_.publish(output);
}
