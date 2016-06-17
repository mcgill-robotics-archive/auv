
#include <pcl/point_cloud.h>
#include "sonar_proc/scan_preprocessor.h"

ScanPreprocessor::ScanPreprocessor(ros::NodeHandle& nh) :
    nh_(nh)
{
    full_scan_sub_ = nh_.subscribe<sensor_msgs::PointCloud>("/full_scan", 10, &ScanPreprocessor::fullScanCallback);
    filtered_scan_pub_ = nh_.advertise<sensor_msgs::PointCloud>("pcl_filtered", 1);

    updater_ = nh_.createTimer(ros::Duration(0.2), &ScanPreprocessor::updateCallback, this);
}

void ScanPreprocessor::fullScanCallback(const sensor_msgs::PointCloud::ConstPtr& msg)
{
    cloud_ = *msg;
}

void ScanPreprocessor::updateCallback(const ros::TimerEvent& e)
{
    filtered_scan_pub_.publish(cloud_);
}
