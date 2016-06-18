/*
 * Scan Preprocessor node.
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

class ScanPreprocessor
{
public:
  ScanPreprocessor(ros::NodeHandle& nh);

private:
  ros::NodeHandle nh_;

  ros::Subscriber full_scan_sub_;
  ros::Publisher filtered_scan_pub_;

  ros::Timer updater_;

  sensor_msgs::PointCloud2 cloud_;

  void fullScanCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void updateCallback(const ros::TimerEvent& e);
};
