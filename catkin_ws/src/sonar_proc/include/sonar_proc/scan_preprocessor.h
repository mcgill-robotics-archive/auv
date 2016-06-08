/*
 * Scan Preprocessor node.
 * \author: Jana Pavlasek
 * lol guys i don't know cpp
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>

class ScanPreprocessor
{
public:
    ScanPreprocessor(ros::NodeHandle& nh);

private:
    ros::NodeHandle nh_;

    ros::Subscriber full_scan_sub_;
    ros::Publisher filtered_scan_pub_;

    ros::Timer updater_;

    sensor_msgs::PointCloud cloud_;

    void fullScanCallback(const sensor_msgs::PointCloud::ConstPrt& msg);
    void updateCallback(const ros::TimerEvent& e);
};
