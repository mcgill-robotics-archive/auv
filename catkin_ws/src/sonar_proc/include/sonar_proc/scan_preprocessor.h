/*
 * Scan Preprocessor node.
 */

#include <pcl/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>

class ScanPreprocessor
{
public:
  ScanPreprocessor(ros::NodeHandle& nh);

private:
  ros::Subscriber full_scan_sub_;
  ros::Publisher filtered_scan_pub_;

  double INTENSITY_THRESHOLD_;
  double RADIUS_THRESHOLD_;
  double X_LEAF_SIZE;
  double Y_LEAF_SIZE;
  double Z_LEAF_SIZE;
  double RADIUS_SEARCH;
  double MIN_NEIGHBORS_IN_RADIUS;

  /**
   * @brief Callback for the /full_scan topic.
   * Listens to the full scan and filters it accordingly, before publishing a
   * filtered scan.
   * @param msg PointCloud message.
   */
  void fullScanCallback(const sensor_msgs::PointCloud::ConstPtr& msg);

  /*****************
  ** FILTER TYPES **
  ******************/

  /**.
   * @param  cloud Unfiltered ROS PointCloud.
   * @return       Filtered ROS PointCloud.
   */
  sensor_msgs::PointCloud voxelGrid(const sensor_msgs::PointCloud& cloud);

  sensor_msgs::PointCloud statisticalOutlierRemoval(const sensor_msgs::PointCloud& cloud); 

  sensor_msgs::PointCloud passThrough(const sensor_msgs::PointCloud& cloud);

  sensor_msgs::PointCloud radiusOutlierRemoval(const sensor_msgs::PointCloud& cloud);


  /**
   * @brief Radius removal and intensity threshold filter.
   * Removes all points a given radius from the center of the cloud, which
   * removes representations of the diver, robot, noise, etc. Also removes all
   * points less than a given intensity from the cloud. These two filters are
   * combined because they are the first, most basic filters always used and
   * it is desirable to avoid iterating through the cloud twice.
   * @param  cloud Unfiltered ROS PointCloud.
   * @return       Filtered ROS PointCloud.
   */
sensor_msgs::PointCloud radiusThresholdFilter(const sensor_msgs::PointCloud& cloud);

};

