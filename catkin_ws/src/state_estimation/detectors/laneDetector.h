/**
 * laneDetector.h
 * @description A class to detect the lane using OpenCV filters.
 * @authors Jana Pavlasek, Paul Wu
 */

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <auv_msgs/Lane.h>

class LaneDetector
{
public:
  LaneDetector(ros::NodeHandle& nh);

private:
  ros::Subscriber image_sub_;
  ros::Publisher lane_pub_;

  /**
   * @brief Callback for the image topic.
   * Listens to the camera image and finds the lane position.
   * @param msg Image message.
   */
  void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
};
