/**
 * laneDetector.cpp
 * @description A class to detect the lane using OpenCV filters.
 * @authors Jana Pavlasek, Paul Wu
 */

#include <laneDetector.h>

LaneDetector::LaneDetector(ros::NodeHandle& nh)
{
  // If you have constants to declare, do that here. If they need to be tuned
  // often, make them ROS params.
  // ROS PUB/SUB
  image_sub_ = nh.subscribe<sensor_msgs::Image>("camera/image_down", 10, &LaneDetector::imageCallback, this);
  lane_pub_ = nh.advertise<auv_msgs::Lane>("state_estimation/lane", 10);
}

void LaneDetector::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  // This function will be called every time the camera receives an image. That
  // may be too fast, in which case, skip some.

  // Do your processing of the image here. OpenCV will have functions to change
  // sensor_msgs/Image into an OpenCV data type.

  // The current image is in the msg variable.

  auv_msgs::Lane lane;
  // When you're done, put your results in as the 4 corners of the lane message.
  lane_pub_.publish(lane);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lane_detector");
  ros::NodeHandle nh;
  LaneDetector laneDetector(nh);

  ros::spin();
  return 0;
}
