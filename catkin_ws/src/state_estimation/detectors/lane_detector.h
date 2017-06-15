/**
 * laneDetector.h
 * @description A class to detect the lane using OpenCV filters.
 * @authors Jana Pavlasek, Paul Wu
 */


#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PolygonStamped.h>
#include <std_srvs/SetBool.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

using namespace cv;
using namespace std;

class LaneDetector
{
public:
  LaneDetector(ros::NodeHandle& nh);

private:
  ros::Subscriber image_sub_;
  ros::Publisher lane_pub_;
  ros::ServiceServer toggle_;

  bool detect_;
  bool visualize_;

  /**
   * @brief Callback for the image topic.
   * Listens to the camera image and finds the lane position.
   * @param msg Image message.
   */
  void imageCallback(const sensor_msgs::Image::ConstPtr& msg);

  /**
   * @brief Callback for the set state service.
   * Turns detection on or off to save computational power when the lane
   * detector is not being used.
   * @param  req Request object.
   * @param  res Response object.
   */
  bool setStateCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

  /**
   * @brief Finds the lane from a list of contours.
   * @param  contours List of contours found in the image.
   * @param  img_size Size of image, for display purposes only.
   * @return          Lane message to publish.
   */
  geometry_msgs::PolygonStamped findLane(vector<vector<Point> > &contours, Size img_size);
};
