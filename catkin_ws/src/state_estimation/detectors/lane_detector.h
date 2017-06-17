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
  geometry_msgs::PolygonStamped lane_;
  std::vector<geometry_msgs::Point32> pts_;

  // The actual ratio is 8:1.
  const float LANE_DIM_RATIO_LOWER_BOUND = 12;
  const float LANE_DIM_RATIO_UPPER_BOUND = 5;
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
  void findLane(vector<vector<Point> > &contours, Size img_size);

  /**
   * @brief Given a valid bounding box, extract the vertices.
   * @param lane      The lane message to be populated.
   * @param img_size  Size of image, for display purposes only.
   * @return          Lane message to publish.
   */
  void extractLanePoints(Size img_size, RotatedRect lane_rect);

  /**
   * @brief Return true if the ratio of sides falls within an acceptable range of what we expect for the lane task.
   */
  bool rectangleSideRatioFilter(RotatedRect lane_rect);
};
