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
  // The actual ratio is 8:1.
  float lane_dim_ratio_lower_bound_;
  float lane_dim_ratio_upper_bound_;
  float area_ratio_lower_bound_;
  float area_ratio_upper_bound_;

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
   * @return          Whether or not the lane we detect appears valid.
   */
  std::vector<geometry_msgs::Point32> findLane(vector<vector<Point> > &contours, Size img_size);

  /**
   * @brief Given a valid bounding box, extract the vertices.
   * @param img_size    Size of image, for display purposes only.
   * @param lane_rect   The bounding rectangle of the lane.
   */
  void extractLanePoints(Size img_size, RotatedRect lane_rect, std::vector<geometry_msgs::Point32> * pts);

  /**
   * @brief Return true if the ratio of sides falls within an acceptable range of what we expect for the lane task.
   * @param lane_rect   The bounding rectangle of the lane.
   * @return            True if the side ratio is within range, false otherwise.
   */
  bool rectangleSideRatioFilter(RotatedRect lane_rect);

  /**
   * @brief Return true if the ratio of areas falls within an acceptable range of what we expect for the lane task.
   * @param lane_rect   The bounding rectangle of the lane.
   * @param blob_area   The actual area of the blob detected by OpenCV.
   * @return            True if the side ratio is within range, false otherwise.
   */
  bool rectangleAreaRatioFilter(RotatedRect lane_rect, float blob_area);
};
