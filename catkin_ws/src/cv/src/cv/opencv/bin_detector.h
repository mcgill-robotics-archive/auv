#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>
#include <std_srvs/SetBool.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PolygonStamped.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

class BinDetector
{
public:
  BinDetector(ros::NodeHandle& nh);
  ~BinDetector() {};

private:
  ros::Subscriber image_sub_;
  ros::Publisher bin_pub_;
  ros::ServiceServer toggle_;

  bool detect_;
  bool visualize_;
  float slope_threshold_;

  /**
   * @brief Callback for the image topic.
   * Listens to the camera image and finds the bin position.
   * @param msg Image message.
   */
  void imageCallback(const sensor_msgs::Image::ConstPtr& msg);

  geometry_msgs::PolygonStamped processContours(std::vector<std::vector<Point> >& contours, Size img_size);

  float getDistance(Point2f& pt1, Point& pt2);
  float getDistance(Point2f& pt1, Point2f& pt2);

  std::vector<geometry_msgs::Point32> combineRectangles(RotatedRect& rect1, RotatedRect& rect2, Size img_size);
  float getSlope(Point2f& pt1, Point2f& pt2);

  bool isRectangle(RotatedRect& rectangle, std::vector<Point>& contour);

  /**
   * @brief Callback for the set state service.
   * Turns detection on or off to save computational power when the lane
   * detector is not being used.
   * @param  req Request object.
   * @param  res Response object.
   */
  bool setStateCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
};
