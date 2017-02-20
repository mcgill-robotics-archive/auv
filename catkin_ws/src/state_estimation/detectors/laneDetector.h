/**
 * laneDetector.h
 * @description A class to detect the lane using OpenCV filters.
 * @authors Jana Pavlasek, Paul Wu
 */


#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <auv_msgs/Lane.h>
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
  ros::NodeHandle nh;

  float bwThresh;
  float thetaDiffThresh;
  float thetaLastTime;
  float rho1LastTime;
  float rho2LastTime;
  //These 3 variables take record of the current data and performs as a feedback of the next call
  int wrongTime;
  bool foundLastTime;
  bool detect_;

  struct Cluster
  {
    float rho;
    float theta;
  };

  /**
   * @brief Callback for the image topic.
   * Listens to the camera image and finds the lane position.
   * @param msg Image message.
   */
  void imageCallback(const sensor_msgs::Image::ConstPtr& msg);

  bool setStateCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

  /**
   * Resets a bunch of paramaters.
   */
  void init();

  /**
   * Creates some matrix of some image or something.
   * @param  src        [description]
   * @param  grayThresh [description]
   * @return            [description]
   */
  Mat imageToBinary(Mat src, double grayThresh);

  auv_msgs::Lane findLane(vector<Vec2f> lines, Mat bw);
};
