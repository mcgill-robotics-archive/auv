#include "bin_detector.h"

#include <vector>
#include <utility>
#include <algorithm>

BinDetector::BinDetector(ros::NodeHandle& nh) :
  detect_(false),
  visualize_(false),
  slope_threshold_(10)
{
  image_sub_ = nh.subscribe<sensor_msgs::Image>("camera_down/image_color", 1, &BinDetector::imageCallback, this);
  bin_pub_ = nh.advertise<geometry_msgs::PolygonStamped>("state_estimation/bins", 10);
  toggle_ = nh.advertiseService("bin_detector/set_state", &BinDetector::setStateCallback, this);
}

void BinDetector::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  if (!detect_)
  {
    return;
  }

  cv_bridge::CvImageConstPtr cv_ptr;

  // Convert from ROS image to openCV image.
  try
  {
    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  Mat bgr_image = cv_ptr->image;

  // Check for invalid input.
  if (! bgr_image.data )
  {
    ROS_ERROR("No data was received.");
    return;
  }

  // Convert input image to HSV.
  Mat hsv_image;
  cvtColor(bgr_image, hsv_image, COLOR_BGR2HSV);

  Mat small_image;
  resize(hsv_image, small_image, Size(), 0.5, 0.5, INTER_CUBIC);

  // Threshold the HSV image, keep only the orange and white pixels.
  Mat orange_filter;
  inRange(small_image, Scalar(0, 0, 0), Scalar(180, 135, 255), orange_filter);

  // Blur to remove black spots.
  medianBlur(orange_filter, orange_filter, 5);

  // Find contours
  std::vector<std::vector<Point> > contours;
  std::vector<Vec4i> hierarchy;
  findContours(orange_filter, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

  geometry_msgs::PolygonStamped bins = processContours(contours, orange_filter.size());

  bin_pub_.publish(bins);
}

geometry_msgs::PolygonStamped BinDetector::processContours(std::vector<std::vector<Point> >& contours, Size img_size)
{
  geometry_msgs::PolygonStamped bins;
  bins.header.stamp = ros::Time::now();
  bins.header.frame_id = "robot";

  // Draw contours and get the points as geometry_msgs/Point.
  Mat drawing = Mat::zeros(img_size, CV_8UC3);

  std::vector<std::pair<float, int> > sorted_contours;
  for (size_t i = 0; i < contours.size(); ++i)
  {
    std::pair<float, int> curr_pair(contourArea(contours[i]), i);
    sorted_contours.push_back(curr_pair);
  }

  std::sort(sorted_contours.begin(), sorted_contours.end());
  size_t largest_area_num = std::min(contours.size(), static_cast<size_t>(2));

  // The significant clusters are the largest.
  std::vector<std::vector<Point> > significant_contours;
  for (size_t i = sorted_contours.size() - largest_area_num; i < sorted_contours.size(); ++i)
  {
    if (sorted_contours[i].first > 3000.0)
    {
      significant_contours.push_back(contours[sorted_contours[i].second]);
    }
  }

  // Find the rectangle of best fit to the lane contour.
  std::vector<RotatedRect> rectangles;
  for (size_t i = 0; i < significant_contours.size(); ++i)
  {
    RotatedRect curr_rect;
    curr_rect = minAreaRect(Mat(significant_contours[i]));

    rectangles.push_back(curr_rect);
  }

  if (significant_contours.size() < 2)
  {
    return bins;
  }

  Point2f vertices1[4];
  rectangles[0].points(vertices1);

  float vertical1 = getSlope(vertices1[0], vertices1[1]);
  float horizontal1 = getSlope(vertices1[1], vertices1[2]);

  // We want the horizontal slope to be lower.
  if (fabs(horizontal1) > fabs(vertical1))
  {
    float tmp = horizontal1;
    horizontal1 = vertical1;
    vertical1 = tmp;
  }

  Point2f vertices2[4];
  rectangles[1].points(vertices2);

  float vertical2 = getSlope(vertices2[0], vertices2[1]);
  float horizontal2 = getSlope(vertices2[1], vertices2[2]);

  // We want the horizontal slope to be lower.
  if (fabs(horizontal2) > fabs(vertical2))
  {
    float tmp = horizontal2;
    horizontal2 = vertical2;
    vertical2 = tmp;
  }

  // If the slopes are good enough, populate the rectangle.
  if (fabs(vertical1 - vertical2) < slope_threshold_ &&
      fabs(horizontal1 - horizontal2) < slope_threshold_)
  {
    bins.polygon.points = combineRectangles(rectangles[0], rectangles[1], img_size);
  }

  return bins;
}

std::vector<geometry_msgs::Point32> BinDetector::combineRectangles(RotatedRect& rect1, RotatedRect& rect2, Size img_size)
{
  std::vector<geometry_msgs::Point32> points;

  Point2f vertices1[4];
  rect1.points(vertices1);
  Point2f vertices2[4];
  rect2.points(vertices2);

  std::vector<std::pair<float, std::pair<size_t, size_t> > > dists;

  for (size_t i = 0; i < 4; ++i)
  {
    for (size_t j = 0; j < 4; ++j)
    {
      dists.push_back(std::make_pair(getDistance(vertices1[i], vertices2[j]), std::make_pair(i, j)));
    }
  }

  std::sort(dists.begin(), dists.end());

  Point2f draw_vertices[4];
  int draw_idx = 0;
  for (size_t i = 0; i < 4; ++i)
  {
    if (i != dists[0].second.first && i != dists[1].second.first)
    {
      geometry_msgs::Point32 pt;
      pt.x = vertices1[i].x;
      pt.y = vertices1[i].y;
      points.push_back(pt);
      draw_vertices[draw_idx] = vertices1[i];
      draw_idx++;
    }
  }

  for (size_t i = 0; i < 4; ++i)
  {
    if (i != dists[0].second.second && i != dists[1].second.second)
    {
      geometry_msgs::Point32 pt;
      pt.x = vertices2[i].x;
      pt.y = vertices2[i].y;
      points.push_back(pt);
      draw_vertices[draw_idx] = vertices2[i];
      draw_idx++;
    }
  }

  if (visualize_)
  {
    Mat drawing = Mat::zeros(img_size, CV_8UC3);
    for (int i = 0; i < 4; ++i)
    {
      line(drawing, draw_vertices[i], draw_vertices[(i+1)%4], Scalar(0,255,0));
    }

    namedWindow("bin contour!!", WINDOW_NORMAL);
    imshow("bin contour!!", drawing);
    waitKey(10);
  }

  return points;
}

float BinDetector::getSlope(Point2f& pt1, Point2f& pt2)
{
  return (pt2.y - pt1.y) / (pt2.x - pt1.x);
}

float BinDetector::getDistance(Point2f& pt1, Point& pt2)
{
  return sqrt(pow(pt1.x - pt2.x, 2) + pow(pt1.y - pt2.y, 2));
}

float BinDetector::getDistance(Point2f& pt1, Point2f& pt2)
{
  return sqrt(pow(pt1.x - pt2.x, 2) + pow(pt1.y - pt2.y, 2));
}

bool BinDetector::setStateCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
  detect_ = req.data;
  res.success = true;

  if (req.data)
  {
    res.message = "Bin detection turned on.";
  }
  else
  {
    res.message = "Bin detection turned off.";
  }

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "bin_detector");
  ros::NodeHandle nh;
  BinDetector binDetector(nh);

  ros::spin();
  return 0;
}
