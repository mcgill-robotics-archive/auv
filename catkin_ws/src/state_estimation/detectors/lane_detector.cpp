/**
 * laneDetector.cpp
 * @description A class to detect the lane using OpenCV filters.
 * @authors Jana Pavlasek, Paul Wu, Malcolm Watt
 */

#include "lane_detector.h"

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <geometry_msgs/Point32.h>


LaneDetector::LaneDetector(ros::NodeHandle& nh) : detect_(false)
{
  // PARAMS
  ros::param::param<bool>("~visualize_lane", visualize_, false);
  // PUBLISHERS & SUBSCRIBERS
  image_sub_ = nh.subscribe<sensor_msgs::Image>("camera_down/image_color", 1, &LaneDetector::imageCallback, this);
  lane_pub_ = nh.advertise<geometry_msgs::PolygonStamped>("state_estimation/lane", 10);
  toggle_ = nh.advertiseService("lane_detector/set_state", &LaneDetector::setStateCallback, this);
}

bool LaneDetector::setStateCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
  detect_ = req.data;
  res.success = true;

  if (req.data)
  {
    res.message = "Lane detection turned on.";
  }
  else
  {
    res.message = "Lane detection turned off.";
  }

  return true;
}

void LaneDetector::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  // Do not do any detection unless it is turned on.
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
  if(! bgr_image.data )
  {
    ROS_ERROR("No data was received.");
    return;
  }

  // Remove the blue channel.
  Mat BGRChannels[3];
  split(bgr_image, BGRChannels); // split the BGR channesl
  BGRChannels[0] = Mat::zeros(bgr_image.rows, bgr_image.cols, CV_8UC1);// removing blue channel
  merge(BGRChannels, 3, bgr_image); // pack the image

  // Convert input image to HSV.
  Mat hsv_image;
  cvtColor(bgr_image, hsv_image, COLOR_BGR2HSV);

  // Threshold the HSV image, keep only the orange pixels.
  Mat orange_filter;
  inRange(hsv_image, Scalar(0, 40, 40), Scalar(27, 255, 255), orange_filter);

  // Blur to remove black spots.
  medianBlur(orange_filter, orange_filter, 5);

  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;

  /// Find contours
  findContours(orange_filter, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

  /*
   * We need to only publish if the lane we find is actually a valid lane.
   */
  geometry_msgs::PolygonStamped lane = findLane(contours, orange_filter.size());
  lane_pub_.publish(lane);
}

geometry_msgs::PolygonStamped LaneDetector::findLane(vector<vector<Point> > &contours, Size img_size)
{
  geometry_msgs::PolygonStamped lane;

  std::vector<geometry_msgs::Point32> pts;

  lane.header.stamp = ros::Time::now();
  lane.header.frame_id = "down_cam";

  // If there are no contours, return an empty message.
  if (contours.size() == 0)
  {
    return lane;
  }

  // Assume that the lane is the object with the largest area.
  float max_area = 0.0;
  int max_idx = 0;

  // Find the contour with the largest area.
  for(int i = 0; i < contours.size(); i++)
  {
    float current_area = contourArea(contours[i]);
    if (current_area > max_area)
    {
      max_area = current_area;
      max_idx = i;
    }
  }

  // Find the rectangle of best fit to the lane contour.
  RotatedRect lane_rect;

  lane_rect = minAreaRect(Mat(contours[max_idx]));

  // We know the approximate ratio of the side lengths of the lane, which we use here to filter out false positives.
  float long_side = max(lane_rect.size.width, lane_rect.size.height);
  float short_side = min(lane_rect.size.width, lane_rect.size.height);

  float side_ratio = long_side / short_side;

  // Check that side_ratio is between the lower and upper bound of the lane ratio.
  if (side_ratio - LANE_DIM_RATIO_LOWER_BOUND <= LANE_DIM_RATIO_UPPER_BOUND - LANE_DIM_RATIO_LOWER_BOUND)
  {
    return extractLanePoints(lane, img_size);
  }
}

geometry_msgs::PolygonStamped extractLanePoints(geometry_msgs::PolygonStamped lane, Size img_size)
{
  // Get the points of the rectangle.
  Point2f rect_points[4];
  lane_rect.points(rect_points);

  // Draw contours and get the points as geometry_msgs/Point.
  Mat drawing = Mat::zeros(img_size, CV_8UC3);
  Scalar color = Scalar(0, 150, 255);

  for(int i = 0; i < 4; i++)
  {
    line(drawing, rect_points[i], rect_points[(i + 1) % 4], color, 5, 8);

    geometry_msgs::Point32 pt;
    pt.x = rect_points[i].x;
    pt.y = rect_points[i].y;
    pts.push_back(pt);
  }

  // Only visualize if requested.
  if (visualize_)
  {
    namedWindow("Lane!!", WINDOW_NORMAL);
    imshow("Lane!!", drawing);
    waitKey(10);
  }

  lane.polygon.points = pts;

  return lane;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lane_detector");
  ros::NodeHandle nh;
  LaneDetector laneDetector(nh);

  ros::spin();
  return 0;
}
