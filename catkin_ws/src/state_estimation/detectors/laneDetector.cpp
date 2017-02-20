/**
 * laneDetector.cpp
 * @description A class to detect the lane using OpenCV filters.
 * @authors Jana Pavlasek, Paul Wu
 */

#include <laneDetector.h>

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#if CV_MAJOR_VERSION == 2
// do opencv 2 code
#elif CV_MAJOR_VERSION == 3
// do opencv 3 code
#endif
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

namespace enc = sensor_msgs::image_encodings;


LaneDetector::LaneDetector(ros::NodeHandle& nh) :
  bwThresh(0.9),
  thetaDiffThresh(1.5),
  thetaLastTime(500),
  rho1LastTime(0),
  rho2LastTime(0),
  wrongTime(0),
  foundLastTime(0),
  detect_(false)
{
  // ROS PUB/SUB
  image_sub_ = nh.subscribe<sensor_msgs::Image>("camera_down/image_color", 10, &LaneDetector::imageCallback, this);
  lane_pub_ = nh.advertise<auv_msgs::Lane>("state_estimation/lane", 10);
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

// Implementation of the function "image to binary image".
Mat LaneDetector::imageToBinary(Mat src, double grayThresh){

  Mat dst;

  threshold(src, dst, 255*grayThresh, 255, CV_THRESH_BINARY);

  return dst;
}

void LaneDetector::init()
{
  thetaLastTime = 500;
  wrongTime = 0;
  rho1LastTime = 0;
  rho2LastTime = 0;
  foundLastTime = 0;
}

void LaneDetector::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  // Do not do any detection unless it is turned on.
  if (!detect_)
  {
    return;
  }

  cv_bridge::CvImageConstPtr cv_ptr;

  try
  {
    cv_ptr = cv_bridge::toCvShare(msg, enc::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Convert from ROS image to openCV image.
  Mat RGB = cv_ptr->image;

  // Check for invalid input
  if(! RGB.data )
  {
    ROS_ERROR("No data.\n");
    return;
  }

  ROS_INFO("wrongTime = %d\n", wrongTime);

  // Create a window to show the image.
  namedWindow("MyWindow", WINDOW_NORMAL);

  Mat gray;

  // Converting to gray image
  cvtColor(RGB, gray, CV_RGB2GRAY);

  // Converting to binary image
  Mat bw = imageToBinary(gray, bwThresh);

  Mat bw_filter, cdst;

  medianBlur(bw, bw_filter, 5);

  // Median filtering
  Canny(bw_filter, bw, 50, 150, 3);

  // Show debug window.
  imshow("MyWindow", bw);

  waitKey(10);

  // Hough transformation.
  cvtColor(bw, cdst, CV_GRAY2BGR);

  vector<Vec2f> lines;
  HoughLines(bw, lines, 1, CV_PI/180, 100, 0, 0);

  lane_pub_.publish(findLane(lines, bw));
}

auv_msgs::Lane LaneDetector::findLane(vector<Vec2f> lines, Mat bw)
{
  auv_msgs::Lane lane;

  if (lines.size() == 0)
  {
    if(wrongTime >= 3){
      init();
    }
    ROS_WARN("No lines were detected.");
    return lane;
  }

  // Constrcting a struct to cluster detected lines

  int len = lines.size();
  Cluster s[len];
  // construct a struct array with length "len", equal to the size of "lines"

  s[0].rho = lines[0][0];
  s[0].theta = lines[0][1];

  for(int i = 1; i < len; i++)
  {
    s[i].rho = 0.0;
    s[i].theta = 0.0;
  }
  // Initializing the array

  int numCluster = 1;
  int flag = 0;

  for(int i = 1; i < len; i++)
  {
    flag = 0;
    for(int j = 0; j < numCluster; j++)
    {
      if(abs(lines[i][0] - s[j].rho) < 45 && abs(lines[i][1] - s[j].theta) * 180 / CV_PI < 2){
        flag = -1;
        // j=-1 means the line is added to a cluster
        break;
      }
    }

    if(flag != -1)
    {
      s[numCluster].rho = lines[i][0];
      s[numCluster].theta = lines[i][1];
      numCluster++;
    }
    // if the line is added as a new cluster, increase numCluster
  }

  Mat cdst1;

  cvtColor(bw, cdst1, CV_GRAY2BGR);

  for(int i = 0; i < numCluster; i++)
  {
    float rho = s[i].rho, theta = s[i].theta;
    Point pt1, pt2;
    double a = cos(theta), b = sin(theta);
    double x0 = a*rho, y0 = b*rho;
    pt1.x = cvRound(x0 + 1000*(-b));
    pt1.y = cvRound(y0 + 1000*(a));
    pt2.x = cvRound(x0 - 1000*(-b));
    pt2.y = cvRound(y0 - 1000*(a));
    line(cdst1, pt1, pt2, Scalar(0,0,255), 3, CV_AA);

    imshow("MyWindow", cdst1);
    waitKey(10);
    theta = theta * 180 / CV_PI;

    if(theta > 90.0)
    {
      theta = theta - 180.0;
    }
  }

  bool found = false;

  for(int i = 0; i < numCluster - 1; i++)
  {
    for(int j = i + 1; j < numCluster; j++)
    {
        float rhoDiff = abs( s[i].rho - s[j].rho );
        float thetaDiff = abs( s[i].theta - s[j].theta ) * 180 / CV_PI;

        if( thetaDiff < thetaDiffThresh && rhoDiff > 50 && rhoDiff < 100)
        {
          //if the parallel pair of line is found, do the following thing:

          float theta1 = s[i].theta * 180 /CV_PI;
          float theta2 = s[j].theta * 180 /CV_PI;
          //take record of the current theta values

          if(theta1 > 90.0 || theta2 > 90.0)
          {
            theta1 = theta1 - 180.0;
            theta2 = theta2 - 180.0;
          }
          // adjust the value of theta to proper range

          // if(thetaLastTime != 500){
          if(foundLastTime == 1)
          {
            if(abs(thetaLastTime - theta1) < 10)
            {
              // if the difference between the current theta and the theta last time is with in 10
              ROS_INFO("Not the first time.\n");
              ROS_INFO("theta difference with last time: %f\n", abs(thetaLastTime - theta1));
              ROS_INFO("rho difference with last time: %f\n", abs((s[i].rho + s[j].rho) / 2 - (rho1LastTime + rho2LastTime)/2));
              wrongTime = 0;
              lane.theta1 = theta1;
              lane.theta2 = theta2;
              lane.rho1 = s[i].rho;
              lane.rho2 = s[j].rho;

              found = true;
              foundLastTime = 1;
              ROS_INFO("rhoDIff = %f\n", rhoDiff);
              ROS_INFO("thetaDiff = %f\n", thetaDiff);
              thetaLastTime = theta1;
              rho1LastTime = s[i].rho;
              rho2LastTime = s[j].rho;
            }
            // publish data, and take record of the data published
            else
            {
              found = false;
            }
          }
          else
          {
            // if it's the first time searching for lane
            // if the lane found is reliable with 3 times found
            ROS_INFO("First time found!\n");
            lane.theta1 = theta1;
            lane.theta2 = theta2;
            lane.rho1 = s[i].rho;
            lane.rho2 = s[j].rho;
            found = true;
            foundLastTime = 1;
            ROS_INFO("rhoDIff = %f\n", rhoDiff);
            ROS_INFO("thetaDiff = %f\n", thetaDiff);
            thetaLastTime = theta1;
            rho1LastTime = s[i].rho;
            rho2LastTime = s[j].rho;
            // publish the original data.
          }
        }
    }
      if(found)
      {
        break;
      }
  }

    if(found)
    {
      ROS_INFO("Lane found.\n");
      ROS_INFO("Lane message: %f %f %f %f\n", lane.theta1,lane.theta2, lane.rho1, lane.rho2);
      return lane;
    }

    else
    {
      if(wrongTime <= 3)
      {
        ROS_INFO("Lane not found but there is at least 1 line detected OR it could be random error OR quitting.\n");
        lane.theta1 = thetaLastTime;
        lane.theta2 = thetaLastTime;
        lane.rho1 = rho1LastTime;
        lane.rho2 = rho2LastTime;
        wrongTime++;

        if(foundLastTime == 1)
        {
          ROS_INFO("Message published.\n");
          ROS_INFO("Lane message: %f %f %f %f\n", lane.theta1, lane.theta2, lane.rho1, lane.rho2);
          return lane;
        }
      }
      else
      {
        init();
      }
    }

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
