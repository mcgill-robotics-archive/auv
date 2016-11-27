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

using namespace cv;
using namespace std;
namespace enc = sensor_msgs::image_encodings;



float thresh=0.9;

float thetaLastTime=500;
float rho1LastTime=0;
float rho2LastTime=0;
//These 3 variables take record of the current data and performs as a feedback of the next call
int wrongTime=0;



LaneDetector::LaneDetector(ros::NodeHandle& nh)
{
  // If you have constants to declare, do that here. If they need to be tuned
  // often, make them ROS params.
  // ROS PUB/SUB
  image_sub_ = nh.subscribe<sensor_msgs::Image>("camera/image_down", 10, &LaneDetector::imageCallback, this);
  lane_pub_ = nh.advertise<auv_msgs::Lane>("state_estimation/lane", 10);
}

Mat im2bw(Mat src, double grayThresh){

    Mat dst;

    threshold(src,dst,255*grayThresh,255,CV_THRESH_BINARY);

    return dst;

}


void LaneDetector::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  // This function will be called every time the camera receives an image. That
  // may be too fast, in which case, skip some.

  // Do your processing of the image here. OpenCV will have functions to change
  // sensor_msgs/Image into an OpenCV data type.

  // The current image is in the msg variable.

  auv_msgs::Lane lane;
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

  Mat RGB = cv_ptr->image;

  if(! RGB.data )                              // Check for invalid input
    {
      return;
    }

  Mat gray;cvtColor( RGB, gray, CV_RGB2GRAY );

  Mat bw = im2bw(gray, thresh);

  Mat bwfilt, cdst;

  //median filtering

  medianBlur ( bw, bwfilt, 5 );

  //canny detection

  Canny(bwfilt, bw, 50, 150, 3);

  cvtColor(bw, cdst, CV_GRAY2BGR);

  //Hough transformation

  vector<Vec2f> lines;
  HoughLines(bw, lines, 1, CV_PI/180, 100, 0, 0 );

  if(lines.size() == 0){
    return;
  }

  struct cluster{
    float rho;
    float theta;
  };

  int len = lines.size();
  cluster s[len];
  //construct a struct array with length "len", equal to the size of "lines"

  s[0].rho = lines[0][0];
  s[0].theta = lines[0][1];

  for(int i = 1; i < len; i++){
    s[i].rho = 0.0;
    s[i].theta = 0.0;
  }
  //initializing the array

  int numCluster = 1;
  int flag = 0;
  //this variable takes the record of total number of length
  for(int i = 1; i < len; i++){
      flag = 0;
      for(int j = 0; j < numCluster; j++){

          if(abs(lines[i][0] - s[j].rho) < 45 && abs(lines[i][1] - s[j].theta) * 180 / CV_PI < 2){
              flag = -1;
              //j=-1 means the line is added to a cluster
              break;
          }

      }

      if(flag != -1){
          s[numCluster].rho = lines[i][0];
          s[numCluster].theta = lines[i][1];
          numCluster++;
      }
      //if the line is added as a new cluster, increase numCluster; otherwise increment
  }

  int found = 0;
  for(int i = 0; i < numCluster-1; i++){

      for(int j = i + 1; j < numCluster; j++){

          float thetaDiff = abs( s[i].theta - s[j].theta ) * 180 / CV_PI;
          if( thetaDiff < 3.1){
          //if the parallel pair of line is found, do the following thing:

              float theta1 = s[i].theta * 180 /CV_PI;
              float theta2 = s[j].theta * 180 /CV_PI;
              //take record of the current theta values

              if(theta1 > 90.0 || theta2 > 90.0){
                theta1 = theta1 - 180.0;
                theta2 = theta2 - 180.0;
              }
              //adjust the value of theta to proper range

              if(thetaLastTime != 500){
                  if(abs(thetaLastTime - theta1) < 10 && (s[i].rho + s[j].rho) / 2 - (rho1LastTime + rho2LastTime)/2 < 20){
                    //if the difference between the current theta and the theta last time is with in 10
                      lane.theta1 = theta1;
                      lane.theta2 = theta2;
                      lane.rho1 = s[i].rho;
                      lane.rho2 = s[j].rho;
                      //printf("%f\n", theta);
                      found=1;
                      thetaLastTime = theta1;
                      rho1LastTime = s[i].rho;
                      rho2LastTime = s[j].rho;
                  }
                  //publish data, and take record of the data published
                  else{
                    //if something else happens, it could be some random error.
                    if(wrongTime<10){
                      lane.theta1 = thetaLastTime;
                      lane.theta2 = thetaLastTime;
                      lane.rho1 = rho1LastTime;
                      lane.rho2 = rho2LastTime;

                      wrongTime++;
                    }
                    //if so, we publish the data we got last time.
                    else{
                      lane.theta1 = theta1;
                      lane.theta2 = theta2;
                      lane.rho1 = s[i].rho;
                      lane.rho2 = s[j].rho;
                      wrongTime=0;
                    }
                    //It can also be that the data we obtained previously is inaccurate.
                  }

              }

              else{
              //if it's the first time searching for lane
                    lane.theta1 = theta1;
                    lane.theta2 = theta2;
                    lane.rho1 = s[i].rho;
                    lane.rho2 = s[j].rho;
                    found=1;
                    thetaLastTime = theta1;
                    rho1LastTime = s[i].rho;
                    rho2LastTime = s[j].rho;
                //publish data.
              }

          }
      }
  }


    if(found==1){
        // When you're done, put your results in as the 4 corners of the lane message.
        lane_pub_.publish(lane);
    }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lane_detector");
  ros::NodeHandle nh;
  LaneDetector laneDetector(nh);

  ros::spin();
  return 0;
}
