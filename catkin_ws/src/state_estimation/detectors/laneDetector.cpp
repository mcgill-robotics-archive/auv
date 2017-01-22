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



float bwThresh = 0.9;
float thetaDiffThresh = 1.5;
float thetaLastTime = 500;
float rho1LastTime = 0;
float rho2LastTime = 0;
//These 3 variables take record of the current data and performs as a feedback of the next call
int wrongTime = 0;
bool foundLastTime = 0;



LaneDetector::LaneDetector(ros::NodeHandle& nh)
{
  // If you have constants to declare, do that here. If they need to be tuned
  // often, make them ROS params.
  // ROS PUB/SUB
  image_sub_ = nh.subscribe<sensor_msgs::Image>("camera_down/image_color", 10, &LaneDetector::imageCallback, this);
  lane_pub_ = nh.advertise<auv_msgs::Lane>("state_estimation/lane", 10);
}

Mat im2bw(Mat src, double grayThresh){

    Mat dst;

    threshold(src,dst,255*grayThresh,255,CV_THRESH_BINARY);

    return dst;

}
//Implementation of the function "image to binary image"

void init(){
  thetaLastTime = 500;
  wrongTime = 0;
  rho1LastTime = 0;
  rho2LastTime = 0;
  foundLastTime = 0;
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

  Mat RGB = cv_ptr->image;                     // Converting from ROS image to openCV image

  if(! RGB.data )                              // Check for invalid input
    {
      printf("No data.\n");
      return;
    }

    printf("\n");
  // printf("thetaLastTime = %f\n", thetaLastTime);
  // printf("rho1LastTime = %f\n", rho1LastTime);
  // printf("rho2LastTime = %f\n", rho2LastTime);
  printf("wrongTime = %d\n", wrongTime);

  namedWindow("MyWindow", WINDOW_NORMAL); //create a window with the name "MyWindow"

  Mat gray;cvtColor( RGB, gray, CV_RGB2GRAY ); // Converting to gray image

  Mat bw = im2bw(gray, bwThresh);                // Converting to binary image

  Mat bwfilt, cdst;


  medianBlur ( bw, bwfilt, 5 );

  // median filtering

  Canny(bwfilt, bw, 50, 150, 3);

  // canny detection
  imshow("MyWindow", bw);

  waitKey(10);



  //Hough transformation section

  cvtColor(bw, cdst, CV_GRAY2BGR);

  vector<Vec2f> lines;
  HoughLines(bw, lines, 1, CV_PI/180, 100, 0, 0 );

  // for( size_t i = 0; i < lines.size(); i++ )
  // {
  //   float rho = lines[i][0], theta = lines[i][1];
  //   Point pt1, pt2;
  //   double a = cos(theta), b = sin(theta);
  //   double x0 = a*rho, y0 = b*rho;
  //   pt1.x = cvRound(x0 + 1000*(-b));
  //   pt1.y = cvRound(y0 + 1000*(a));
  //   pt2.x = cvRound(x0 - 1000*(-b));
  //   pt2.y = cvRound(y0 - 1000*(a));
  //   line( cdst, pt1, pt2, Scalar(0,0,255), 3, CV_AA);
  //
  // }
  // imshow("MyWindow", cdst);
  //
  // waitKey(10);


  if(lines.size() == 0){
    if(wrongTime >= 3){
      init();
    }
    return;
  }

  struct cluster{
    float rho;
    float theta;
  };
  // Constrcting a struct to cluster detected lines

  int len = lines.size();
  cluster s[len];
  // construct a struct array with length "len", equal to the size of "lines"

  s[0].rho = lines[0][0];
  s[0].theta = lines[0][1];

  for(int i = 1; i < len; i++){
    s[i].rho = 0.0;
    s[i].theta = 0.0;
  }
  // initializing the array

  int numCluster = 1;
  int flag = 0;
  // this variable takes the record of total number of length
  for(int i = 1; i < len; i++){
      flag = 0;
      for(int j = 0; j < numCluster; j++){

          if(abs(lines[i][0] - s[j].rho) < 45 && abs(lines[i][1] - s[j].theta) * 180 / CV_PI < 2){
              flag = -1;
              // j=-1 means the line is added to a cluster
              break;
          }

      }

      if(flag != -1){
          s[numCluster].rho = lines[i][0];
          s[numCluster].theta = lines[i][1];
          numCluster++;
      }
      // if the line is added as a new cluster, increase numCluster
  }

  Mat cdst1;
  cvtColor(bw, cdst1, CV_GRAY2BGR);
  for(int i=0;i<numCluster;i++){
      float rho = s[i].rho, theta = s[i].theta;
      Point pt1, pt2;
      double a = cos(theta), b = sin(theta);
      double x0 = a*rho, y0 = b*rho;
      pt1.x = cvRound(x0 + 1000*(-b));
      pt1.y = cvRound(y0 + 1000*(a));
      pt2.x = cvRound(x0 - 1000*(-b));
      pt2.y = cvRound(y0 - 1000*(a));
      line( cdst1, pt1, pt2, Scalar(0,0,255), 3, CV_AA);

      imshow("MyWindow", cdst1);
      waitKey(10);
      theta=theta*180/CV_PI;

      if(theta>90.0){
        theta=theta-180.0;
      }

      //printf("rho:%f\ntheta:%f\n", rho,theta);

  }
  bool wrongTimePlus = 0;
  bool found = 0;

  for(int i = 0; i < numCluster-1; i++){

      for(int j = i + 1; j < numCluster; j++){
          // if(i == 0 && j == 1){
          //   float theta1 = s[i].theta * 180 /CV_PI;
          //   float theta2 = s[j].theta * 180 /CV_PI;
          //   //take record of the current theta values
          //
          //   if(theta1 > 90.0 || theta2 > 90.0){
          //     theta1 = theta1 - 180.0;
          //     theta2 = theta2 - 180.0;
          //   }
          //   printf("Here I am.\n");
          //   printf("thetaLastTime - theta1: %f\n", abs(thetaLastTime - theta1));
          //   printf("rho difference with last time: %f\n", abs((s[i].rho + s[j].rho) / 2 - (rho1LastTime + rho2LastTime)/2));
          // }
          float rhoDiff = abs( s[i].rho - s[j].rho );
          float thetaDiff = abs( s[i].theta - s[j].theta ) * 180 / CV_PI;

          //printf("thetaDiff-%d%d = %f\n",i,j,thetaDiff);

          if( thetaDiff < thetaDiffThresh && rhoDiff > 50 && rhoDiff < 100){
          //if the parallel pair of line is found, do the following thing:

              float theta1 = s[i].theta * 180 /CV_PI;
              float theta2 = s[j].theta * 180 /CV_PI;
              //take record of the current theta values

              if(theta1 > 90.0 || theta2 > 90.0){
                theta1 = theta1 - 180.0;
                theta2 = theta2 - 180.0;
              }
              // adjust the value of theta to proper range

              // if(thetaLastTime != 500){
              if(foundLastTime == 1){
                  if(abs(thetaLastTime - theta1) < 10){//&& abs((s[i].rho + s[j].rho) / 2 - (rho1LastTime + rho2LastTime)/2) < 1){
                    // if the difference between the current theta and the theta last time is with in 10
                      printf("Not the first time.\n");
                      printf("theta difference with last time: %f\n", abs(thetaLastTime - theta1));
                      printf("rho difference with last time: %f\n", abs((s[i].rho + s[j].rho) / 2 - (rho1LastTime + rho2LastTime)/2));
                      wrongTime = 0;
                      lane.theta1 = theta1;
                      lane.theta2 = theta2;
                      lane.rho1 = s[i].rho;
                      lane.rho2 = s[j].rho;

                      // printf("%f\n", theta);
                      found=1;
                      foundLastTime = 1;
                      printf("rhoDIff = %f\n", rhoDiff);
                      printf("thetaDiff = %f\n", thetaDiff);
                      thetaLastTime = theta1;
                      rho1LastTime = s[i].rho;
                      rho2LastTime = s[j].rho;

                  }
                  // publish data, and take record of the data published
                  else{
                    found = 0;
                    // during the process of tracing, it's beneficial to avoid noise and keep sending
                    // the correct data(angle, position parameter)
                    // if something else happens, it could be some random error.
                    // if so, we publish the data collected last time.
                    // if(wrongTime <= 3){
                    //   printf("Position 2\n");
                    //   lane.theta1 = thetaLastTime;
                    //   lane.theta2 = thetaLastTime;
                    //   lane.rho1 = rho1LastTime;
                    //   lane.rho2 = rho2LastTime;
                    //   found = 1;
                    //   printf("rhoDIff = %f\n", rhoDiff);
                    //   printf("thetaDiff = %f\n", thetaDiff);
                    //   wrongTime++;
                    //   wrongTimePlus = 1;
                    // }
                    //
                    // else{
                    //   printf("Position 3\n");
                    //   // after 3 wrong times ,the lane is probably
                    //   // let found = 0, thetaLastTime = 500
                    //   lane.theta1 = theta1;
                    //   lane.theta2 = theta2;
                    //   lane.rho1 = s[i].rho;
                    //   lane.rho2 = s[j].rho;
                    //   printf("rhoDIff = %f\n", rhoDiff);
                    //   printf("thetaDiff = %f\n", thetaDiff);
                    //   found = 1;
                    //   wrongTime=0;
                    //   thetaLastTime = 500;
                    //}

                    // It can also be that the data we obtained previously is inaccurate.
                  }
              }

              else{// if thetaLastTime = 500
                // if it's the first time searching for lane
                    // if the lane found is reliable with 3 times found
                    printf("First time found!\n");
                    lane.theta1 = theta1;
                    lane.theta2 = theta2;
                    lane.rho1 = s[i].rho;
                    lane.rho2 = s[j].rho;
                    found = 1;
                    foundLastTime = 1;
                    printf("rhoDIff = %f\n", rhoDiff);
                    printf("thetaDiff = %f\n", thetaDiff);
                    thetaLastTime = theta1;
                    rho1LastTime = s[i].rho;
                    rho2LastTime = s[j].rho;
                // publish the original data.
              }
          }
      }
      if(found == 1){
        break;
      }
  }



    if(found == 1){
        lane_pub_.publish(lane);
        printf("Lane found.\n");
        printf("Lane message: %f %f %f %f\n", lane.theta1,lane.theta2, lane.rho1, lane.rho2);
    }

    else{

      if(wrongTime <= 3){
        printf("Lane not found but there is at least 1 line detected OR it could be random error OR quitting.\n");
        lane.theta1 = thetaLastTime;
        lane.theta2 = thetaLastTime;
        lane.rho1 = rho1LastTime;
        lane.rho2 = rho2LastTime;
        wrongTime++;

        if(foundLastTime == 1){
          printf("Message published.\n");
          lane_pub_.publish(lane);
          printf("Lane message: %f %f %f %f\n", lane.theta1,lane.theta2, lane.rho1, lane.rho2);
        }
      }
      else{
        init();
      }
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
