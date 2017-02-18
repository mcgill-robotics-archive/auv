/**
 * torpedoTargeter.cpp
 * @description A class to detect the torpedo target using OpenCV filters.
 * @authors Auguste Lalande
 */

#include <torpedoTargeter.h>


RNG rng(12345);


TorpedoTargeter::TorpedoTargeter(ros::NodeHandle& nh) {
  image_sub_ = nh.subscribe<sensor_msgs::Image>(
    "camera_front/image_color", 1, &TorpedoTargeter::imageCallback, this);
  lane_pub_ = nh.advertise<auv_msgs::TorpedoTarget>("state_estimation/torpedo_target", 10);
}


void TorpedoTargeter::imageCallback(const sensor_msgs::Image::ConstPtr& msg) {

  auv_msgs::TorpedoTarget torpedoTarget;
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

  Mat img = cv_ptr->image;  // Converting from ROS image to openCV image

  if(!img.data) {
    printf("No data.\n");
    return;
  }

  Mat small_img;
  Mat filtered;

  // downsample and apply blur filter
  resize(img, small_img, Size(), 0.5, 0.5, INTER_CUBIC);
  medianBlur(small_img, filtered, 25);

  uint8_t* pixelPtr = (uint8_t*)filtered.data;
  int cn = filtered.channels();
  int nRows = filtered.rows;
  int nCols = filtered.cols;

  Mat gray(nRows, nCols, CV_8UC1, Scalar(0));
  uint8_t* grayPtr = (uint8_t*)gray.data;

  // create grayscale based on the rule that pixels where blue channel is less then green channel
  // should be black. And the inverse should be white
  for(int i = 0; i < nRows; i++) {
    for(int j = 0; j < nCols; j++) {
      if (pixelPtr[i * nCols * cn + j * cn] < pixelPtr[i * nCols * cn + j * cn + 1]) // B < G
        grayPtr[i * nCols + j] = 255;
    }
  }

  Mat grayCopy = gray.clone();
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
  // extract contours of apparent objects
  findContours(grayCopy, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

  int largest_area = 0;
  int largest_contour_index = 0;
  // find largest contour
  for (int i = 0; i < contours.size(); i++) {
    double a = contourArea(contours[i], false);
    if (a > largest_area) {
      largest_area = a;
      largest_contour_index = i;
    }
  }

  if (!largest_area) {
    ROS_WARN("No significant object in frame");
    return;
  }


  RotatedRect rect = minAreaRect(Mat(contours[largest_contour_index]));
  // TODO check that min area rect is similar to contour area


  // extract information only from within largest contour
  Mat cropped = Mat::zeros(gray.size(), CV_8U);
  Mat mask_image(gray.size(), CV_8U, Scalar(0));
  drawContours(mask_image, contours, largest_contour_index, Scalar(255), CV_FILLED);
  gray.copyTo(cropped, mask_image);

  // Point2f rect_points[4];
  // rect.points(rect_points);
  // Scalar color = Scalar(0, 0, 255);
  // for(int i = 0; i < 4; i++)
  //   line(filtered, rect_points[i], rect_points[(i+1)%4], color, 2, 8 );

  // color = Scalar(255, 0, 0);
  // drawContours(filtered, contours, largest_contour_index, color, 2, 8, hierarchy, 0, Point());

  namedWindow("Torpedo", WINDOW_NORMAL);
  imshow("Torpedo", cropped);
  waitKey(10);
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "torpedo_targeter");
  ros::NodeHandle nh;
  TorpedoTargeter TorpedoTargeter(nh);

  ros::spin();
  return 0;
}
