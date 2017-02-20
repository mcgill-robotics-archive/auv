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
  torpedo_pub_ = nh.advertise<auv_msgs::TorpedoTarget>("state_estimation/torpedo_target", 10);
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


  vector<vector<Point> > inner_contours;
  vector<Vec4i> inner_hierarchy;
  // extract contours of inside target
  findContours(cropped, inner_contours, inner_hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

  // sort contours by area
  vector<pair<double, int> > contour_areas;

  for (int i = 0; i < inner_contours.size(); i++) {
    double a = contourArea(inner_contours[i], false);
    contour_areas.push_back(make_pair(a, i));
  }

  if (contour_areas.size() < 2) {
    ROS_WARN("No hole detected!");
    return;
  }

  sort(contour_areas.rbegin(), contour_areas.rend());
  int target_ind = contour_areas[1].second;

  rect = minAreaRect(Mat(inner_contours[target_ind]));
  // TODO check that min area rect is similar to contour area

  Moments M = moments(inner_contours[target_ind], false);
  int cx = M.m10 / M.m00;
  int cy = M.m01 / M.m00;

  circle(small_img, Point(cx, cy), 7, Scalar(0, 0, 255), 7);

  torpedoTarget.x_hole = cx;
  torpedoTarget.y_hole = cy;

  // Point2f rect_points[4];
  // rect.points(rect_points);
  // Scalar color = Scalar(0, 0, 255);
  // for(int i = 0; i < 4; i++)
  //   line(filtered, rect_points[i], rect_points[(i+1)%4], color, 2, 8 );

  // color = Scalar(255, 0, 0);
  // drawContours(filtered, contours, largest_contour_index, color, 2, 8, hierarchy, 0, Point());

  namedWindow("Torpedo", WINDOW_NORMAL);
  imshow("Torpedo", small_img);
  waitKey(10);

  torpedo_pub_.publish(torpedoTarget);
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "torpedo_targeter");
  ros::NodeHandle nh;
  TorpedoTargeter TorpedoTargeter(nh);

  ros::spin();
  return 0;
}
