/**
 * torpedoTargeter.h
 * @description A class to detect the torpedo target using OpenCV filters.
 * @authors Auguste Lalande , Jeremy Mallette
 */

#ifndef TORPEDO_TARGETER_H_
#define TORPEDO_TARGETER_H_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <auv_msgs/TorpedoTarget.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <std_srvs/SetBool.h>

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>


using namespace cv;
using namespace std;
namespace enc = sensor_msgs::image_encodings;


class TorpedoTargeter
{

public:
    TorpedoTargeter(ros::NodeHandle& nh);

private:
    ros::Subscriber image_sub_;
    ros::Publisher  torpedo_pub_;
    ros::ServiceServer toggle_;

    // Params
    bool m_detect;
    bool m_visualize;

    double scale_factor;
    int blur_value;
    int threshold_value ;
    float radius_acceptance_limit;

    bool setStateCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
    void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
};

#endif //END TORPEDO_TARGET_H
