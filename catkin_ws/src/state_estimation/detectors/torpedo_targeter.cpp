/**
 * torpedoTargeter.cpp
 * @description A class to detect the torpedo target using OpenCV filters.
 * @author Auguste Lalande , Jeremy Mallette
 */

#include "torpedo_targeter.h"

TorpedoTargeter::TorpedoTargeter(ros::NodeHandle& nh) :
    m_detect(true),
    m_visualize(true)
{
    ros::param::param<bool>("~visualize_torpedo", m_visualize, true);
    ros::param::param<double>("~torpedo_targeter/scale_factor", scale_factor, 0.5);
    ros::param::param<int>("~torpedo_targeter/blur_value", blur_value, 25);
    ros::param::param<int>("~torpedo_targeter/threshold_value", threshold_value, 128);
    ros::param::param<float>("~torpedo_targeter/radius_acceptance_limit", radius_acceptance_limit, 0);

    image_sub = nh.subscribe<sensor_msgs::Image>("camera_front/image_color", 1, &TorpedoTargeter::imageCallback, this);
    /*FOR COMP*/torpedo_pub = nh.advertise<geometry_msgs::PolygonStamped>("geometry_msgs/PolygonStamped", 10);
    toggle = nh.advertiseService("torpedo_targeter/set_state", &TorpedoTargeter::setStateCallback, this);
}

bool TorpedoTargeter::setStateCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
  m_detect = req.data;
  res.success = true;

  if (req.data)
  {
    res.message = "Torpedo targeter turned on.";
  }
  else
  {
    res.message = "Torpedo targeter turned off.";
  }

  return true;
}

void TorpedoTargeter::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    if (!m_detect)
    {
        return;
    }

    // I/O
    cv_bridge::CvImageConstPtr cv_ptr;
    geometry_msgs::PolygonStamped target;                      // FOR COMP
    //auv_msgs::TorpedoTarget torpedoTarget;

    // Images
    Mat input_img,
        filter_img,
        small_img,
        gray_img,
        thresh_img,
        border_contour_img,
        border_img,
        disp_img;

    // Contours
    vector<vector<Point> > border_contours;
    vector<Vec4i> border_hierarchy;
    vector<vector<Point> > target_contours;
    vector<Vec4i> target_hierarchy;

    // Detection
    RotatedRect border;
    vector<Point2f> centres;
    vector<float> radii;

    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Ros Image -> OpenCV Image
    input_img = cv_ptr->image;

    if(!input_img.data)
    {
        ROS_WARN("No data!");
        return;
    }

    // Remove Blue Channel
    Mat bgrChannels[3];
    split(input_img, bgrChannels);
    bgrChannels[0] = Mat::zeros(input_img.rows, input_img.cols, CV_8UC1);
    merge(bgrChannels, 3, input_img);   // Repack into the input_img

    // Filtering (Downsample and Blur)
    resize(input_img, small_img, Size(), scale_factor, scale_factor, INTER_CUBIC);
    medianBlur(small_img, filter_img, blur_value);

    // Convert to Grayscale
    cvtColor(filter_img, gray_img, CV_BGR2GRAY);

    // Threshold and Find Contours
    threshold(gray_img, thresh_img, threshold_value, 255, CV_THRESH_BINARY);
    findContours(border_contour_img, border_contours, border_hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    int largest_area = 0;
    int largest_contour_index = 0;
    // find largest contour
    for (int i = 0; i < border_contours.size(); i++)
    {
        double area = contourArea(border_contours[i], false);
        if (area > largest_area)
        {
            largest_area = area;
            largest_contour_index = i;
        }
    }

    if (!largest_area)  // Maybe put in an actual min value
    {
        ROS_WARN("No significant object in frame");
        vector<geometry_msgs::Point32> empty_pts;
        target.header.stamp = ros::Time::now();
        target.header.frame_id = "camera_front";
        target.polygon.points = empty_pts;
        torpedo_pub.publish(target);
        return;
    }

    // TODO Either during the for loop or after, use color/size ratios to do validation...
    border = minAreaRect(Mat(border_contours[largest_contour_index]));

    // FOR COMP
    Size contour_img_size = border_contour_img.size();
    target.header.stamp = ros::Time::now();
    target.header.frame_id = "camera_front";
    target.polygon.points = extractLanePoints(contour_img_size, border);
    torpedo_pub.publish(target);

    return;

    // DEAD CODE FOR NOW =======================================================
    //
    //    |
    //    |
    //    |
    //   \ /
    //    '

    // Copy all pixels within the largest contour to another image
    border_img = Mat::zeros(border_contour_img.size(), CV_8U);
    Mat mask_image(border_contour_img.size(), CV_8U, Scalar(0));
    drawContours(mask_image, border_contours, largest_contour_index, Scalar(255), CV_FILLED);
    border_contour_img.copyTo(border_img, mask_image);

    // Find contours within this largest contour
    findContours(border_contour_img, target_contours, target_hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    for (int j = 0; j < target_contours.size(); j++)
    {
        Point2f centre;
        float radius;

        minEnclosingCircle(target_contours[j], centre, radius);

        if (radius > radius_acceptance_limit)
        {
            centres.push_back(centre);
            radii.push_back(radius);
        }
    }

    // Draw Visulize Output
    if (m_visualize)
    {
        disp_img = border_contour_img.clone();

        for (int k = 0; k < target_contours.size(); k++)
        {
            drawContours(disp_img, target_contours, k, Scalar(255, 0, 0), 2, 8, target_hierarchy, 0, Point());
        }

        for (int l = 0; l < centres.size(); l++)
        {
            circle(disp_img, centres[l], 5, Scalar(0, 0, 255), -1, 8);
        }

        namedWindow("~ Torpedo ~", WINDOW_NORMAL);
        imshow("~ Torpedo ~", disp_img);
    }

    waitKey(10);

    if (!centres.empty())
    {
        ROS_DEBUG("Possible targets identified.");
        //torpedo_pub_.publish(TODO);
    }
    else
    {
        ROS_DEBUG("No possible targets identified.");
        //torpedo_pub_.publish(TODO)
    }
}

vector<geometry_msgs::Point32> TorpedoTargeter::extractLanePoints(Size& img_size, RotatedRect& lane_rect)
{
    // Get the points of the rectangle.
    Point2f rect_points[4];
    lane_rect.points(rect_points);
    vector<geometry_msgs::Point32> pts;

    for(int i = 0; i < 4; i++)
    {
        geometry_msgs::Point32 pt;
        pt.x = rect_points[i].x;
        pt.y = rect_points[i].y;
        pts.push_back(pt);
    }

    return pts;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "torpedo_targeter");
    ros::NodeHandle nh;
    TorpedoTargeter TorpedoTargeter(nh);

    ros::spin();
    return 0;
}
