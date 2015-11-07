#ifndef ROS_SLAM_H_
#define ROS_SLAM_H_

#include "ros/ros.h"
#include "ukf_slam.h"
#include "auv_msgs/RangeBearingElevation.h"
#include "auv_msgs/ConfidenceRadius.h"
#include "auv_msgs/XYConfidenceRadius.h"
#include "auv_msgs/Covariance.h"
#include <tf/transform_listener.h>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"

class ros_slam
{
  public:
    ros_slam(ros::NodeHandle&);
            
  private:
    ros::Publisher pub;
    ros::Subscriber sub;
    ros::Subscriber subDepth;
    ros::ServiceServer confidence_radius_srv;
    ros::ServiceServer xy_confidence_radius_srv;
    ros::ServiceServer covariance_srv;
    ukf_slam estimator;
    tf::TransformListener listener;
    int currentIndex;
    typedef boost::unordered_map<std::string,int> map_type;
    map_type map;
    tf::StampedTransform tf_sensor_transform;
    Affine3d sensor_transform;
    double depthMeasurement;	//Holds depth measurements
    double depthCovariance;	//Holds covar of the above
    Vector3d measurement; //Holds the range bearing and elevation
    Vector3d covariance; // Holds the covariance of the above
    VectorXd position;
    void dataCallback(const auv_msgs::RangeBearingElevation::ConstPtr& input);
    void depthDataCallback(const std_msgs::Float64::ConstPtr& input);
    const MatrixXd getCovariance(std::string, int=3);
    bool confidenceRadiusCallback(auv_msgs::ConfidenceRadius::Request&,
        auv_msgs::ConfidenceRadius::Response&);
    bool xyConfidenceRadiusCallback(auv_msgs::XYConfidenceRadius::Request&,
        auv_msgs::XYConfidenceRadius::Response&);
    bool covarianceCallback(auv_msgs::Covariance::Request&,
        auv_msgs::Covariance::Response&);
};

#endif 
