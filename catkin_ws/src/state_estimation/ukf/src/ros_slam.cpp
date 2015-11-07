#include "ros_slam.h"
#include <cmath>
#include "geometry_msgs/Vector3.h"
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include "auv_msgs/SlamEstimate.h"
#include <string.h>
#include <math.h>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/unordered_map.hpp>

ros_slam::ros_slam(ros::NodeHandle& node) :
  estimator(3),
  sub(node.subscribe("slam/measurement", 100, &ros_slam::dataCallback, this)),
  subDepth(node.subscribe("/state_estimation/depth", 100, &ros_slam::depthDataCallback, this)),
  pub(node.advertise<auv_msgs::SlamEstimate>("map_data", 100)),
  confidence_radius_srv(node.advertiseService("slam/confidence_radius",
      &ros_slam::confidenceRadiusCallback, this)),
  xy_confidence_radius_srv(node.advertiseService("slam/xy_confidence_radius",
      &ros_slam::xyConfidenceRadiusCallback, this)),
  covariance_srv(node.advertiseService("slam/covariance",
      &ros_slam::covarianceCallback, this)),
  currentIndex(0)
{}

void ros_slam::depthDataCallback(const std_msgs::Float64::ConstPtr& input)	{
	depthMeasurement = input->data;
	depthCovariance = 0.002;	//TODO : Fix
	estimator.updateDepth(depthMeasurement, depthCovariance);
}

void ros_slam::dataCallback(const auv_msgs::RangeBearingElevation::ConstPtr& input) {
  static tf::TransformBroadcaster broadcaster;
  
  measurement << log(input->range), input->bearing, input->elevation;
  covariance << input->ln_range_variance, input->bearing_variance,
      input->elevation_variance;
  if (map.find(input->name) == map.end())
  {
	  currentIndex += 3;	//Increment current index to accommodate for new object which is not already in hashmap
	  estimator.append(3);
	  map[input->name] = currentIndex;	//Link new name to new index
  }
  
  //TODO: Error handling
  int objectIndex = map.at(input->name);
  
  // TODO make sure this works well if the transforms don't exist or are being published slowly
  // there are various exceptions we still need to catch
  // transform from "/north" frame to the sensor frame
  // Also, we should switch to using the version that takes a fixed frame so
  // we can use odometry to compensate for sensor processing latency
  listener.lookupTransform("north", input->header.frame_id,
      input->header.stamp, tf_sensor_transform);
      
  tf::transformTFToEigen(tf_sensor_transform, sensor_transform);
  
  position = estimator.update(objectIndex, sensor_transform, measurement, covariance); 
  
  BOOST_FOREACH(map_type::value_type& item, map) {
    int i = item.second;
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion::getIdentity(),
            tf::Vector3(position(i), position(i + 1), position(i + 2))),
        ros::Time::now(),
        "north",
        boost::lexical_cast<std::string>(i)    
      )
    );
    
    auv_msgs::SlamEstimate estimate;
    estimate.ObjectID = item.first;
    estimate.xPos = position(i);
    estimate.yPos = position(i+1);
    MatrixXd covar = getCovariance(item.first);
    estimate.var_xx = covar(0,0);
    estimate.var_xy = covar(0,1);
    estimate.var_yy = covar(1,1);
    pub.publish(estimate);
  }

}

//TODO: Convert to the covariance relative to the robot
const MatrixXd ros_slam::getCovariance(std::string name, int size) {
  //TODO: Error handling. What if string is invalid?
  int index = map.at(name);
  return  estimator.estimator.covariance.block(0, 0, size, size)
       + estimator.estimator.covariance.block(index, index, size, size)
       - estimator.estimator.covariance.block(0, index, size, size)
       - estimator.estimator.covariance.block(index, 0, size, size);
}

// Returns the radius of the sphere of 95% confidence
bool ros_slam::confidenceRadiusCallback(auv_msgs::ConfidenceRadius::Request &req,
    auv_msgs::ConfidenceRadius::Response &res) {
  res.confidence_radius = 2. * sqrt(getCovariance(req.name).trace());
  return true;
}

// Returns the radius of the circle in the xy-plane of 95% confidence
bool ros_slam::xyConfidenceRadiusCallback(auv_msgs::XYConfidenceRadius::Request &req, 
    auv_msgs::XYConfidenceRadius::Response &res) {
  res.xy_confidence_radius =  2. * sqrt(getCovariance(req.name, 2).trace()); 
  return true;
}

bool ros_slam::covarianceCallback(auv_msgs::Covariance::Request &req,
    auv_msgs::Covariance::Response &res) {
  tf::matrixEigenToMsg(getCovariance(req.name), res.covariance);
  return true;
}


int main (int argc, char **argv) {
  ros::init(argc, argv, "slam_ukf");
  ros::NodeHandle node;
  // We have to give this object a name here because otherwise c++ doesn't
  // create it! WTF c++?
  ros_slam s(node);
  ros::spin();
  return 0;
}

