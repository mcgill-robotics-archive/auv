#include <ros/ros.h>
#include <tf/transform_listener.h> 
#include "geometry_msgs/Vector3.h"

geometry_msgs::Vector3 vectorDifference(tf::Vector3 theoretical, geometry_msgs::Vector3 actual) {
	geometry_msgs::Vector3 differenceVector;

	differenceVector.x = theoretical.x() - actual.x;
	differenceVector.y = theoretical.y() - actual.y;
	differenceVector.z = theoretical.z() - actual.z; 

	return differenceVector;
}

void testcallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
	ros::Time time(ros::Time::now());
	tf::TransformListener listener;
	tf::StampedTransform slam_transform;
	listener.lookupTransform("/robot", "/gate", time, slam_transform);

	tf::Vector3 slam_vector = slam_transform.getOrigin();

	//Calculate difference between vectors 
	geometry_msgs::Vector3 difference = vectorDifference(slam_vector, *msg);

	ROS_INFO("SLAM and Acutal Difference \n"  
		"x: %f \n" 
		"y: %f \n" 
		"z: %f \n",
		difference.x, difference.y, difference.z );	
}
	
int main(int argc, char** argv) {
	ros::init(argc, argv, "verify_slam");
	ros::NodeHandle node;

	ros::Subscriber test_vector =  node.subscribe("noisy_relative_obj1_position", 1000, testcallback);
	
	ros::spin();

	return 0;
}
