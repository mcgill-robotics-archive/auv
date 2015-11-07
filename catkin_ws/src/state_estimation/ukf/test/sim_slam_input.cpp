#include <ros/ros.h>
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "auv_msgs/RangeBearingElevation.h"
#include <tf/transform_broadcaster.h>
#include <math.h>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/lexical_cast.hpp>

boost::mt19937 rng; // I don't seed it on purpouse (it's not relevant)
boost::normal_distribution<> nd(0.0, 1.0);
boost::variate_generator<boost::mt19937&, 
    boost::normal_distribution<> > var_nor(rng, nd);


geometry_msgs::Point update_position(geometry_msgs::Vector3 velocity, geometry_msgs::Point previous_position) {
	geometry_msgs::Point new_position;
	new_position.x = previous_position.x + velocity.x;
	new_position.y = previous_position.y + velocity.y;
	new_position.z = previous_position.z + velocity.z;
	return new_position;
}

geometry_msgs::Vector3 relative_position(geometry_msgs::Point robot_position, geometry_msgs::Point object_position) {
	geometry_msgs::Vector3 relative_position;
	relative_position.x = object_position.x - robot_position.x;
	relative_position.y = object_position.y - robot_position.y;
	relative_position.z = object_position.z - robot_position.z;
	return relative_position;
}

double range(geometry_msgs::Vector3 v) {
  return sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
}

double bearing(geometry_msgs::Vector3 v) {
  return atan2(v.y, v.x);
}

double elevation(geometry_msgs::Vector3 v) {
  return atan2(v.z, sqrt(v.x*v.x + v.y*v.y));
}

geometry_msgs::Vector3 add_noise(geometry_msgs::Vector3 vector, float sigma) {
        geometry_msgs::Vector3 noisy_vector;
        noisy_vector.x = vector.x + var_nor() * sigma;
        noisy_vector.y = vector.y + var_nor() * sigma;
        noisy_vector.z = 0.;;
        return noisy_vector;
}

int main(int argc, char **argv)
{
	srand(time(NULL));	

	ros::init(argc, argv, "sim_slam_input");

	ros::NodeHandle n;
	tf::TransformBroadcaster broadcaster;

	ros::Publisher velocity_pub = n.advertise<geometry_msgs::Vector3>("velocity", 1000);
	ros::Publisher position_pub = n.advertise<geometry_msgs::Vector3Stamped>("sim_slam/position/actual", 1000);	
	ros::Publisher noisy_velocity_pub = n.advertise<geometry_msgs::Vector3>("noisy_velocity", 1000);
	ros::Publisher noisy_position_pub = n.advertise<auv_msgs::RangeBearingElevation>("slam/measurement", 1000);
	
	//Update rate
	ros::Rate loop_rate(10);
	
	//Define starting position of the robot
	geometry_msgs::Point robot_position;
	robot_position.x = 0; robot_position.y = 0; robot_position.z = 0;

	//Define velocity of the robot
	geometry_msgs::Vector3 robot_velocity;
	robot_velocity.x = 0.0; robot_velocity.y = 0; robot_velocity.z = 0;
	
	//Define object locations
	const int numObjs = 4;
	geometry_msgs::Point objs[4];
  objs[0].x = 3; objs[0].y = 0; objs[0].z = 0;
  objs[1].x = 0; objs[1].y = 3; objs[1].z = 0;
  objs[2].x = -3; objs[2].y = 0; objs[2].z = 0;
  objs[3].x = 0; objs[3].y = -3; objs[3].z = 0;
    	
	//Define noise
	double ln_range_noise = 0.5;
	double bearing_noise = 0.1;
	double elevation_noise = 0.1;
	float v_noise = 1;

	geometry_msgs::Vector3 distance;
	geometry_msgs::Vector3 noisy_robot_velocity;

	while (ros::ok())
	{
		
		//Add noise to outputs
		noisy_robot_velocity = add_noise(robot_velocity, v_noise);
		
		//Publish outputs
		velocity_pub.publish(robot_velocity);
		noisy_velocity_pub.publish(noisy_robot_velocity);
		
		for (int i = 0; i < numObjs; i++) {
		  auv_msgs::RangeBearingElevation noisy_msg;
		  std_msgs::Header header;
		  header.frame_id = "/north";
		  header.stamp = ros::Time::now();
		  noisy_msg.header = header;
		  noisy_msg.name = boost::lexical_cast<std::string>(i);
		  distance = relative_position(robot_position, objs[i]);
		  noisy_msg.range = fabs((1+ln_range_noise*var_nor()) * range(distance));
		  noisy_msg.bearing = bearing(distance) + bearing_noise*var_nor();
		  noisy_msg.elevation = elevation(distance) + elevation_noise*var_nor();
		  noisy_msg.ln_range_variance = ln_range_noise*ln_range_noise;
		  noisy_msg.bearing_variance = bearing_noise*bearing_noise;
		  noisy_msg.elevation_variance = elevation_noise*elevation_noise;
		  noisy_position_pub.publish(noisy_msg);
		  geometry_msgs::Vector3Stamped position_msg;
		  header.frame_id = boost::lexical_cast<std::string>(i);
		  position_msg.header = header;
		  position_msg.vector = distance;
		  position_pub.publish(position_msg);
		}

		robot_position = update_position(robot_velocity, robot_position);
		
		ros::spinOnce();
		
		loop_rate.sleep();
	}
	
	return 0;

}
