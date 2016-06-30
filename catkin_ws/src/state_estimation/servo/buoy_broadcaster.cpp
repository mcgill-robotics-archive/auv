#include <string>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Point.h>

//TODO: Change msg type to match CV published topic
void servoCallback(const geometry_msgs::Point::ConstPtr& msg) {
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(msg->x, msg->y, msg->z) );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "robot", "buoy"));

}


int main (int argc, char** argv) {
    ros::init(argc, argv, "buoy_broadcaster");
    ros::NodeHandle node;
    
    //TODO: Change CVdummytopic to real CV published topic
    ros::Subscriber CVsub = node.subscribe("/buoy/red", 1000, servoCallback);

    ros::spin();

    return 0;
}

