#include <string>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Point.h>


void servoCallback(const geometry_msgs::Point::ConstPtr& msg) {
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(msg->x, msg->y, msg->z) );
    transform.setRotation( tf::createQuaternionFromRPY(0, 0, 0) );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "robot", "target"));

}


int main (int argc, char** argv) {
    ros::init(argc, argv, "target_broadcaster");
    ros::NodeHandle node;

    ros::Subscriber CVsub = node.subscribe("target", 10, servoCallback);

    ros::spin();

    return 0;
}

