//Libraries etc. ---------------------------------------------------------------
#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"

//ROS --------------------------------------------------------------------------
ros::Publisher pub;
ros::Subscriber sub;

//Main -------------------------------------------------------------------------
int main (int argc, char **argv) {
    ros::init(argc, argv, "double_int_pose_test");
    ros::NodeHandle node;

    pub = node.advertise<geometry_msgs::Vector3>("test/acc", 100);

    geometry_msgs::Vector3 acc = geometry_msgs::Vector3();
    acc.y = 0.0;
    acc.z = 0.0;

    ros::Duration d(0.1);
    for(int i = 0; i <= 30; i++) {
        acc.x = 1.0;
        pub.publish(acc);
        ros::spinOnce();
        d.sleep();
    }
    for(int k = 0; k <= 30; k++) {
        acc.x = 0.0;
        pub.publish(acc);
        ros::spinOnce();
        d.sleep();
    }
    for(int l = 0; l <= 30; l++) {
        acc.x = -1.0;
        pub.publish(acc);
        ros::spinOnce();
        d.sleep();
    }

    return 0;
}
