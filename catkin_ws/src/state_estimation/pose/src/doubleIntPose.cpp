/*
 *==============================================================================
 *Name: Double Integration Pose
 *Creator: Jeremy Mallette
 *Last Updated: 30/10/2016
 *Updated By:
 *
 *Notes:
 *  This is a fairly inaccurate method of calculating the position from a set
 *      of acceleration data-points, given by the IMU.
 *
 *==============================================================================
 */

//Libraries etc. ---------------------------------------------------------------
#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"

//Definitions ------------------------------------------------------------------
class acceleration {
    public:
        double x;
        double y;
        double z;
        unsigned long t;
} accIn, accArr[3];

class velocity {
    public:
        double x;
        double y;
        double z;
        unsigned long t;
} velArr[2];


class position {
    public:
        double x;
        double y;
        float z;
} position;

//ROS --------------------------------------------------------------------------
ros::Publisher pub, pub2;
ros::Subscriber sub;

//Callback ---------------------------------------------------------------------
void dataCallback(const geometry_msgs::Vector3ConstPtr& data) {
    accIn.x = data->x;
    accIn.y = data->y;
    accIn.z = data->z;
    accIn.t = ros::Time::now().toNSec();
}

//Update Array -----------------------------------------------------------------
void update(int i) {
    accArr[i].x = accIn.x * pow(10.0,-18);
    accArr[i].y = accIn.y * pow(10.0,-18);
    accArr[i].z = accIn.z * pow(10.0,-18);
    accArr[i].t = accIn.t;
}

//Take First Derivative --------------------------------------------------------
void firstIntegral() {
    velArr[0].x = (accArr[1].x - accArr[0].x) * (accArr[1].t - accArr[0].t);
    velArr[0].y = (accArr[1].y - accArr[0].y) * (accArr[1].t - accArr[0].t);
    velArr[0].z = (accArr[1].z - accArr[0].z) * (accArr[1].t - accArr[0].t);
    velArr[0].t = accArr[1].t;

    velArr[1].x = (accArr[2].x - accArr[1].x) * (accArr[2].t - accArr[1].t);
    velArr[1].y = (accArr[2].y - accArr[1].y) * (accArr[2].t - accArr[1].t);
    velArr[1].z = (accArr[2].z - accArr[1].z) * (accArr[2].t - accArr[1].t);
    velArr[1].t = accArr[2].t;
}
//Take Second Derivative -------------------------------------------------------
void secondIntegral() {
    position.x += ((velArr[1].x - velArr[0].x) * (velArr[1].t - velArr[0].t)) / pow(10.0, -18);
    position.y += ((velArr[1].y - velArr[0].y) * (velArr[1].t - velArr[0].t)) / pow(10.0, -18);
    position.z += ((velArr[1].z - velArr[0].z) * (velArr[1].t - velArr[0].t)) / pow(10.0, -18);
}

//Publish Results --------------------------------------------------------------
void publish() {
    //Velocity
    geometry_msgs::TwistStamped intVelocity = geometry_msgs::TwistStamped();
    intVelocity.twist.linear.x = velArr[1].x;
    intVelocity.twist.linear.y = velArr[1].y;
    intVelocity.twist.linear.z = velArr[1].z;
    pub.publish(intVelocity);
    //Position
    geometry_msgs::PoseStamped doubleIntPose = geometry_msgs::PoseStamped();
    doubleIntPose.pose.position.x = position.x;
    doubleIntPose.pose.position.y = position.y;
    doubleIntPose.pose.position.z = position.z;
    pub2.publish(doubleIntPose);
}

//Print Data -------------------------------------------------------------------
void printData() {
    //Raw Data
    ROS_INFO("DEBUG: Acceleration from IMU: %lf", accIn.x);
    ROS_INFO("DEBUG: ROS Time: %li", accIn.t);
    ROS_INFO("DEBUG: %li", accArr[1].t);
    ROS_INFO("DEBUG: %li", accArr[0].t);
    ROS_INFO("DEBUG: Time difference sample: %li", (accArr[1].t - accArr[0].t));
    //Velocity
    ROS_INFO("Speed in x direction: %lf", velArr[1].x);
    ROS_INFO("Speed in y direction: %lf", velArr[1].y);
    //Position
    ROS_INFO("Position in x direction: %lf", position.x);
    ROS_INFO("Position in y direction: %lf\n", position.y);
}

//Shift Data -------------------------------------------------------------------
void dataShift() {
    for(int j = 0; j < 2; j++) {
        accArr[j].x = accArr[j+1].x;
        accArr[j].y = accArr[j+1].y;
        accArr[j].z = accArr[j+1].z;
        accArr[j].t = accArr[j+1].t;
    }
}

//Main -------------------------------------------------------------------------
int main (int argc, char **argv) {
    unsigned int count = 0;

    ros::init(argc, argv, "double_int_pose");
    ros::NodeHandle node;

    sub = node.subscribe("state_estimation/acc", 1000, &dataCallback);

    pub = node.advertise<geometry_msgs::TwistStamped>("imu/vel", 100);
    pub2 = node.advertise<geometry_msgs::PoseStamped>("imu/pose", 100);

    //Fill First Data Point
    ros::spinOnce();
    update(0);

    //Fill Second Data Point
    ros::spinOnce();
    update(1);

    ros::Rate r(10);
    while(ros::ok()) {
        //Updating Third Datapoint
        ros::spinOnce();
        update(2);

        //Calculate Integrals
        firstIntegral();
        secondIntegral();

        //Publish Results and shift data
        publish();
        printData();
        dataShift();

        r.sleep();
    }

    return 0;
}

/*
 *==============================================================================
 */
