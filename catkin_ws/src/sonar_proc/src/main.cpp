
#include <ros/ros.h>
#include "sonar_proc/scan_preprocessor.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sonar_node");
    ros::NodeHandle nh("sonar_node");
    ScanPreprocessor process(nh);

    ros::spin();
    return 0;
}