#include <ros/ros.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl_conversions/pcl_conversions.h>

/*Class that allows you to convert pointcloud data to pointcloud2*/

class PointcloudConverter{

public:
  PointcloudConverter(void);
  ~PointcloudConverter(void);
  static pcl::PCLPointCloud2* PointCloudToPCLPointCloud2(const sensor_msgs::PointCloud &input);

private:

};

PointcloudConverter::PointcloudConverter(void){
}

PointcloudConverter::~PointcloudConverter(void){
}

pcl::PCLPointCloud2* PointcloudConverter::PointCloudToPCLPointCloud2(const sensor_msgs::PointCloud &input){

  /*Convert to PCLPointCloud2ConstPtr*/
  sensor_msgs::PointCloud2 output;
  sensor_msgs::convertPointCloudToPointCloud2(input, output);

  pcl::PCLPointCloud2 *cloud_PCL = new pcl::PCLPointCloud2;
  //void pcl_conversions::toPCL ( const sensor_msgs::PointCloud2 &  pc2, pcl::PCLPointCloud2 & pcl_pc2C
  pcl_conversions::toPCL(output, *cloud_PCL);

  //returns pcl::PCLPointCloud2
  return cloud_PCL;

}


