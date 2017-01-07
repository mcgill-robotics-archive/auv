#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
//#include <pcl/features/pfh.h>

#include <math.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include "sonar_proc/scan_preprocessor.h"
#include "sonar_proc/pointcloud_converter.h"

/*Instead of the conversion done in each types, I want it to be done from the start*/

ScanPreprocessor::ScanPreprocessor(ros::NodeHandle& nh)
{
  // ROSPARAMS
  ros::param::param<double>("~intensity_threshold", INTENSITY_THRESHOLD_, 80.0);
  ros::param::param<double>("~radius_threshold", RADIUS_THRESHOLD_, 1.5);

  // ROSPARAMS for voxelGrid
  ros::param::param<double>("~x_leaf_size", X_LEAF_SIZE, 0.3);
  ros::param::param<double>("~y_leaf_size", Y_LEAF_SIZE, 0.3);
  ros::param::param<double>("~z_leaf_size", Z_LEAF_SIZE, 0.3);

  // ROSPARAMS for radiusOutlierRemoval
  ros::param::param<double>("~radius_search", RADIUS_SEARCH, 0.5);
  ros::param::param<double>("~min_neighbors_in_radius", MIN_NEIGHBORS_IN_RADIUS, 40.0);

  // ROS PUB/SUB
  full_scan_sub_ = nh.subscribe<sensor_msgs::PointCloud>("/full_scan", 10, &ScanPreprocessor::fullScanCallback, this);
  //filtered_scan_pub_ = nh.advertise<sensor_msgs::PointCloud>("pcl_filtered", 10);
  filtered_scan_pub_ = nh.advertise<sensor_msgs::PointCloud>("/filered_scan/pcl_filtered", 1);
}  

void ScanPreprocessor::fullScanCallback(const sensor_msgs::PointCloud::ConstPtr &msg)
{
  // Create holder for filtered message "cloud_filtered"and run basic filter.
  sensor_msgs::PointCloud cloud_filtered(radiusThresholdFilter(*msg));

  //Create object
  pcl::PCLPointCloud2 * cloud_pcl_filtered;
  cloud_pcl_filtered =  PointcloudConverter::PointCloudToPCLPointCloud2(cloud_filtered);
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud_pcl_filtered);

  //Assemble a local 3D grid over a given PointCloud, and downsamples + filters the data.
  voxelGrid(cloudPtr, cloud_pcl_filtered);

  //Removes outliers if the number of neighbors in a certain search radius is smaller than a given K
  radiusOutlierRemoval(cloudPtr, cloud_pcl_filtered);

  //PFHFeatures(cloudPtr, cloud_pcl_filtered);

  // Convert to ROS msg for publishing purposes.
  sensor_msgs::PointCloud2 output_pcl2;
  sensor_msgs::PointCloud output_pcl1;
  pcl_conversions::fromPCL(*cloud_pcl_filtered, output_pcl2);
  sensor_msgs::convertPointCloud2ToPointCloud(output_pcl2, output_pcl1);

  // Publish the data.
  filtered_scan_pub_.publish(output_pcl1);
}

sensor_msgs::PointCloud ScanPreprocessor::radiusThresholdFilter(const sensor_msgs::PointCloud &cloud)
{

  // Make holder for a new filtered cloud.
  sensor_msgs::PointCloud* filtered = new sensor_msgs::PointCloud;
  filtered->header = cloud.header;

  // Create a channel object for the intensities.
  sensor_msgs::ChannelFloat32* channel = new sensor_msgs::ChannelFloat32;
  channel->name = "intensity";

  // Create vector holders for points and values to go into the filtered cloud.
  std::vector<float> values;
  std::vector<geometry_msgs::Point32> points;

  // Iterate through all points in the cloud.
  for (int i = 0; i < cloud.points.size(); ++i)
  {
    // Get the intensity and radius of the given point.
    double radius = sqrt(pow(cloud.points.at(i).x, 2.0) + pow(cloud.points.at(i).y, 2.0));
    double intensity = cloud.channels.at(0).values.at(i);

    // Only add the point to the new cloud if it is under both thresholds.
    if (radius >= RADIUS_THRESHOLD_ && intensity >= INTENSITY_THRESHOLD_)
    {
      points.push_back(cloud.points.at(i));
      values.push_back(cloud.channels.at(0).values.at(i));
    }
  }

  // Populate message.
  channel->values = values;
  filtered->points = points;
  filtered->channels.push_back(*channel);

  return *filtered;
}

//Filtering algorithm
void ScanPreprocessor::voxelGrid(const pcl::PCLPointCloud2ConstPtr &cloud, pcl::PCLPointCloud2* ptr)
{
  //Create the filtering object
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(X_LEAF_SIZE, Y_LEAF_SIZE, Z_LEAF_SIZE);
  sor.filter(*ptr);

}

void ScanPreprocessor::radiusOutlierRemoval(const pcl::PCLPointCloud2ConstPtr &cloud, pcl::PCLPointCloud2* ptr)
{
  //Create the filtering object
  pcl::RadiusOutlierRemoval<pcl::PCLPointCloud2> sor;
  sor.setInputCloud(cloud);
  sor.setRadiusSearch(RADIUS_SEARCH);
  sor.setMinNeighborsInRadius(MIN_NEIGHBORS_IN_RADIUS);
  sor.filter(*ptr);

}

/*
void ScanPreprocessor::PFHFeatures(const pcl::PCLPointCloud2ConstPtr &cloud, pcl::PCLPointCloud2* ptr ){

  //Convert from pcl::PCLPointCloud2 to pcl::PointCloudXYZ
  pcl::PointCloud<pcl::PointXYZ> *temp_cloud = new pcl::PointCloud<pcl::PointXYZ>;
  pcl::fromPCL(cloud,*temp_cloud);

  //Create the PFH estimation class, and pass the input dataset+normals to it
  pcl::PFHEstimation<pcl::PointXYZ, pcl::PointNormal, pcl::PFHSignature125> pfh;
  pfh.setInputCloud (tem_cloud);
  pfh.setInputNormals (tem_cloud);

  // Create an empty kdtree representation, and pass it to the PFH estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree = new pcl::search::KdTree<pcl::PointXYZ> ();
  //pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ()); -- older call for PCL 1.5-
  pfh.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs = new pcl::PointCloud<pcl::PFHSignature125> ();

  pfh.setRadiusSearch (0.05);

  // Compute the features
  pfh.compute (*pfhs);

  //pcl::PCLPointCloud2 *ptr = new pcl::PCLPointCloud2;
  pcl_conversions::toPCL(pfhs, *ptr);

    }
*/


int main(int argc, char **argv)
{
    ros::init(argc, argv, "scan_preprocessor");
    ros::NodeHandle nh("filtered_scan");
    ScanPreprocessor process(nh);

    ros::spin();
    return 0;
}
