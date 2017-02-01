# Sonar Proc Package

This package processes sonar data in order to determine objects in the viscinity
of the Robot.

The goal is to do this in four stages:

1. Scan Stitcher: Stitches PointCloud slices together and publishes to /full_scan
2. Scan prepocessor : Filters the /full_scan sonar PointCloud and published to /filtered_scan
3. Meanshift Clustering : Cluster the filtered PointCloud data and publishes to /visualization_marker_array and sonar_proc/cluster_data
4. Cluster Interpreter: Determines the most significant cluster

## Utilities

Available utilities include:

* PointCloud to LaserScan: Converts to LaserScan (pretty lossy)


## Running Sonar Processing

```bash
roscore
rosbag play <path to your rosbag> --loop
roslaunch sonar_proc sonar_proc.launch
rosrun rviz rviz
```
