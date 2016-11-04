#!/bin/bash

# Setup McGill Robotics AUV workspace

# ROS package dependencies
echo "Installing ROS package dependencies..."
rosdep update
rosdep install -r -i --from-paths catkin_ws
echo

echo "Setup complete."
