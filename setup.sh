#!/bin/bash

# Setup McGill Robotics AUV workspace

# Update Git repo
echo "Pulling latest chages..."
git pull && git submodule update --init --recursive
echo

# ROS package dependencies
echo "Installing ROS package dependencies..."
rosdep update
rosdep install -r -i --from-paths catkin_ws
echo

echo "Setup complete."
