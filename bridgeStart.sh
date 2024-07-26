#!/bin/bash

# Set the IP address for the ROS TCP bridge
export ROS_IP=192.168.0.225

# Source the ROS setup script
source install/setup.bash 

# Start the ROS TCP bridge
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=192.168.0.225
