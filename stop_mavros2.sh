#\!/bin/bash

echo "Stopping MAVROS2..."

# Kill any running mavros_node processes
pkill -f mavros_node

# Kill any ROS2 processes that might be hanging
pkill -f ros2

echo "MAVROS2 stopped."
