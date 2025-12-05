#\!/bin/bash

echo "Starting MAVROS2 connection to Pixhawk on /dev/ttyAMA0..."
echo "Press Ctrl+C to stop"

cd ~/drone
source /opt/ros/humble/setup.bash

# Start MAVROS2 with serial connection to Pixhawk
ros2 run mavros mavros_node --ros-args -p fcu_url:=/dev/ttyAMA0:57600
