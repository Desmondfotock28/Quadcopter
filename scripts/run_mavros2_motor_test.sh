#\!/bin/bash

echo "========================================"
echo "ROS2 MAVROS2 Motor Test"
echo "========================================"
echo "This script will:"
echo "1. Launch MAVROS2 to connect to Pixhawk"
echo "2. Run motor test that will:"
echo "   - Wait for FCU connection"
echo "   - Arm the motors"
echo "   - Turn motors ON for 2 seconds (low throttle)"
echo "   - Turn motors OFF"
echo "   - Disarm and exit"
echo ""
echo "SAFETY WARNING: Ensure drone is secured and propellers are safe\!"
echo "Press Ctrl+C to cancel, or Enter to continue..."
read

# Change to drone directory
cd ~/drone

# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "Starting MAVROS2..."
# Launch MAVROS2 in background
ros2 run mavros mavros_node --ros-args -p fcu_url:=/dev/ttyAMA0:57600 &
MAVROS_PID=$\!

# Wait a moment for MAVROS to start
sleep 3

echo "Starting motor test..."
# Run the motor test
ros2 run drone_control motor_test_mavros2

# Kill MAVROS when done
echo "Stopping MAVROS2..."
kill $MAVROS_PID 2>/dev/null

echo "Motor test complete\!"
