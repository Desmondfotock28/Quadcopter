# ROS2 RealSense PX4 Offboard Controller

This workspace contains ROS2 nodes for the custom PX4 Gazebo Classic F450 quadcopter with an Intel RealSense D415 camera mounted on the front of the vehicle.

The main controller uses the RealSense RGB image to detect the target shape, draws bounding boxes, shows RGB and depth frames, and sends PX4 offboard position setpoints so the drone can hover and align side-to-side/up-down with the selected target.

## Workspace Layout

```text
ros2_realsense_ws/
  src/
    my_drone_vision/
      my_drone_vision/
        realsense_square_detector.py
        realsense_square_offboard_controller.py
      package.xml
      setup.py
    px4_msgs/
    realsense2_description/
    realsense_gazebo_plugin/
    realsense-ros/
```

## Requirements

- Ubuntu 22.04
- ROS2 Humble
- PX4 SITL
- Gazebo Classic
- Micro XRCE-DDS Agent
- OpenCV / `cv_bridge`
- PX4 ROS2 message package matching the PX4-Autopilot checkout

The controller expects the Gazebo Classic quadcopter model to publish these RealSense topics:

```text
/camera/color/image_raw
/camera/aligned_depth_to_color/image_raw
```

It also expects PX4 uORB DDS bridge topics such as:

```text
/fmu/out/vehicle_local_position_v1
/fmu/out/vehicle_status_v1
/fmu/out/vehicle_command_ack
/fmu/in/offboard_control_mode
/fmu/in/trajectory_setpoint
/fmu/in/vehicle_command
```

## Build

From the workspace root:

```bash
cd /home/udeme/ros2_realsense_ws
source /opt/ros/humble/setup.bash
PYTHONNOUSERSITE=1 colcon build --packages-select px4_msgs my_drone_vision --event-handlers console_direct+
```

Source the workspace after building:

```bash
source /opt/ros/humble/setup.bash
source /home/udeme/ros2_realsense_ws/install/setup.bash
```

## Run PX4 SITL

Start the custom F450 quadcopter in the custom Gazebo Classic world from the PX4-Autopilot checkout:

```bash
cd /home/udeme/PX4-Autopilot
make px4_sitl gazebo-classic_quad_f450_camera__test_world
```

In another terminal, start the Micro XRCE-DDS Agent:

```bash
MicroXRCEAgent udp4 -p 8888
```

Verify that ROS2 can see the camera and PX4 topics:

```bash
source /opt/ros/humble/setup.bash
source /home/udeme/ros2_realsense_ws/install/setup.bash
ros2 topic list
```

## Run The Controller

```bash
source /opt/ros/humble/setup.bash
source /home/udeme/ros2_realsense_ws/install/setup.bash
ros2 run my_drone_vision realsense_square_offboard_controller
```

The controller will:

- Subscribe to the RealSense RGB and aligned depth images.
- Detect the largest visible target shape.
- Publish annotated detections on `/camera/squares/annotated`.
- Publish bounding boxes on `/camera/squares/bounding_boxes`.
- Publish PX4 offboard position setpoints.
- Auto-request Offboard mode and arm by default.
- Stop publishing offboard heartbeat/setpoints when PX4 leaves Offboard mode, so external landing commands can take control.

## Useful Parameters

Run with ROS2 parameter overrides like this:

```bash
ros2 run my_drone_vision realsense_square_offboard_controller --ros-args -p display_scale:=0.35
```

Common parameters:

```text
image_topic                         /camera/color/image_raw
depth_topic                         /camera/aligned_depth_to_color/image_raw
show_window                         true
display_scale                       0.5
takeoff_height                      -1.5
lateral_local_axis                  y
lateral_axis_sign                   1.0
center_deadband_px                  45.0
max_lateral_velocity                0.08
max_vertical_velocity               0.2
position_limit_enabled              true
position_x_min_m                    -0.35
position_x_max_m                    2.95
position_y_min_m                    -6.70
position_y_max_m                    -0.10
auto_arm                            true
auto_offboard                       true
release_on_external_mode_change     true
```

If the drone moves in the wrong side direction, flip the lateral sign:

```bash
ros2 run my_drone_vision realsense_square_offboard_controller --ros-args -p lateral_axis_sign:=-1.0
```

If the lateral correction is mapped to the wrong PX4 local axis, switch it:

```bash
ros2 run my_drone_vision realsense_square_offboard_controller --ros-args -p lateral_local_axis:=x
```

To run without the OpenCV display window:

```bash
ros2 run my_drone_vision realsense_square_offboard_controller --ros-args -p show_window:=false
```

## Detector-Only Node

The package also includes a detector-only node that publishes annotated images and bounding boxes without commanding PX4:

```bash
source /opt/ros/humble/setup.bash
source /home/udeme/ros2_realsense_ws/install/setup.bash
ros2 run my_drone_vision realsense_square_detector
```

## Landing

The offboard controller releases control when PX4 leaves Offboard mode. To land from the PX4 shell or QGroundControl, first switch out of Offboard/command land. Once PX4 leaves Offboard, the controller stops sending heartbeat and trajectory setpoints.

Example from the PX4 shell:

```bash
commander land
```

## Notes

- The controller does not command forward/backward motion. It holds the initial local x/y position and only adjusts the configured lateral axis plus altitude.
- Yaw and yaw speed setpoints are sent as `NaN`, allowing PX4 to keep the current yaw instead of rotating the drone during arming/offboard entry.
- The RealSense optical frame uses image coordinates where x is right, y is down, and depth is forward. This controller uses image-center error for visual servoing and does not convert pixel coordinates into full world coordinates.
- Keep `px4_msgs` synchronized with the PX4-Autopilot version used for SITL.
