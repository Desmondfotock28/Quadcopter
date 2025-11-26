# NMPC PX4 ROS2 Workspace

ROS2 workspace for NMPC-based quadcopter control with PX4 SITL (Software-In-The-Loop) simulation. This workspace integrates the NMPC algorithms with PX4 and Gazebo for testing before real hardware deployment.

## Packages

| Package | Description |
|---------|-------------|
| `nmpc_px4_ros2/` | Main NMPC flight mode node - trajectory tracking controller |
| `nmpc_px4_ros2_utils/` | Utility nodes (reference trajectory publisher, odometry republisher) |
| `nmpc_px4_ros2_interfaces/` | Custom ROS2 message/service definitions |
| `px4_msgs/` | PX4 message definitions for ROS2 |
| `px4-ros2-interface-lib/` | PX4-ROS2 interface library (third-party) |
| `3rd_party/` | Additional dependencies |

## Building

```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Build all packages
colcon build

# Source the workspace
source install/setup.bash
```

## Running

### Launch NMPC Controller with Visualization
```bash
ros2 launch nmpc_px4_ros2 bringup.launch.py
```

### Launch Reference Trajectory Publisher
```bash
ros2 run nmpc_px4_ros2_utils reference_trajectory_node
```

## Key Nodes

### NMPC Flight Mode Node
- Subscribes to: Vehicle odometry, reference trajectory
- Publishes to: Actuator commands (thrust + torques)
- Implements: Real-time NMPC using Acados solver

### Reference Trajectory Node
- Publishes: Reference trajectories (circle, spiral, figure-8, etc.)
- Configurable: Trajectory type, speed, radius

## Configuration

Parameters are defined in `nmpc_px4_ros2/config/nmpc_params.yaml`:
- Mass, gravity, thrust coefficients
- NMPC horizon and timing
- Trajectory parameters

## Documentation

For detailed information, see:
- [ROS Integration Wiki](https://github.com/Desmondfotock28/Quadcopter/wiki/ROS-Integration)
- [NMPC Implementation Wiki](https://github.com/Desmondfotock28/Quadcopter/wiki/NMPC-Implementation)
