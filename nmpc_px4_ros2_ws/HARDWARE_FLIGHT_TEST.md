# NMPC Hardware Flight Test Deployment

Safety-first deployment workflow for running the NMPC controller on the Raspberry Pi 4B with a Pixhawk flight controller.

> **Important**
>
> The current NMPC controller uses the PX4 ROS2 interface and direct actuator setpoints. For hardware flight testing, the controller path is:
>
> ```text
> NMPC ROS2 Node (RPi 4B) -> Micro XRCE-DDS Agent -> Pixhawk -> PWM Outputs -> ESCs/Motors
> ```
>
> MAVROS2/MAVLink remains useful for hardware checks and the existing motor test, but the NMPC flight-mode node itself is not a MAVROS2 motor-test node.

## Safety Gates

Do not bypass these gates.

### Gate 1: Bench Validation

- Propellers removed.
- Frame secured.
- Battery restrained and accessible for immediate disconnect.
- RC transmitter powered, bound, and configured with a known disarm/kill/manual fallback.
- Pixhawk safety switch connected and visible.
- QGroundControl connected and showing vehicle status.
- Raspberry Pi connected to Pixhawk over the expected telemetry/DDS interface.

The safety switch should remain in the safe state while checking ROS2 topics, parameters, and trajectory publication.

### Gate 2: Actuator-Output Validation

Only enter this gate after Gate 1 passes.

- Propellers still removed.
- Operator has physical access to the battery connector.
- RC fallback has been tested.
- QGroundControl can disarm the vehicle.
- Pixhawk reports `pre_flight_checks_pass: true`.
- Pixhawk safety switch is intentionally pressed only when actuator output is expected.

For PX4 `vehicle_status`, the safety switch fields are:

```text
safety_button_available: true
safety_off: true
```

`safety_off: true` means the safety switch has been pressed and outputs are no longer locked by the safety switch.

### Gate 3: First Flight Attempt

Only enter this gate after actuator-output validation passes.

- Use a controlled open area.
- Start with conservative trajectory parameters, preferably static hover or very small motion.
- Keep the RC transmitter in hand.
- Keep QGroundControl visible.
- Have one person responsible only for abort/disarm.
- Do not test vision, SLAM, or aggressive trajectories during the first NMPC flight.

## Hardware Bringup

On the Raspberry Pi:

```bash
cd ~/Quadcopter/nmpc_px4_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Start the Micro XRCE-DDS Agent using the Pixhawk serial device used for PX4 ROS2 communication:

```bash
MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600
```

If your Pixhawk ROS2 bridge uses a different device, replace `/dev/ttyAMA0`.

## Read-Only Preflight Check

Run the read-only checker before launching the controller:

```bash
cd ~/Quadcopter/nmpc_px4_ros2_ws
./scripts/preflight_nmpc_hardware_check.sh
```

This check does not arm the vehicle and does not send motor commands.

## Safe Launch Sequence

### 1. Topic/odometry check only

This starts only non-actuating utility nodes:

```bash
ros2 launch nmpc_px4_ros2_bringup hardware_safety_bringup.launch.py
```

Default behavior:

```text
enable_nmpc_controller:=false
enable_reference_publisher:=false
use_rviz:=false
```

### 2. Add reference trajectory publishing

Use this after odometry and PX4 status topics are visible:

```bash
ros2 launch nmpc_px4_ros2_bringup hardware_safety_bringup.launch.py \
  enable_reference_publisher:=true \
  ref_traj:=static
```

Use `static` first. Do not start with `spiral`, `circle`, or aggressive motion for hardware testing.

### 3. Enable NMPC controller

Only enable this after the safety gates pass:

```bash
ros2 launch nmpc_px4_ros2_bringup hardware_safety_bringup.launch.py \
  enable_reference_publisher:=true \
  enable_nmpc_controller:=true \
  ref_traj:=static
```

The controller still does not arm by itself. Use QGroundControl and the RC transmitter as the operator-controlled arming and mode-selection interface.

When launched through `hardware_safety_bringup.launch.py`, the NMPC node also applies a software safety gate before publishing actuator setpoints:

```text
hardware_safety_gate: true
require_safety_switch: true
require_preflight_checks: true
```

With these settings, the controller waits for `/fmu/out/vehicle_status`, blocks actuator setpoint publication during failsafe, requires PX4 preflight checks to pass, and requires the Pixhawk safety switch to report `safety_off: true`.

## Abort Procedure

Use the fastest available safe action:

```text
1. Switch out of NMPC/Offboard/custom mode using RC or QGroundControl.
2. Disarm from RC or QGroundControl.
3. Use kill switch if configured and required.
4. Disconnect battery if the vehicle does not respond.
```

## Data To Record

- `/fmu/out/vehicle_status`
- `/fmu/out/vehicle_odometry`
- `/fmu/out/vehicle_control_mode`
- `ref_traj`
- `optimal_traj`
- NMPC node logs, including solver status and actuator setpoints
- QGroundControl messages
- External video of the test

## First Result To Capture

For the first hardware NMPC deployment result, record:

```text
Test name:
Date:
Location:
Propellers installed: no/yes
Safety switch available:
Safety switch state before arming:
RC fallback verified:
QGroundControl disarm verified:
PX4 ROS2 topics visible:
Reference trajectory:
NMPC controller enabled:
Arming method:
Mode switch method:
Observed behavior:
Issues:
Conclusion:
Evidence:
```
