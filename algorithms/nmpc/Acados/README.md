# Acados NMPC Implementations

Production-ready NMPC implementations using the [Acados](https://docs.acados.org/) solver framework. These are optimized for real-time embedded control and integration with ROS2.

## Implementations

| Folder | Description | Use Case |
|--------|-------------|----------|
| `Quadcopter_nmpc/` | Basic NMPC with Euler angles | Standard trajectory tracking |
| `Quadcopter_nmpc_quaternions/` | NMPC with quaternion representation | Avoids gimbal lock, better for aggressive maneuvers |
| `FBL_Quadcopter/` | Feedback linearization MPC | Simplified control structure |
| `FBL_Quadcopter_DOB/` | FBL-MPC with Disturbance Observer | Robust to external disturbances |
| `X500_nmpc/` | NMPC for X500 platform | Different drone configuration |

## Which to Use?

- **Starting out?** Use `Quadcopter_nmpc/` for basic trajectory tracking
- **Aggressive flight?** Use `Quadcopter_nmpc_quaternions/` to avoid gimbal lock
- **Windy conditions?** Use `FBL_Quadcopter_DOB/` for disturbance rejection
- **X500 drone?** Use `X500_nmpc/` for correct dynamics model

## Running

Each implementation has a `main.py` entry point:

```bash
cd Quadcopter_nmpc
python main.py

cd ../Quadcopter_nmpc_quaternions
python main.py
```

## File Structure

Each implementation follows the same pattern:
- `main.py` - Entry point and simulation loop
- `Quadcopter.py` or similar - Dynamics model and Acados OCP setup
- `utils.py` - Plotting and helper functions

## Dependencies

Requires Acados to be installed. See [Acados installation guide](https://docs.acados.org/installation/).

## Documentation

For detailed theory and implementation analysis, see:
- [NMPC Implementation Wiki](https://github.com/Desmondfotock28/Quadcopter/wiki/NMPC-Implementation)
