# F450 Quadcopter - Deployment Guide

## Overview

This project supports three deployment scenarios:
1. **Development Machine with Docker** (Ubuntu 24.04) - For simulation and development
2. **Raspberry Pi Native** - For running on the actual drone hardware
3. **Demo/Sharing** - Docker-based for showing results on any PC

## Quick Start

### Universal Command
```bash
./run_drone.sh start  # Auto-detects environment and mode
```

## Development Machine (Ubuntu 24.04)

Since Ubuntu 24.04 doesn't support ROS2 Humble natively, use Docker:

```bash
# One-time setup
./start_simulation.sh setup

# Run simulation
./start_simulation.sh start

# Enter container for development
./start_simulation.sh shell

# Test motors in simulation
./start_simulation.sh motor-test
```

## Raspberry Pi Deployment

### Initial Setup (One Time)

1. **Clone repository on Raspberry Pi:**
```bash
git clone https://github.com/your-username/Quadcopter.git
cd Quadcopter
```

2. **Run setup script:**
```bash
chmod +x setup_rpi.sh
./setup_rpi.sh
# This installs ROS2 Humble, MAVROS2, and configures serial ports
# Reboot when prompted
```

3. **After reboot, verify setup:**
```bash
cd ~/Quadcopter
./run_drone.sh start  # Should auto-detect hardware mode
```

### Daily Usage on Raspberry Pi

```bash
# Terminal 1: Start MAVROS connection to Pixhawk
./run_drone.sh start

# Terminal 2: Test motors (REMOVE PROPELLERS!)
./run_drone.sh test

# Monitor connection
./run_drone.sh monitor
```

## Environment Auto-Detection

The `run_drone.sh` script automatically detects:
- **Environment**: Docker, Native ROS2, or Raspberry Pi
- **Mode**: Hardware (if Pixhawk connected) or Simulation

### Manual Override
```bash
# Force simulation mode
./run_drone.sh start --sim

# Force hardware mode
./run_drone.sh start --hw

# Force Docker even if native available
./run_drone.sh start --docker
```

## Switching Between Simulation and Hardware

The same code works for both! The only difference is the MAVROS connection URL:

| Mode | Connection URL | Auto-detected by |
|------|---------------|------------------|
| Simulation | `udp://:14540@127.0.0.1:14557` | No serial ports found |
| Hardware | `/dev/ttyAMA0:57600` | Serial port exists |

## File Structure for Deployment

```
Quadcopter/
├── run_drone.sh          # Universal runner (works everywhere)
├── setup_rpi.sh          # Raspberry Pi setup
├── start_simulation.sh   # Docker simulation control
├── docker/               # Docker files (dev machine only)
├── src/
│   └── drone_control/    # ROS2 package (same code for sim & hw)
└── algorithms/           # Python algorithms
```

## Workflow Examples

### Development Workflow (Ubuntu 24.04)
```bash
# 1. Develop in Docker
./start_simulation.sh shell
# Edit code in src/drone_control/
colcon build --packages-select drone_control

# 2. Test in simulation
./run_drone.sh test

# 3. Commit changes
git add . && git commit -m "Updated control algorithm"
git push
```

### Deployment to Raspberry Pi
```bash
# On Raspberry Pi:
cd ~/Quadcopter
git pull                  # Get latest changes
./run_drone.sh build      # Rebuild if needed
./run_drone.sh start      # Connect to real Pixhawk
```

### Demo on Another PC
```bash
# On any Linux machine with Docker:
git clone https://github.com/your-username/Quadcopter.git
cd Quadcopter
./start_simulation.sh setup
./start_simulation.sh start  # Full simulation demo
```

## Python Dependencies

Both environments use the same `requirements.txt`:
- **Docker**: Installed in container
- **Raspberry Pi**: Install with `pip3 install -r requirements.txt`

Key packages:
- `numpy`, `casadi` - For NMPC algorithms
- `pyros-genmsg` - ROS2 message generation
- `matplotlib` - Visualization

## Troubleshooting

### Raspberry Pi Issues

**Serial port not working:**
```bash
# Check serial port exists
ls -l /dev/ttyAMA0

# Verify user in dialout group
groups | grep dialout

# Check if Bluetooth disabled (required)
sudo systemctl status hciuart  # Should be inactive
```

**MAVROS can't connect:**
```bash
# Check Pixhawk wiring (TX→RX, RX→TX, GND→GND)
# Verify baud rate: 57600
# Try different serial port:
ros2 run mavros mavros_node --ros-args -p fcu_url:=/dev/ttyUSB0:57600
```

### Docker Issues

**GUI not working:**
```bash
xhost +local:docker  # Allow X11 forwarding
echo $DISPLAY        # Should show :0 or :1
```

**Out of space:**
```bash
docker system prune -a  # Clean up Docker
```

## Safety Notes

⚠️ **ALWAYS remove propellers when testing motors on hardware!**

The motor test (`motor_test_mavros2`) will:
1. Arm the drone
2. Spin motors at different speeds
3. Disarm after testing

## Performance Comparison

| Aspect | Docker (Dev) | Raspberry Pi (Native) |
|--------|-------------|----------------------|
| Setup | Easy, isolated | More complex, system-wide |
| Performance | Slower (virtualization) | Faster (native) |
| Resource Usage | Higher | Lower |
| Debugging | Easier (consistent env) | Hardware-specific issues |
| Deployment | Not suitable | Production ready |

## Next Steps

1. **For Development**: Use Docker on Ubuntu 24.04
2. **For Testing**: Deploy to Raspberry Pi frequently
3. **For Demos**: Share Docker setup with others

The beauty of this setup is that the same `src/drone_control/` code works everywhere - just the environment changes!