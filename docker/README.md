# Docker Environment for F450 Drone Simulation

This directory contains the Docker configuration for the F450 drone simulation environment.

## 📁 Files in this Directory

- **`Dockerfile`** - Defines the container with ROS2 Humble, PX4 SITL, Gazebo, and MAVROS2
- **`docker-compose.yml`** - Orchestrates the simulation container with proper volumes and networking
- **`setup_docker.sh`** - Helper script to install Docker if not present

## 🚀 Usage

All Docker operations are handled by the main script in the parent directory:

```bash
# From the project root:
../start_simulation.sh setup    # Build Docker image
../start_simulation.sh start    # Run simulation
../start_simulation.sh shell    # Enter container
```

## 🐳 Docker Configuration

### Environment Variables
The container includes:
- ROS2 Humble
- PX4 SITL dependencies  
- Gazebo Classic
- MAVROS2
- Python packages for PX4

### Volumes
- Source code is mounted for live development
- PX4-Autopilot is stored in a persistent volume
- X11 forwarding enabled for GUI applications

### Networking
- Uses host network mode for simplicity
- Ports: 14550 (QGroundControl), 14540 (MAVROS2), 14557 (PX4 SITL)

## 🔧 Troubleshooting

If you encounter issues:
1. Ensure Docker is installed: `docker --version`
2. Check you're in docker group: `groups | grep docker`
3. For GPU acceleration, install nvidia-container-toolkit
4. For X11 issues, run: `xhost +local:docker`

---

For usage, see the main `../start_simulation.sh` script.