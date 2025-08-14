# F450 Drone Simulator - Quick Start

## 🚁 Simple Usage

Just run:
```bash
./sim.sh
```

That's it! The simulation will start with a GUI showing your drone.

## 📝 Commands

- `./sim.sh` - Start simulation with GUI
- `./sim.sh test` - Test motor control  
- `./sim.sh stop` - Stop simulation
- `./sim.sh help` - Show all commands

## 🎯 First Time Setup

1. **Install Docker** (if not installed):
   ```bash
   # Ubuntu/Debian
   sudo apt install docker.io docker-compose
   sudo usermod -aG docker $USER
   # Log out and back in
   ```

2. **Build Docker image** (one-time):
   ```bash
   cd docker
   docker compose build
   ```

3. **Run simulation**:
   ```bash
   ./sim.sh
   ```

## 🔧 For Raspberry Pi

To deploy on the actual drone hardware:
```bash
./setup_rpi.sh  # Run on Raspberry Pi
```

## 📁 Project Structure

```
Quadcopter/
├── sim.sh           # Main simulation script
├── setup_rpi.sh     # Raspberry Pi setup (for real hardware)
├── docker/          # Docker configuration
├── src/             # Source code
│   ├── drone_control/   # ROS2 control code
│   └── simulation/      # Simulation assets
└── algorithms/      # Control algorithms (NMPC, etc.)
```

## ⚠️ Notes

- First run takes ~5-10 minutes (builds PX4)
- GUI takes 10-30 seconds to appear
- Press Ctrl+C to stop simulation
- Simulation works on Ubuntu 24.04 without GPU

## 🐛 Troubleshooting

**No GUI appears?**
- Wait 30 seconds - it's slow to start
- Check if window opened behind others

**Docker permission denied?**
```bash
sudo usermod -aG docker $USER
# Then log out and back in
```

**Want to reset everything?**
```bash
./sim.sh clean  # Removes all Docker images
```