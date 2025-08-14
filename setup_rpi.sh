#!/bin/bash

# Raspberry Pi Setup Script for F450 Quadcopter
# This script sets up the Raspberry Pi 4B to run the drone code natively
# Assumes Raspberry Pi OS (Debian Bookworm) or Ubuntu 22.04 Server for RPi

set -e

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

print_status() { echo -e "${GREEN}[INFO]${NC} $1"; }
print_warning() { echo -e "${YELLOW}[WARNING]${NC} $1"; }
print_error() { echo -e "${RED}[ERROR]${NC} $1"; }
print_header() { echo -e "${BLUE}[STEP]${NC} $1"; }

# Detect OS
OS_INFO=$(lsb_release -d 2>/dev/null || cat /etc/os-release | grep PRETTY_NAME)
print_status "Detected OS: $OS_INFO"

print_header "Raspberry Pi 4B Setup for F450 Quadcopter"
echo "This script will install:"
echo "  - ROS2 Humble (native)"
echo "  - MAVROS2 for Pixhawk communication"
echo "  - Python dependencies"
echo "  - Serial port configuration"
echo ""
echo "Continue? (y/N)"
read -n 1 -r
echo
[[ ! $REPLY =~ ^[Yy]$ ]] && exit 1

# Update system
print_header "Updating system packages..."
sudo apt update && sudo apt upgrade -y

# Install basic dependencies
print_header "Installing basic dependencies..."
sudo apt install -y \
    curl \
    git \
    python3-pip \
    python3-venv \
    build-essential \
    cmake \
    software-properties-common

# Setup ROS2 Humble
print_header "Installing ROS2 Humble..."

# Add ROS2 repository
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble (base, not desktop on RPi)
sudo apt update
sudo apt install -y \
    ros-humble-ros-base \
    ros-humble-mavros \
    ros-humble-mavros-extras \
    python3-colcon-common-extensions \
    python3-rosdep

# Initialize rosdep
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

# Install GeographicLib datasets for MAVROS
print_header "Installing GeographicLib datasets..."
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh

# Configure serial port for Pixhawk
print_header "Configuring serial port for Pixhawk..."

# Disable serial console on /dev/ttyAMA0
if grep -q "console=serial0" /boot/cmdline.txt 2>/dev/null; then
    print_status "Disabling serial console on /dev/ttyAMA0..."
    sudo sed -i 's/console=serial0,115200 //g' /boot/cmdline.txt
    sudo sed -i 's/console=ttyAMA0,115200 //g' /boot/cmdline.txt
fi

# Disable Bluetooth (it uses the good UART)
if [ -f /boot/config.txt ]; then
    if ! grep -q "dtoverlay=disable-bt" /boot/config.txt; then
        print_status "Disabling Bluetooth to free up UART..."
        echo "dtoverlay=disable-bt" | sudo tee -a /boot/config.txt
        sudo systemctl disable hciuart
    fi
fi

# Add user to dialout group for serial access
sudo usermod -a -G dialout $USER

# Install Python dependencies
print_header "Installing Python dependencies..."
pip3 install --user -r requirements.txt

# Create workspace
print_header "Setting up ROS2 workspace..."
mkdir -p ~/ros2_ws/src

# Link this repository to workspace
ln -sf $(pwd)/src/drone_control ~/ros2_ws/src/
print_status "Linked drone_control package to ROS2 workspace"

# Build ROS2 packages
print_header "Building ROS2 packages..."
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select drone_control

# Create environment configuration
print_header "Creating environment configuration..."

cat > ~/drone_env.sh << 'EOF'
#!/bin/bash
# F450 Drone Environment Setup for Raspberry Pi

# Detect if we're on hardware (RPi) or simulation
if [ -e /dev/ttyAMA0 ] || [ -e /dev/ttyUSB0 ]; then
    export DRONE_MODE="HARDWARE"
    export MAVROS_FCU_URL="/dev/ttyAMA0:57600"
    echo "🤖 Hardware Mode: Connected to Pixhawk"
else
    export DRONE_MODE="SIMULATION"
    export MAVROS_FCU_URL="udp://:14540@127.0.0.1:14557"
    echo "🖥️  Simulation Mode: Using PX4 SITL"
fi

# ROS2 setup
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash 2>/dev/null

# Convenience functions
start_mavros() {
    echo "Starting MAVROS with FCU URL: $MAVROS_FCU_URL"
    ros2 run mavros mavros_node --ros-args -p fcu_url:=$MAVROS_FCU_URL
}

motor_test() {
    ros2 run drone_control motor_test_mavros2
}

check_connection() {
    ros2 topic echo /mavros/state --once
}

# Aliases
alias build_drone='cd ~/ros2_ws && colcon build --packages-select drone_control && source install/setup.bash'
alias drone_topics='ros2 topic list | grep mavros'
alias drone_status='ros2 topic echo /mavros/state'

echo "Available commands:"
echo "  start_mavros    - Start MAVROS connection"
echo "  motor_test      - Run motor test"
echo "  check_connection - Check Pixhawk connection"
echo "  build_drone     - Rebuild drone_control package"
echo "  drone_topics    - List MAVROS topics"
echo "  drone_status    - Monitor connection status"
EOF

chmod +x ~/drone_env.sh

# Add to bashrc
if ! grep -q "drone_env.sh" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# F450 Drone Environment" >> ~/.bashrc
    echo "[ -f ~/drone_env.sh ] && source ~/drone_env.sh" >> ~/.bashrc
fi

# Create systemd service for auto-start (optional)
print_header "Creating systemd service (optional)..."

sudo tee /etc/systemd/system/drone-mavros.service > /dev/null << EOF
[Unit]
Description=MAVROS for F450 Drone
After=network.target

[Service]
Type=simple
User=$USER
Environment="ROS_DOMAIN_ID=0"
ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && source /home/$USER/ros2_ws/install/setup.bash && ros2 run mavros mavros_node --ros-args -p fcu_url:=/dev/ttyAMA0:57600'
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

print_status "Systemd service created but not enabled"
print_status "To auto-start MAVROS on boot: sudo systemctl enable drone-mavros"

# Final setup
print_header "================== Setup Complete! =================="
echo ""
print_warning "IMPORTANT: Reboot required for serial port changes!"
echo ""
echo "After reboot:"
echo "  1. Open a terminal"
echo "  2. The environment will auto-load"
echo "  3. Run 'start_mavros' to connect to Pixhawk"
echo "  4. Run 'motor_test' to test motors (REMOVE PROPELLERS!)"
echo ""
echo "To switch between simulation and hardware:"
echo "  - The system auto-detects based on serial port availability"
echo "  - Same code works for both modes!"
echo ""
print_warning "Reboot now? (y/N)"
read -n 1 -r
echo
[[ $REPLY =~ ^[Yy]$ ]] && sudo reboot