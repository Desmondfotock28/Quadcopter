#!/bin/bash

# Gazebo Harmonic Installation Script

echo "Installing Gazebo Harmonic (Latest LTS)..."

# Install dependencies
sudo apt update
sudo apt install -y wget lsb-release gnupg

# Add Gazebo GPG key
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

# Add repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Update and install Gazebo
sudo apt update
sudo apt install -y gz-harmonic

# Install ROS2-Gazebo integration packages
sudo apt install -y ros-humble-ros-gz

echo "Gazebo Harmonic installation complete!"
echo "Test with: gz sim"