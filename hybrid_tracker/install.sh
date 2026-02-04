#!/bin/bash
# Installation script for hybrid_tracker ROS 2 package

set -e

echo "=========================================="
echo "Hybrid Tracker - Installation Script"
echo "=========================================="
echo ""

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check if running on Ubuntu 22.04
if [ -f /etc/os-release ]; then
    . /etc/os-release
    if [ "$VERSION_ID" != "22.04" ]; then
        echo -e "${YELLOW}Warning: This script is designed for Ubuntu 22.04${NC}"
        echo "Current version: $VERSION_ID"
        read -p "Continue anyway? (y/n) " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            exit 1
        fi
    fi
fi

# Check if ROS 2 Humble is installed
if [ ! -f "/opt/ros/humble/setup.bash" ]; then
    echo -e "${RED}Error: ROS 2 Humble not found!${NC}"
    echo "Please install ROS 2 Humble first:"
    echo "https://docs.ros.org/en/humble/Installation.html"
    exit 1
fi

echo -e "${GREEN}✓ ROS 2 Humble detected${NC}"

# Source ROS 2
source /opt/ros/humble/setup.bash

# Install system dependencies
echo ""
echo "Installing system dependencies..."
sudo apt update
sudo apt install -y \
    ros-humble-cv-bridge \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-mavros \
    ros-humble-mavros-extras \
    python3-pip \
    python3-colcon-common-extensions

echo -e "${GREEN}✓ System dependencies installed${NC}"

# Install Python dependencies
echo ""
echo "Installing Python dependencies..."
pip3 install --user ultralytics filterpy opencv-python numpy

echo -e "${GREEN}✓ Python dependencies installed${NC}"

# Check if workspace exists
if [ -z "$ROS_WORKSPACE" ]; then
    ROS_WORKSPACE="$HOME/ros2_ws"
fi

echo ""
echo "ROS 2 workspace: $ROS_WORKSPACE"

# Create workspace if it doesn't exist
if [ ! -d "$ROS_WORKSPACE" ]; then
    echo "Creating ROS 2 workspace..."
    mkdir -p "$ROS_WORKSPACE/src"
    cd "$ROS_WORKSPACE"
    colcon build
    echo -e "${GREEN}✓ Workspace created${NC}"
fi

# Copy package to workspace
echo ""
echo "Copying hybrid_tracker package to workspace..."
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PACKAGE_DIR="$SCRIPT_DIR"

if [ ! -d "$ROS_WORKSPACE/src/hybrid_tracker" ]; then
    cp -r "$PACKAGE_DIR" "$ROS_WORKSPACE/src/"
    echo -e "${GREEN}✓ Package copied${NC}"
else
    echo -e "${YELLOW}Package already exists in workspace${NC}"
    read -p "Overwrite? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        rm -rf "$ROS_WORKSPACE/src/hybrid_tracker"
        cp -r "$PACKAGE_DIR" "$ROS_WORKSPACE/src/"
        echo -e "${GREEN}✓ Package updated${NC}"
    fi
fi

# Build package
echo ""
echo "Building hybrid_tracker package..."
cd "$ROS_WORKSPACE"
colcon build --packages-select hybrid_tracker

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Package built successfully${NC}"
else
    echo -e "${RED}✗ Build failed${NC}"
    exit 1
fi

# Source workspace
source "$ROS_WORKSPACE/install/setup.bash"

echo ""
echo "=========================================="
echo -e "${GREEN}Installation Complete!${NC}"
echo "=========================================="
echo ""
echo "To use the package:"
echo "1. Source your workspace:"
echo "   source $ROS_WORKSPACE/install/setup.bash"
echo ""
echo "2. Launch the tracker:"
echo "   ros2 launch hybrid_tracker tracking.launch.py"
echo ""
echo "Add this to your ~/.bashrc to auto-source:"
echo "   echo 'source $ROS_WORKSPACE/install/setup.bash' >> ~/.bashrc"
echo ""
