#!/bin/bash
set -e  # Exit on error

# Ensure script is run inside WSL or Ubuntu
if [[ "$(uname -s)" != "Linux" ]]; then
    echo "This script should be run inside WSL 2 or a native Ubuntu system."
    exit 1
fi

# Check if ROS 2 is installed
if ! command -v ros2 &> /dev/null; then
    echo "ROS 2 not found. Installing ROS 2 Humble..."
    sudo apt update && sudo apt install -y software-properties-common
    sudo add-apt-repository universe
    sudo apt update && sudo apt install -y \
        ros-humble-desktop python3-colcon-common-extensions \
        python3-rosdep python3-argcomplete

    # Initialize rosdep
    sudo rosdep init || true  # Avoid errors if already initialized
    rosdep update
fi

# Source ROS 2 setup script
source /opt/ros/humble/setup.bash

# Create and activate a virtual environment
if [ ! -d "venv" ]; then
    echo "Creating virtual environment..."
    python3 -m venv venv
fi
source venv/bin/activate

# Install uv (Universal Virtualenv Package Manager)
if ! command -v uv &> /dev/null; then
    echo "Installing uv..."
    pip install uv
fi

# Install dependencies using uv
echo "Installing dependencies from pyproject.toml..."
uv pip install

# Done!
echo "Environment setup complete!"
echo "Run 'source venv/bin/activate' to activate your virtual environment."
