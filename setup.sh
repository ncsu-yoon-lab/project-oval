#!/bin/bash
set -e  # Exit on error

# Ensure script is run inside WSL or Ubuntu
if [[ "$(uname -s)" != "Linux" ]]; then
    echo "This script should be run inside WSL 2 or a native Ubuntu system."
    exit 1
fi

# Add the ROS 2 repository if not already added
if ! grep -q "packages.ros.org" /etc/apt/sources.list.d/ros2.list 2>/dev/null; then
    echo "Adding ROS 2 package repository..."
    sudo apt update && sudo apt install -y curl gnupg2 lsb-release

    # Add the official ROS 2 GPG key
    sudo mkdir -p /etc/apt/keyrings
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo tee /etc/apt/keyrings/ros-archive-keyring.gpg > /dev/null

    # Add the ROS 2 Humble repository
    echo "deb [signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    # Update package list after adding ROS 2 repository
    sudo apt update
fi

# Check if ROS 2 is installed
if ! command -v ros2 &> /dev/null; then
    echo "ROS 2 not found. Installing ROS 2 Humble..."
    sudo apt install -y ros-humble-desktop python3-colcon-common-extensions \
        python3-rosdep python3-argcomplete

    # Initialize rosdep
    sudo rosdep init || true  # Avoid errors if already initialized
    rosdep update
fi

# Source ROS 2 setup script
source /opt/ros/humble/setup.bash

# Ensure rclpy is installed via apt (not PyPI)
if ! dpkg -l | grep -q ros-humble-rclpy; then
    echo "Installing rclpy system-wide..."
    sudo apt install -y ros-humble-rclpy
fi

# Install UV if missing
if ! command -v uv &> /dev/null; then
    echo "Installing UV..."
    python3 -m pip install --user uv
    export PATH="$HOME/.local/bin:$PATH"
fi

# Ensure UV is available in PATH
if ! command -v uv &> /dev/null; then
    echo "UV installation failed or not found in PATH. Please add ~/.local/bin to PATH."
    exit 1
fi

# Create and activate a UV-managed virtual environment
if [ ! -d ".venv" ]; then
    echo "Creating virtual environment with UV..."
    uv venv .venv
fi

# Install dependencies from pyproject.toml using UV
echo "Installing dependencies with uv pip install .dev..."
uv pip install -e ".[dev]"

# Done!
echo "Environment setup complete!"

#!/bin/bash
echo '#!/bin/bash
source /opt/ros/humble/setup.bash
source .venv/bin/activate' > activate.sh

chmod +x activate.sh
source activate.sh