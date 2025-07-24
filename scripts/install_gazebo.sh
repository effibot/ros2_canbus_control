#!/bin/bash

# Script to install Gazebo for ROS 2
# This script should be run as root or with sudo privileges
# Usage: install_gazebo.sh [ROS_DISTRO]
# If ROS_DISTRO is not provided, it will use the environment variable

set -euo pipefail

# Get ROS_DISTRO from command line argument or environment variable
if [ $# -eq 1 ]; then
    ROS_DISTRO_ARG="$1"
    echo "Using ROS_DISTRO from command line: $ROS_DISTRO_ARG"
elif [ -n "${ROS_DISTRO:-}" ]; then
    ROS_DISTRO_ARG="$ROS_DISTRO"
    echo "Using ROS_DISTRO from environment: $ROS_DISTRO_ARG"
else
    echo "Error: ROS_DISTRO must be provided as argument or environment variable"
    echo "Usage: $0 [ROS_DISTRO]"
    echo "Example: $0 jazzy"
    exit 1
fi

echo "Installing Gazebo for ROS 2 $ROS_DISTRO_ARG..."

# Add Gazebo repository key and source
curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

# Add Gazebo repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Update package list
apt-get update

# Define Gazebo version mapping for ROS 2 distributions
declare -A version=(
    ["rolling"]="ionic"
    ["jazzy"]="harmonic"
    ["iron"]="fortress"
)

# Get the appropriate Gazebo version for the current ROS distribution
GAZEBO_VERSION=${version[$ROS_DISTRO_ARG]}

echo "Installing Gazebo ${GAZEBO_VERSION} for ROS ${ROS_DISTRO_ARG}..."

# Install Gazebo
apt install -y gz-"${GAZEBO_VERSION}" --no-install-recommends

# Clean up
apt-get clean && rm -rf /var/lib/apt/lists/*

echo "Gazebo ${GAZEBO_VERSION} installation complete!"
