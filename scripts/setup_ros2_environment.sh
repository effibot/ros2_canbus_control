#!/bin/bash

# ROS2 Environment Setup Script
# This script configures the ROS2 environment for both bash and zsh shells

set -e

# Set ZDOTDIR if not already set (adapts to ZSH configuration)
export ZDOTDIR="${ZDOTDIR:-${HOME}/.config/zsh}"

# Check if required variables are set
if [ -z "$ROS_DISTRO" ]; then
    echo "Error: ROS_DISTRO environment variable is not set"
    exit 1
fi

if [ -z "$USERNAME" ]; then
    echo "Error: USERNAME environment variable is not set"
    exit 1
fi

if [ -z "$HOME" ]; then
    echo "Error: HOME environment variable is not set"
    exit 1
fi

if [ -z "$WORKSPACE" ]; then
    echo "Error: WORKSPACE environment variable is not set"
    exit 1
fi

echo "Setting up ROS2 environment for user: $USERNAME"
echo "ROS Distribution: $ROS_DISTRO"
echo "Workspace: $WORKSPACE"

# Setup .bashrc
echo "Configuring .bashrc..."
cat >> $HOME/.bashrc << EOF

# ROS2 Environment Setup
source /opt/ros/${ROS_DISTRO}/setup.bash

# Source workspace setup if it exists
if [ -f ${WORKSPACE}/install/setup.bash ]; then
    source ${WORKSPACE}/install/setup.bash
fi

# Colcon argcomplete
if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then
    source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
fi

# Colcon cd
if [ -f /usr/share/colcon_cd/function/colcon_cd.bash ]; then
    source /usr/share/colcon_cd/function/colcon_cd.bash
    export _colcon_cd_root=/opt/ros/${ROS_DISTRO}
fi
EOF

# Setup .zshrc
echo "Configuring .zshrc..."

# Ensure the ZSH config directory exists
mkdir -p "${ZDOTDIR}"

cat >> "${ZDOTDIR}/.zshrc" << EOF

# ROS2 Environment Setup
source /opt/ros/${ROS_DISTRO}/setup.zsh

# Source workspace setup if it exists
if [ -f ${WORKSPACE}/install/setup.zsh ]; then
    source ${WORKSPACE}/install/setup.zsh
fi

# Colcon argcomplete
if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then
    source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
fi

# Colcon cd
if [ -f /usr/share/colcon_cd/function/colcon_cd.sh ]; then
    source /usr/share/colcon_cd/function/colcon_cd.sh
    export _colcon_cd_root=/opt/ros/${ROS_DISTRO}
fi
EOF

echo "ROS2 environment setup completed successfully!"
