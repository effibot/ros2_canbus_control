#!/bin/bash

# Script to properly set up X11 forwarding and start the ROS2 container

echo "Setting up X11 authentication for Docker container..."

# Create a temporary X11 auth file for Docker
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]; then
    touch $XAUTH
fi

# Remove any existing auth entries for the display
xauth -f $XAUTH remove :0 2>/dev/null || true

# Add current display authorization to the temp file
if [ ! -z "$DISPLAY" ]; then
    xauth -f $XAUTH add $DISPLAY . $(xauth list $DISPLAY | awk '{print $3}')
    echo "Added X11 authorization for display $DISPLAY"
else
    echo "Warning: DISPLAY environment variable is not set"
fi

# Set proper permissions for the auth file
chmod 644 $XAUTH

echo "Starting Docker container..."
cd "$(dirname "$0")/.."
docker compose -f docker/compose.yaml up -d

echo "Container started! You can now run GUI applications like rviz2."
echo "To attach to the container, run:"
echo "  docker exec -it ros2_jazzy bash"
