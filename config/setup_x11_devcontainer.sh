#!/bin/bash

# Script to set up X11 authentication for VS Code Dev Containers
# Run this on your HOST machine before starting the dev container

echo "Setting up X11 authentication for Dev Container..."

# Ensure .Xauthority exists
if [ ! -f "$HOME/.Xauthority" ]; then
    touch "$HOME/.Xauthority"
    echo "Created .Xauthority file"
fi

# Set proper permissions
chmod 600 "$HOME/.Xauthority"

# For systems without xhost, we can use a different approach
# Allow local connections (this is less secure but works for development)
if command -v xhost >/dev/null 2>&1; then
    xhost +local:docker
    echo "Enabled X11 forwarding for Docker using xhost"
else
    echo "xhost not available, relying on .Xauthority file"
fi

# Make sure X11 socket has proper permissions
sudo chmod 777 /tmp/.X11-unix

echo "X11 setup complete. You can now start your Dev Container."
echo "After the container starts, test with: rviz2"
