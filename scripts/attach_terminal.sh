#!/usr/bin/env bash

# Detect if there is nvidia GPU
if command -v nvidia-smi &> /dev/null
then
    NAME="ros2_jazzy_nvidia"
else
    NAME="ros2_jazzy"
fi

# Get the container ID
CONTAINER=$(docker container ls | grep -E "$NAME" | awk '{print $1}')

# Execute the entrypoint script inside the container
docker container exec -it "$CONTAINER"  /entrypoint.sh tmux