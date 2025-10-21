#! /bin/bash

set -e
if [ -e "${WORKSPACE}" ]; then
    pushd "${WORKSPACE}" || exit
        echo "Setting up ROS 2 workspace..."
        envsubst < src/ros2.repos | vcs import src
        sudo apt update -qq >/dev/null 2>&1 || true
        rosdep update --rosdistro="$ROS_DISTRO"
        rosdep install --from-paths src --ignore-src -y --rosdistro="$ROS_DISTRO" >/dev/null 2>&1 || true
    popd || exit
else
    echo "Error: ROS 2 workspace not found at ${WORKSPACE}."
fi