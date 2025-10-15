#!/bin/bash
# shellcheck disable=SC1090,SC2086,SC1091
# Helper script to build ROS 2 workspace and install dependencies in the container

if [ -e "${WORKSPACE}" ]; then
    pushd "${WORKSPACE}" || exit
        echo "Building ROS 2 workspace..."
        sudo apt update -qq >/dev/null 2>&1 || true
        rosdep install --from-paths src --ignore-src -r -y >/dev/null 2>&1 || true
        colcon build --symlink-install  >/dev/null 2>&1  || true
    popd || exit
    source "${WORKSPACE}/install/setup.bash"
else
    echo "Error: ROS 2 workspace not found at ${WORKSPACE}."
fi