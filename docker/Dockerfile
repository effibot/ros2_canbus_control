ARG BASE_DOCKER_IMAGE=osrf/ros:jazzy-desktop-full
FROM $BASE_DOCKER_IMAGE AS base

# Set the shell to support the "source" command
SHELL ["/bin/bash", "-euo", "pipefail", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

# Create a non-root user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Delete user and home folder of user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u ${USER_UID} ; then userdel $(id -unr ${USER_UID}); fi

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -m \
  -s /bin/bash \
  -G sudo,plugdev,dialout \
  --uid $USER_UID \
  --gid $USER_GID \
  $USERNAME

RUN apt update && apt full-upgrade -y \
    && apt install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

# Basic tools
RUN apt update && apt install -y \
    apt-utils \
    bash-completion \
    bat \
    btop \
    build-essential \
    curl \
    gawk \
    gdb \
    git \
    iproute2 \
    less \
    mesa-utils \
    net-tools \
    locales \
    lsb-release \
    nano \
    neovim \
    python3-argcomplete \
    python3-colcon-* \
    python3-pip \
    python3-vcstool \
    ros-dev-tools \
    rsync \
    strace \
    tmux \
    tzdata \
    unzip \
    vim \
    wget \
    xterm \
    zsh \
    && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Configure Git to accept different ownerships for local repository clones
RUN git config --system --add safe.directory '*'

# Install nano syntax highlighting
RUN curl https://raw.githubusercontent.com/scopatz/nanorc/master/install.sh | sh

ENV EDITOR=nano

USER ${USERNAME}
ENV HOME=/home/${USERNAME}
ENV USER=${USERNAME}
ENV PATH=${HOME}/.local/bin:${PATH}

# Set up the workspace
ARG WORKSPACE=${HOME}/ros_ws
ENV WORKSPACE=$WORKSPACE

# Install Rust for the user
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
ENV PATH="${HOME}/.cargo/bin:${PATH}"

# Install eza via cargo
RUN ${HOME}/.cargo/bin/cargo install eza

# Install individual terminal enhancement tools (split for better error detection)
COPY scripts/install_*.sh /tmp/scripts/
RUN sudo chmod +x /tmp/scripts/*.sh

# Install fzf
RUN /tmp/scripts/install_fzf.sh

# Install zoxide
RUN /tmp/scripts/install_zoxide.sh

# Install starship
RUN /tmp/scripts/install_starship.sh

# Install ble.sh
RUN /tmp/scripts/install_ble.sh

# Configure integrations between tools and remove installation scripts
RUN /tmp/scripts/install_integration_conf.sh && \
    sudo rm -rf /tmp/scripts

# Source ros2 setup script in .bashrc
# This ensures that the ROS 2 environment is sourced automatically when a new terminal is opened.
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> ${HOME}/.bashrc

WORKDIR $WORKSPACE

# Clean up
RUN sudo apt autoremove -y \
    && sudo apt clean \
    && sudo rm -rf /var/lib/apt/lists/*

ENV DEBIAN_FRONTEND=

FROM base AS full

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# Install core ROS 2 packages (non-simulation)
RUN sudo apt update && sudo apt install -y \
    ros-${ROS_DISTRO}-plotjuggler-ros \
    ros-${ROS_DISTRO}-rqt* \
    ros-${ROS_DISTRO}-ament-* \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-rosbag2* \
    ros-${ROS_DISTRO}-canopen* \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    && sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*

# Install Python dependencies
COPY requirements.txt /tmp/requirements.txt
RUN pip install --no-cache-dir -r /tmp/requirements.txt || true

# Clean up
RUN sudo apt autoremove -y \
    && sudo apt clean \
    && sudo rm -rf /var/lib/apt/lists/*

ENV DEBIAN_FRONTEND=

FROM full AS sim

ENV DEBIAN_FRONTEND=noninteractive

# Install Gazebo
COPY scripts/install_gazebo.sh /tmp/install_gazebo.sh
RUN sudo chmod +x /tmp/install_gazebo.sh && sudo /tmp/install_gazebo.sh ${ROS_DISTRO} && sudo rm /tmp/install_gazebo.sh

# Install simulation-specific ROS 2 packages
RUN sudo apt update && sudo apt install -y \
    ros-${ROS_DISTRO}-ros-gz* \
    && sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*
    #ros-${ROS_DISTRO}-rtabmap-ros

# Clean up
RUN sudo apt autoremove -y \
    && sudo apt clean \
    && sudo rm -rf /var/lib/apt/lists/*

ENV DEBIAN_FRONTEND=

FROM sim AS app

ENV DEBIAN_FRONTEND=