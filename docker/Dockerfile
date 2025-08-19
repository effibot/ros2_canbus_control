ARG BASE_DOCKER_IMAGE=osrf/ros:jazzy-desktop-full
FROM $BASE_DOCKER_IMAGE AS base

# Set the shell to support the "source" command
SHELL ["/bin/bash", "-euo", "pipefail", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

# Create a non-root user but stay as root for all installations
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

# Configure sudo properly
RUN apt update && apt full-upgrade -y \
    && apt install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

# Create sudo protection script for use at each stage
RUN echo '#!/bin/bash' > /usr/local/bin/fix-sudo.sh \
    && echo 'chown root:root /usr/bin/sudo 2>/dev/null || true' >> /usr/local/bin/fix-sudo.sh \
    && echo 'chmod 4755 /usr/bin/sudo 2>/dev/null || true' >> /usr/local/bin/fix-sudo.sh \
    && echo 'chown root:root /etc/sudo.conf 2>/dev/null || true' >> /usr/local/bin/fix-sudo.sh \
    && echo 'chmod 644 /etc/sudo.conf 2>/dev/null || true' >> /usr/local/bin/fix-sudo.sh \
    && echo 'chown root:root /etc/sudoers 2>/dev/null || true' >> /usr/local/bin/fix-sudo.sh \
    && echo 'chmod 440 /etc/sudoers 2>/dev/null || true' >> /usr/local/bin/fix-sudo.sh \
    && echo 'chown root:root /etc/sudoers.d/* 2>/dev/null || true' >> /usr/local/bin/fix-sudo.sh \
    && echo 'chmod 440 /etc/sudoers.d/* 2>/dev/null || true' >> /usr/local/bin/fix-sudo.sh \
    && chmod +x /usr/local/bin/fix-sudo.sh \
    && /usr/local/bin/fix-sudo.sh

# Basic tools installation (staying as root)
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
    usbutils \
    vim \
    wget \
    xterm \
    zsh \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Configure Git to accept different ownerships for local repository clones
RUN git config --system --add safe.directory '*'

# Install nano syntax highlighting
RUN curl https://raw.githubusercontent.com/scopatz/nanorc/master/install.sh | sh

ENV EDITOR=nano

# Set up user environment variables (but stay as root)
ENV HOME=/home/${USERNAME}
ENV USER=${USERNAME}
ENV PATH=/home/${USERNAME}/.local/bin:${PATH}

# Set up the workspace
ARG WORKSPACE=/home/${USERNAME}/ros_ws
ENV WORKSPACE=$WORKSPACE

# Install Rust for the user (as root but in user's home)
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
ENV PATH="/home/${USERNAME}/.cargo/bin:${PATH}"

# Fix ownership of .cargo directory
RUN chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}/.cargo

# Install eza via cargo (as root but install to user's cargo)
RUN runuser -l ${USERNAME} -c '/home/ros/.cargo/bin/cargo install eza'

# Install terminal enhancement tools (staying as root)
COPY --chmod=755 scripts/install_*.sh /tmp/scripts/

# Install fzf for the user
RUN runuser -l ${USERNAME} -c '/tmp/scripts/install_fzf.sh'

# Install zoxide for the user  
RUN runuser -l ${USERNAME} -c '/tmp/scripts/install_zoxide.sh'

# Install starship for the user
RUN runuser -l ${USERNAME} -c '/tmp/scripts/install_starship.sh'

# Install ble.sh for the user
RUN runuser -l ${USERNAME} -c '/tmp/scripts/install_ble.sh'

# Configure integrations
RUN runuser -l ${USERNAME} -c '/tmp/scripts/install_integration_conf.sh'

# Source ros2 setup script in .bashrc for the user
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/${USERNAME}/.bashrc

# Clean up installation scripts
RUN rm -rf /tmp/scripts

# Clean up apt cache
RUN apt autoremove -y \
    && apt clean \
    && rm -rf /var/lib/apt/lists/*

# Set workspace directory ownership
RUN mkdir -p ${WORKSPACE} && chown -R ${USERNAME}:${USERNAME} ${WORKSPACE}

ENV DEBIAN_FRONTEND=

FROM base AS full

# Stay as root for package installations
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# Install core ROS 2 packages (non-simulation)
RUN apt update && apt install -y \
    ros-${ROS_DISTRO}-plotjuggler-ros \
    ros-${ROS_DISTRO}-rqt* \
    ros-${ROS_DISTRO}-ament-* \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-rosbag2* \
    ros-${ROS_DISTRO}-canopen* \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Install Python dependencies for the user
COPY requirements.txt /tmp/requirements.txt
RUN runuser -l ${USERNAME} -c 'pip install --no-cache-dir -r /tmp/requirements.txt' || true
RUN rm /tmp/requirements.txt

# Final cleanup
RUN apt autoremove -y \
    && apt clean \
    && rm -rf /var/lib/apt/lists/*

ENV DEBIAN_FRONTEND=

FROM full AS sim

# Stay as root for package installations
ENV DEBIAN_FRONTEND=noninteractive

# Install Gazebo
COPY --chmod=755 scripts/install_gazebo.sh /tmp/install_gazebo.sh
RUN /tmp/install_gazebo.sh ${ROS_DISTRO} && rm /tmp/install_gazebo.sh

# Install simulation-specific ROS 2 packages
RUN apt update && apt install -y \
    ros-${ROS_DISTRO}-ros-gz* \
    && apt-get clean && rm -rf /var/lib/apt/lists/*
#ros-${ROS_DISTRO}-rtabmap-ros

# Final cleanup
RUN apt autoremove -y \
    && apt clean \
    && rm -rf /var/lib/apt/lists/*

ENV DEBIAN_FRONTEND=

FROM sim AS app

# Set working directory and switch to user ONLY at the very end
WORKDIR ${WORKSPACE}

# Fix sudo configuration before switching to non-root user
RUN /usr/local/bin/fix-sudo.sh

USER ${USERNAME}

ENV DEBIAN_FRONTEND=