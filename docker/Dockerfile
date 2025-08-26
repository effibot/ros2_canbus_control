# Use an official ROS 2 image as a base
# This image already has a lot of ROS 2 packages installed and provide
# a non-root user named 'ubuntu' with sudo permissions.

ARG BASE_DOCKER_IMAGE=osrf/ros:jazzy-desktop-full
FROM $BASE_DOCKER_IMAGE AS base

ENV DEBIAN_FRONTEND=noninteractive

# Install basic utilities, dependencies, and development tools
# These include:
# - C/C++ toolchain and debuggers
# - Python 3 interpreter, testers, basic modules and scientific libraries
# - Linters
# - OpenCV dependencies for x86 systems
# - Boost C++ libraries
# - System utilities
# - Zsh shell
# Then, add universe repository
RUN apt update && sudo apt full-upgrade -y && apt install -y --no-install-recommends \
    apt-utils \
    build-essential \
    ca-certificates \
    ccache \
    cmake \
    cppcheck \
    curl \
    dialog \
    direnv \
    dirmngr \
    dmidecode \
    file \
    fd-find \
    fuse3 \
    g++ \
    gcc \
    gdb \
    gdbserver \
    git \
    gnupg2 \
    htop \
    iproute2 \
    less \
    lcov \
    libasio-dev \
    libboost-all-dev \
    libssl-dev \
    libtbb-dev \
    libtinyxml2-dev \
    libxml2-utils \
    locales \
    lsb-release \
    lsof \
    make \
    mesa-utils \
    minicom \
    micro \
    nano \
    net-tools \
    nfs-common \
    ninja-build \
    openssh-client \
    openssl \
    pkg-config \
    python3 \
    python3-argcomplete \
    python3-autopep8 \
    python3-dev \
    python3-pip \
    python3-pygments \
    python3-pytest-pylint \
    python3-venv \
    shellcheck \
    shfmt \
    software-properties-common \
    sshfs \
    sudo \
    uncrustify \
    unzip \
    valgrind \
    vim \
    wget \
    whois \
    zip \
    zsh \
    zsh-doc \
    zstd && \
    apt-get autoremove -y && \
    apt-get autoclean && \
    rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*/apt/lists/*

# Configure Git to accept different ownerships for local repository clones
RUN git config --system --add safe.directory '*'

# Install nano syntax highlighting
RUN curl https://raw.githubusercontent.com/scopatz/nanorc/master/install.sh | sh

# Set up user environment variables
ENV USERNAME=ubuntu
ENV EDITOR=micro
ENV HOME=/home/${USERNAME}
ENV PATH="/home/ubuntu/.cargo/bin:/home/ubuntu/.local/bin:/home/ubuntu/.fzf/bin:${PATH}"
ENV SHELL=/bin/zsh

# Set up the workspace
ARG WORKSPACE=/home/${USERNAME}/ros_ws
ENV WORKSPACE=$WORKSPACE

RUN echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

# Set workspace directory ownership
RUN mkdir -p ${WORKSPACE} && chown -R ${USERNAME}:${USERNAME} ${WORKSPACE}

# Condigure Git to accept different ownerships for local repository clones
RUN git config --system --add safe.directory '*'

# Configure ZSH for the user
COPY --chmod=755 scripts/configure_zsh.sh /tmp/configure_zsh.sh
RUN runuser -l ${USERNAME} -c '/tmp/configure_zsh.sh ${USERNAME} ${HOME}' && rm /tmp/configure_zsh.sh

# Set ZDOTDIR environment variable to match ZSH configuration
ENV ZDOTDIR=/home/${USERNAME}/.config/zsh

# Install Rust for the user
RUN runuser -l ${USERNAME} -c "curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y" \
    && chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}/.cargo

# Install eza via cargo
RUN runuser -l ${USERNAME} -c '${HOME}/.cargo/bin/cargo install eza'

# Install terminal enhancement tools
COPY --chmod=755 scripts/install_*.sh /tmp/scripts/

# Install fzf for the user
RUN runuser -l ${USERNAME} -c '/tmp/scripts/install_fzf.sh'

# Install starship for the user
RUN /tmp/scripts/install_starship.sh

# Install zoxide for the user
RUN runuser -l ${USERNAME} -c '/tmp/scripts/install_zoxide.sh'

# Fix ownership of every file and directory under home directory
#RUN chown -R ${USERNAME}:${USERNAME} ${HOME}

# Clean up installation scripts and apt cache
RUN rm -rf /tmp/scripts \
    && apt autoremove -y \
    && apt autoclean \
    && rm -rf /var/lib/apt/lists/*



ENV DEBIAN_FRONTEND=

FROM base AS full

# Stay as root for package installations
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# Setup ROS2 environment using script
COPY --chmod=755 scripts/setup_ros2_environment.sh /tmp/setup_ros2_environment.sh
RUN /tmp/setup_ros2_environment.sh && rm /tmp/setup_ros2_environment.sh

# Install core ROS 2 packages
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
RUN pip install --no-cache-dir -r /tmp/requirements.txt || true \
    && rm /tmp/requirements.txt

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
    ros-${ROS_DISTRO}-ros-gz*

# Final cleanup
RUN apt autoremove -y \
    && apt clean \
    && rm -rf /var/lib/apt/lists/*

ENV DEBIAN_FRONTEND=

FROM sim AS app

# Set working directory
WORKDIR ${WORKSPACE}

ENV DEBIAN_FRONTEND=