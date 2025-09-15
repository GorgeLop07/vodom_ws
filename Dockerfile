# ROS2 Humble Dockerfile for vo_ws
FROM ubuntu:22.04

# Set environment variables for non-interactive installation
ENV DEBIAN_FRONTEND=noninteractive

# Install required packages
RUN apt-get update && apt-get install -y \
    locales \
    curl \
    gnupg2 \
    lsb-release \
    build-essential \
    python3-pip \
    git \
    libopencv-dev \
    && rm -rf /var/lib/apt/lists/*

# Set locale
RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Add ROS2 apt repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list

# Install ROS2 Humble
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Source ROS2 setup
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Create workspace directory
WORKDIR /workspace

# Copy workspace
COPY . /workspace

# Build the workspace
RUN source /opt/ros/humble/setup.bash && \
    cd /workspace && \
    colcon build --symlink-install

# Source local setup when container starts
CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && bash"]
