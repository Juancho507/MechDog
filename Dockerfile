# =============================================================================
# MECHDOG MULTI-STAGE DOCKERFILE
# Arquitectura modular con separación estricta de responsabilidades
# =============================================================================

# =============================================================================
# STAGE 1: BASE - Core del Sistema ROS Noetic
# =============================================================================
FROM osrf/ros:noetic-desktop-full AS base

# Prevent interactive prompts during build
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=America/New_York

# Install system dependencies and core ROS packages
RUN apt-get update && apt-get install -y \
    # Build tools
    python3-catkin-tools \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential \
    python3-pip \
    git \
    vim \
    wget \
    # Gazebo dependencies
    ros-noetic-gazebo-ros \
    ros-noetic-gazebo-plugins \
    ros-noetic-gazebo-ros-control \
    ros-noetic-joint-state-publisher-gui \
    ros-noetic-urdf \
    ros-noetic-rosbridge-suite \
    # Navigation dependencies
    ros-noetic-navigation \
    ros-noetic-move-base \
    ros-noetic-tf2 \
    ros-noetic-tf2-ros \
    # Visualization (required for RViz)
    ros-noetic-rviz \
    ros-noetic-rqt \
    ros-noetic-rqt-common-plugins \
    # Robot description and URDF tools
    ros-noetic-xacro \
    ros-noetic-robot-state-publisher \
    ros-noetic-joint-state-publisher \
    ros-noetic-joint-state-publisher-gui \
    ros-noetic-urdf \
    # Websocket telemetry bridge (Foxglove Studio)
    ros-noetic-rosbridge-suite \
    # Python scientific dependencies
    python3-numpy \
    python3-scipy \
    python3-matplotlib \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep update

# Set working directory
WORKDIR /app

# =============================================================================
# STAGE 2: BUILDER - Workspace Compilation
# =============================================================================
FROM base AS builder

# Copy only source code for workspace compilation
COPY catkin_ws/src /app/catkin_ws/src

# Build the catkin workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    cd /app/catkin_ws && \
    catkin_make"

# Setup environment for subsequent stages
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "source /app/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Inline entrypoint: sources ROS environment then runs the given command
RUN echo '#!/bin/bash\nsource /opt/ros/noetic/setup.bash\nsource /app/catkin_ws/devel/setup.bash\nexec "$@"' \
    > /ros_entrypoint.sh && chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

# =============================================================================
# STAGE 3: VISUALIZER - noVNC Web Visualization Layer
# =============================================================================
FROM builder AS visualizer

# Install web visualization stack (noVNC + Virtual Display)
RUN apt-get update && apt-get install -y \
    # Virtual display server
    xvfb \
    # VNC server
    x11vnc \
    # Lightweight window manager
    fluxbox \
    # Web VNC client
    novnc \
    websockify \
    # Terminal emulator (for right-click → Terminal in noVNC)
    xterm \
    # Network utilities
    net-tools \
    && rm -rf /var/lib/apt/lists/*

# Configure VNC environment variables
ENV DISPLAY=:0
ENV RESOLUTION=1920x1080

# Copy VNC startup script
COPY start_vnc.sh /start_vnc.sh
RUN chmod +x /start_vnc.sh

# Expose noVNC web port
EXPOSE 6080

# Set entrypoint to VNC initialization script
ENTRYPOINT ["/start_vnc.sh"]
CMD ["bash"]
