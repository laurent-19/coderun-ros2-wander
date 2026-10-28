# CodeRun 2026 - Minimal ROS 2 Humble image for path planning challenge
# Based on official ros:humble-ros-core (smallest ROS 2 base)

FROM ros:humble-ros-core

# Avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Install minimal dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Build tools
    python3-pip \
    python3-colcon-common-extensions \
    # ROS 2 packages (minimal set)
    ros-humble-stage-ros2 \
    ros-humble-rviz2 \
    ros-humble-nav-msgs \
    ros-humble-visualization-msgs \
    # Python dependencies
    python3-numpy \
    python3-scipy \
    python3-pil \
    python3-yaml \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /root/ros_ws/src

# Copy challenge packages
COPY challenge/src/coderun_challenge ./coderun_challenge
COPY challenge/src/coderun_student ./coderun_student

# Build
WORKDIR /root/ros_ws
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install && \
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /root/ros_ws/install/setup.bash" >> ~/.bashrc

# Default command
CMD ["bash"]
