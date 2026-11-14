# CodeRun 2026 - Minimal ROS 2 Humble image
FROM ros:humble-ros-core

ENV DEBIAN_FRONTEND=noninteractive

# Install build tools and rosdep
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    build-essential \
    cmake \
    python3-rosdep \
    git \
    nano \
    openssh-client \
    && rm -rf /var/lib/apt/lists/* \
    && printf 'set tabsize 4\nset tabstospaces\n' >> /etc/nanorc

# Initialize rosdep
RUN rosdep init && rosdep update

WORKDIR /root/ros_ws/src

# Clone stage_ros2
RUN git clone --depth 1 https://github.com/tuw-robotics/Stage.git && \
    git clone --depth 1 https://github.com/tuw-robotics/stage_ros2.git

# Copy challenge packages
COPY challenge/src/coderun_challenge ./coderun_challenge
COPY challenge/src/coderun_student ./coderun_student

# Install dependencies and build
WORKDIR /root/ros_ws
RUN apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/* && \
    . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install && \
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /root/ros_ws/install/setup.bash" >> ~/.bashrc

CMD ["bash"]
