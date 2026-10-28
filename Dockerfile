# CodeRun 2026 - Path planning challenge
# Based on ros2-efac which includes stage_ros2, rviz2, and dependencies

FROM laurentiupopa/ros2-efac:ros2

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
