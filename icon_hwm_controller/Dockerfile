FROM ros:jazzy AS builder

# Match the user's workspace path so absolute symlinks remain valid during the build process
WORKDIR /usr/local/google/home/nilsb/ros2_ws

# Use a bind-mount to access the workspace (avoiding the need to copy the 11GB build folder into context).
# Since we mount straight to the exact original path, absolute symlinks resolve correctly.
# We use `cp -rL` to follow and dereference all symlinks, outputting clean files into /opt/install.
RUN --mount=type=bind,target=/usr/local/google/home/nilsb/ros2_ws \
    mkdir -p /opt/install && \
    cp -rL install/* /opt/install/ && \
    chmod -R a+rX /opt/install

FROM ros:jazzy

RUN apt-get update && apt-get install -y ros-jazzy-xacro ros-jazzy-backward-ros python3-filelock ros-jazzy-ros2-control ros-jazzy-ros2-controllers && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws
ENV HOME=/tmp

# Copy only the dereferenced files from the builder stage
COPY --from=builder /opt/install /ros2_ws/install

# Execute the setup script and the ROS2 launch file
CMD ["/bin/bash", "-c", "source install/setup.bash && ros2 launch ros2_control_demo_example_7 r6bot_controller.launch.py gui:=false"]
