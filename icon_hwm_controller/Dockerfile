FROM icon_hwm_base:latest

WORKDIR /ros2_ws
ENV HOME=/tmp

# Copy the staged files from the context
COPY . install/

# Ensure permissions are correct
RUN chmod -R a+rX install

# Execute the setup script and the ROS2 launch file
CMD ["/bin/bash", "-c", "source install/setup.bash && ros2 launch ros2_control_demo_example_7 r6bot_controller.launch.py gui:=false"]
