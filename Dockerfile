FROM ros:humble-ros-base

# Install dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-rmf-door-msgs \
        python3-pip && \
        pip3 install websockets websocket-client requests gmqtt && \
    rm -rf /var/lib/apt/lists/*

# Clone the repository
WORKDIR /door_adapter_megazo_ws/src
COPY ./door_adapter_megazo door_adapter_megazo/

# Setup the workspace
WORKDIR /door_adapter_megazo_ws
RUN apt-get update && rosdep install --from-paths src --ignore-src --rosdistro=$ROS_DISTRO -y \
  && rm -rf /var/lib/apt/lists/*

# # Build the workspace
RUN . /opt/ros/$ROS_DISTRO/setup.sh \
  && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# # Ensure the entrypoint script sources the ROS setup
RUN echo 'source /door_adapter_megazo_ws/install/setup.bash' >> /ros_entrypoint.sh

# # Ensure proper permissions for entrypoint
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]

