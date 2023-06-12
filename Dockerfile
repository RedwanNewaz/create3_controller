# Use the OSRF ROS Humble Desktop base image
FROM osrf/ros:humble-desktop

# Install additional dependencies
RUN apt-get update && apt-get install -y \
    git \
    vim \
    && rm -rf /var/lib/apt/lists/*

# Create a new workspace directory
WORKDIR /root/colcon_ws

# Copy the source code into the workspace
COPY src src/

RUN apt-get update -y

RUN rosdep install --from-path src -yi


# Build the workspace
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && colcon build --symlink-install"

# Set up the environment variables
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc
RUN echo "source /root/colcon_ws/install/setup.bash" >> /root/.bashrc

# Set the default command to source the workspace setup file
CMD /bin/bash -c "source /root/.bashrc && /bin/bash"
