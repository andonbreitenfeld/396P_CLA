# Base Humble Image
FROM osrf/ros:humble-desktop
ENV DEBIAN_FRONTEND=noninteractive

# Use bash so we can 'source' setup files in RUN
SHELL ["/bin/bash", "-c"]

# Core tools + ROS deps
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-rosdep python3-colcon-common-extensions python3-vcstool \
    build-essential git \
    ros-humble-navigation2 ros-humble-nav2-bringup \
    ros-humble-turtlebot3* \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-cartographer ros-humble-cartographer-ros \
    ros-humble-nav2-map-server \
    ros-humble-rmw-cyclonedds-cpp \
    xdg-utils vim-tiny nano \
 && rm -rf /var/lib/apt/lists/*

# Create workspace and copy packages
WORKDIR /ros2_ws
COPY src/ src/

# Install package dependencies
RUN source /opt/ros/humble/setup.bash \
 && rosdep update \
 && rosdep install --from-paths src --ignore-src -r -y

# Update Motion Model YAML for AMCL
RUN sed -i 's/^ *robot_model_type: "differential"/robot_model_type: "nav2_amcl::DifferentialMotionModel"/' \
    /opt/ros/humble/share/turtlebot3_navigation2/param/waffle.yaml || true

# Colcon Build ws
RUN source /opt/ros/humble/setup.bash \
 && colcon build --symlink-install

# Adds ROS sources to bashrc
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc \
 && echo 'source /ros2_ws/install/setup.bash' >> /root/.bashrc

# Launch (change to your package/launch file)
# Re-Add in later if demo-ifying, can make launch file so user does nothing!
# CMD ["bash", "-lc", "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch my_pkg my_launch.py"]
