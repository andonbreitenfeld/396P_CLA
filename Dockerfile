# Base Humble Image
FROM osrf/ros:humble-desktop

# No Interactive Prompts
ENV DEBIAN_FRONTEND=noninteractive

# Install Dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-turtlebot3\* \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    ros-humble-nav2-map-server \
    ros-humble-rviz2 \
    ros-humble-rmw-cyclonedds-cpp \
 && rm -rf /var/lib/apt/lists/*

# Switch to Cyclone DDS
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Sets Correct URDF Model
ENV TURTLEBOT3_MODEL=waffle

# Updates Motion Model YAML for AMCL
RUN sed -i 's/^ *robot_model_type: "differential"/robot_model_type: "nav2_amcl::DifferentialMotionModel"/' \
    /opt/ros/humble/share/turtlebot3_navigation2/param/waffle.yaml || true

# Source ROS 2 setup for all bash sessions
SHELL ["/bin/bash", "-lc"]
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Sets default command to bash
CMD ["bash"]