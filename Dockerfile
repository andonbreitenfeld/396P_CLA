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
    ros-humble-rmw-cyclonedds-cpp \
    xdg-utils vim-tiny nano \
    git \ 
 && rm -rf /var/lib/apt/lists/*

 RUN mkdir -p /root/ws/src && \
    git clone  https://github.com/arshadlab/tb3_multi_robot.git -b humble /root/ws/src
    
# Updates Motion Model YAML for AMCL
RUN sed -i 's/^ *robot_model_type: "differential"/robot_model_type: "nav2_amcl::DifferentialMotionModel"/' \
    /opt/ros/humble/share/turtlebot3_navigation2/param/waffle.yaml || true

# Auto-source ROS in interactive shells
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Sets default command to bash
CMD ["bash"]