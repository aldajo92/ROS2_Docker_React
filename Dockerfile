# humble, jazzy
ARG ROS_DISTRO=humble

FROM osrf/ros:${ROS_DISTRO}-desktop
ENV ROS_DISTRO=${ROS_DISTRO}

RUN apt update && apt install -y \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs

RUN apt update && apt install -y \
    ros-${ROS_DISTRO}-cv-bridge

RUN mkdir -p /ros2_ws/src

# Install React
# Install Node.js and npm
RUN curl -fsSL https://deb.nodesource.com/setup_18.x | bash -
RUN apt-get install -y nodejs

# Create React app directory
RUN mkdir -p /react_app


RUN echo "alias bros2='cd /ros2_ws && source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build && source /ros2_ws/install/setup.bash'" >> ~/.bashrc
RUN echo "alias sros2='source /opt/ros/${ROS_DISTRO}/setup.bash && source /ros2_ws/install/setup.bash'" >> ~/.bashrc

WORKDIR /ros2_ws

