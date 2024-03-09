# Pull the image
FROM osrf/ros:humble-desktop-full


# Change to bash
SHELL ["/bin/bash", "-c"]


# Install ubuntu packages
COPY china.list /etc/apt/sources.list
RUN apt-get update 
RUN apt install -y wget \
                   curl \
                   gnupg


# Install Gazebo & ROS2 packages
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get update 
RUN apt install -y gazebo \
                   ros-humble-controller-manager \
                   ros-humble-gazebo-ros-pkgs \
                   ros-humble-joint-state-publisher \
                   ros-humble-joint-trajectory-controller \
                   ros-humble-joint-state-broadcaster \
                   ros-humble-gazebo-ros2-control \
                   ros-humble-joint-state-publisher-gui \
                   ros-humble-ros2-controllers \
                   ros-humble-ros2-control \
                   ros-humble-moveit \
                   ros-humble-gripper-controllers


# Copy the ROS project into the container
RUN mkdir -p /root/ros2_ws


# Set ROS in .bashrc
RUN echo source /opt/ros/humble/setup.bash >> /root/.bashrc
RUN echo source /root/ros2_ws/install/setup.bash >> /root/.bashrc