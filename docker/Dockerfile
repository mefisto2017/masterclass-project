# Pull the image
FROM osrf/ros:humble-desktop-full


# Change to bash
SHELL ["/bin/bash", "-c"]


# Install ubuntu packages
COPY china.list /etc/apt/sources.list
RUN apt-get update 
RUN apt install -y wget \
                   curl \
                   gnupg \
                   python3-pip

# Install python modules
COPY ros2_numpy /ros2_numpy
RUN cd /ros2_numpy && \
    pip3 install .
RUN pip3 install transforms3d


# Install webserver
RUN mkdir -p /root/nvm
ENV NVM_DIR /root/nvm
RUN echo 'export NVM_DIR="/root/nvm"' >> /root/.bashrc && \
    echo '[ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh"' >> /root/.bashrc
RUN curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.38.0/install.sh | bash && \
    . $NVM_DIR/nvm.sh && \
    nvm install v14 && \
    nvm alias default v14 && \
    nvm use default && \
    npm install -g http-server


# Install Gazebo & ROS2 packages
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get update 
RUN apt install -y gazebo \
                   ros-humble-controller-manager \
                   ros-humble-joint-state-publisher \
                   ros-humble-joint-trajectory-controller \
                   ros-humble-joint-state-broadcaster \
                   ros-humble-gazebo-ros2-control \
                   ros-humble-joint-state-publisher-gui \
                   ros-humble-ros2-controllers \
                   ros-humble-ros2-control \
                   ros-humble-moveit \
                   ros-humble-gripper-controllers \
                   ros-humble-grasping-msgs \
                   ros-humble-rqt-tf-tree \
                   ros-humble-tf-transformations \
                   ros-humble-rosbridge-server \
                   ros-humble-async-web-server-cpp \
                   ros-humble-moveit-planners-chomp \
                   ros-humble-moveit-chomp-optimizer-adapter
                   # ros-humble-gazebo-ros-pkgs \ it is an outdated version depth camera doesnt generate ordered pointclouds (pc)


# Copy the ROS project into the container
RUN mkdir -p /root/ros2_ws


# Set ROS in .bashrc
RUN echo source /opt/ros/humble/setup.bash >> /root/.bashrc
RUN echo source /root/ros2_ws/install/setup.bash >> /root/.bashrc
