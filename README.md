# MasterClass Project

<a name="readme-top"></a>

<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#software-prerequisites">Software Prerequisites</a></li>
        <li><a href="#hardware-prerequisites">Hardware Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#update">Update</a></li>
  </ol>
</details>


## About The Project
Reinforcement learning project for a robotic arm, the main goal is to train the arm to pick up a coffee cup and place it
on a delivery robot platform. 


<!-- GETTING STARTED -->
## Getting Started

### Software Prerequisites
* Ubuntu 22.04
* ROS2 Humble
* Docker
* Docker-compose


<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- INSTALLATION -->
### Installation
1. Clone the repo:
   ```sh
   cd ~ && \
   git clone https://github.com/mefisto2017/Ranger-Mini-Project
   ```
2. Compile the image:
   ```sh
   cd Ranger-Mini-Project && \
   docker build .
   ```
3. Tag the image:
   ```sh
   docker image tag XXXXX engosim:dev
   ```
4. Setup the docker compose file:
   ```yaml
   environment:
      - DISPLAY=:0 # Select the dispaly to be shared, can be replaced by $DISPLAY
      - GAZEBO_MODEL_PATH=/root/ros2_ws/src/the_construct_office_gazebo/models:/root/ros2_ws/src/the_construct_office_gazebo/barista_ros2/barista_description:/root/ros2_ws/src/ur_arm:$${GAZEBO_MODEL_PATH} # No need to change
   volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix # No need to change
      - /dev/shm:/dev/shm  # No need to change
      - /home/mefisto/masterclass/ros2_ws:/root/ros2_ws # Change the first part to your ros2_ws path
   ```
     
<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- USAGE -->
## Usage
1. Allow the container to use the screen:
```sh
xhost +
```
2. Start the container:
```sh
docker-compose run masterclass_project /bin/bash
```
3. Compile and launch the simulation:
```sh
cd /root/ros2_ws && \
colcon build && \
source /root/ros2_ws/install/setup.bash && \
ros2 launch the_construct_office_gazebo starbots_ur3e.launch.xml
```
4. Moveit:
```sh
source /root/ros2_ws/install/setup.bash && \
ros2 launch my_moveit_config move_group.launch.py && \
ros2 launch my_moveit_config moveit_rviz.launch.py
```

gazebo-ros-pkgs descargado de la branch ros2 arregla el problema de la pointcloud no ordenada en la depth camara