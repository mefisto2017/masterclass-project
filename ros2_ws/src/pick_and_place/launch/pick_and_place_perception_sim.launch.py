import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("name", package_name="my_moveit_config").to_moveit_configs()

    ppp_node = Node(
        name="pick_and_place_perception_sim",
        package="pick_and_place",
        executable="pick_and_place_perception_sim",
        output="screen",
        parameters=[moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    {'use_sim_time': True},],
    )
    
    return LaunchDescription([
        ppp_node
    ])
