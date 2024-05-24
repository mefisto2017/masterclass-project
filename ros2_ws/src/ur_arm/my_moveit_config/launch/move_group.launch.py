import os
from ament_index_python.packages import get_package_share_directory

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # moveit_config = MoveItConfigsBuilder("name", package_name="my_moveit_config").to_moveit_configs()

    moveit_config = (
        MoveItConfigsBuilder("name", package_name="my_moveit_config")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )
    
    # Move Group Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"trajectory_execution.allowed_execution_duration_scaling": 2.0,},
            {"publish_robot_description_semantic": True},
            {"use_sim_time": True},
        ],
    )

    scene_planner_node = Node(
        package="moveit_ros_planning",
        executable="moveit_publish_scene_from_text",
        output="screen",
    )

    return LaunchDescription(
        [move_group_node]
    )