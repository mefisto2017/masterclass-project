import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Initialize Arguments
    moveit_package = get_package_share_directory("real_my_moveit_config")
    ur_package = get_package_share_directory("ur_description")

    # Find paths
    scene_file_path = PathJoinSubstitution([moveit_package, "scenes", "coffee_shop.scene"])
    srdf_file_path = PathJoinSubstitution([moveit_package, "config", "name.srdf"])
    controller_file_path = PathJoinSubstitution([moveit_package, "config", "moveit_controllers.yaml"])
    urdf_file_path = PathJoinSubstitution([ur_package, "urdf", "ur.urdf.xacro"]) 

    moveit_config = MoveItConfigsBuilder("name", package_name="real_my_moveit_config").to_moveit_configs()

    ppp_node = Node(
        name="pick_and_place_perception",
        package="pick_and_place",
        executable="pick_and_place_perception",
        output="screen",
        parameters=[moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    {'use_sim_time': False},],
    )

    scene_pub = Node(
        name="scene_publisher",
        package="moveit_ros_planning",
        executable="moveit_publish_scene_from_text",
        output="screen",
        parameters=[],
        arguments=[
            '--scene', scene_file_path]
    )
    
    return LaunchDescription([
        ppp_node,
        scene_pub
    ])
