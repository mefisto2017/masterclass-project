import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Initialize Arguments
    moveit_package = LaunchConfiguration("my_moveit_config")
    ur_package = LaunchConfiguration("ur_description")

    # Find paths
    scene_file_path = PathJoinSubstitution([FindPackageShare(moveit_package), "scenes", "coffee_shop.scene"])
    srdf_file_path = PathJoinSubstitution([FindPackageShare(moveit_package), "config", "name.srdf"])
    urdf_file_path = PathJoinSubstitution([FindPackageShare(ur_package), "urdf", "ur.urdf.xacro"]) 
    
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

    scene_pub = Node(
        name="scene_publisher",
        package="moveit_ros_planning",
        executable="moveit_publish_scene_from_text",
        output="screen",
        parameters=[
            {'robot_description': urdf_file_path},
            {'robot_description_semantic':srdf_file_path}
        ],
        remappings=[
            ('__node:=moveit_publish_scene_from_text')
        ],
        arguments=[scene_file_path]
    )
    
    return LaunchDescription([
        ppp_node
    ])
