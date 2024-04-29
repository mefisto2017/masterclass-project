import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    hole_detector_node = Node(
        package='hole_detector',
        executable='holedetector.py',
        name='hole_detector_node',
        output='screen',
        #remappings=[('/depth/points' , '/wrist_rgbd_depth_sensor/points')],  # Add remappings if necessary
    )

    return LaunchDescription([
        hole_detector_node,
    ])
