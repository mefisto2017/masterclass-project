import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    hole_detector_node = Node(
        package='hole_detector',
        executable='holedetector_real.py',
        name='hole_detector_node',
        output='screen',
    )

    return LaunchDescription([
        hole_detector_node,
    ])
