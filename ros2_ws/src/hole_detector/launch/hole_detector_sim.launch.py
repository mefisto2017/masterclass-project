import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    hole_detector_node = Node(
        package='hole_detector',
        executable='holedetector_sim.py',
        name='hole_detector_node',
        output='screen',
        #remappings=[('/wrist_rgbd_depth_sensor/points', '/D415/depth/color/points' ),
        #            ('/wrist_rgbd_depth_sensor/image_raw', '/D415/color/image_raw')],  # Add remappings if necessary
    )

    return LaunchDescription([
        hole_detector_node,
    ])
