import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
  
    # Include launch file for servers
    launch_include = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("servers"),
            "launch/servers_launch.yaml"
        )
    )

    return LaunchDescription([
        launch_include, 
        Node(
            package = "ros_sm",
            executable = "ros_sm",
        )
    ])