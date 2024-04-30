from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(package="starling", executable="circle-flight", name="starling"), 
            Node(package="starling", executable="custom-pc-pub", name="starling"), 
            Node(package="voxl_mpa_to_ros2",
                  executable="voxl_mpa_to_ros2_node",
                    name="voxl-mpa")
        ]
    )
