#!/bin/bash
source /opt/ros/foxy/setup.bash
echo "2" | voxl-configure-microdds
source /opt/ros/foxy/mpa_to_ros2/install/setup.bash
ros2 run voxl_mpa_to_ros2 voxl_mpa_to_ros2_node &