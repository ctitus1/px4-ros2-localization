#!/bin/bash

source /opt/ros/humble/setup.bash && source ~/IdeaProjects/px4-ros2-localization/ws_sensor_combined/install/local_setup.bash

ros2 run drone_visualizer visualizer_node
