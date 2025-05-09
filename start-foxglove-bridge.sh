#!/bin/bash

source /opt/ros/humble/setup.bash && source ~/IdeaProjects/px4-ros2-localization/ws_sensor_combined/install/local_setup.bash

ros2 launch foxglove_bridge foxglove_bridge_launch.xml