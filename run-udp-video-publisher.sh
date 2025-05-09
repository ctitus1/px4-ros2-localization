#!/bin/bash

source /opt/ros/humble/setup.bash && source ~/IdeaProjects/px4-ros2-localization/ws_sensor_combined/install/local_setup.bash

ros2 run udp_video_pkg udp_video_publisher