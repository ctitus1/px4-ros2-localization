#!/bin/bash

source /opt/ros/humble/setup.bash && source ~/IdeaProjects/px4-ros2-localization/ws_sensor_combined/install/local_setup.bash

# Start Micro XRCE Agent in background
MicroXRCEAgent udp4 -p 8888 > /dev/null 2>&1 &
AGENT_PID=$!

# Start PX4 simulation in background
PX4_GZ_WORLD=aruco make -C ./PX4-Autopilot px4_sitl gz_x500_gimbal &
PX4_PID=$!

# Cleanup function to kill background processes
cleanup() {
    echo "Shutting down..."
    kill $PX4_PID $AGENT_PID 2>/dev/null
    wait $PX4_PID $AGENT_PID 2>/dev/null
    echo "Exited cleanly."
    exit 0
}

# Trap INT (Ctrl+C), TERM, and EXIT signals
trap cleanup INT TERM EXIT

# Wait for background jobs
wait
