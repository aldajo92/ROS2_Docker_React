#!/bin/bash

echo "Starting ROS2 rosbridge server..."
echo "This will allow web applications to connect to ROS2 via WebSocket on port 9090"

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Start rosbridge server
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
