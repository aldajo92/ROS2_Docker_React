#!/bin/bash

echo "🌉 Starting ROS Bridge Server..."

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Check if ROS2 is sourced properly
if ! command -v ros2 &> /dev/null; then
    echo "❌ ROS2 not found. Make sure ROS2 is installed."
    exit 1
fi

# Start rosbridge server
echo "📡 Launching rosbridge_websocket on port 9090..."
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &

# Store the PID
ROSBRIDGE_PID=$!
echo "📋 ROS Bridge PID: $ROSBRIDGE_PID"

# Wait a bit to let it initialize
sleep 3

# Check if rosbridge is running
if ps -p $ROSBRIDGE_PID > /dev/null; then
    echo "✅ ROS Bridge Server started successfully"
    echo "🌐 WebSocket available at ws://localhost:9090"
else
    echo "❌ Failed to start ROS Bridge Server"
    exit 1
fi

# Keep the script running
wait $ROSBRIDGE_PID 