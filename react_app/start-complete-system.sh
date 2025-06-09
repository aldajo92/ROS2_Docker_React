#!/bin/bash

# Complete system startup script for React + ROS2 integration
# This script launches:
# 1. Robot description (robot_state_publisher + joint_state_publisher)
# 2. Rosbridge websocket server
# 3. React development server

echo "🚀 Starting complete ROS2 + React system..."

# Function to kill background processes on exit
cleanup() {
    echo "🛑 Shutting down services..."
    kill $ROBOT_PID $ROSBRIDGE_PID 2>/dev/null || true
    echo "✅ Services stopped"
    exit 0
}

# Set up signal handlers
trap cleanup SIGINT SIGTERM

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Build workspace
cd /ros2_ws
echo "🔨 Building ROS2 workspace..."
colcon build --packages-select waver_description
source /ros2_ws/install/setup.bash

# Start robot description in background
echo "🤖 Starting robot description..."
ros2 launch waver_description view_gazebo_rviz.launch.py &
ROBOT_PID=$!

# Wait for robot description to initialize
echo "⏳ Waiting for robot description to initialize..."
sleep 3

# Start rosbridge server in background
echo "📡 Starting rosbridge websocket server..."
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
ROSBRIDGE_PID=$!

# Wait for rosbridge to initialize
echo "⏳ Waiting for rosbridge to initialize..."
sleep 3

# Check if services are running
if ps -p $ROBOT_PID > /dev/null 2>&1; then
    echo "✅ Robot description is running (PID: $ROBOT_PID)"
else
    echo "❌ Robot description failed to start"
    exit 1
fi

if ps -p $ROSBRIDGE_PID > /dev/null 2>&1; then
    echo "✅ Rosbridge is running (PID: $ROSBRIDGE_PID)"
else
    echo "❌ Rosbridge failed to start"
    exit 1
fi

# Start React app in foreground
echo "⚛️  Starting React development server..."
cd /react_app
npm start

# If we get here, React app has exited
echo "🛑 React app exited, cleaning up..."
cleanup 