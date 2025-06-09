#!/bin/bash

# Script to start robot description for React app
# This launches only the robot state publisher and joint state publisher
# without Gazebo or RViz

echo "🤖 Starting robot description for React app..."

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Build the workspace if needed
cd /ros2_ws
echo "🔨 Building ROS2 workspace..."
colcon build --packages-select waver_description

# Source the built workspace
source /ros2_ws/install/setup.bash

echo "🚀 Launching robot description..."
ros2 launch waver_description view_gazebo_rviz.launch.py

echo "✅ Robot description launched successfully!"
echo "📡 Robot state should now be available on ROS topics:"
echo "   - /robot_description (parameter)"
echo "   - /joint_states (topic)"
echo "   - /tf (topic)"
echo "   - /tf_static (topic)" 