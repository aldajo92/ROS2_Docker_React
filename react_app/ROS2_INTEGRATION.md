# ROS2 React Integration Guide

This React application can connect to ROS2 and subscribe to topics in real-time using WebSocket communication.

## Architecture

```
ROS2 System ← → rosbridge_server ← → WebSocket (port 9090) ← → React App (port 3000)
```

## Setup Instructions

### 1. Build and Start the Docker Container

```bash
# Build the Docker image (if not already built)
docker build -t ros2_react_image .

# Start the container
./scripts/run.sh
```

### 2. Start rosbridge Server (In Container Terminal 1)

```bash
# Inside the container
cd /react_app
./start-rosbridge.sh
```

This will start the rosbridge server on port 9090, allowing WebSocket connections.

### 3. Start React App (In Container Terminal 2)

```bash
# Open another terminal to the same container
docker exec -it ros2_react_container bash

# Start React
cd /react_app
./start-docker.sh
```

### 4. Access the Web Interface

Open your browser and go to: `http://localhost:3000`

## Features

### 🔗 Real-time ROS2 Connection
- Automatic connection to rosbridge server
- Connection status indicator
- Error handling and reconnection

### 📡 Topic Subscription
- Subscribe to any ROS2 topic
- Real-time message display
- Message history (last 10 messages)
- JSON formatting for complex messages

### 🚀 Quick Access to Common Topics
- `/clock` - System clock
- `/rosout` - ROS logging messages  
- `/parameter_events` - Parameter changes
- `/cmd_vel` - Velocity commands
- `/odom` - Odometry data
- `/scan` - Laser scan data
- `/tf` - Transform data

### ✨ Custom Topic Support
- Add any topic by name and message type
- Dynamic subscription management
- Remove subscriptions as needed

## Usage Examples

### Subscribe to a Standard Topic

1. Click on one of the common topics (e.g., `/clock`)
2. The topic will appear in the active subscriptions
3. Messages will start appearing in real-time

### Subscribe to a Custom Topic

1. Enter the topic name (e.g., `/my_robot/status`)
2. Enter the message type (e.g., `std_msgs/msg/String`)
3. Click "Subscribe"
4. Monitor the messages in real-time

### Test with ROS2 Commands

In another container terminal, you can publish test messages:

```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Publish a string message
ros2 topic pub /test_topic std_msgs/msg/String "data: 'Hello from ROS2!'" --once

# Publish geometry twist
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}" --once
```

## File Structure

```
react_app/
├── src/
│   ├── components/
│   │   ├── ROS2Dashboard.js      # Main dashboard component
│   │   └── TopicSubscriber.js    # Individual topic subscriber
│   ├── hooks/
│   │   └── useROS.js            # ROS connection hook
│   └── App.js                   # Main app component
├── start-docker.sh              # Start React with Docker config
├── start-rosbridge.sh           # Start rosbridge server
└── package.json                 # Dependencies including roslib
```

## Dependencies

- **roslib**: JavaScript library for ROS communication
- **rosbridge_suite**: ROS2 package providing WebSocket interface

## Troubleshooting

### Connection Issues

1. **Check rosbridge is running**: Make sure port 9090 is accessible
2. **Verify Docker ports**: Ensure both 3000 and 9090 are mapped
3. **Check browser console**: Look for WebSocket connection errors

### Topic Not Receiving Messages

1. **Verify topic exists**: Use `ros2 topic list` in container
2. **Check message type**: Use `ros2 topic info /topic_name`
3. **Test with ros2 commands**: Publish test messages to verify

### Performance Issues

1. **Limit subscriptions**: Too many active subscriptions can slow down the interface
2. **Message history**: Only last 10 messages are kept per topic
3. **Browser resources**: Close unused tabs to free up resources

## Advanced Usage

### Publishing Messages

You can extend the dashboard to publish messages by adding publisher functionality to the React components:

```javascript
// Example: Publishing to a topic
const publisher = new ROSLIB.Topic({
  ros: ros,
  name: '/cmd_vel',
  messageType: 'geometry_msgs/msg/Twist'
});

const twist = new ROSLIB.Message({
  linear: { x: 1.0, y: 0.0, z: 0.0 },
  angular: { x: 0.0, y: 0.0, z: 0.5 }
});

publisher.publish(twist);
```

### Service Calls

You can also call ROS2 services:

```javascript
const service = new ROSLIB.Service({
  ros: ros,
  name: '/my_service',
  serviceType: 'std_srvs/srv/Empty'
});

const request = new ROSLIB.ServiceRequest({});

service.callService(request, (result) => {
  console.log('Service call result:', result);
});
``` 