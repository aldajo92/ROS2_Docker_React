# ROS2 React Dashboard

A modern web dashboard for ROS2 built with React and Docker. This project provides a comprehensive web interface for monitoring and interacting with ROS2 topics, featuring real-time topic subscription, message visualization, and a complete test suite.

## Features

- 🌐 **Web-based ROS2 Dashboard**: Modern React interface for ROS2 interaction
- 📡 **Real-time Topic Monitoring**: Subscribe to and visualize ROS2 topics in real-time
- 📋 **Topic Discovery**: List all available ROS2 topics with one click
- 🔌 **WebSocket Integration**: Uses rosbridge for seamless ROS2-web communication
- 🧪 **Comprehensive Testing**: 39 tests with excellent coverage (89.65% for main components)
- 🐳 **Docker Environment**: Fully containerized development and deployment
- ⚡ **Hot Reload**: Development server with instant updates

## Quick Start

### Prerequisites
- Docker and Docker Compose
- Git

### 1. Clone and Setup
```bash
git clone <repository-url>
cd Docker_ROS2_Humble_React
```

### 2. Build Docker Image
```bash
./scripts/build.sh
```

### 3. Run ROS2 Container
```bash
./scripts/run.sh
```

### 4. Start React Dashboard
```bash
./ros2_scripts/start_react.sh
```

The dashboard will be available at `http://localhost:3000`

## Development

### Running Tests
Execute the comprehensive test suite:
```bash
./ros2_scripts/run_test.sh
```

### Project Structure
```
├── react_app/                 # React dashboard application
│   ├── src/
│   │   ├── components/        # React components
│   │   │   ├── ROS2Dashboard.js      # Main dashboard
│   │   │   ├── TopicSubscriber.js    # Topic subscription component
│   │   │   └── __tests__/            # Component tests
│   │   ├── hooks/             # Custom React hooks
│   │   │   ├── useROS.js             # ROS connection hook
│   │   │   └── __tests__/            # Hook tests
│   │   └── App.js             # Main app component
│   ├── public/                # Static files
│   └── package.json           # Dependencies
├── ros2_ws/                   # ROS2 workspace
├── scripts/                   # Docker scripts
└── ros2_scripts/              # Application scripts
```

## Dashboard Features

### Topic Management
- **Quick Subscribe**: One-click subscription to common topics (/clock, /rosout, /cmd_vel, /odom)
- **Custom Topics**: Subscribe to any topic with custom message types
- **Real-time Display**: Live message visualization with timestamps
- **Message History**: Keep track of the last 10 messages per topic

### Topic Discovery
- **List All Topics**: Discover all available ROS2 topics
- **Service Integration**: Uses ROS2 services with automatic fallback
- **Loading States**: User-friendly loading indicators

### Connection Management
- **Connection Status**: Real-time ROS bridge connection monitoring
- **Error Handling**: Comprehensive error display and recovery
- **WebSocket URL**: Configurable ROS bridge connection

## Testing

The project includes a comprehensive test suite with 39 tests:

- ✅ **App Component**: Basic rendering and integration tests
- ✅ **useROS Hook**: Connection management and state handling
- ✅ **ROS2Dashboard**: UI interactions and topic management
- ✅ **TopicSubscriber**: Component rendering and prop validation

**Test Coverage:**
- Overall: 62.22% statements, 70.21% branches
- ROS2Dashboard: 89.65% statements (excellent coverage)
- useROS Hook: 100% statements (perfect coverage)

## Docker Commands

### Basic Operations
```bash
# Build the image
./scripts/build.sh

# Run container
./scripts/run.sh

# Clean build (remove cache)
./scripts/clean_build.sh

# Access container bash
./scripts/bash.sh
```

### Development Commands
```bash
# Start React development server
./ros2_scripts/start_react.sh

# Run test suite
./ros2_scripts/run_test.sh

# Build and serve for production (port 5050)
./ros2_scripts/build_production.sh

# Serve existing production build (port 5050)
./ros2_scripts/serve_production.sh
```

## Technology Stack

- **Frontend**: React 18, JavaScript ES6+
- **ROS2**: Humble distribution
- **WebSocket**: rosbridge_suite for ROS2-web communication
- **Testing**: Jest, React Testing Library
- **Containerization**: Docker, Ubuntu 22.04
- **Build Tools**: Create React App, npm

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Run tests: `./ros2_scripts/run_test.sh`
5. Submit a pull request

## License

MIT

**Author**: [Alejandro Daniel Jose Gomez Florez](https://www.linkedin.com/in/aldajo92/)

---

## Troubleshooting

### Container Issues
- Ensure Docker is running
- Check container status: `docker ps`
- Restart container: `./scripts/run.sh`

### React App Issues
- Check if rosbridge is running
- Verify WebSocket connection at `ws://localhost:9090`
- Check browser console for errors

### ROS2 Issues
- Ensure ROS2 nodes are running
- Check topic availability: `ros2 topic list`
- Verify rosbridge_suite is installed
