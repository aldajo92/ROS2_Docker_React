#!/bin/bash

PROJECT_ROOT="$(cd "$(dirname "$0")"; cd ..; pwd)"
source ${PROJECT_ROOT}/config_docker.sh

echo "🌐 Serving ROS2 React Dashboard (Production Build)..."

# Function to check if container is running
check_container() {
    docker ps --filter "name=${DOCKER_CONTAINER_NAME}" --filter "status=running" --format "{{.Names}}" | grep -q "^${DOCKER_CONTAINER_NAME}$"
    return $?
}

# Check if container is running
if ! check_container; then
    echo "❌ Container ${DOCKER_CONTAINER_NAME} is not running!"
    echo "   Please start the container first with: ./scripts/run.sh"
    exit 1
fi

echo "📦 Container ${DOCKER_CONTAINER_NAME} is running"

# Function to check if rosbridge is running
check_rosbridge() {
    docker exec ${DOCKER_CONTAINER_NAME} /bin/bash -c "curl -s http://localhost:9090 > /dev/null 2>&1"
    return $?
}

# Serve pre-built React app using Python HTTP server
docker exec -it ${DOCKER_CONTAINER_NAME} /bin/bash -c "
    echo '📡 Starting rosbridge server in background...'
    cd /react_app
    ./start-rosbridge.sh &
    ROSBRIDGE_PID=\$!
    
    echo '⏳ Waiting for rosbridge to initialize...'
    sleep 5
    
    # Check if build directory exists
    if [ ! -d 'build' ]; then
        echo '❌ Build directory not found!'
        echo '🔨 Please run the build script first: ./ros2_scripts/build_production.sh'
        kill \$ROSBRIDGE_PID 2>/dev/null || true
        exit 1
    fi
    
    echo '🌐 Starting production server on port 5050...'
    echo '📱 Dashboard available at: http://localhost:5050'
    echo '🛑 Press Ctrl+C to stop the server'
    echo ''
    
    cd build
    
    # Use Python 3 HTTP server
    python3 -m http.server 5050
    
    # If server exits, kill rosbridge
    echo '🛑 Shutting down rosbridge...'
    kill \$ROSBRIDGE_PID 2>/dev/null || true
" 