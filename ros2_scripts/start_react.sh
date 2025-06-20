#!/bin/bash

PROJECT_ROOT="$(cd "$(dirname "$0")"; cd ..; pwd)"
source ${PROJECT_ROOT}/config_docker.sh

echo "🚀 Starting ROS2 React Integration..."

# Function to check if rosbridge is running
check_rosbridge() {
    docker exec ${DOCKER_CONTAINER_NAME} /bin/bash -c "curl -s http://localhost:9090 > /dev/null 2>&1"
    return $?
}

# Start rosbridge in background and React in foreground
docker exec -it ${DOCKER_CONTAINER_NAME} /bin/bash -c "
    echo '📡 Starting rosbridge server in background...'
    cd /react_app
    ./start-rosbridge.sh &
    ROSBRIDGE_PID=\$!
    
    echo '⏳ Waiting for rosbridge to initialize...'
    sleep 5
    
    echo '⚛️  Starting React application...'
    ./start-docker.sh
    
    # If React app exits, kill rosbridge
    echo '🛑 Shutting down rosbridge...'
    kill \$ROSBRIDGE_PID 2>/dev/null || true
"
 