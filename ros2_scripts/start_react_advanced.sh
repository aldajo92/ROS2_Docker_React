#!/bin/bash

PROJECT_ROOT="$(cd "$(dirname "$0")"; cd ..; pwd)"
source ${PROJECT_ROOT}/config_docker.sh

echo "🚀 Starting ROS2 React Integration (Advanced)..."

# Cleanup function
cleanup() {
    echo "🛑 Shutting down services..."
    docker exec ${DOCKER_CONTAINER_NAME} /bin/bash -c "pkill -f rosbridge" 2>/dev/null || true
    docker exec ${DOCKER_CONTAINER_NAME} /bin/bash -c "pkill -f react-scripts" 2>/dev/null || true
    exit 0
}

# Set up signal handlers
trap cleanup SIGINT SIGTERM

# Start both services with tmux for better process management
docker exec -it ${DOCKER_CONTAINER_NAME} /bin/bash -c "
    # Install tmux if not available (for session management)
    apt-get update -qq && apt-get install -y tmux curl 2>/dev/null || true
    
    echo '📡 Starting rosbridge server...'
    # Start rosbridge in tmux session
    tmux new-session -d -s rosbridge 'cd /react_app && ./start-rosbridge.sh'
    
    echo '⏳ Waiting for rosbridge to initialize...'
    # Wait for rosbridge to be ready
    for i in {1..30}; do
        if curl -s http://localhost:9090 >/dev/null 2>&1; then
            echo '✅ Rosbridge is ready!'
            break
        fi
        echo \"Waiting for rosbridge... (\$i/30)\"
        sleep 1
    done
    
    echo '⚛️  Starting React application...'
    # Start React in foreground
    cd /react_app
    ./start-docker.sh
"

# Cleanup on exit
cleanup 