#!/bin/bash

PROJECT_ROOT="$(cd "$(dirname "$0")"; cd ..; pwd)"
source ${PROJECT_ROOT}/config_docker.sh

echo "🧪 Running ROS2 React Tests..."

# Function to check if container is running
check_container() {
    docker ps --filter "name=${DOCKER_CONTAINER_NAME}" --filter "status=running" --format "{{.Names}}" | grep -q "^${DOCKER_CONTAINER_NAME}$"
    return $?
}

# Check if container is running
if ! check_container; then
    echo "❌ Container ${DOCKER_CONTAINER_NAME} is not running!"
    echo "   Please start the container first with: ./ros2_scripts/start_docker.sh"
    exit 1
fi

echo "📦 Container ${DOCKER_CONTAINER_NAME} is running"
echo "🚀 Starting test execution..."

# Run tests in the container
docker exec -it ${DOCKER_CONTAINER_NAME} /bin/bash -c "
    echo '📂 Navigating to React app directory...'
    cd /react_app
    
    echo '🔍 Checking Node.js and npm versions...'
    node --version
    npm --version
    
    echo '📋 Installing/updating dependencies if needed...'
    npm install --silent
    
    echo '🧪 Running test suite...'
    echo '====================================='
    
    # Run tests with coverage and verbose output
    # --watchAll=false: Don't watch for file changes (CI mode)
    # --verbose: Show individual test results
    # --coverage: Generate coverage report
    # --passWithNoTests: Don't fail if no tests found
    npm test -- --watchAll=false --verbose --coverage --passWithNoTests
    
    TEST_EXIT_CODE=\$?
    
    echo '====================================='
    if [ \$TEST_EXIT_CODE -eq 0 ]; then
        echo '✅ All tests passed successfully!'
    else
        echo '❌ Some tests failed!'
    fi
    
    exit \$TEST_EXIT_CODE
"

TEST_RESULT=$?

if [ $TEST_RESULT -eq 0 ]; then
    echo "🎉 Test execution completed successfully!"
else
    echo "💥 Test execution failed with exit code: $TEST_RESULT"
    exit $TEST_RESULT
fi 