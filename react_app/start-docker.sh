#!/bin/bash
export HOST=0.0.0.0
export PORT=3000
export FAST_REFRESH=false
export WATCHPACK_POLLING=true

echo "Starting React development server..."
echo "HOST: $HOST"
echo "PORT: $PORT"

cd /react_app
npm start 