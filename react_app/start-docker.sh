#!/bin/bash

echo "⚛️  Starting React Development Server..."

# Check if package.json exists
if [ ! -f "package.json" ]; then
    echo "❌ package.json not found. Are you in the right directory?"
    exit 1
fi

# Check if node_modules exists
if [ ! -d "node_modules" ]; then
    echo "📦 Installing dependencies..."
    npm install
fi

echo "🚀 Starting React app in development mode..."
echo "🌐 App will be available at http://localhost:3000"
echo "🛑 Press Ctrl+C to stop the server"
echo ""

# Use the npm script with Docker configuration (includes GENERATE_SOURCEMAP=false)
npm run start:docker 