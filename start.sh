#!/bin/bash

# Husky Robot Control Server Startup Script
# This script sources ROS2 environment and starts the control server

# Source ROS2 environment
source /opt/ros/jazzy/setup.bash

# Activate virtual environment if it exists
if [ -d ".venv" ]; then
    source .venv/bin/activate
    echo "✅ Virtual environment activated"
else
    uv venv -p 3.12
fi

# Start the Husky Robot Control Server
echo "🚀 Starting Husky Robot Control Server..."
uv run husky_rcs.py