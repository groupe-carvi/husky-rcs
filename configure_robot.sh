#!/bin/bash
# configure_robot.sh - Configure Husky robot for WebSocket control

echo "🤖 Configuring Husky Robot for WebSocket Control"
echo "================================================"

# Function to check if a ROS2 node is running
check_node() {
    local node_name=$1
    if ros2 node list | grep -q "$node_name"; then
        echo "✅ Node '$node_name' is running"
        return 0
    else
        echo "❌ Node '$node_name' is not running"
        return 1
    fi
}

# Function to check topic publishing rate
check_topic_rate() {
    local topic_name=$1
    echo "📊 Checking publishing rate for $topic_name..."
    timeout 3s ros2 topic hz "$topic_name" 2>/dev/null || echo "❌ No data on $topic_name"
}

echo
echo "1. Checking current ROS2 nodes..."
echo "--------------------------------"
ros2 node list

echo
echo "2. Checking velocity-related topics..."
echo "------------------------------------"
ros2 topic list | grep -E "(cmd_vel|teleop|joy|websocket)" || echo "No velocity topics found"

echo
echo "3. Checking topic rates..."
echo "-------------------------"
check_topic_rate "/cmd_vel"
check_topic_rate "/teleop/cmd_vel" 
check_topic_rate "/joy/cmd_vel"

echo
echo "4. Solutions to resolve gamepad interference:"
echo "============================================"

echo
echo "Option A: Disable gamepad controller node"
echo "----------------------------------------"
echo "If you see a gamepad/joy node running, kill it with:"
echo "  ros2 lifecycle set /joy_node shutdown"
echo "  # or"
echo "  pkill -f joy_node"
echo "  pkill -f teleop"

echo
echo "Option B: Use twist_mux to prioritize WebSocket over gamepad"
echo "------------------------------------------------------------"
echo "Configure twist_mux (if available) to prioritize your WebSocket commands:"
echo "  ros2 param set /twist_mux topics.websocket.topic /cmd_vel"
echo "  ros2 param set /twist_mux topics.websocket.timeout 2.0"
echo "  ros2 param set /twist_mux topics.websocket.priority 100"
echo "  ros2 param set /twist_mux topics.joy.priority 50"

echo
echo "Option C: Use twist_mux for topic prioritization"
echo "-----------------------------------------------"
echo "Configure twist_mux to prioritize your WebSocket commands:"
cat << 'EOF'
twist_mux:
  topics:
    websocket:
      topic: /websocket/cmd_vel
      timeout: 2.0
      priority: 100
    gamepad:
      topic: /teleop/cmd_vel  
      timeout: 1.0
      priority: 50
  cmd_vel_out: /cmd_vel
EOF

echo
echo "5. Test your WebSocket control:"
echo "=============================="
echo "After applying solutions, test with:"
echo "  python3 example_client.py"
echo "  # and monitor with:"
echo "  ros2 topic echo /cmd_vel"

echo
echo "6. Emergency stop:"
echo "=================="
echo "If robot moves unexpectedly:"
echo "  ros2 topic pub /cmd_vel geometry_msgs/TwistStamped '{}' --once"

echo
echo "🎯 Quick fix: Run this to disable gamepad and start WebSocket control:"
echo "sudo pkill -f 'joy.*node' && sudo pkill -f 'teleop.*joy' && python3 husky_rcs.py"