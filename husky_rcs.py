#!/usr/bin/env python3
"""
Husky Robot Control Server - ROS2 Node with WebSocket Server
Receives joystick input via WebSocket and publishes to ROS2 cmd_vel topic.
"""

import asyncio
import json
import logging
import threading
from typing import Dict, Any

import websockets
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class HuskyWebSocketServer:
    """WebSocket server for receiving joystick commands for Husky robot."""
    
    def __init__(self, ros2_node, host="localhost", port=8767):
        self.host = host
        self.port = port
        self.clients = set()
        self.latest_joystick_data = {"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, 
                                   "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}
        self.ros2_node = ros2_node

    async def handle_client(self, websocket):
        """Handle a new WebSocket client connection."""
        self.clients.add(websocket)
        logger.info(f"New client connected from {websocket.remote_address}")
        
        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                    await self.process_joystick_data(data)
                except json.JSONDecodeError:
                    logger.error(f"Invalid JSON received: {message}")
                except Exception as e:
                    logger.error(f"Error processing message: {e}")
        except websockets.ConnectionClosed:
            logger.info("Client disconnected")
        except Exception as e:
            logger.error(f"WebSocket error: {e}")
        finally:
            self.clients.discard(websocket)

    async def process_joystick_data(self, data: Dict[str, Any]):
        """Process joystick data and update latest state."""
        if "linear" in data and "angular" in data:
            self.latest_joystick_data = data
            logger.debug(f"Updated joystick data: {data}")
            
            # Publish to ROS2 immediately
            self.ros2_node.publish_twist(data)
        else:
            logger.warning(f"Invalid joystick data format: {data}")

    async def start_server(self):
        """Start the WebSocket server."""
        logger.info(f"Starting WebSocket server on {self.host}:{self.port}")
        return await websockets.serve(self.handle_client, self.host, self.port)

    def get_latest_data(self) -> Dict[str, Any]:
        """Get the latest joystick data."""
        return self.latest_joystick_data


class HuskyROS2Node(Node):
    """ROS2 node for publishing cmd_vel messages."""
    
    def __init__(self):
        super().__init__('husky_control_server')
        
        # Create QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Create publisher for cmd_vel
        self.cmd_vel_publisher = self.create_publisher(
            Twist, 
            'cmd_vel', 
            qos_profile
        )
        
        logger.info("ROS2 Husky node initialized successfully")

    def publish_twist(self, joystick_data: Dict[str, Any]):
        """Publish a Twist message from joystick data."""
        try:
            twist_msg = Twist()
            
            # Extract linear velocities
            linear = joystick_data.get("linear", {})
            twist_msg.linear.x = float(linear.get("x", 0.0))
            twist_msg.linear.y = float(linear.get("y", 0.0))
            twist_msg.linear.z = float(linear.get("z", 0.0))
            
            # Extract angular velocities
            angular = joystick_data.get("angular", {})
            twist_msg.angular.x = float(angular.get("x", 0.0))
            twist_msg.angular.y = float(angular.get("y", 0.0))
            twist_msg.angular.z = float(angular.get("z", 0.0))
            
            # Publish the message
            self.cmd_vel_publisher.publish(twist_msg)
            
            logger.debug(f"Published Twist: linear=({twist_msg.linear.x}, {twist_msg.linear.y}, {twist_msg.linear.z}), "
                        f"angular=({twist_msg.angular.x}, {twist_msg.angular.y}, {twist_msg.angular.z})")
            
        except Exception as e:
            logger.error(f"Error publishing Twist message: {e}")


def run_ros2_node():
    """Run the ROS2 node in a separate thread."""
    rclpy.init()
    
    try:
        node = HuskyROS2Node()
        rclpy.spin(node)
    except KeyboardInterrupt:
        logger.info("ROS2 node interrupted")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


async def main():
    """Main function to run both WebSocket server and ROS2 node."""
    logger.info("Starting Husky Robot Control Server")
    
    # Initialize ROS2
    rclpy.init()
    ros2_node = HuskyROS2Node()
    
    # Start ROS2 node in separate thread
    ros2_thread = threading.Thread(target=lambda: rclpy.spin(ros2_node), daemon=True)
    ros2_thread.start()
    
    # Create and start WebSocket server
    websocket_server = HuskyWebSocketServer(ros2_node)
    logger.info("Starting WebSocket server on localhost:8767")
    
    try:
        # Start the WebSocket server
        server = await websocket_server.start_server()
        logger.info("WebSocket server started successfully")
        
        # Keep the server running
        await server.wait_closed()
        
    except KeyboardInterrupt:
        logger.info("Server interrupted by user")
    except Exception as e:
        logger.error(f"Server error: {e}")
    finally:
        logger.info("Shutting down...")
        ros2_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("Application interrupted")
    except Exception as e:
        logger.error(f"Application error: {e}")