#!/usr/bin/env python3
"""
Husky Robot Control Server - ROS2 Node with WebSocket Server
Receives joystick input via WebSocket and publishes to ROS2 cmd_vel topic.
"""

import asyncio
import json
import logging
import os
import threading
import time
from typing import Dict, Any

import websockets
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import TwistStamped

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class HuskyWebSocketServer:
    """WebSocket server for receiving joystick commands for Husky robot."""
    
    def __init__(self, ros2_node, host="0.0.0.0", port=8767, publish_rate=20.0, timeout_seconds=2.0):
        self.host = host
        self.port = port
        self.clients = set()
        self.latest_joystick_data = {
            "twist": {
                "linear": {"x": 0.0, "y": 0.0, "z": 0.0}, 
                "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
            }
        }
        self.ros2_node = ros2_node
        self.publish_rate = publish_rate  # Hz - rate to publish commands
        self.timeout_seconds = timeout_seconds  # seconds before auto-stop
        self.last_command_time = 0.0
        self.is_publishing = False
        self.publish_task = None
        
    async def start_command_publisher(self):
        """Start the background task that publishes commands at a fixed rate."""
        if self.publish_task is None or self.publish_task.done():
            self.publish_task = asyncio.create_task(self._command_publisher_loop())
            logger.info(f"Started command publisher at {self.publish_rate} Hz")
    
    async def _command_publisher_loop(self):
        """Background loop that publishes the latest command at a fixed rate."""
        self.is_publishing = True
        publish_interval = 1.0 / self.publish_rate
        
        try:
            while self.is_publishing:
                current_time = time.time()
                
                # Check if we should timeout and stop
                if current_time - self.last_command_time > self.timeout_seconds:
                    # Send stop command if no recent updates
                    stop_data = {
                        "twist": {
                            "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
                            "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
                        }
                    }
                    self.ros2_node.publish_twist_stamped(stop_data)
                    logger.debug("Published auto-stop command due to timeout")
                else:
                    # Publish the latest command
                    self.ros2_node.publish_twist_stamped(self.latest_joystick_data)
                    logger.debug("Published current command")
                
                await asyncio.sleep(publish_interval)
                
        except asyncio.CancelledError:
            logger.info("Command publisher loop cancelled")
        except Exception as e:
            logger.error(f"Error in command publisher loop: {e}")
        finally:
            self.is_publishing = False
    
    async def stop_command_publisher(self):
        """Stop the background command publisher."""
        self.is_publishing = False
        if self.publish_task and not self.publish_task.done():
            self.publish_task.cancel()
            try:
                await self.publish_task
            except asyncio.CancelledError:
                pass
        # Send final stop command
        stop_data = {
            "twist": {
                "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
            }
        }
        self.ros2_node.publish_twist_stamped(stop_data)
        logger.info("Stopped command publisher and sent final stop command")

    async def handle_client(self, websocket):
        """Handle a new WebSocket client connection."""
        self.clients.add(websocket)
        logger.info(f"New client connected from {websocket.remote_address}")
        
        # Start the command publisher when first client connects
        if len(self.clients) == 1:
            await self.start_command_publisher()
        
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
            logger.info(f"Client {websocket.remote_address} disconnected")
        except Exception as e:
            logger.error(f"WebSocket error: {e}")
        finally:
            self.clients.discard(websocket)
            # Stop publisher when last client disconnects
            if len(self.clients) == 0:
                await self.stop_command_publisher()

    async def process_joystick_data(self, data: Dict[str, Any]):
        """Process joystick data and update latest state with timestamp."""
        # Support both legacy Twist format and new TwistStamped format
        if "twist" in data:
            # New TwistStamped format
            if "linear" in data["twist"] and "angular" in data["twist"]:
                self.latest_joystick_data = data
                self.last_command_time = time.time()  # Update timestamp
                logger.debug(f"Updated TwistStamped data: {data}")
            else:
                logger.warning(f"Invalid TwistStamped data format: {data}")
        elif "linear" in data and "angular" in data:
            # Legacy Twist format - convert to TwistStamped
            converted_data = {"twist": data}
            self.latest_joystick_data = converted_data
            self.last_command_time = time.time()  # Update timestamp
            logger.debug(f"Converted legacy Twist to TwistStamped: {converted_data}")
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
    """ROS2 node for publishing cmd_vel messages to Husky controller."""
    
    def __init__(self):
        super().__init__('husky_control_server')
        
        # Create QoS profile for reliable, high-priority communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=1  # Reduced depth for lower latency
        )
        
        # Create publisher for cmd_vel - Husky controller listens to /cmd_vel
        self.teleop_cmd_vel_publisher = self.create_publisher(
            TwistStamped, 
            'teleop/cmd_vel',  # Main Husky controller topic
            qos_profile
        )
        
        logger.info("ROS2 Husky node initialized successfully")

    def publish_twist_stamped(self, joystick_data: Dict[str, Any]):
        """Publish a TwistStamped message from joystick data."""
        try:
            twist_stamped_msg = TwistStamped()
            
            # Set header
            twist_stamped_msg.header.stamp = self.get_clock().now().to_msg()
            header_data = joystick_data.get("header", {})
            twist_stamped_msg.header.frame_id = header_data.get("frame_id", "base_link")
            
            # Extract twist data
            twist_data = joystick_data.get("twist", {})
            
            # Extract linear velocities
            linear = twist_data.get("linear", {})
            twist_stamped_msg.twist.linear.x = float(linear.get("x", 0.0))
            twist_stamped_msg.twist.linear.y = float(linear.get("y", 0.0))
            twist_stamped_msg.twist.linear.z = float(linear.get("z", 0.0))
            
            # Extract angular velocities
            angular = twist_data.get("angular", {})
            twist_stamped_msg.twist.angular.x = float(angular.get("x", 0.0))
            twist_stamped_msg.twist.angular.y = float(angular.get("y", 0.0))
            twist_stamped_msg.twist.angular.z = float(angular.get("z", 0.0))
            
            # Publish the message
            self.teleop_cmd_vel_publisher.publish(twist_stamped_msg)
            
            logger.debug(f"Published TwistStamped: frame_id={twist_stamped_msg.header.frame_id}, "
                        f"linear=({twist_stamped_msg.twist.linear.x}, {twist_stamped_msg.twist.linear.y}, {twist_stamped_msg.twist.linear.z}), "
                        f"angular=({twist_stamped_msg.twist.angular.x}, {twist_stamped_msg.twist.angular.y}, {twist_stamped_msg.twist.angular.z})")
            
        except Exception as e:
            logger.error(f"Error publishing TwistStamped message: {e}")


def run_ros2_node():
    """Run the ROS2 node in a separate thread."""
    rclpy.init()
    node = None
    
    try:
        node = HuskyROS2Node()
        rclpy.spin(node)
    except KeyboardInterrupt:
        logger.info("ROS2 node interrupted")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


async def main():
    """Main function to run both WebSocket server and ROS2 node."""
    logger.info("Starting Husky Robot Control Server with Delta Control")
    
    # Initialize ROS2
    rclpy.init()
    ros2_node = HuskyROS2Node()
    
    # Start ROS2 node in separate thread
    ros2_thread = threading.Thread(target=lambda: rclpy.spin(ros2_node), daemon=True)
    ros2_thread.start()
    
    # Get WebSocket configuration from environment variables or use defaults
    websocket_host = os.getenv('HUSKY_WEBSOCKET_HOST', '0.0.0.0')
    websocket_port = int(os.getenv('HUSKY_WEBSOCKET_PORT', 8767))
    publish_rate = float(os.getenv('HUSKY_PUBLISH_RATE', 20.0))  # Hz
    timeout_seconds = float(os.getenv('HUSKY_TIMEOUT_SECONDS', 2.0))  # seconds
    
    # Create and start WebSocket server
    websocket_server = HuskyWebSocketServer(
        ros2_node, 
        host=websocket_host, 
        port=websocket_port,
        publish_rate=publish_rate,
        timeout_seconds=timeout_seconds
    )
    logger.info(f"Starting WebSocket server on {websocket_host}:{websocket_port}")
    logger.info(f"Command publish rate: {publish_rate} Hz, timeout: {timeout_seconds}s")
    
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
        # Stop the command publisher
        await websocket_server.stop_command_publisher()
        ros2_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("Application interrupted")
    except Exception as e:
        logger.error(f"Application error: {e}")