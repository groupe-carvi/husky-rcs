#!/usr/bin/env python
"""
Example WebSocket client for testing the Husky Robot Control Server.
Sends joystick commands to control the robot.
"""

import asyncio
import json
import websockets
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class JoystickClient:
    """Example joystick client that sends commands to the Husky WebSocket server."""
    
    def __init__(self, uri="ws://localhost:8767"):
        self.uri = uri
        
    async def send_joystick_command(self, websocket, linear_x: float = 0.0, linear_y: float = 0.0, 
                                  angular_z: float = 0.0):
        """Send a joystick command to the WebSocket server."""
        command = {
            "twist": {
                "linear": {
                    "x": linear_x,
                    "y": linear_y,
                    "z": 0.0
                },
                "angular": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": angular_z
                }
            }
        }
        
        await websocket.send(json.dumps(command))
        logger.info(f"Sent TwistStamped command: linear_x={linear_x}, linear_y={linear_y}, angular_z={angular_z}")
        
    async def run_demo(self):
        """Run a demo sequence of robot movements."""
        try:
            async with websockets.connect(self.uri) as websocket:
                logger.info(f"Connected to {self.uri}")
                
                # Demo sequence
                commands = [
                    (1.0, 0.0, 0.0),   # Move forward
                    (0.0, 0.0, 1.0),   # Rotate left
                    (-1.0, 0.0, 0.0),  # Move backward
                    (0.0, 0.0, -1.0),  # Rotate right
                    (0.0, 0.0, 0.0),   # Stop
                ]
                
                for linear_x, linear_y, angular_z in commands:
                    await self.send_joystick_command(websocket, linear_x, linear_y, angular_z)
                    await asyncio.sleep(2)  # Wait 2 seconds between commands
                    
                logger.info("Demo sequence completed")
                
        except Exception as e:
            logger.error(f"Error connecting to WebSocket server: {e}")


async def main():
    """Main function to run the joystick client demo."""
    client = JoystickClient()
    await client.run_demo()


if __name__ == "__main__":
    asyncio.run(main())