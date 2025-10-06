#!/usr/bin/env python
"""
Example WebSocket client demonstrating DELTA CONTROL for Husky Robot Control Server.
Shows best practices for teleoperation with the new rate-managed system.
"""

import asyncio
import json
import websockets
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class DeltaControlClient:
    """
    Example client demonstrating delta control system.
    Only sends commands when joystick values change (not continuously).
    """
    
    def __init__(self, uri="ws://localhost:8767"):
        self.uri = uri
        self.last_command = None  # Track last sent command
        
    async def send_delta_command(self, websocket, linear_x: float = 0.0, linear_y: float = 0.0, 
                                angular_z: float = 0.0):
        """Send command only if it's different from the last one (delta control)."""
        command = {
            "twist": {
                "linear": {"x": linear_x, "y": linear_y, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": angular_z}
            }
        }
        
        # Only send if command changed (delta control)
        if command != self.last_command:
            await websocket.send(json.dumps(command))
            logger.info(f"🎮 Delta command: linear_x={linear_x:.1f}, angular_z={angular_z:.1f}")
            self.last_command = command
        else:
            logger.debug("Command unchanged, not sending (delta control)")
    
    async def send_stop_command(self, websocket):
        """Send explicit stop command."""
        await self.send_delta_command(websocket, 0.0, 0.0, 0.0)
        logger.info("🛑 Stop command sent")
        
    async def simulate_joystick_demo(self):
        """Simulate realistic joystick control with delta commands."""
        try:
            async with websockets.connect(self.uri) as websocket:
                logger.info(f"🔗 Connected to {self.uri}")
                logger.info("🎮 Starting delta control demo...")
                
                # Simulate joystick movements (only send when values change)
                movements = [
                    ("Move forward", 1.0, 0.0),
                    ("Continue forward", 1.0, 0.0),  # Same as previous - won't send
                    ("Turn while moving", 1.0, 0.5),
                    ("Sharp left turn", 0.5, 1.0),
                    ("Reverse", -0.8, 0.0),
                    ("Stop", 0.0, 0.0),
                    ("Rotate in place", 0.0, 1.0),
                    ("Final stop", 0.0, 0.0),
                ]
                
                for description, linear_x, angular_z in movements:
                    logger.info(f"📱 Joystick: {description}")
                    await self.send_delta_command(websocket, linear_x, 0.0, angular_z)
                    
                    # Simulate varying client delays (real-world scenario)
                    await asyncio.sleep(1.5)
                
                logger.info("✅ Demo completed - server will auto-stop after timeout")
                
        except websockets.ConnectionClosed:
            logger.info("🔌 Connection closed - server will auto-stop robot")
        except Exception as e:
            logger.error(f"❌ Error: {e}")

    async def simulate_mobile_app_behavior(self):
        """Simulate mobile app with intermittent connectivity (Flutter/React Native)."""
        try:
            async with websockets.connect(self.uri) as websocket:
                logger.info("📱 Simulating mobile app with delta control...")
                
                # Simulate user touching joystick
                await self.send_delta_command(websocket, 1.0, 0.0)  # Start moving
                await asyncio.sleep(0.5)
                
                # User adjusts joystick (only send changes)
                await self.send_delta_command(websocket, 1.0, 0.3)  # Add turning
                await asyncio.sleep(1.0)
                
                # Network issue simulation - no commands sent for 3 seconds
                logger.info("📶 Simulating network issue (no commands for 3s)...")
                await asyncio.sleep(3.0)  # Server will auto-stop after timeout
                
                # Connection recovered - send new command
                logger.info("📶 Network recovered")
                await self.send_delta_command(websocket, 0.5, -0.5)  # Different command
                await asyncio.sleep(1.0)
                
                # User releases joystick (send stop)
                await self.send_stop_command(websocket)
                
        except Exception as e:
            logger.error(f"❌ Mobile simulation error: {e}")


class LegacyClient:
    """Example client using legacy Twist format (for backward compatibility)."""
    
    def __init__(self, uri="ws://localhost:8767"):
        self.uri = uri
        
    async def send_legacy_command(self, websocket, linear_x: float, angular_z: float):
        """Send command using legacy Twist format."""
        command = {
            "linear": {"x": linear_x, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": angular_z}
        }
        await websocket.send(json.dumps(command))
        logger.info(f"📤 Legacy Twist: linear_x={linear_x}, angular_z={angular_z}")
        
    async def run_legacy_demo(self):
        """Demonstrate legacy format support."""
        try:
            async with websockets.connect(self.uri) as websocket:
                logger.info("🔄 Testing legacy Twist format compatibility...")
                
                await self.send_legacy_command(websocket, 0.5, 0.0)
                await asyncio.sleep(2)
                await self.send_legacy_command(websocket, 0.0, 0.0)  # Stop
                
                logger.info("✅ Legacy format test completed")
                
        except Exception as e:
            logger.error(f"❌ Legacy test error: {e}")


async def main():
    """Run demonstration of delta control system."""
    print("🤖 Husky Delta Control Client Demo")
    print("=" * 40)
    
    # Choose demo type
    import sys
    if len(sys.argv) > 1:
        demo_type = sys.argv[1]
    else:
        demo_type = "delta"
    
    if demo_type == "mobile":
        client = DeltaControlClient()
        await client.simulate_mobile_app_behavior()
    elif demo_type == "legacy":
        client = LegacyClient()
        await client.run_legacy_demo()
    else:
        # Default: delta control demo
        client = DeltaControlClient()
        await client.simulate_joystick_demo()


if __name__ == "__main__":
    print("Usage:")
    print("  python example_client.py          # Delta control demo")
    print("  python example_client.py mobile   # Mobile app simulation")
    print("  python example_client.py legacy   # Legacy format test")
    print()
    asyncio.run(main())