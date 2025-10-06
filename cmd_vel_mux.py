#!/usr/bin/env python3
"""
Command Velocity Multiplexer for Husky Robot
Prioritizes WebSocket teleop commands over gamepad controller commands.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import TwistStamped, Twist
import time


class CmdVelMultiplexer(Node):
    """
    Multiplexer that prioritizes teleop/cmd_vel over gamepad cmd_vel.
    
    Topic Priority (highest to lowest):
    1. /teleop/cmd_vel (WebSocket commands) - TwistStamped
    2. /joy/cmd_vel (Gamepad commands) - Twist  
    3. /cmd_vel (Default/other commands) - Twist
    
    Publishes to: /husky_velocity_controller/cmd_vel
    """
    
    def __init__(self):
        super().__init__('cmd_vel_multiplexer')
        
        # Timeout settings (seconds)
        self.teleop_timeout = 2.0  # WebSocket command timeout
        self.joy_timeout = 1.0     # Gamepad command timeout
        
        # Last command timestamps
        self.last_teleop_time = 0.0
        self.last_joy_time = 0.0
        self.last_default_time = 0.0
        
        # Last received commands
        self.last_teleop_cmd = None
        self.last_joy_cmd = None
        self.last_default_cmd = None
        
        # QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Publishers
        self.cmd_vel_publisher = self.create_publisher(
            Twist, 
            '/husky_velocity_controller/cmd_vel',  # Main robot control topic
            qos_profile
        )
        
        # Subscribers with priority order
        self.teleop_subscriber = self.create_subscription(
            TwistStamped,
            '/teleop/cmd_vel',  # WebSocket commands (highest priority)
            self.teleop_callback,
            qos_profile
        )
        
        self.joy_subscriber = self.create_subscription(
            Twist,
            '/joy/cmd_vel',  # Gamepad commands (medium priority)
            self.joy_callback,
            qos_profile
        )
        
        self.default_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',  # Default commands (lowest priority)
            self.default_callback,
            qos_profile
        )
        
        # Timer for publishing active command
        self.timer = self.create_timer(0.05, self.publish_active_command)  # 20Hz
        
        self.get_logger().info('Command Velocity Multiplexer started')
        self.get_logger().info('Priority: /teleop/cmd_vel > /joy/cmd_vel > /cmd_vel')
        self.get_logger().info('Publishing to: /husky_velocity_controller/cmd_vel')
    
    def teleop_callback(self, msg):
        """Handle WebSocket teleop commands (highest priority)."""
        self.last_teleop_time = time.time()
        self.last_teleop_cmd = msg.twist
        self.get_logger().debug('Received teleop command')
    
    def joy_callback(self, msg):
        """Handle gamepad joy commands (medium priority)."""
        self.last_joy_time = time.time()
        self.last_joy_cmd = msg
        self.get_logger().debug('Received joy command')
    
    def default_callback(self, msg):
        """Handle default cmd_vel commands (lowest priority)."""
        self.last_default_time = time.time()
        self.last_default_cmd = msg
        self.get_logger().debug('Received default command')
    
    def publish_active_command(self):
        """Publish the highest priority active command."""
        current_time = time.time()
        active_cmd = None
        source = "none"
        
        # Check teleop (highest priority)
        if (self.last_teleop_cmd and 
            (current_time - self.last_teleop_time) < self.teleop_timeout):
            active_cmd = self.last_teleop_cmd
            source = "teleop"
        
        # Check joy (medium priority)
        elif (self.last_joy_cmd and 
              (current_time - self.last_joy_time) < self.joy_timeout):
            active_cmd = self.last_joy_cmd
            source = "joy"
        
        # Check default (lowest priority)
        elif (self.last_default_cmd and 
              (current_time - self.last_default_time) < self.joy_timeout):
            active_cmd = self.last_default_cmd
            source = "default"
        
        # Publish active command or stop
        if active_cmd:
            self.cmd_vel_publisher.publish(active_cmd)
            if source != getattr(self, '_last_source', None):
                self.get_logger().info(f'Active control source: {source}')
                self._last_source = source
        else:
            # No active commands - publish stop
            stop_cmd = Twist()
            self.cmd_vel_publisher.publish(stop_cmd)
            if getattr(self, '_last_source', None) != "stop":
                self.get_logger().info('No active commands - stopping robot')
                self._last_source = "stop"


def main(args=None):
    rclpy.init(args=args)
    
    try:
        multiplexer = CmdVelMultiplexer()
        rclpy.spin(multiplexer)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()