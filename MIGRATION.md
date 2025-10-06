# Migration Guide: Upgrading to Delta Control System

## 📋 Overview

The Husky Robot Control Server has been upgraded from a continuous command system to a **Delta Control System** that dramatically improves teleoperation reliability, especially for mobile applications. This guide helps existing clients migrate to the new implementation.

## 🔄 What Changed

### Before (Old System)
- **Continuous Commands**: Clients had to send 20+ commands per second
- **Rate Dependency**: Robot movement quality depended on client sending rate
- **Network Sensitive**: Poor network = jerky robot movement
- **Manual Safety**: No automatic stop on disconnect

### After (New Delta Control System)
- **Delta Commands**: Send only when joystick values change
- **Server-Managed Rate**: Consistent 20Hz publishing regardless of client
- **Network Resilient**: Smooth movement even with poor connectivity
- **Auto-Safety**: Automatic stop on timeout or disconnect

## 🚀 Migration Steps

### Step 1: Update Message Format (Optional)

The server now prefers `TwistStamped` format but maintains backward compatibility:

#### New Preferred Format (TwistStamped)

```javascript
// NEW: TwistStamped format
const command = {
    twist: {
        linear: { x: 1.0, y: 0.0, z: 0.0 },
        angular: { x: 0.0, y: 0.0, z: 0.5 }
    },
    header: {  // Optional
        frame_id: "base_link"
    }
};
```

#### Legacy Format (Still Supported)

```javascript
// OLD: Still works for backward compatibility
const command = {
    linear: { x: 1.0, y: 0.0, z: 0.0 },
    angular: { x: 0.0, y: 0.0, z: 0.5 }
};
```

### Step 2: Implement Delta Control Logic

#### ❌ Remove Continuous Sending

```javascript
// OLD: Don't do this anymore
setInterval(() => {
    sendCommand(currentJoystickState);  // Sends even if unchanged
}, 50); // 20Hz continuous sending
```

#### ✅ Implement Delta Control

```javascript
// NEW: Delta control implementation
class HuskyTeleopClient {
    constructor(wsUrl) {
        this.ws = new WebSocket(wsUrl);
        this.lastCommand = null;
    }
    
    sendTeleopCommand(linearX, linearY, angularZ) {
        const command = {
            twist: {
                linear: { x: linearX, y: linearY, z: 0.0 },
                angular: { x: 0.0, y: 0.0, z: angularZ }
            }
        };
        
        // Only send if command changed (delta control)
        if (!this.commandsEqual(command, this.lastCommand)) {
            this.ws.send(JSON.stringify(command));
            this.lastCommand = command;
            console.log(`Delta command sent: ${linearX}, ${angularZ}`);
        }
    }
    
    commandsEqual(cmd1, cmd2) {
        if (!cmd1 || !cmd2) return false;
        return (
            cmd1.twist.linear.x === cmd2.twist.linear.x &&
            cmd1.twist.angular.z === cmd2.twist.angular.z
        );
    }
    
    // Call this when joystick values change
    onJoystickChange(linearX, angularZ) {
        this.sendTeleopCommand(linearX, 0.0, angularZ);
    }
    
    // Call this when user releases joystick
    onJoystickRelease() {
        this.sendTeleopCommand(0.0, 0.0, 0.0);  // Stop command
    }
}
```

### Step 3: Update Event Handlers

#### Web Applications

```javascript
// Joystick library integration
joystick.on('move', (data) => {
    // Only send when values actually change
    client.onJoystickChange(data.x, data.y);
});

joystick.on('release', () => {
    // Always send stop when released
    client.onJoystickRelease();
});
```

#### Flutter Mobile App

```dart
class HuskyController {
  WebSocketChannel? _channel;
  Map<String, dynamic>? _lastCommand;
  
  void connectToRobot(String serverUrl) {
    _channel = WebSocketChannel.connect(Uri.parse(serverUrl));
  }
  
  void sendDeltaCommand(double linearX, double angularZ) {
    final command = {
      'twist': {
        'linear': {'x': linearX, 'y': 0.0, 'z': 0.0},
        'angular': {'x': 0.0, 'y': 0.0, 'z': angularZ}
      }
    };
    
    // Only send if command changed
    if (!_commandsEqual(command, _lastCommand)) {
      _channel?.sink.add(jsonEncode(command));
      _lastCommand = command;
      print('Delta command: $linearX, $angularZ');
    }
  }
  
  bool _commandsEqual(Map<String, dynamic>? cmd1, Map<String, dynamic>? cmd2) {
    if (cmd1 == null || cmd2 == null) return false;
    return cmd1['twist']['linear']['x'] == cmd2['twist']['linear']['x'] &&
           cmd1['twist']['angular']['z'] == cmd2['twist']['angular']['z'];
  }
  
  // Call when joystick moves
  void onJoystickMove(double x, double y) {
    sendDeltaCommand(x, y);
  }
  
  // Call when joystick released
  void onJoystickStop() {
    sendDeltaCommand(0.0, 0.0);
  }
}
```

#### React Native

```javascript
export class HuskyTeleopClient {
    constructor(serverUrl) {
        this.ws = new WebSocket(serverUrl);
        this.lastCommand = null;
    }
    
    sendDeltaCommand(linearX, angularZ) {
        const command = {
            twist: {
                linear: { x: linearX, y: 0.0, z: 0.0 },
                angular: { x: 0.0, y: 0.0, z: angularZ }
            }
        };
        
        if (!this.commandsEqual(command, this.lastCommand)) {
            this.ws.send(JSON.stringify(command));
            this.lastCommand = command;
        }
    }
    
    commandsEqual(cmd1, cmd2) {
        return cmd1?.twist?.linear?.x === cmd2?.twist?.linear?.x &&
               cmd1?.twist?.angular?.z === cmd2?.twist?.angular?.z;
    }
}

// Usage in React Native component
const handleJoystickMove = useCallback((data) => {
    client.sendDeltaCommand(data.x, data.y);
}, [client]);
```

## 📱 Mobile App Considerations

### Network Resilience

The new system is perfect for mobile apps because:

- **Intermittent connectivity** doesn't affect robot smoothness
- **Variable network latency** is handled by server-side rate management
- **Background app behavior** is safer (auto-stop on disconnect)

### Battery Optimization

- **Reduced network traffic** = better battery life
- **No continuous timer loops** needed
- **Event-driven architecture** is more efficient

## 🔧 Server Configuration

Update your server configuration for optimal performance:

### Environment Variables

```bash
# .env file
HUSKY_WEBSOCKET_HOST=0.0.0.0        # Allow remote clients
HUSKY_WEBSOCKET_PORT=8767           # WebSocket port
HUSKY_PUBLISH_RATE=20.0             # ROS2 publish rate (Hz)
HUSKY_TIMEOUT_SECONDS=2.0           # Auto-stop timeout
```

### Launch Configuration

```bash
# Start with delta control
./start.sh

# Or with environment variables
HUSKY_PUBLISH_RATE=30.0 HUSKY_TIMEOUT_SECONDS=1.5 ./start.sh
```

## 🧪 Testing Your Migration

### 1. Test Delta Control

```bash
# Run the example client
python example_client.py delta
```

### 2. Test Mobile Simulation

```bash
# Simulate mobile app behavior
python example_client.py mobile
```

### 3. Test Legacy Compatibility

```bash
# Verify old format still works
python example_client.py legacy
```

### 4. Monitor ROS2 Topic

```bash
# Verify consistent 20Hz publishing
ros2 topic hz /teleop/cmd_vel
```

## ⚠️ Common Migration Issues

### Issue 1: Over-Sending Commands

**Problem**: Still sending commands continuously  
**Solution**: Implement proper delta comparison

```javascript
// Add command comparison logic
if (JSON.stringify(newCommand) !== JSON.stringify(lastCommand)) {
    sendCommand(newCommand);
}
```

### Issue 2: Missing Stop Commands

**Problem**: Robot doesn't stop when expected  
**Solution**: Always send explicit stop commands

```javascript
// On joystick release, disconnection, or error
client.sendTeleopCommand(0.0, 0.0, 0.0);
```

### Issue 3: Rapid Command Changes

**Problem**: Too many delta commands in quick succession  
**Solution**: Add debouncing

```javascript
const debouncedSend = debounce((cmd) => {
    client.sendDeltaCommand(cmd.x, cmd.z);
}, 100); // 100ms debounce
```

## 📊 Performance Benefits

### Before vs After Migration

| Metric | Before | After | Improvement |
|--------|--------|-------|-------------|
| Client Send Rate | 20+ Hz | On-change only | 80-95% reduction |
| Network Traffic | High | Minimal | ~90% reduction |
| Robot Smoothness | Variable | Consistent 20Hz | Always smooth |
| Mobile Battery | Poor | Good | Significant improvement |
| Safety | Manual | Automatic | Built-in protection |

## 🎯 Migration Checklist

- [ ] Update message format to TwistStamped (optional)
- [ ] Remove continuous command loops
- [ ] Implement delta control logic
- [ ] Add command comparison function
- [ ] Update joystick event handlers
- [ ] Add explicit stop commands
- [ ] Test with example client
- [ ] Verify ROS2 topic publishing rate
- [ ] Test timeout behavior
- [ ] Update client documentation

## 📖 Code Examples Repository

For complete working examples of delta control implementation:

- **Python Example**: [`example_client.py`](example_client.py) - Complete delta control examples
- **Server Configuration**: [`.env.example`](.env.example) - Environment variables
- **Installation**: [`install.sh`](install.sh) - Automated setup
- **Full Documentation**: [`README.md`](README.md) - Complete API reference

## 🔗 Additional Resources

### Official Documentation

- [WebSocket API Documentation](README.md#-websocket-api-for-teleoperation)
- [Delta Control System](README.md#-delta-control-system)
- [Configuration Options](README.md#-configuration)

### Example Implementations

```bash
# Clone and test the examples
git clone https://github.com/groupe-carvi/husky-rcs.git
cd husky-rcs

# Test delta control
uv run python example_client.py delta

# Test mobile simulation
uv run python example_client.py mobile
```

## 🆘 Support

If you encounter issues during migration:

1. **Test with example client**: `python example_client.py`
2. **Check server logs**: `journalctl -u husky-control-server -f`
3. **Monitor ROS2**: `ros2 topic echo /teleop/cmd_vel`
4. **Verify network**: Ensure server listens on `0.0.0.0:8767`

### Debug Commands

```bash
# Check server status
systemctl status husky-control-server

# Monitor delta control in logs
journalctl -u husky-control-server -f | grep -E "(Delta|timeout|auto-stop)"

# Test WebSocket connection
websocat ws://localhost:8767

# Verify ROS2 topic rate
ros2 topic hz /teleop/cmd_vel
```

## 🏁 Conclusion

The delta control system makes teleoperation more reliable and efficient, especially for mobile applications. The migration process is straightforward and maintains backward compatibility during the transition.

**Key Benefits After Migration:**
- 🎯 **Reliable Control**: Consistent robot movement regardless of network
- 📱 **Mobile-First**: Perfect for Flutter, React Native, and web apps
- 🔋 **Battery Friendly**: Significant reduction in network traffic
- 🛡️ **Built-in Safety**: Automatic stop on disconnect or timeout
- 🚀 **Performance**: 80-95% reduction in command frequency

Happy migrating! 🚀

---

*For questions or support, please refer to the main documentation or create an issue in the repository.*