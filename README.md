# Husky Robot Control Server 🤖

Teleoperation server bridging WebSocket joystick commands to ROS2 `geometry_msgs/TwistStamped` messages on `/teleop/cmd_vel`.

## 🚀 Features

- Async WebSocket server (multi‑client)
- Publishes directly to ROS2 `/teleop/cmd_vel` (TwistStamped)
- Standard JSON TwistStamped schema (linear / angular)
- Threaded ROS2 spin + asyncio loop
- Configurable port via `HUSKY_WEBSOCKET_PORT`
- Graceful handling of malformed JSON & disconnects
- Systemd deployment auto-setup via `install.sh`

## 📋 Prerequisites

### System Requirements

- Ubuntu 24.04 LTS (Noble)
- ROS2 Jazzy
- Python 3.12+

### ROS2 Dependencies

```bash
sudo apt update
sudo apt install ros-jazzy-desktop-full
sudo apt install python3-websockets
```

### Python Dependencies

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh   # if uv not installed
uv sync
```

## 🛠️ Installation

### Quick (recommended)

```bash
git clone https://github.com/groupe-carvi/husky-rcs.git
cd husky-rcs
./install.sh   # installs deps + systemd service
```

### Manual

```bash
git clone https://github.com/groupe-carvi/husky-rcs.git
cd husky-rcs
source /opt/ros/jazzy/setup.bash
curl -LsSf https://astral.sh/uv/install.sh | sh  # if needed
uv sync
chmod +x start.sh
```

Optional: copy `.env.example` to `.env` and adjust values.

## 🎮 Usage

Start (script):

```bash
./start.sh
```
Start (manual):

```bash
source /opt/ros/jazzy/setup.bash
python3 husky_rcs.py
```
Background:

```bash
nohup python3 husky_rcs.py &
```
Test client:

```bash
python3 example_client.py
```
Monitor topic:

```bash
ros2 topic echo /teleop/cmd_vel
```


## 📡 WebSocket API for Teleoperation

### Connection Setup

- **URL**: `ws://localhost:8767` (local) or `ws://<server-ip>:8767` (remote)
- **Host**: Listens on all interfaces (0.0.0.0) by default - supports remote clients
- **Port**: Configurable via `HUSKY_WEBSOCKET_PORT` environment variable
- **Protocol**: WebSocket (RFC 6455)
- **Format**: JSON messages
- **Frequency**: Real-time (no rate limiting on server side)

### Client Implementation Guide

#### 1. Establishing Connection

```javascript
// JavaScript example
const ws = new WebSocket('ws://localhost:8767');

ws.onopen = function() {
    console.log('Connected to Husky teleoperation server');
};

ws.onerror = function(error) {
    console.error('WebSocket error:', error);
};
```

```python
# Python example
import asyncio
import websockets
import json

async def connect_to_husky():
    async with websockets.connect('ws://localhost:8767') as websocket:
        # Send commands here
        await send_teleop_command(websocket, 1.0, 0.0, 0.5)
```

#### 2. Message Format (TwistStamped)

The server accepts `geometry_msgs/TwistStamped` format and publishes to `/teleop/cmd_vel`:

```jsonc
{
   "header": {                    // Optional header
      "stamp": "auto",            // "auto" or ISO8601, server fills if omitted
      "frame_id": "base_link"     // Reference frame (default: "base_link")
   },
   "twist": {                     // Required twist data
      "linear": {                 // Linear velocities (m/s)
         "x": 1.0,               // Forward/backward
         "y": 0.0,               // Left/right (typically 0 for wheeled robots)
         "z": 0.0                // Up/down (typically 0 for ground robots)
      },
      "angular": {                // Angular velocities (rad/s)
         "x": 0.0,               // Roll (typically 0 for wheeled robots)
         "y": 0.0,               // Pitch (typically 0 for wheeled robots)
         "z": 0.5                // Yaw (rotation around vertical axis)
      }
   }
}
```

#### 3. Simplified Format (Legacy Support)

For backward compatibility, the server also accepts direct Twist format:

```jsonc
{
   "linear": { "x": 1.0, "y": 0.0, "z": 0.0 },
   "angular": { "x": 0.0, "y": 0.0, "z": 0.5 }
}
```

### Teleoperation Command Examples

```jsonc
// Move forward at 1 m/s
{"twist": {"linear": {"x": 1.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}}

// Move backward at 0.5 m/s
{"twist": {"linear": {"x": -0.5, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}}

// Turn right while moving forward
{"twist": {"linear": {"x": 0.5, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": -0.5}}}

// Turn left while moving forward
{"twist": {"linear": {"x": 0.5, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.5}}}

// Rotate in place (left)
{"twist": {"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 1.0}}}

// Emergency stop
{"twist": {"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}}
```

### Complete Client Examples

#### JavaScript Web Client

```javascript
class HuskyTeleopClient {
    constructor(url = 'ws://localhost:8767') {
        this.ws = new WebSocket(url);
        this.setupEventHandlers();
    }
    
    setupEventHandlers() {
        this.ws.onopen = () => console.log('Connected to Husky');
        this.ws.onerror = (error) => console.error('Error:', error);
        this.ws.onclose = () => console.log('Disconnected');
    }
    
    sendTeleopCommand(linearX, linearY, angularZ) {
        const command = {
            twist: {
                linear: { x: linearX, y: linearY, z: 0.0 },
                angular: { x: 0.0, y: 0.0, z: angularZ }
            }
        };
        this.ws.send(JSON.stringify(command));
    }
    
    // Convenience methods
    moveForward(speed = 1.0) { this.sendTeleopCommand(speed, 0, 0); }
    moveBackward(speed = 1.0) { this.sendTeleopCommand(-speed, 0, 0); }
    turnLeft(speed = 1.0) { this.sendTeleopCommand(0, 0, speed); }
    turnRight(speed = 1.0) { this.sendTeleopCommand(0, 0, -speed); }
    stop() { this.sendTeleopCommand(0, 0, 0); }
}
```

#### Python Asyncio Client

```python
import asyncio
import websockets
import json

class HuskyTeleopClient:
    def __init__(self, uri="ws://localhost:8767"):
        self.uri = uri
    
    async def send_teleop_command(self, websocket, linear_x=0.0, linear_y=0.0, angular_z=0.0):
        command = {
            "twist": {
                "linear": {"x": linear_x, "y": linear_y, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": angular_z}
            }
        }
        await websocket.send(json.dumps(command))
    
    async def teleop_session(self):
        async with websockets.connect(self.uri) as websocket:
            # Example: move forward for 2 seconds, then stop
            await self.send_teleop_command(websocket, linear_x=1.0)
            await asyncio.sleep(2)
            await self.send_teleop_command(websocket, 0, 0, 0)  # Stop
```

### Best Practices for Clients

1. **Connection Management**:
   - Implement reconnection logic for network issues
   - Handle WebSocket close events gracefully
   - Send periodic heartbeat/stop commands to maintain safety

2. **Safety Considerations**:
   - Always send stop commands when:
     - Client disconnects
     - User releases control inputs
     - Error conditions occur
   - Implement deadman switch functionality
   - Validate velocity limits before sending

3. **Performance Tips**:
   - Send commands at reasonable frequencies (10-50 Hz typical)
   - Don't flood the server with identical commands
   - Use efficient JSON serialization

4. **Error Handling**:
   - Check WebSocket connection state before sending
   - Implement timeout mechanisms
   - Log failed transmissions for debugging

### Security Considerations for Remote Access

⚠️ **Important**: The server now listens on all network interfaces (0.0.0.0) by default to support remote clients.

**For production deployments**:
- Consider using `HUSKY_WEBSOCKET_HOST=localhost` for local-only access
- Implement authentication/authorization if needed
- Use firewall rules to restrict access to trusted networks
- Consider using SSL/TLS with `wss://` protocol for encrypted connections
- Monitor for unauthorized access attempts

**Network Configuration**:
```bash
# Secure: Local access only
export HUSKY_WEBSOCKET_HOST=localhost

# Open: Remote access allowed (default)
export HUSKY_WEBSOCKET_HOST=0.0.0.0

# Specific interface: Bind to specific IP
export HUSKY_WEBSOCKET_HOST=192.168.1.100
```

## 🏗️ Architecture

```
┌─────────────────┐    WebSocket     ┌──────────────────────┐       ROS2        ┌─────────────┐
│   Web Client    │◄─────────────────►│  Husky Control      │◄─────────────────►│   Husky     │
│ (Joystick/Web)  │   (Port 8767)    │      Server          │ (/teleop/cmd_vel) │   Robot     │
└─────────────────┘                  └──────────────────────┘                   └─────────────┘
```

### Main Components

1. `HuskyWebSocketServer` – Accepts clients & validates JSON
2. `HuskyROS2Node` – Publishes `TwistStamped` to `/teleop/cmd_vel`
3. Threading – ROS2 spin in thread, asyncio loop in main thread

## 📁 Project Structure

```
husky-rcs/
├── husky_rcs.py          # Main server
├── example_client.py     # Test WebSocket client
├── start.sh              # Convenience launcher
├── install.sh            # Installer + systemd setup
├── .env.example          # Environment template
├── launch/
│   └── husky_control.launch.py
├── pyproject.toml        # Python project config (uv compatible)
├── README.md
└── LICENSE.md
```

## 🔧 Configuration

### Environment Variables

`.env` (or export):

```bash
# WebSocket server host (default: 0.0.0.0 - all interfaces)
HUSKY_WEBSOCKET_HOST=0.0.0.0
# WebSocket server port (default 8767)
HUSKY_WEBSOCKET_PORT=8767
# Optional ROS domain
ROS_DOMAIN_ID=0
```

### Change Host/Port (runtime)

```bash
# Listen only on localhost (more secure)
export HUSKY_WEBSOCKET_HOST=localhost
export HUSKY_WEBSOCKET_PORT=8080
./start.sh

# Listen on all interfaces (default - allows remote clients)
export HUSKY_WEBSOCKET_HOST=0.0.0.0
export HUSKY_WEBSOCKET_PORT=8767
./start.sh
```

### Change Host/Port (code)

Edit constructor in `husky_rcs.py` (`HuskyWebSocketServer`):
```python
# For localhost only
websocket_server = HuskyWebSocketServer(ros2_node, host="localhost", port=8767)

# For all interfaces (allows remote clients)
websocket_server = HuskyWebSocketServer(ros2_node, host="0.0.0.0", port=8767)
```

### Change ROS2 Topic

Modify publisher line in `HuskyROS2Node`:
```python
self.teleop_cmd_vel_publisher = self.create_publisher(TwistStamped, 'teleop/cmd_vel', qos_profile)
```

## 🐛 Troubleshooting

| Issue | Diagnosis | Fix |
|-------|-----------|-----|
| Port already in use | `lsof -i :8767` | Change `HUSKY_WEBSOCKET_PORT` or kill PID |
| `No module named 'rclpy'` | ROS env not sourced | `source /opt/ros/jazzy/setup.bash` |
| No messages on `/teleop/cmd_vel` | `ros2 node list` empty | Restart server / verify logs |
| Wrong topic name | `ros2 topic list` | Ensure publisher topic matches consumer |

## 🧪 Testing

Basic flow:
```bash
# Terminal 1
source /opt/ros/jazzy/setup.bash && python3 husky_rcs.py
# Terminal 2
python3 example_client.py
# Terminal 3
ros2 topic echo /teleop/cmd_vel
```

Manual WebSocket (websocat):

```bash
sudo apt install -y websocat
websocat ws://localhost:8767
{"twist": {"linear": {"x": 0.5, "y": 0, "z": 0}, "angular": {"x": 0, "y": 0, "z": 0.2}}}
```

## 📊 Monitoring

```bash
ros2 node list
ros2 node info /husky_control_server
ros2 topic hz /teleop/cmd_vel
ros2 topic bw /teleop/cmd_vel
```
Optional: `rqt_graph` for topology.

## 🚀 Deployment

### Automated (install.sh)

```bash
./install.sh
sudo systemctl start husky-control-server.service
sudo systemctl status husky-control-server.service
journalctl -u husky-control-server.service -f
```

### Manual systemd Unit
`/etc/systemd/system/husky-control-server.service`:

```ini
[Unit]
Description=Husky Robot Control Server
After=network.target

[Service]
Type=simple
User=robot
WorkingDirectory=/path/to/husky-rcs
Environment=ROS_DOMAIN_ID=0
ExecStartPre=/bin/bash -c 'source /opt/ros/jazzy/setup.bash'
ExecStart=/usr/bin/python3 husky_rcs.py
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```
Enable & start:

```bash
sudo systemctl enable husky-control-server.service
sudo systemctl start husky-control-server.service
```

## 📝 Development

### Contributing
1. Fork
2. Branch: `feat/<name>`
3. Commit (concise, conventional if possible)
4. PR with summary & test notes

### Style
- PEP 8
- Docstrings for public classes/functions
- Keep logging informative (INFO for flow, DEBUG for data)

## 📄 License

Apache 2.0 – see `LICENSE.md`.

## 👥 Authors

Groupe Carvi – <dev@carvi.ai>

## 🙏 Acknowledgments

- Norlab (Husky access)
- Python WebSocket community
- Clearpath Robotics

## 🔗 Useful Links

- ROS2 Jazzy Docs: https://docs.ros.org/en/jazzy/
- Husky Docs: https://docs.clearpathrobotics.com/docs/robots/outdoor_robots/husky/
- WebSocket RFC: https://www.rfc-editor.org/rfc/rfc6455
- geometry_msgs/TwistStamped: https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/TwistStamped.html
