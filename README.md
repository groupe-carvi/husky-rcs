# Husky Robot Control Server 🤖

A teleoperation control server for Husky robots using WebSocket and ROS2. This project enables remote control of a Husky robot via joystick commands sent through WebSocket and converted to ROS2 `geometry_msgs/Twist` messages.



## 🚀 Features

- **WebSocket Server**: Real-time joystick command reception
- **ROS2 Integration**: Native publishing to `/cmd_vel` topic
- **Asynchronous Architecture**: Simultaneous handling of multiple WebSocket clients
- **Standard Format**: `geometry_msgs/Twist` messages compatible with all ROS2 robots
- **Detailed Logging**: Complete tracking of connections and commands



## 📋 Prerequisites

### System Requirements
- Ubuntu 24.04 LTS (Noble)
- ROS2 Jazzy
- Python 3.12+

### ROS2 Dependencies
```bash
# Install ROS2 Jazzy (if not already installed)
sudo apt update
sudo apt install ros-jazzy-desktop-full

# Install required ROS2 packages
sudo apt install python3-websockets
```

### Python Dependencies
```bash

# Install dependencies with uv (recommended)
curl -LsSf https://astral.sh/uv/install.sh | sh
uv sync
```
## 🛠️ Installation

1. **Clone the repository**
   ```bash
   git clone https://github.com/groupe-carvi/husky-rcs.git
   cd husky-rcs
   ```

2. **Configure ROS2 environment**
   ```bash
   source /opt/ros/jazzy/setup.bash
   ```

3. **Install dependencies**
   ```bash
   # With uv (recommended)
   uv sync
   
   # Or system Python
   sudo apt install python3-websockets
   ```

4. **Make startup script executable**
   ```bash
   chmod +x start_husky_control.sh
   ```

## 🎮 Usage

### Starting the Server

#### Option 1: Automatic script
```bash
./start_husky_control.sh
```

#### Option 2: Manual startup
```bash
source /opt/ros/jazzy/setup.bash
python3 husky_control_server.py
```

#### Option 3: Background mode
```bash
source /opt/ros/jazzy/setup.bash
python3 husky_control_server.py &
```

The server starts on `localhost:8767` by default.

### Testing with Example Client

In another terminal:
```bash
source .venv/bin/activate  # If using uv
python3 example_client.py
```

### Monitoring ROS2 Messages
```bash
# Listen to messages on cmd_vel topic
source /opt/ros/jazzy/setup.bash
ros2 topic echo /cmd_vel

# List all active topics
ros2 topic list

# View topic information
ros2 topic info /cmd_vel
```

## 📡 WebSocket API

### Connection
- **URL**: `ws://localhost:8767`
- **Format**: JSON

### Message Format
```json
{
  "linear": {
    "x": 1.0,    // Linear velocity forward/backward (m/s)
    "y": 0.0,    // Linear velocity left/right (m/s)
    "z": 0.0     // Linear velocity up/down (m/s)
  },
  "angular": {
    "x": 0.0,    // Rotation around X axis (rad/s)
    "y": 0.0,    // Rotation around Y axis (rad/s)
    "z": 0.5     // Rotation around Z axis - yaw (rad/s)
  }
}
```

### Command Examples
```json
// Move forward
{"linear": {"x": 1.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}

// Move backward
{"linear": {"x": -1.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}

// Turn right
{"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": -1.0}}

// Turn left
{"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 1.0}}

// Stop
{"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}
```

## 🏗️ Architecture

```
┌─────────────────┐    WebSocket     ┌──────────────────────┐    ROS2     ┌─────────────┐
│   Web Client    │◄─────────────────►│  Husky Control      │◄────────────►│   Husky     │
│   (Joystick)    │   (Port 8767)    │      Server          │  (/cmd_vel)  │   Robot     │
└─────────────────┘                  └──────────────────────┘              └─────────────┘
```

### Main Components
1. **`HuskyWebSocketServer`**: WebSocket connection management
2. **`HuskyROS2Node`**: ROS2 node for message publishing
3. **Threading**: Parallel execution of WebSocket server and ROS2 node
```

## 📁 Project Structure

```
husky-rcs/
├── husky_control_server.py    # Main server
├── example_client.py          # Test client
├── start_husky_control.sh     # Startup script
├── launch/
│   └── husky_control.launch.py # ROS2 launch file
├── pyproject.toml            # Project configuration
├── README.md                 # Documentation
└── LICENSE.md               # License
```

## 🔧 Configuration

### Changing Port
Edit `husky_control_server.py`:
```python
def __init__(self, ros2_node, host="localhost", port=8767):
```

### Changing ROS2 Topic
Edit the `HuskyROS2Node` class:
```python
self.cmd_vel_publisher = self.create_publisher(
    Twist, 
    'cmd_vel',  # Change topic name here
    qos_profile
)
```

## 🐛 Troubleshooting

### Error "Address already in use"
```bash
# Check which process is using the port
lsof -i :8767

# Change port in husky_control_server.py or kill the process
kill -9 <PID>
```

### Error "No module named 'rclpy'"
```bash
# Verify ROS2 is sourced
source /opt/ros/jazzy/setup.bash

# Check ROS2 installation
ros2 --version
```

### Messages not received on topic
```bash
# Check if node is active
ros2 node list

# Check if topic exists
ros2 topic list | grep cmd_vel

# Check publication frequency
ros2 topic hz /cmd_vel
## 🧪 Testing

### Automated Test
```bash
# Terminal 1: Start server
source /opt/ros/jazzy/setup.bash
python3 husky_control_server.py &

# Terminal 2: Run test client
source .venv/bin/activate
python3 example_client.py

# Terminal 3: Verify ROS2 messages
source /opt/ros/jazzy/setup.bash
ros2 topic echo /cmd_vel
```

### Manual Test with websocat
```bash
# Install websocat
sudo apt install websocat

# Connect manually
websocat ws://localhost:8767

# Send JSON command
{"linear": {"x": 1.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}
## 📊 Monitoring

### Server Logs
Logs include:
- Client connections/disconnections
- Commands received and processed
- ROS2 messages published
- JSON parsing errors

### ROS2 Tools
```bash
# View active nodes
ros2 node list

# Node information
ros2 node info /husky_control_server

# Topic graph
rqt_graph

# Real-time monitoring
ros2 topic hz /cmd_vel
ros2 topic bw /cmd_vel
## 🚀 Deployment

### systemd Service (Optional)
Create `/etc/systemd/system/husky-control.service`:
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
ExecStart=/usr/bin/python3 husky_control_server.py
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

Enable the service:
```bash
sudo systemctl enable husky-control.service
sudo systemctl start husky-control.service
## 📝 Development

### Contributing
1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit changes (`git commit -m 'Add amazing feature'`)
4. Push to branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

### Code Style
- Follow PEP 8 conventions
- Use docstrings for all functions
- Add tests for new features## 📄 License

This project is licensed under the Apache 2.0 License. See the [LICENSE.md](LICENSE.md) file for details.

## 👥 Authors

- **Groupe Carvi** - [dev@carvi.ai](mailto:dev@carvi.ai)

## 🙏 Acknowledgments

- Norlab for access to the Husky robot
- Python WebSocket community
- Clearpath Robotics for Husky robots

---

## 🔗 Useful Links

- [ROS2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [Clearpath Husky Documentation](https://docs.clearpathrobotics.com/docs/robots/outdoor_robots/husky/)
- [WebSocket RFC 6455](https://tools.ietf.org/html/rfc6455)
- [geometry_msgs Documentation](https://docs.ros.org/en/noetic/api/geometry_msgs/html/index.html)
