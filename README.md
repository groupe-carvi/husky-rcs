# Husky Robot Control Server 🤖

Teleoperation server bridging WebSocket joystick commands to ROS2 `geometry_msgs/TwistStamped` messages on `/cmd_vel`.

## 🚀 Features

- Async WebSocket server (multi‑client)
- Publishes directly to ROS2 `/cmd_vel` (TwistStamped)
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
ros2 topic echo /cmd_vel
```


## 📡 WebSocket API

### Connection

- URL: `ws://localhost:8767`
- Format: JSON

### Message Format (TwistStamped)

```jsonc
{
   "header": {
      "stamp": "auto", // or ISO8601 timestamp, optional (server will fill if omitted)
      "frame_id": "base_link" // optional
   },
   "twist": {
      "linear": { "x": 1.0, "y": 0.0, "z": 0.0 },   // m/s
      "angular": { "x": 0.0, "y": 0.0, "z": 0.5 }   // rad/s (yaw)
   }
}
```

### Command Examples

```jsonc
// Forward
{"twist": {"linear": {"x": 1.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}}
// Backward
{"twist": {"linear": {"x": -1.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}}
// Turn Right (yaw -)
{"twist": {"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": -1.0}}}
// Turn Left (yaw +)
{"twist": {"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 1.0}}}
// Stop
{"twist": {"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}}
```

## 🏗️ Architecture

```
┌─────────────────┐    WebSocket     ┌──────────────────────┐    ROS2     ┌─────────────┐
│   Web Client    │◄─────────────────►│  Husky Control      │◄────────────►│   Husky     │
│ (Joystick/Web)  │   (Port 8767)    │      Server          │  (/cmd_vel)  │   Robot     │
└─────────────────┘                  └──────────────────────┘              └─────────────┘
```

### Main Components

1. `HuskyWebSocketServer` – Accepts clients & validates JSON
2. `HuskyROS2Node` – Publishes `TwistStamped` to `/cmd_vel`
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
# WebSocket server port (default 8767)
HUSKY_WEBSOCKET_PORT=8767
# Optional ROS domain
ROS_DOMAIN_ID=0
```

### Change Port (runtime)

```bash
export HUSKY_WEBSOCKET_PORT=8080
./start.sh
```

### Change Port (code)

Edit constructor in `husky_rcs.py` (`HuskyWebSocketServer`).

### Change ROS2 Topic

Modify publisher line in `HuskyROS2Node`:
```python
self.cmd_vel_publisher = self.create_publisher(TwistStamped, 'cmd_vel', qos_profile)
```

## 🐛 Troubleshooting

| Issue | Diagnosis | Fix |
|-------|-----------|-----|
| Port already in use | `lsof -i :8767` | Change `HUSKY_WEBSOCKET_PORT` or kill PID |
| `No module named 'rclpy'` | ROS env not sourced | `source /opt/ros/jazzy/setup.bash` |
| No messages on `/cmd_vel` | `ros2 node list` empty | Restart server / verify logs |
| Wrong topic name | `ros2 topic list` | Ensure publisher topic matches consumer |

## 🧪 Testing

Basic flow:
```bash
# Terminal 1
source /opt/ros/jazzy/setup.bash && python3 husky_rcs.py
# Terminal 2
python3 example_client.py
# Terminal 3
ros2 topic echo /cmd_vel
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
ros2 topic hz /cmd_vel
ros2 topic bw /cmd_vel
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
