# Copilot Instructions for Husky Robot Control Server

## Project Overview
- This project provides a WebSocket-to-ROS2 bridge for teleoperating a Husky robot using joystick commands.
- The main entry point is `husky_rcs.py` (sometimes referenced as `husky_control_server.py` in docs).
- WebSocket clients send JSON joystick commands, which are published as ROS2 `geometry_msgs/Twist` messages to the `/cmd_vel` topic.
- The server is designed for real-time, multi-client operation and runs both a WebSocket server and a ROS2 node in parallel (threaded).

## Key Components
- `HuskyWebSocketServer`: Handles WebSocket connections, receives and validates joystick data, and passes it to ROS2.
- `HuskyROS2Node`: ROS2 node that publishes `Twist` messages to the configured topic.
- `example_client.py`: Minimal client for testing WebSocket API.
- `launch/husky_control.launch.py`: ROS2 launch file for deployment.

## Developer Workflows
- **Start the server:**
  - `./start.sh` (preferred)
  - Or: `python3 husky_rcs.py` (ensure ROS2 environment is sourced)
- **Test client:** `python3 example_client.py`
- **Monitor ROS2:** `ros2 topic echo /cmd_vel`
- **Change WebSocket port:** Set `HUSKY_WEBSOCKET_PORT` environment variable or edit `HuskyWebSocketServer.__init__` in `husky_rcs.py`.
- **Change ROS2 topic:** Edit `HuskyROS2Node` publisher topic in `husky_rcs.py`.
- **Install dependencies:** Use `uv sync` (preferred) or `pip install -r requirements.txt` if present.

## Patterns & Conventions
- All joystick commands must be JSON with `linear` and `angular` keys, each mapping to `{x, y, z}` floats.
- Logging is via Python's `logging` module; info and error logs are important for debugging.
- The server is designed to be robust to malformed JSON and client disconnects.
- Follows PEP 8 and uses docstrings for all functions/classes.
- ROS2 node and WebSocket server run in separate threads for concurrency.

## Integration & External Dependencies
- Requires ROS2 Jazzy and Python 3.12+.
- Uses `websockets`, `rclpy`, and `geometry_msgs`.
- Systemd deployment is supported (see README for example service file).
- Environment variables: `HUSKY_WEBSOCKET_PORT` (default: 8767) for WebSocket port configuration.

## Troubleshooting & Testing
- Common issues: port conflicts, missing ROS2 environment, missing dependencies.
- Use `ros2 topic list`, `ros2 node list`, and `ros2 topic echo /cmd_vel` for debugging.
- Automated and manual test instructions are in the README.

## Example WebSocket Command
```json
{"linear": {"x": 1.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.5}}
```

## References
- See `README.md` for full architecture, usage, and troubleshooting details.
- Key files: `husky_rcs.py`, `example_client.py`, `start.sh`, `launch/husky_control.launch.py`.
