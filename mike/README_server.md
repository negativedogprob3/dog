# Go2 Dog Remote Control Server

A comprehensive server system that allows remote computers to connect and control the Unitree Go2 robot via WebSocket and REST API.

## Features

### 🚀 Server Capabilities
- **WebSocket Real-time Control**: Low-latency command and data streaming
- **REST API**: HTTP endpoints for basic operations
- **Multi-client Support**: Multiple remote computers can connect simultaneously
- **Camera Streaming**: Real-time video feed from robot's front camera
- **Safety Features**: Emergency stop, command timeouts, movement limits
- **Web Interface**: Built-in browser-based control panel

### 🎮 Control Options
- **Movement Commands**: Forward/backward, left/right, rotation
- **Pose Control**: Stand, sit, lie down, body height adjustment
- **Emergency Stop**: Immediate safety shutdown
- **State Monitoring**: Battery, position, orientation, temperature

## Quick Start

### 1. Start the Server (on robot-connected machine)
```bash
cd mike
python3 -m pip install websockets
python3 dog_server.py --interface eth0
```

### 2. Access Control Interfaces

**Web Interface:**
- Open browser to `http://[robot-ip]:8081`
- Interactive control panel with movement buttons and camera feed

**WebSocket API:**
- Connect to `ws://[robot-ip]:8080`
- Send JSON commands for real-time control

**REST API:**
- POST commands to `http://[robot-ip]:8081/api/command`
- GET state from `http://[robot-ip]:8081/api/state`

### 3. Use Client Examples
```bash
python3 client_example.py  # Choose from demo options
```

## API Reference

### WebSocket Messages

#### Send Commands
```json
{
  "type": "command",
  "data": {
    "velocity_x": 0.5,      // Forward/backward (-1.0 to 1.0)
    "velocity_y": 0.0,      // Left/right (-1.0 to 1.0)
    "angular_velocity": 0.0, // Rotation (-1.0 to 1.0)
    "body_height": 0.0,     // Height adjustment
    "mode": "walk"          // "walk", "stand", "sit", "lie"
  }
}
```

#### Get State
```json
{"type": "get_state"}
```

#### Get Camera Frame
```json
{"type": "get_camera"}
```

#### Emergency Stop
```json
{"type": "emergency_stop"}
```

### REST API Endpoints

| Method | Endpoint | Description |
|--------|----------|-------------|
| GET | `/` | Web interface |
| GET | `/api/state` | Robot state JSON |
| GET | `/api/camera` | Camera JPEG image |
| POST | `/api/command` | Send movement command |
| POST | `/api/emergency_stop` | Emergency stop |

## Architecture

```
┌─────────────────┐    WebSocket/HTTP     ┌──────────────────┐
│  Remote Client  │ ◄──────────────────► │   Dog Server     │
│   (Your PC)     │                       │ (Robot Machine)  │
└─────────────────┘                       └──────────────────┘
                                                    │
                                                    │ SDK/DDS
                                                    ▼
                                          ┌──────────────────┐
                                          │  Unitree Go2     │
                                          │     Robot        │
                                          └──────────────────┘
```

### Server Components
- **DogServer**: Main WebSocket/HTTP server
- **Go2RobotController**: Robot interface and safety logic
- **HTTP Handler**: REST API and web interface
- **State Management**: Real-time robot state updates

## Safety Features

### Command Timeout
- Commands automatically expire after 2 seconds
- Robot stops if no commands received within timeout
- Prevents runaway behavior from network issues

### Emergency Stop
- Immediate movement cessation
- Accessible via any client interface
- Broadcasts stop to all connected clients

### Movement Limits  
- Velocity scaling to safe robot limits
- Body height constraints
- Mode validation

## Configuration

### Command Line Options
```bash
python3 dog_server.py --help

Options:
  --host HOST         Server host (default: 0.0.0.0)
  --port PORT         WebSocket port (default: 8080)
  --interface IFACE   Robot network interface (default: eth0)
  --debug            Enable debug logging
```

### Network Setup

**Ethernet Connection:**
- Connect robot to network via ethernet
- Use interface name like `eth0`, `enp2s0`
- Robot typically available at `192.168.12.1`

**WiFi Connection:**
- Connect to robot's WiFi network `GO2_XXXXXX`
- Use interface name like `wlan0`

## Client Development

### Python WebSocket Client
```python
import asyncio
import websockets
import json

async def control_robot():
    async with websockets.connect("ws://192.168.12.1:8080") as ws:
        # Send movement command
        command = {
            "type": "command", 
            "data": {"velocity_x": 0.5, "mode": "walk"}
        }
        await ws.send(json.dumps(command))
        
        # Get response
        response = await ws.recv()
        print(json.loads(response))

asyncio.run(control_robot())
```

### JavaScript Web Client
```javascript
const ws = new WebSocket('ws://192.168.12.1:8080');

ws.onopen = () => {
    // Send command
    ws.send(JSON.stringify({
        type: 'command',
        data: {velocity_x: 0.5, mode: 'walk'}
    }));
};

ws.onmessage = (event) => {
    const data = JSON.parse(event.data);
    console.log('Robot state:', data);
};
```

### cURL REST Examples
```bash
# Get robot state  
curl http://192.168.12.1:8081/api/state

# Send movement command
curl -X POST http://192.168.12.1:8081/api/command \
     -H "Content-Type: application/json" \
     -d '{"velocity_x": 0.5, "mode": "walk"}'

# Emergency stop
curl -X POST http://192.168.12.1:8081/api/emergency_stop

# Get camera image
curl http://192.168.12.1:8081/api/camera -o camera.jpg
```

## Troubleshooting

### Connection Issues
1. **Server won't start**: Check network interface name with `ip link show`
2. **Robot won't respond**: Verify SDK installation and robot connection
3. **Camera not working**: Ensure GStreamer pipeline is working
4. **High latency**: Use ethernet instead of WiFi connection

### Common Errors
- **"Failed to connect to robot"**: Robot may be powered off or network issue
- **"No camera available"**: Camera pipeline failed, check interface name
- **"Command timeout"**: Robot safety timeout triggered, send commands regularly

### Debug Mode
```bash
python3 dog_server.py --debug
```
Enables detailed logging for troubleshooting connection and command issues.

## Integration with Phase 2 Challenge

This server directly enables **Phase 2** requirements:

✅ **Programmatic camera connection**: Camera streaming via WebSocket/REST
✅ **Robot controller**: Movement API with safety features  
✅ **Manual piloting**: Web interface and client examples
✅ **Sensor data streaming**: Real-time state updates

Use this server as the foundation for autonomous ball retrieval in Phase 3!