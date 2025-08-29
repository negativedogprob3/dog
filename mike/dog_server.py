#!/usr/bin/env python3
"""
Unitree Go2 Remote Control Server

A WebSocket and REST API server that allows remote computers to control
the Unitree Go2 robot. Provides both movement control and sensor data streaming.

Features:
- WebSocket real-time control and data streaming
- REST API for basic commands
- Camera feed streaming
- Movement commands (walk, turn, pose)
- Sensor data (IMU, position, battery)
- Multiple client support
- Safety features (emergency stop, limits)

Usage:
    python3 dog_server.py [--host 0.0.0.0] [--port 8080]
"""

import asyncio
import websockets
import json
import threading
import time
import cv2
import numpy as np
import base64
from http.server import HTTPServer, SimpleHTTPRequestHandler
from socketserver import ThreadingMixIn
import urllib.parse
from typing import Dict, List, Optional, Any
import logging
from dataclasses import dataclass, asdict
import argparse
import socket
import subprocess
import platform

# Try to import Unitree SDK (mock if not available)
try:
    # These would be the actual SDK imports
    # from unitree_sdk2_python.core.channel import ChannelFactoryInitialize
    # from unitree_sdk2_python.idl.default import unitree_go_msg_dds__SportModeCmd_
    # from unitree_sdk2_python.idl.unitree_go.msg.dds_ import SportModeState_
    SDK_AVAILABLE = False  # Set to False for now since SDK may not be installed
except ImportError:
    SDK_AVAILABLE = False

def get_local_ip() -> str:
    """Get the local IP address that can be reached by other computers"""
    try:
        # Connect to a remote address to determine local IP
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))
            return s.getsockname()[0]
    except:
        return "127.0.0.1"

def detect_network_interfaces() -> List[str]:
    """Detect available network interfaces"""
    interfaces = []
    
    try:
        if platform.system() == "Darwin":  # macOS
            result = subprocess.run(["ifconfig"], capture_output=True, text=True)
            lines = result.stdout.split('\n')
            current_iface = None
            
            for line in lines:
                if line and not line.startswith('\t') and not line.startswith(' '):
                    current_iface = line.split(':')[0]
                elif current_iface and 'inet ' in line and 'inet 127.0.0.1' not in line:
                    ip = line.split('inet ')[1].split(' ')[0]
                    if ip.startswith(('192.168.', '10.', '172.')):
                        interfaces.append(current_iface)
                        
        elif platform.system() == "Linux":
            result = subprocess.run(["ip", "link", "show"], capture_output=True, text=True)
            for line in result.stdout.split('\n'):
                if ': ' in line and 'state UP' in line:
                    iface = line.split(': ')[1].split('@')[0]
                    if not iface.startswith('lo'):
                        interfaces.append(iface)
        
        # Common interface names to try if detection fails
        fallback_interfaces = ['wlan0', 'eth0', 'enp2s0', 'en0', 'wlp3s0']
        for iface in fallback_interfaces:
            if iface not in interfaces:
                interfaces.append(iface)
                
    except Exception as e:
        logging.warning(f"Interface detection failed: {e}")
        interfaces = ['eth0', 'wlan0', 'en0']
    
    return interfaces

def find_go2_interface() -> Optional[str]:
    """Try to find the interface connected to Go2 robot"""
    interfaces = detect_network_interfaces()
    
    # Try to ping Go2's default IP on each interface
    go2_ip = "192.168.12.1"
    
    for interface in interfaces:
        try:
            # Try a quick ping test
            if platform.system() == "Darwin":
                result = subprocess.run(
                    ["ping", "-c", "1", "-W", "1000", "-I", interface, go2_ip],
                    capture_output=True, timeout=2
                )
            else:
                result = subprocess.run(
                    ["ping", "-c", "1", "-W", "1", "-I", interface, go2_ip],
                    capture_output=True, timeout=2
                )
            
            if result.returncode == 0:
                logging.info(f"Found Go2 robot on interface {interface}")
                return interface
                
        except Exception:
            continue
    
    # If no ping success, return the first wireless interface
    wifi_interfaces = [i for i in interfaces if any(x in i.lower() for x in ['wlan', 'wifi', 'wl'])]
    if wifi_interfaces:
        logging.info(f"Using WiFi interface {wifi_interfaces[0]} (no Go2 ping response)")
        return wifi_interfaces[0]
    
    # Fallback to first interface
    if interfaces:
        logging.info(f"Using fallback interface {interfaces[0]}")
        return interfaces[0]
    
    return "eth0"

@dataclass
class RobotState:
    """Current state of the robot"""
    position: Dict[str, float]  # x, y, z
    orientation: Dict[str, float]  # roll, pitch, yaw
    velocity: Dict[str, float]  # vx, vy, vz
    battery_level: float
    temperature: float
    mode: str  # "idle", "walking", "standing", etc.
    connected: bool
    timestamp: float

@dataclass
class MovementCommand:
    """Movement command structure"""
    velocity_x: float = 0.0  # Forward/backward (-1.0 to 1.0)
    velocity_y: float = 0.0  # Left/right (-1.0 to 1.0) 
    angular_velocity: float = 0.0  # Rotation (-1.0 to 1.0)
    body_height: float = 0.0  # Body height adjustment
    mode: str = "walk"  # "walk", "stand", "sit", "lie"

class Go2RobotController:
    """Interface to Unitree Go2 robot"""
    
    def __init__(self):
        self.connected = False
        self.state = RobotState(
            position={"x": 0, "y": 0, "z": 0},
            orientation={"roll": 0, "pitch": 0, "yaw": 0},
            velocity={"vx": 0, "vy": 0, "vz": 0},
            battery_level=100.0,
            temperature=25.0,
            mode="idle",
            connected=False,
            timestamp=time.time()
        )
        self.camera = None
        self.last_command_time = 0
        self.safety_timeout = 2.0  # Stop if no commands for 2 seconds
        
    def connect(self, interface: str = "eth0") -> bool:
        """Connect to the robot"""
        connected_components = []
        
        try:
            # Try to ping the robot first
            import subprocess
            import platform
            
            go2_ip = "192.168.12.1"
            if platform.system() == "Darwin":
                ping_result = subprocess.run(
                    ["ping", "-c", "1", "-W", "1000", go2_ip], 
                    capture_output=True, timeout=3
                )
            else:
                ping_result = subprocess.run(
                    ["ping", "-c", "1", "-W", "1", go2_ip], 
                    capture_output=True, timeout=3
                )
            
            robot_reachable = ping_result.returncode == 0
            
            # Try SDK connection if available
            if SDK_AVAILABLE and robot_reachable:
                # Initialize actual SDK connection
                # ChannelFactoryInitialize(0, interface)
                # self.sport_client = SportClient()
                connected_components.append("🤖 Control API")
            
            # Try to connect to camera
            self.camera = self._setup_camera(interface)
            if self.camera:
                connected_components.append("📹 Camera")
            
            if robot_reachable:
                connected_components.append("🌐 Network")
            
            # Consider connected if we have any components
            self.connected = len(connected_components) > 0
            self.state.connected = robot_reachable
            
            if connected_components:
                self.log_green(f"✅ Connected: {' + '.join(connected_components)}")
                return True
            else:
                # Build failure message with details
                failures = []
                if not robot_reachable:
                    failures.append("robot unreachable")
                if not self.camera:
                    failures.append("camera failed")
                
                failure_details = ", ".join(failures)
                logging.error(f"❌ No robot connections ({failure_details}) - running in dogless mode")
                return False
            
        except Exception as e:
            logging.error(f"❌ Connection failed: {e}")
            self.connected = False
            self.state.connected = False
            return False
    
    def _setup_camera(self, interface: str):
        """Setup camera connection"""
        try:
            pipeline = (
                f"udpsrc address=230.1.1.1 port=1720 multicast-iface={interface} ! "
                "application/x-rtp, media=video, encoding-name=H264 ! "
                "rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! "
                "video/x-raw,width=1280,height=720,format=BGR ! appsink drop=1"
            )
            cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            return cap if cap.isOpened() else None
        except:
            return None
    
    def execute_command(self, cmd: MovementCommand):
        """Execute movement command"""
        self.last_command_time = time.time()
        
        try:
            # Log what we're sending to the robot
            if SDK_AVAILABLE and self.connected:
                scaled_vx = cmd.velocity_x * 1.5
                scaled_vy = cmd.velocity_y * 1.0
                scaled_vyaw = cmd.angular_velocity * 1.5
                
                print(f"{time.strftime('%H:%M:%S')} - 🤖 Sending to robot: vx={scaled_vx:.1f}, vy={scaled_vy:.1f}, vyaw={scaled_vyaw:.1f}")
                
                # Send actual command to robot via SDK
                # sport_req = unitree_go_msg_dds__SportModeCmd_()
                # sport_req.vx = scaled_vx
                # sport_req.vy = scaled_vy
                # sport_req.vyaw = scaled_vyaw
                # sport_req.body_height = cmd.body_height
                # self.sport_client.Move(sport_req)
            else:
                # Show what we would send if connected
                scaled_vx = cmd.velocity_x * 1.5
                scaled_vy = cmd.velocity_y * 1.0
                scaled_vyaw = cmd.angular_velocity * 1.5
                
                print(f"{time.strftime('%H:%M:%S')} - 🚷 Would send to robot: vx={scaled_vx:.1f}, vy={scaled_vy:.1f}, vyaw={scaled_vyaw:.1f}")
            
            # Update mock state
            self.state.velocity["vx"] = cmd.velocity_x
            self.state.velocity["vy"] = cmd.velocity_y 
            self.state.mode = cmd.mode
            self.state.timestamp = time.time()
            
            return True
            
        except Exception as e:
            logging.error(f"❌ Command execution failed: {e}")
            return False
    
    def get_camera_frame(self) -> Optional[np.ndarray]:
        """Get current camera frame"""
        if self.camera is None:
            return None
        
        ret, frame = self.camera.read()
        return frame if ret else None
    
    def emergency_stop(self):
        """Emergency stop the robot"""
        stop_cmd = MovementCommand(velocity_x=0, velocity_y=0, angular_velocity=0, mode="stand")
        self.execute_command(stop_cmd)
        logging.warning("🛑 Emergency stop executed")
    
    def update_state(self):
        """Update robot state from sensors"""
        if not self.connected:
            return
        
        # Safety timeout check
        if time.time() - self.last_command_time > self.safety_timeout:
            if self.state.mode != "idle":
                self.emergency_stop()
                self.state.mode = "idle"
        
        # In real implementation, read from actual sensors
        if SDK_AVAILABLE:
            # state = self.sport_client.GetState()
            # Update self.state from actual robot data
            pass
        
        self.state.timestamp = time.time()

class DogServer:
    """WebSocket and HTTP server for robot control"""
    
    def __init__(self, host: str = "0.0.0.0", port: int = 8080):
        self.host = host
        self.port = port
        self.robot = Go2RobotController()
        self.connected_clients: Dict[str, websockets.WebSocketServerProtocol] = {}
        self.running = False
        
        # Setup logging with custom formatter for colors
        class ColoredFormatter(logging.Formatter):
            COLORS = {
                'ERROR': '\033[91m',    # Red
                'WARNING': '\033[93m',  # Yellow
                'INFO': '\033[0m',      # White (default)
                'DEBUG': '\033[94m',    # Blue
                'GREEN': '\033[92m',    # Green (special)
                'RESET': '\033[0m'      # Reset
            }
            
            def format(self, record):
                # Check if message explicitly requests green color
                if hasattr(record, 'green') and record.green:
                    color = self.COLORS['GREEN']
                else:
                    color = self.COLORS.get(record.levelname, self.COLORS['RESET'])
                
                record.msg = f"{color}{record.msg}{self.COLORS['RESET']}"
                return super().format(record)
        
        handler = logging.StreamHandler()
        handler.setFormatter(ColoredFormatter('%(asctime)s - %(message)s', datefmt='%H:%M:%S'))
        
        logger = logging.getLogger()
        logger.setLevel(logging.INFO)
        logger.handlers.clear()
        logger.addHandler(handler)
        
        # Suppress websockets library logging
        logging.getLogger('websockets.server').setLevel(logging.WARNING)
        
    def log_green(self, message: str):
        """Log a message in green color"""
        logger = logging.getLogger()
        record = logger.makeRecord(
            logger.name, logging.INFO, __file__, 0, message, (), None
        )
        record.green = True
        logger.handle(record)
    
    async def register_client(self, websocket, path=None):
        """Register new WebSocket client"""
        client_id = f"{websocket.remote_address[0]}:{websocket.remote_address[1]}"
        self.connected_clients[client_id] = websocket
        
        print(f"{time.strftime('%H:%M:%S')} - 🔗 Client connected: {client_id}")
        
        try:
            # Send initial state
            await websocket.send(json.dumps({
                "type": "state",
                "data": asdict(self.robot.state)
            }))
            
            async for message in websocket:
                await self.handle_websocket_message(websocket, client_id, message)
                
        except websockets.exceptions.ConnectionClosed:
            print(f"{time.strftime('%H:%M:%S')} - ⛓️‍💥  Client disconnected: {client_id}")
        except Exception as e:
            logging.error(f"❌ Client error {client_id}: {e}")
        finally:
            self.connected_clients.pop(client_id, None)
    
    async def handle_websocket_message(self, websocket, client_id: str, message: str):
        """Handle incoming WebSocket message"""
        try:
            data = json.loads(message)
            msg_type = data.get("type")
            
            if msg_type == "command":
                # Movement command
                cmd_data = data.get("data", {})
                command = MovementCommand(**cmd_data)
                
                # Log the received command with emoji
                client_ip = client_id.split(':')[0]  # Extract just the IP part
                if command.velocity_x > 0:
                    print(f"{time.strftime('%H:%M:%S')} - ⬆️  Forward from {client_ip}")
                elif command.velocity_x < 0:
                    print(f"{time.strftime('%H:%M:%S')} - ⬇️  Backward from {client_ip}")
                elif command.velocity_y < 0:
                    print(f"{time.strftime('%H:%M:%S')} - ⬅️  Left from {client_ip}")
                elif command.velocity_y > 0:
                    print(f"{time.strftime('%H:%M:%S')} - ➡️  Right from {client_ip}")
                elif command.angular_velocity < 0:
                    print(f"{time.strftime('%H:%M:%S')} - ↪️  Turn left from {client_ip}")
                elif command.angular_velocity > 0:
                    print(f"{time.strftime('%H:%M:%S')} - ↩️  Turn right from {client_ip}")
                elif command.mode != "walk":
                    print(f"{time.strftime('%H:%M:%S')} - 🎮  {command.mode} from {client_ip}")
                else:
                    print(f"{time.strftime('%H:%M:%S')} - 🛑  Stop from {client_ip}")
                
                success = self.robot.execute_command(command)
                
                await websocket.send(json.dumps({
                    "type": "command_response",
                    "success": success,
                    "timestamp": time.time()
                }))
                
            elif msg_type == "get_state":
                # State request
                await websocket.send(json.dumps({
                    "type": "state",
                    "data": asdict(self.robot.state)
                }))
                
            elif msg_type == "get_camera":
                # Camera frame request
                frame = self.robot.get_camera_frame()
                if frame is not None:
                    # Encode frame as base64 JPEG
                    _, buffer = cv2.imencode('.jpg', frame)
                    frame_b64 = base64.b64encode(buffer).decode('utf-8')
                    
                    await websocket.send(json.dumps({
                        "type": "camera_frame",
                        "data": frame_b64,
                        "timestamp": time.time()
                    }))
                else:
                    await websocket.send(json.dumps({
                        "type": "camera_frame",
                        "data": None,
                        "error": "No camera available"
                    }))
                    
            elif msg_type == "emergency_stop":
                # Emergency stop
                self.robot.emergency_stop()
                await self.broadcast_message({
                    "type": "emergency_stop",
                    "timestamp": time.time()
                })
                
        except json.JSONDecodeError:
            await websocket.send(json.dumps({
                "type": "error",
                "message": "Invalid JSON format"
            }))
        except Exception as e:
            await websocket.send(json.dumps({
                "type": "error", 
                "message": str(e)
            }))
    
    async def broadcast_message(self, message: dict):
        """Broadcast message to all connected clients"""
        if self.connected_clients:
            await asyncio.gather(
                *[client.send(json.dumps(message)) for client in self.connected_clients.values()],
                return_exceptions=True
            )
    
    async def periodic_state_broadcast(self):
        """Periodically broadcast robot state to all clients"""
        while self.running:
            try:
                self.robot.update_state()
                if self.connected_clients:  # Only broadcast if there are clients
                    await self.broadcast_message({
                        "type": "state_update",
                        "data": asdict(self.robot.state)
                    })
                await asyncio.sleep(1.0)  # Reduce to 1Hz to avoid overwhelming clients
                
            except Exception as e:
                logging.error(f"❌ State broadcast error: {e}")
                await asyncio.sleep(1)
    
    def create_http_handler(self):
        """Create HTTP request handler for REST API"""
        server_ref = self
        
        class DogHTTPHandler(SimpleHTTPRequestHandler):
            def do_GET(self):
                """Handle GET requests"""
                if self.path == "/":
                    self.send_response(200)
                    self.send_header('Content-type', 'text/html')
                    self.end_headers()
                    self.wfile.write(self.get_web_interface().encode())
                    
                elif self.path == "/api/state":
                    self.send_json_response(asdict(server_ref.robot.state))
                    
                elif self.path == "/api/camera":
                    frame = server_ref.robot.get_camera_frame()
                    if frame is not None:
                        _, buffer = cv2.imencode('.jpg', frame)
                        self.send_response(200)
                        self.send_header('Content-type', 'image/jpeg')
                        self.end_headers()
                        self.wfile.write(buffer.tobytes())
                    else:
                        self.send_error(404, "Camera not available")
                        
                else:
                    self.send_error(404, "Not found")
            
            def do_POST(self):
                """Handle POST requests"""
                if self.path == "/api/command":
                    content_length = int(self.headers['Content-Length'])
                    post_data = self.rfile.read(content_length)
                    
                    try:
                        data = json.loads(post_data.decode('utf-8'))
                        command = MovementCommand(**data)
                        success = server_ref.robot.execute_command(command)
                        
                        self.send_json_response({
                            "success": success,
                            "timestamp": time.time()
                        })
                        
                    except Exception as e:
                        self.send_json_response({
                            "error": str(e)
                        }, status=400)
                        
                elif self.path == "/api/emergency_stop":
                    server_ref.robot.emergency_stop()
                    self.send_json_response({
                        "success": True,
                        "message": "Emergency stop executed"
                    })
                    
                else:
                    self.send_error(404, "Not found")
            
            def send_json_response(self, data, status=200):
                """Send JSON response"""
                self.send_response(status)
                self.send_header('Content-type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(json.dumps(data).encode())
            
            def get_web_interface(self):
                """Return basic web interface HTML"""
                return """
<!DOCTYPE html>
<html>
<head>
    <title>Go2 Robot Controller</title>
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; }
        .controls { display: flex; gap: 20px; }
        .panel { border: 1px solid #ccc; padding: 15px; border-radius: 5px; }
        button { padding: 10px 15px; margin: 5px; }
        .emergency { background-color: #ff4444; color: white; }
        #status { font-weight: bold; }
    </style>
</head>
<body>
    <h1>🐕 Go2 Robot Remote Controller</h1>
    
    <div id="status">Status: Connecting...</div>
    
    <div class="controls">
        <div class="panel">
            <h3>Movement</h3>
            <div>
                <button onclick="sendCommand({velocity_x: 0.5})">Forward</button><br>
                <button onclick="sendCommand({velocity_y: -0.5})">Left</button>
                <button onclick="sendCommand({velocity_x: 0, velocity_y: 0, angular_velocity: 0})">Stop</button>
                <button onclick="sendCommand({velocity_y: 0.5})">Right</button><br>
                <button onclick="sendCommand({velocity_x: -0.5})">Backward</button>
            </div>
            <div>
                <button onclick="sendCommand({angular_velocity: -0.5})">Turn Left</button>
                <button onclick="sendCommand({angular_velocity: 0.5})">Turn Right</button>
            </div>
        </div>
        
        <div class="panel">
            <h3>Poses</h3>
            <button onclick="sendCommand({mode: 'stand'})">Stand</button>
            <button onclick="sendCommand({mode: 'sit'})">Sit</button>
            <button onclick="sendCommand({mode: 'lie'})">Lie Down</button>
        </div>
        
        <div class="panel">
            <h3>Emergency</h3>
            <button class="emergency" onclick="emergencyStop()">🛑 EMERGENCY STOP</button>
        </div>
    </div>
    
    <div class="panel">
        <h3>Camera Feed</h3>
        <img id="camera" src="/api/camera" style="max-width: 640px;" />
        <br>
        <button onclick="toggleCamera()">Toggle Camera</button>
    </div>
    
    <script>
        let cameraInterval;
        
        function sendCommand(cmd) {
            fetch('/api/command', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify(cmd)
            });
        }
        
        function emergencyStop() {
            fetch('/api/emergency_stop', {method: 'POST'});
        }
        
        function toggleCamera() {
            if (cameraInterval) {
                clearInterval(cameraInterval);
                cameraInterval = null;
            } else {
                cameraInterval = setInterval(() => {
                    document.getElementById('camera').src = '/api/camera?' + Date.now();
                }, 100);
            }
        }
        
        // Update status
        setInterval(() => {
            fetch('/api/state')
                .then(r => r.json())
                .then(data => {
                    document.getElementById('status').textContent = 
                        `Status: ${data.connected ? 'Connected' : 'Disconnected'} | Mode: ${data.mode} | Battery: ${data.battery_level}%`;
                });
        }, 1000);
        
        // Start camera by default
        toggleCamera();
    </script>
</body>
</html>
                """
        
        return DogHTTPHandler
    
    def start_http_server(self):
        """Start HTTP server in separate thread"""
        handler = self.create_http_handler()
        
        class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
            pass
        
        http_server = ThreadedHTTPServer((self.host, self.port + 1), handler)
        
        # Get local IP for display instead of 0.0.0.0
        display_ip = get_local_ip() if self.host == "0.0.0.0" else self.host
        
        def run_server():
            logging.info(f"🌐 HTTP server running on http://{display_ip}:{self.port + 1}")
            http_server.serve_forever()
        
        thread = threading.Thread(target=run_server, daemon=True)
        thread.start()
        
        return http_server
    
    async def start(self, interface: Optional[str] = None):
        """Start the server"""
        print(f"{time.strftime('%H:%M:%S')} - 🚀 Starting Dog Server...")
        
        # Auto-detect interface if not provided
        if interface is None:
            interface = find_go2_interface()
            print(f"{time.strftime('%H:%M:%S')} - 🔍 Auto-detected interface: {interface}")
        
        # Get actual IP for display
        local_ip = get_local_ip()
        
        # Connect to robot
        self.robot.connect(interface)
        
        # Start HTTP server
        http_server = self.start_http_server()
        
        # Start WebSocket server and state broadcaster
        self.running = True
        
        print(f"🐕 WebSocket server running on ws://{local_ip}:{self.port}")
        print(f"🌐 Web interface available at http://{local_ip}:{self.port + 1}")
        self.log_green("✅ Ready to receive client connections")
        
        # Start periodic state updates
        state_task = asyncio.create_task(self.periodic_state_broadcast())
        
        # Start WebSocket server with keepalive settings
        async with websockets.serve(
            self.register_client, 
            self.host, 
            self.port,
            ping_interval=30,  # Send ping every 30 seconds
            ping_timeout=10    # Wait 10 seconds for pong
        ):
            try:
                await asyncio.Future()  # Run forever
            except KeyboardInterrupt:
                logging.info("🛑 Server shutdown requested")
                self.running = False
                state_task.cancel()
                http_server.shutdown()

def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(description="Unitree Go2 Remote Control Server")
    parser.add_argument("--host", default="0.0.0.0", help="Server host (default: 0.0.0.0)")
    parser.add_argument("--port", type=int, default=8080, help="WebSocket port (default: 8080)")
    parser.add_argument("--interface", help="Network interface for robot connection (auto-detected if not specified)")
    parser.add_argument("--debug", action="store_true", help="Enable debug logging")
    
    args = parser.parse_args()
    
    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)
    
    # Get local IP for display
    local_ip = get_local_ip()
    
    print("🐕 Unitree Go2 Remote Control Server")
    print("=" * 50)
    print(f"🔍 Auto-detecting network interface...")
    if args.interface:
        print(f"📡 Using specified interface: {args.interface}")
    print(f"🐕 WebSocket API: ws://{local_ip}:{args.port}")
    print(f"🌐 HTTP API: http://{local_ip}:{args.port + 1}")
    print(f"🖥️  Web UI: http://{local_ip}:{args.port + 1}")
    print("=" * 50)
    
    server = DogServer(args.host, args.port)
    
    try:
        asyncio.run(server.start(args.interface))
    except KeyboardInterrupt:
        print("\n🛑 Server stopped")

if __name__ == "__main__":
    main()