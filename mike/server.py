#!/usr/bin/env python3

# Set CycloneDDS path if not already set
import os
if "CYCLONEDDS_HOME" not in os.environ:
    cyclone_path = os.path.expanduser("~/cyclonedds/install")
    if os.path.exists(cyclone_path):
        os.environ["CYCLONEDDS_HOME"] = cyclone_path
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

# Try to import Unitree SDK
try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.go2.video.video_client import VideoClient
    from unitree_sdk2py.go2.sport.sport_client import SportClient
    from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
    SDK_AVAILABLE = True
    print("✅ Unitree SDK loaded successfully")
except ImportError as e:
    SDK_AVAILABLE = False
    print(f"⚠️  Unitree SDK not available: {e}")

# Import Claude integration
from claude_interface import ClaudeInterface

# Disable HTTP request logging from Anthropic client
import logging
logging.getLogger("httpx").setLevel(logging.WARNING)

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
        fallback_interfaces = ['en7', 'en0', 'eth0', 'wlan0', 'enp2s0', 'wlp3s0']
        for iface in fallback_interfaces:
            if iface not in interfaces:
                interfaces.append(iface)
                
    except Exception as e:
        logging.warning(f"Interface detection failed: {e}")
        interfaces = ['en7', 'en0', 'eth0', 'wlan0']
    
    return interfaces

def find_go2_interface() -> Optional[str]:
    """Try to find the interface connected to Go2 robot"""
    interfaces = detect_network_interfaces()
    
    # First check if robot is reachable
    go2_ips = ["192.168.123.18", "192.168.12.1"]
    robot_reachable = False
    
    for go2_ip in go2_ips:
        try:
            if platform.system() == "Darwin":
                result = subprocess.run(
                    ["ping", "-c", "1", "-W", "1000", go2_ip],
                    capture_output=True, timeout=2
                )
            else:
                result = subprocess.run(
                    ["ping", "-c", "1", "-W", "1", go2_ip],
                    capture_output=True, timeout=2
                )
            
            if result.returncode == 0:
                robot_reachable = True
                logging.info(f"Found Go2 robot at {go2_ip}")
                break
        except Exception:
            continue
    
    if robot_reachable:
        # Return first ethernet-like interface (en* on macOS)
        eth_interfaces = [i for i in interfaces if i.startswith('en') and i != 'en0']  # Skip en0 (usually wifi)
        if eth_interfaces:
            logging.info(f"Using ethernet interface: {eth_interfaces[0]}")
            return eth_interfaces[0]
    
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
        self.video_client = None
        self.sport_client = None
        self.last_command_time = 0
        self.safety_timeout = 2.0  # Stop if no commands for 2 seconds
        
    def connect(self, interface: str = "eth0") -> bool:
        """Connect to the robot"""
        connected_components = []
        
        try:
            # Try to ping the robot first
            import subprocess
            import platform
            
            # Try both common IP configurations for Go2
            go2_ips = ["192.168.12.1", "192.168.123.18"]
            robot_reachable = False
            
            for go2_ip in go2_ips:
                try:
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
                    
                    if ping_result.returncode == 0:
                        robot_reachable = True
                        logging.info(f"🌐 Robot found at {go2_ip}")
                        break
                except subprocess.TimeoutExpired:
                    continue
                except Exception:
                    continue
            
            # Try SDK connection if available
            if SDK_AVAILABLE and robot_reachable:
                try:
                    # Initialize actual SDK connection
                    ChannelFactoryInitialize(0, interface)
                    
                    # Setup sport client for movement
                    self.sport_client = SportClient()
                    self.sport_client.SetTimeout(3.0)
                    self.sport_client.Init()
                    connected_components.append("🤖 Control API")
                    
                    # Setup camera client
                    self.video_client = VideoClient()
                    self.video_client.SetTimeout(3.0)
                    self.video_client.Init()
                    connected_components.append("📹 Camera")
                    
                except Exception as e:
                    logging.warning(f"⚠️  SDK setup failed: {e}")
            
            # Fallback to GStreamer camera if SDK camera failed
            if not self.video_client:
                self.camera = self._setup_camera(interface)
                if self.camera:
                    connected_components.append("📹 Camera (GStreamer)")
            
            if robot_reachable:
                connected_components.append("🌐 Network")
            
            # Only consider fully connected if we have SDK control
            has_control = any("🤖 Control API" in comp for comp in connected_components)
            self.connected = has_control
            self.state.connected = has_control
            
            if connected_components:
                if has_control:
                    logging.info(f"✅ Connected: {' + '.join(connected_components)}")
                    return True
                else:
                    logging.error(f"❌ Robot found but SDK unavailable - install unitree_sdk2py for real control")
                    logging.warning(f"⚠️  Partial connection: {' + '.join(connected_components)} - running in dogless mode")
                    return False
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
            if self.sport_client and self.connected:
                scaled_vx = cmd.velocity_x * 1.5
                scaled_vy = cmd.velocity_y * 1.0
                scaled_vyaw = cmd.angular_velocity * 1.5
                
                print(f"{time.strftime('%H:%M:%S')} - 🤖 Sending to robot: vx={scaled_vx:.1f}, vy={scaled_vy:.1f}, vyaw={scaled_vyaw:.1f}")
                
                # Send actual command to robot via SDK
                result = self.sport_client.Move(scaled_vx, scaled_vy, scaled_vyaw)
                if result != 0:
                    print(f"⚠️  Move command failed with result: {result}")
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
        # Try SDK camera first
        if self.video_client:
            try:
                code, data = self.video_client.GetImageSample()
                if code == 0:
                    # Convert to numpy image
                    image_data = np.frombuffer(bytes(data), dtype=np.uint8)
                    frame = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
                    return frame
            except Exception as e:
                logging.warning(f"⚠️  SDK camera error: {e}")
        
        # Fallback to GStreamer camera
        if hasattr(self, 'camera') and self.camera is not None:
            ret, frame = self.camera.read()
            return frame if ret else None
            
        return None
    
    def capture_picture(self, filename: Optional[str] = None) -> Optional[str]:
        """Capture a picture from the camera and save to temp directory"""
        frame = self.get_camera_frame()
        if frame is None:
            return None
        
        if filename is None:
            # Use temp file that gets cleaned up
            import tempfile
            temp_file = tempfile.NamedTemporaryFile(suffix='.jpg', delete=False)
            filepath = temp_file.name
            temp_file.close()
        else:
            filepath = os.path.abspath(filename)
        
        success = cv2.imwrite(filepath, frame)
        
        if success:
            print(f"📸 Picture saved to temp: {os.path.basename(filepath)}")
            return filepath
        else:
            print("❌ Failed to save picture")
            return None
    
    def move_robot(self, vx: float, vy: float, vyaw: float) -> bool:
        """Move robot using real SportClient API"""
        try:
            self.last_command_time = time.time()
            
            if SDK_AVAILABLE and self.connected and self.sport_client:
                # Use real SDK like euan branch
                print(f"{time.strftime('%H:%M:%S')} - 🤖 Sending to robot: vx={vx:.1f}, vy={vy:.1f}, vyaw={vyaw:.1f}")
                result = self.sport_client.Move(vx, vy, vyaw)
                return result == 0  # SDK returns 0 for success
            else:
                # Mock mode for testing
                print(f"{time.strftime('%H:%M:%S')} - 🚷 Would send to robot: vx={vx:.1f}, vy={vy:.1f}, vyaw={vyaw:.1f}")
                return True
                
        except Exception as e:
            print(f"❌ Move command failed: {e}")
            return False
    
    def execute_pose_command(self, pose_cmd: str) -> bool:
        """Execute pose command using real SportClient API"""
        try:
            self.last_command_time = time.time()
            
            if SDK_AVAILABLE and self.connected and self.sport_client:
                # Use real SDK methods like euan branch
                if pose_cmd == "standup":
                    print(f"{time.strftime('%H:%M:%S')} - 🤖 Sending StandUp() to robot")
                    return self.sport_client.StandUp() == 0
                elif pose_cmd == "standdown":
                    print(f"{time.strftime('%H:%M:%S')} - 🤖 Sending StandDown() to robot")
                    return self.sport_client.StandDown() == 0
                elif pose_cmd == "recovery":
                    print(f"{time.strftime('%H:%M:%S')} - 🤖 Sending RecoveryStand() to robot")
                    return self.sport_client.RecoveryStand() == 0
                elif pose_cmd == "stop":
                    print(f"{time.strftime('%H:%M:%S')} - 🤖 Sending StopMove() to robot")
                    return self.sport_client.StopMove() == 0
                elif pose_cmd == "damp":
                    print(f"{time.strftime('%H:%M:%S')} - 🤖 Sending Damp() to robot")
                    return self.sport_client.Damp() == 0
                else:
                    print(f"❌ Unknown pose command: {pose_cmd}")
                    return False
            else:
                # Mock mode for testing
                print(f"{time.strftime('%H:%M:%S')} - 🚷 Would send pose command to robot: {pose_cmd}")
                return True
                
        except Exception as e:
            print(f"❌ Pose command failed: {e}")
            return False
    
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
        
        # TODO: Subscribe to robot state messages (SportModeState) for real robot data
        # For now, just update timestamp and keep mock state
        
        self.state.timestamp = time.time()

class DogServer:
    """WebSocket and HTTP server for robot control"""
    
    def __init__(self, host: str = "0.0.0.0", port: int = 8080):
        self.host = host
        self.port = port
        self.robot = Go2RobotController()
        self.claude = ClaudeInterface()
        if self.claude.connected:
            print("🤖 Claude connected and ready")
        else:
            print("❌ Claude not connected - check API key")
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
            async for message in websocket:
                await self.handle_websocket_message(websocket, client_id, message)
                
        except websockets.exceptions.ConnectionClosed:
            print(f"{time.strftime('%H:%M:%S')} - 🔗💥 Client disconnected: {client_id}")
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
                
                # Log the received command in human-readable format
                if command.velocity_x > 0:
                    print(f"{time.strftime('%H:%M:%S')} - 🎮 Received command to move forward")
                elif command.velocity_x < 0:
                    print(f"{time.strftime('%H:%M:%S')} - 🎮 Received command to move backward")
                elif command.velocity_y < 0:
                    print(f"{time.strftime('%H:%M:%S')} - 🎮 Received command to move left")
                elif command.velocity_y > 0:
                    print(f"{time.strftime('%H:%M:%S')} - 🎮 Received command to move right")
                elif command.angular_velocity < 0:
                    print(f"{time.strftime('%H:%M:%S')} - 🎮 Received command to turn left")
                elif command.angular_velocity > 0:
                    print(f"{time.strftime('%H:%M:%S')} - 🎮 Received command to turn right")
                elif command.mode != "walk":
                    print(f"{time.strftime('%H:%M:%S')} - 🎮 Received command to {command.mode}")
                else:
                    print(f"{time.strftime('%H:%M:%S')} - 🛑 Received stop command")
                
                success = self.robot.execute_command(command)
                
                await websocket.send(json.dumps({
                    "type": "command_response",
                    "success": success,
                    "timestamp": time.time()
                }))
                
            elif msg_type == "get_state":
                # State requests removed per user request
                await websocket.send(json.dumps({
                    "type": "error",
                    "message": "State requests not supported"
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
                    
            elif msg_type == "natural_language_command":
                # Natural language command processing via Claude
                command_text = data.get("command", "")
                print(f"{time.strftime('%H:%M:%S')} - 🗣️  Human: '{command_text}'")
                
                # Get current robot state for context
                robot_state = asdict(self.robot.state)
                
                # Process with Claude (robot command mode)
                try:
                    print(f"{time.strftime('%H:%M:%S')} - 🤖 Processing with Claude...")
                    result = await self.claude.process_natural_language_command(command_text, robot_state, use_robot_prompt=True)
                    print(f"{time.strftime('%H:%M:%S')} - 🤖 Claude result: {result}")
                except Exception as claude_error:
                    print(f"{time.strftime('%H:%M:%S')} - ❌ Claude processing error: {claude_error}")
                    import traceback
                    print(f"Full Claude error traceback: {traceback.format_exc()}")
                    result = {"success": False, "error": f"Claude processing failed: {claude_error}"}
                
                if result["success"] and "response" in result:
                    response_data = result["response"]
                    action = response_data.get("action", "info")
                    
                    # Execute robot command(s) based on Claude's response
                    if action == "move" and "command" in response_data:
                        try:
                            cmd_data = response_data["command"]
                            vx = cmd_data.get("vx", 0.0)
                            vy = cmd_data.get("vy", 0.0)
                            vyaw = cmd_data.get("vyaw", 0.0)
                            
                            print(f"{time.strftime('%H:%M:%S')} - 🎮 Move command: vx={vx}, vy={vy}, vyaw={vyaw}")
                            
                            # Use real SportClient API (like we had in our branch)
                            success = self.robot.move_robot(vx, vy, vyaw)
                            if not success:
                                print(f"❌ Failed to execute move command")
                                
                        except Exception as e:
                            print(f"❌ Movement command error: {e}")
                    
                    elif action == "pose" and "command" in response_data:
                        try:
                            pose_cmd = response_data["command"]
                            print(f"{time.strftime('%H:%M:%S')} - 🎮 Pose command: {pose_cmd}")
                            
                            # Use real SportClient pose methods (like we had in our branch)
                            success = self.robot.execute_pose_command(pose_cmd)
                            if not success:
                                print(f"❌ Failed to execute pose command: {pose_cmd}")
                                
                        except Exception as e:
                            print(f"❌ Pose command error: {e}")
                    
                    elif action == "sequence" and "sequence" in response_data:
                        # Execute sequence of commands with timing
                        sequence_str = response_data["sequence"]
                        print(f"{time.strftime('%H:%M:%S')} - 🎮 Robot sequence: {sequence_str}")
                        
                        asyncio.create_task(self.execute_sequence_string(sequence_str))
                    
                    elif action == "capture_picture":
                        # Get frame directly instead of saving to file
                        frame = self.robot.get_camera_frame()
                        if frame is not None:
                            # Encode frame as base64 JPEG for transmission
                            _, buffer = cv2.imencode('.jpg', frame)
                            frame_b64 = base64.b64encode(buffer).decode('utf-8')
                            
                            print(f"📸 Picture captured, analyzing with Claude vision...")
                            
                            # Enhance response with vision analysis
                            try:
                                enhanced_response = await self.claude.analyze_image_and_enhance_response(
                                    frame_b64, response_data
                                )
                                english_message = enhanced_response.get("english_message")
                                dogspeak = enhanced_response.get("dogspeak")
                                print(f"🔍 Vision analysis complete")
                            except Exception as e:
                                print(f"❌ Vision analysis failed: {e}")
                                english_message = response_data.get("english_message", "I took a picture for you!")
                                dogspeak = response_data.get("dogspeak", "📸 Woof! Got a great shot!")
                            
                            await websocket.send(json.dumps({
                                "type": "natural_language_response",
                                "success": True,
                                "english_message": english_message,
                                "dogspeak": dogspeak,
                                "image_data": frame_b64,  # Include image data
                                "timestamp": time.time()
                            }))
                            # Image response sent, skip normal response sending
                            return
                        else:
                            print("❌ Failed to capture picture - camera not available")
                    
                    # Print and send English message and dogspeak to client
                    english_message = response_data.get("english_message", "")
                    dogspeak = response_data.get("dogspeak", "")
                    
                    if english_message:
                        print(f"{time.strftime('%H:%M:%S')} - ☀️  English: '{english_message}'")
                    if dogspeak:
                        print(f"{time.strftime('%H:%M:%S')} - 🐕 Dogspeak: '{dogspeak}'")
                    
                    await websocket.send(json.dumps({
                        "type": "natural_language_response",
                        "success": True,
                        "english_message": english_message,
                        "dogspeak": dogspeak,
                        "timestamp": time.time()
                    }))
                else:
                    # Error in Claude processing
                    await websocket.send(json.dumps({
                        "type": "natural_language_response",
                        "success": False,
                        "error": result.get("error", "Unknown error"),
                        "timestamp": time.time()
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
    
    async def execute_sequence_string(self, sequence_str):
        """Execute a sequence from a string format like 'standdown wait(2) standup wait(2)'"""
        steps = sequence_str.split()
        for i, step in enumerate(steps):
            try:
                if step.startswith('wait(') and step.endswith(')'):
                    # Parse wait command
                    duration_str = step[5:-1]  # Remove 'wait(' and ')'
                    duration = float(duration_str)
                    print(f"{time.strftime('%H:%M:%S')} - 🎮 Step {i+1}: wait ({duration}s)")
                    await asyncio.sleep(duration)
                    
                elif step == 'capture_picture':
                    # Handle camera capture in sequence
                    print(f"{time.strftime('%H:%M:%S')} - 🎮 Step {i+1}: capture_picture")
                    frame = self.robot.get_camera_frame()
                    if frame is not None:
                        # Encode and broadcast image to all connected clients
                        _, buffer = cv2.imencode('.jpg', frame)
                        frame_b64 = base64.b64encode(buffer).decode('utf-8')
                        
                        print(f"📸 Picture captured in sequence, analyzing with Claude vision...")
                        
                        # Do vision analysis for sequence images too
                        try:
                            # Create a simple context for sequence images
                            sequence_context = {
                                "english_message": f"Step {i+1}: Picture captured during sequence",
                                "dogspeak": f"📸 Woof! Sequence photo #{i+1}!"
                            }
                            enhanced_response = await self.claude.analyze_image_and_enhance_response(
                                frame_b64, sequence_context
                            )
                            enhanced_message = enhanced_response.get("english_message")
                            enhanced_dogspeak = enhanced_response.get("dogspeak")
                            print(f"🔍 Sequence vision analysis complete")
                        except Exception as e:
                            print(f"❌ Sequence vision analysis failed: {e}")
                            enhanced_message = f"📸 Step {i+1}: Picture captured during sequence"
                            enhanced_dogspeak = f"📸 Woof! Sequence photo #{i+1}!"
                        
                        # Send to all clients with enhanced description
                        await self.broadcast_message({
                            "type": "sequence_image",
                            "image_data": frame_b64,
                            "message": enhanced_message,
                            "dogspeak": enhanced_dogspeak,
                            "timestamp": time.time()
                        })
                        print(f"📸 Enhanced sequence picture sent to clients")
                    else:
                        print(f"❌ Failed to capture picture in sequence step {i+1}")
                        
                elif step.startswith('move(') and step.endswith(')'):
                    # Parse move command: move(vx,vy,vyaw)
                    params_str = step[5:-1]  # Remove 'move(' and ')'
                    params = [float(x) for x in params_str.split(',')]
                    if len(params) == 3:
                        vx, vy, vyaw = params
                        print(f"{time.strftime('%H:%M:%S')} - 🎮 Step {i+1}: move({vx}, {vy}, {vyaw})")
                        success = self.robot.move_robot(vx, vy, vyaw)
                        if not success:
                            print(f"❌ Failed to execute move in sequence step {i+1}")
                    else:
                        print(f"❌ Invalid move parameters in step {i+1}: {step}")
                        
                else:
                    # Execute pose command using real SDK
                    print(f"{time.strftime('%H:%M:%S')} - 🎮 Step {i+1}: {step}")
                    success = self.robot.execute_pose_command(step)
                    if not success:
                        print(f"❌ Failed to execute sequence step {i+1}: {step}")
                
            except Exception as e:
                print(f"❌ Sequence step {i+1} error: {e}")
            
            # Send keepalive after each step to prevent client timeout
            if i % 3 == 0:  # Every 3 steps
                await self.broadcast_message({
                    "type": "sequence_keepalive",
                    "step": i + 1,
                    "total_steps": len(steps),
                    "timestamp": time.time()
                })
    
    async def broadcast_message(self, message: dict):
        """Broadcast message to all connected clients"""
        if self.connected_clients:
            await asyncio.gather(
                *[client.send(json.dumps(message)) for client in self.connected_clients.values()],
                return_exceptions=True
            )
    
    async def cleanup_temp_file(self, filepath: str, delay: int = 5):
        """Clean up temporary file after a delay"""
        try:
            await asyncio.sleep(delay)
            if os.path.exists(filepath):
                os.unlink(filepath)
                print(f"🗑️  Cleaned up temp file: {os.path.basename(filepath)}")
        except Exception as e:
            print(f"⚠️  Failed to clean up temp file: {e}")
    
    # State broadcasting removed per user request
    
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
        
        # State broadcasting removed per user request
        
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