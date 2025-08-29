#!/usr/bin/env python3
"""
Example client for connecting to the Go2 Dog Server

Demonstrates how to connect and control the robot remotely.
"""

import asyncio
import websockets
import json
import time
import cv2
import base64
import numpy as np
from typing import Optional
import sys
import select
import tty
import termios

# Default server IP - change this if your server is on a different machine
DEFAULT_SERVER_IP = "192.168.2.143"

def print_r(message):
    """Print message with \r to prevent terminal stair-stepping in raw mode"""
    # Replace all \n with \r\n to prevent stair-stepping
    formatted_message = message.replace('\n', '\r\n')
    print(f"{formatted_message}\r")

class NonBlockingInput:
    """Cross-platform non-blocking keyboard input"""
    
    def __init__(self):
        if sys.platform == 'win32':
            import msvcrt
            self.msvcrt = msvcrt
            self.is_windows = True
        else:
            self.is_windows = False
            self.old_settings = termios.tcgetattr(sys.stdin)
            tty.setraw(sys.stdin.fileno())
    
    def get_key(self):
        """Get a key press without blocking"""
        if self.is_windows:
            if self.msvcrt.kbhit():
                return self.msvcrt.getch().decode('utf-8')
            return None
        else:
            if select.select([sys.stdin], [], [], 0.1)[0]:
                return sys.stdin.read(1)
            return None
    
    def restore(self):
        """Restore terminal settings"""
        if not self.is_windows:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

class DogClient:
    """Client for connecting to dog server"""
    
    def __init__(self, server_url: str):
        self.server_url = server_url
        self.websocket = None
        self.connected = False
        
    async def connect(self):
        """Connect to the server"""
        try:
            self.websocket = await websockets.connect(
                self.server_url,
                ping_interval=30,  # Send ping every 30 seconds
                ping_timeout=10    # Wait 10 seconds for pong
            )
            self.connected = True
            print_r(f"✅ Connected to {self.server_url}")
            return True
        except Exception as e:
            print_r(f"❌ Connection failed: {e}")
            return False
    
    async def send_command(self, velocity_x: float = 0, velocity_y: float = 0, 
                          angular_velocity: float = 0, mode: str = "walk"):
        """Send movement command"""
        if not self.connected:
            return False
        
        command = {
            "type": "command",
            "data": {
                "velocity_x": velocity_x,
                "velocity_y": velocity_y, 
                "angular_velocity": angular_velocity,
                "mode": mode
            }
        }
        
        try:
            await self.websocket.send(json.dumps(command))
            response = await self.websocket.recv()
            result = json.loads(response)
            return result.get("success", False)
        except (websockets.exceptions.ConnectionClosed, websockets.exceptions.ConnectionClosedError):
            print_r("🔄 Connection lost, attempting reconnect...")
            self.connected = False
            if await self.connect():
                # Retry the command
                try:
                    await self.websocket.send(json.dumps(command))
                    response = await self.websocket.recv()
                    result = json.loads(response)
                    return result.get("success", False)
                except:
                    return False
            return False
        except Exception as e:
            print_r(f"❌ Command failed: {e}")
            return False
    
    async def get_state(self):
        """Get current robot state"""
        if not self.connected:
            return None
            
        await self.websocket.send(json.dumps({"type": "get_state"}))
        response = await self.websocket.recv()
        result = json.loads(response)
        
        return result.get("data")
    
    async def get_camera_frame(self) -> Optional[np.ndarray]:
        """Get camera frame"""
        if not self.connected:
            return None
            
        await self.websocket.send(json.dumps({"type": "get_camera"}))
        response = await self.websocket.recv()
        result = json.loads(response)
        
        frame_data = result.get("data")
        if frame_data:
            # Decode base64 image
            image_bytes = base64.b64decode(frame_data)
            nparr = np.frombuffer(image_bytes, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            return frame
        
        return None
    
    async def emergency_stop(self):
        """Emergency stop"""
        if not self.connected:
            return False
            
        await self.websocket.send(json.dumps({"type": "emergency_stop"}))
        print_r("🛑 Emergency stop sent")
        return True
    
    async def disconnect(self):
        """Disconnect from server"""
        if self.websocket:
            await self.websocket.close()
            self.connected = False
            print_r("🔌 Disconnected")


async def wasd_control():
    """Real-time WASD robot control"""
    # Get server IP
    server_ip = input(f"Enter server IP address (default: {DEFAULT_SERVER_IP}): ").strip()
    if not server_ip:
        server_ip = DEFAULT_SERVER_IP
    
    server_url = f"ws://{server_ip}:8080"
    client = DogClient(server_url)
    
    if not await client.connect():
        print_r("❌ Failed to connect to server")
        return
    
    print_r("✅ Connected to server")
    print_r("\n🎮 WASD Robot Control")
    print_r("==================")
    print_r("W - Forward")
    print_r("S - Backward") 
    print_r("A - Left")
    print_r("D - Right")
    print_r("SPACE - Stop")
    print_r("ESC or Ctrl+C - Quit")
    print_r("==================")
    print_r("Press keys to move...")
    
    keyboard = NonBlockingInput()
    
    try:
        while True:
            try:
                key = keyboard.get_key()
                
                if key:
                    if key == '\x1b':  # ESC key
                        print_r("\n👋 Disconnecting...")
                        break
                    elif key == '\x03':  # Ctrl+C
                        print_r("\n⚠️ Interrupted by Ctrl+C")
                        break
                    elif key.lower() == 'w':
                        await client.send_command(velocity_x=0.5)
                        print_r("⬆️ Moving forward")
                    elif key.lower() == 's':
                        await client.send_command(velocity_x=-0.5)
                        print_r("⬇️ Moving backward")
                    elif key.lower() == 'a':
                        await client.send_command(velocity_y=-0.5)
                        print_r("⬅️ Moving left")
                    elif key.lower() == 'd':
                        await client.send_command(velocity_y=0.5)
                        print_r("➡️ Moving right")
                    elif key == ' ':
                        await client.send_command()
                        print_r("⏹️ Stopped")
                
                # Small delay to prevent overwhelming the server
                await asyncio.sleep(0.05)
                
            except KeyboardInterrupt:
                print_r("\n⚠️ Interrupted by Ctrl+C")
                break
            
    except KeyboardInterrupt:
        print_r("\n⚠️ Interrupted by Ctrl+C")
    finally:
        keyboard.restore()
        await client.disconnect()

def main():
    """Main entry point"""
    print_r("🐕 Go2 Dog Client - WASD Control")
    asyncio.run(wasd_control())

if __name__ == "__main__":
    main()