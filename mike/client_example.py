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

class DogClient:
    """Client for connecting to dog server"""
    
    def __init__(self, server_url: str = "ws://192.168.12.1:8080"):
        self.server_url = server_url
        self.websocket = None
        self.connected = False
        
    async def connect(self):
        """Connect to the server"""
        try:
            self.websocket = await websockets.connect(self.server_url)
            self.connected = True
            print(f"✅ Connected to {self.server_url}")
            return True
        except Exception as e:
            print(f"❌ Connection failed: {e}")
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
        
        await self.websocket.send(json.dumps(command))
        response = await self.websocket.recv()
        result = json.loads(response)
        
        return result.get("success", False)
    
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
        print("🛑 Emergency stop sent")
        return True
    
    async def disconnect(self):
        """Disconnect from server"""
        if self.websocket:
            await self.websocket.close()
            self.connected = False
            print("🔌 Disconnected")

async def keyboard_control_demo():
    """Demo with keyboard control"""
    print("🐕 Dog Client - Keyboard Control Demo")
    print("=" * 40)
    print("Controls:")
    print("  w/s: Forward/Backward")
    print("  a/d: Left/Right") 
    print("  q/e: Turn Left/Right")
    print("  r: Stand up")
    print("  f: Sit down")
    print("  SPACE: Stop")
    print("  ESC: Emergency stop and exit")
    print("=" * 40)
    
    client = DogClient()
    
    if not await client.connect():
        return
    
    try:
        # Note: This is a simplified demo
        # In practice, you'd want proper keyboard input handling
        while True:
            command = input("Enter command (w/s/a/d/q/e/r/f/SPACE/ESC): ").lower().strip()
            
            if command == "esc":
                await client.emergency_stop()
                break
            elif command == "w":
                await client.send_command(velocity_x=0.5)
                print("Moving forward")
            elif command == "s":
                await client.send_command(velocity_x=-0.5)
                print("Moving backward")
            elif command == "a":
                await client.send_command(velocity_y=-0.5)
                print("Moving left")
            elif command == "d":
                await client.send_command(velocity_y=0.5)
                print("Moving right")
            elif command == "q":
                await client.send_command(angular_velocity=-0.5)
                print("Turning left")
            elif command == "e":
                await client.send_command(angular_velocity=0.5)
                print("Turning right")
            elif command == "r":
                await client.send_command(mode="stand")
                print("Standing up")
            elif command == "f":
                await client.send_command(mode="sit")
                print("Sitting down")
            elif command == " ":
                await client.send_command()
                print("Stopped")
            else:
                print("Unknown command")
            
            # Get and display state
            state = await client.get_state()
            if state:
                print(f"Robot: {state['mode']} | Battery: {state['battery_level']}%")
    
    finally:
        await client.disconnect()

async def camera_stream_demo():
    """Demo showing camera stream"""
    print("🐕 Dog Client - Camera Stream Demo")
    print("Press 'q' to quit")
    
    client = DogClient()
    
    if not await client.connect():
        return
    
    try:
        while True:
            frame = await client.get_camera_frame()
            
            if frame is not None:
                cv2.imshow("Dog Camera", frame)
                
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
            else:
                print("No camera frame available")
                await asyncio.sleep(0.1)
    
    finally:
        cv2.destroyAllWindows()
        await client.disconnect()

async def autonomous_demo():
    """Demo of autonomous control"""
    print("🐕 Dog Client - Autonomous Demo")
    print("Robot will perform a simple sequence")
    
    client = DogClient()
    
    if not await client.connect():
        return
    
    try:
        # Simple autonomous sequence
        commands = [
            ("Standing up", {"mode": "stand"}),
            ("Moving forward", {"velocity_x": 0.3}),
            ("Turning left", {"angular_velocity": -0.5}),
            ("Moving forward", {"velocity_x": 0.3}),
            ("Stopping", {"velocity_x": 0, "angular_velocity": 0}),
            ("Sitting down", {"mode": "sit"})
        ]
        
        for description, cmd in commands:
            print(f"🤖 {description}...")
            await client.send_command(**cmd)
            
            # Show state
            state = await client.get_state()
            if state:
                print(f"   Status: {state['mode']} | Battery: {state['battery_level']}%")
            
            await asyncio.sleep(3)  # Wait between commands
        
        print("✅ Sequence complete")
    
    finally:
        await client.disconnect()

def main():
    """Main demo selector"""
    print("🐕 Go2 Dog Client Examples")
    print("=" * 30)
    print("1. Keyboard Control")
    print("2. Camera Stream")
    print("3. Autonomous Demo")
    print("=" * 30)
    
    choice = input("Select demo (1-3): ").strip()
    
    if choice == "1":
        asyncio.run(keyboard_control_demo())
    elif choice == "2":
        asyncio.run(camera_stream_demo())
    elif choice == "3":
        asyncio.run(autonomous_demo())
    else:
        print("Invalid choice")

if __name__ == "__main__":
    main()