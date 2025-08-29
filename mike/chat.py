#!/usr/bin/env python3
"""
Chat client for natural language robot control via Claude integration.

Examples:
- "What's 2+2?"
- "Make the dog turn left"
- "Move forward slowly"
- "What's the robot's current battery level?"
- "Run some Python code to calculate the square root of 25"
"""

import asyncio
import websockets
import json
import sys
from typing import Optional

# Default server IP - change this if your server is on a different machine
DEFAULT_SERVER_IP = "192.168.2.147"

def print_r(message):
    """Print message with \r to prevent terminal stair-stepping"""
    # Replace all \n with \r\n to prevent stair-stepping
    formatted_message = message.replace('\n', '\r\n')
    print(f"{formatted_message}\r")

class ChatClient:
    """Chat client for natural language robot commands"""
    
    def __init__(self, server_ip: str, port: int = 8080):
        self.server_url = f"ws://{server_ip}:{port}"
        self.websocket = None
        self.connected = False
        
    async def connect(self) -> bool:
        """Connect to the robot server with retry logic"""
        max_retries = 6  # 30 seconds / 5 seconds per retry
        retry_delay = 5  # seconds
        
        for attempt in range(max_retries):
            try:
                self.websocket = await websockets.connect(
                    self.server_url,
                    ping_interval=30,
                    ping_timeout=10
                )
                self.connected = True
                return True
            except Exception as e:
                if attempt < max_retries - 1:
                    print_r(f"⚠️  Connection failed (attempt {attempt + 1}/{max_retries}): {e}")
                    print_r(f"⛓️‍💥  Retrying in {retry_delay} seconds...")
                    await asyncio.sleep(retry_delay)
                else:
                    print_r(f"❌ Connection failed after {max_retries} attempts: {e}")
                    return False
        return False
    
    async def send_command(self, command_text: str) -> Optional[dict]:
        """Send natural language command to server with retry logic"""
        if not self.connected:
            print_r("❌ Not connected to server")
            return None
        
        message = {
            "type": "natural_language_command",
            "command": command_text
        }
        
        try:
            await self.websocket.send(json.dumps(message))
            
            # Listen for response (and handle any sequence images)
            while True:
                response = await self.websocket.recv()
                data = json.loads(response)
                
                # Handle sequence images separately
                if data.get("type") == "sequence_image":
                    await self.handle_sequence_image(data)
                    continue
                    
                # Handle keepalive messages (ignore silently)
                if data.get("type") == "sequence_keepalive":
                    continue
                    
                # Return the main response
                return data
            
        except (websockets.exceptions.ConnectionClosed, websockets.exceptions.ConnectionClosedError):
            print_r("🔄 Connection lost, attempting reconnect...")
            self.connected = False
            if await self.connect():
                # Retry the command
                try:
                    await self.websocket.send(json.dumps(message))
                    response = await self.websocket.recv()
                    return json.loads(response)
                except:
                    return None
            return None
        except Exception as e:
            print_r(f"❌ Command failed: {e}")
            import traceback
            print_r(f"🔍 Full traceback: {traceback.format_exc()}")
            return None
    
    async def handle_sequence_image(self, data):
        """Handle image received during a sequence"""
        try:
            import base64
            import tempfile
            import subprocess
            import platform
            
            image_data = data.get("image_data")
            message = data.get("message", "Sequence image")
            dogspeak = data.get("dogspeak", "")
            
            if image_data:
                # Decode and save image to temp file
                image_bytes = base64.b64decode(image_data)
                temp_file = tempfile.NamedTemporaryFile(suffix='.jpg', delete=False)
                temp_file.write(image_bytes)
                temp_file.close()
                
                # Open the image
                if platform.system() == "Darwin":  # macOS
                    subprocess.run(["open", temp_file.name])
                elif platform.system() == "Windows":
                    subprocess.run(["start", temp_file.name], shell=True)
                else:  # Linux
                    subprocess.run(["xdg-open", temp_file.name])
                
                # Print both enhanced messages
                print_r(f"☀️  {message}")
                if dogspeak:
                    print_r(f"🐕 {dogspeak}")
                
        except Exception as e:
            print_r(f"❌ Failed to handle sequence image: {e}")
    
    async def close(self):
        """Close connection"""
        if self.websocket:
            await self.websocket.close()
            self.connected = False

async def main():
    print_r("🤖 Claude-Powered Robot Chat Client")
    print_r("==================================")
    
    # Get server IP
    server_ip = input(f"Enter server IP (or press Enter for {DEFAULT_SERVER_IP}): ").strip()
    if not server_ip:
        server_ip = DEFAULT_SERVER_IP
    
    client = ChatClient(server_ip)
    
    # Connect to server
    print_r(f"🔄 Connecting to {server_ip}...")
    if not await client.connect():
        print_r("❌ Failed to connect to server")
        return
    
    print_r("✅ Connected to robot server")
    print_r("")
    print_r("🎯 Examples:")
    print_r("  • What's 2+2?")
    print_r("  • Make the dog turn left")
    print_r("  • Move forward slowly")  
    print_r("  • What's the robot's battery level?")
    print_r("  • Run Python: print('Hello from robot!')")
    print_r("")
    print_r("Type 'quit' or 'exit' to stop")
    print_r("=" * 50)
    
    try:
        while True:
            # Get user input
            try:
                command = input("\n🗣️  You: ").strip()
            except (EOFError, KeyboardInterrupt):
                break
                
            if not command:
                continue
                
            if command.lower() in ['quit', 'exit', 'q']:
                break
            
            # Send command to server
            response = await client.send_command(command)
            
            if response:
                if response.get("success"):
                    # Handle image data if present
                    image_data = response.get("image_data")
                    if image_data:
                        try:
                            import base64
                            import tempfile
                            import subprocess
                            import platform
                            
                            # Decode and save image to temp file
                            image_bytes = base64.b64decode(image_data)
                            temp_file = tempfile.NamedTemporaryFile(suffix='.jpg', delete=False)
                            temp_file.write(image_bytes)
                            temp_file.close()
                            
                            # Open the image
                            if platform.system() == "Darwin":  # macOS
                                subprocess.run(["open", temp_file.name])
                            elif platform.system() == "Windows":
                                subprocess.run(["start", temp_file.name], shell=True)
                            else:  # Linux
                                subprocess.run(["xdg-open", temp_file.name])
                            
                            print_r(f"📸 Picture received and opened!")
                            
                        except Exception as e:
                            print_r(f"❌ Failed to handle image: {e}")
                    
                    # Print English message with sun emoji
                    english_message = response.get("english_message", "")
                    if english_message:
                        print_r(f"☀️  {english_message}")
                    
                    # Print dogspeak message with dog emoji
                    dogspeak = response.get("dogspeak", "")
                    if dogspeak:
                        print_r(f"🐕 {dogspeak}")
                else:
                    error = response.get("error", "Unknown error")
                    print_r(f"❌ Error: {error}")
            else:
                print_r("❌ No response from server")
                
    except KeyboardInterrupt:
        print_r("\r\n⚠️  Interrupted by Ctrl+C")
    finally:
        print_r("🔌 Disconnecting...")
        await client.close()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print_r("👋 Goodbye!")
    except Exception as e:
        print_r(f"❌ Unexpected error: {e}")