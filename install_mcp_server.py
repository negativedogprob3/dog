#!/usr/bin/env python3
"""
Installation script for Unitree Go2 MCP Server

This script helps set up the MCP server for Claude to control the Go2 robot.
"""

import json
import os
import sys
from pathlib import Path

def create_mcp_settings():
    """Create MCP server settings for Claude Desktop"""
    
    # Get current directory
    current_dir = os.getcwd()
    server_path = os.path.join(current_dir, "mcp_robot_server_minimal.py")
    
    # MCP server configuration for Claude Desktop
    mcp_config = {
        "mcpServers": {
            "unitree-go2-robot": {
                "command": "/Users/dogtrial4/.pyenv/shims/python",
                "args": [server_path, "en8"],  # Default network interface
                "env": {}
            }
        }
    }
    
    # Determine Claude Desktop config path based on OS
    home = Path.home()
    
    if sys.platform == "darwin":  # macOS
        config_dir = home / "Library" / "Application Support" / "Claude"
    elif sys.platform == "win32":  # Windows
        config_dir = home / "AppData" / "Roaming" / "Claude"
    else:  # Linux
        config_dir = home / ".config" / "claude"
    
    config_file = config_dir / "claude_desktop_config.json"
    
    # Create directory if it doesn't exist
    config_dir.mkdir(parents=True, exist_ok=True)
    
    # Read existing config or create new
    if config_file.exists():
        with open(config_file, 'r') as f:
            existing_config = json.load(f)
    else:
        existing_config = {}
    
    # Merge configurations
    if "mcpServers" not in existing_config:
        existing_config["mcpServers"] = {}
    
    existing_config["mcpServers"]["unitree-go2-robot"] = mcp_config["mcpServers"]["unitree-go2-robot"]
    
    # Write updated config
    with open(config_file, 'w') as f:
        json.dump(existing_config, f, indent=2)
    
    print(f"✅ MCP server configuration added to: {config_file}")
    return config_file

def install_dependencies():
    """Install required Python packages"""
    import subprocess
    
    packages = [
        "mcp",
        "opencv-python", 
        "numpy"
    ]
    
    print("📦 Installing MCP server dependencies...")
    
    for package in packages:
        try:
            print(f"Installing {package}...")
            result = subprocess.run([
                sys.executable, "-m", "pip", "install", package, "--break-system-packages"
            ], capture_output=True, text=True, check=True)
            print(f"✅ {package} installed successfully")
        except subprocess.CalledProcessError as e:
            print(f"❌ Failed to install {package}: {e}")
            print(f"   You may need to install manually: pip install {package}")

def check_robot_connection():
    """Check if robot connection is available"""
    import subprocess
    
    print("🔍 Checking robot connection...")
    
    try:
        # Try to ping robot
        result = subprocess.run([
            "ping", "-c", "1", "192.168.123.18"
        ], capture_output=True, timeout=5)
        
        if result.returncode == 0:
            print("✅ Robot connection available at 192.168.123.18")
            return True
        else:
            print("⚠️  Robot not responding at 192.168.123.18")
            print("   Make sure robot is connected via Ethernet")
            print("   Set PC IP to 192.168.123.51/24")
            return False
    except Exception as e:
        print(f"❌ Connection check failed: {e}")
        return False

def main():
    print("🚀 Unitree Go2 MCP Server Installation")
    print("=" * 50)
    
    # Check if we're in the right directory
    if not os.path.exists("mcp_robot_server_minimal.py"):
        print("❌ mcp_robot_server_minimal.py not found in current directory")
        print("   Please run this script from the directory containing the MCP server")
        return 1
    
    # Install dependencies
    print("\n1. Installing dependencies...")
    install_dependencies()
    
    # Create MCP configuration
    print("\n2. Setting up MCP server configuration...")
    config_file = create_mcp_settings()
    
    # Check robot connection
    print("\n3. Checking robot connection...")
    robot_connected = check_robot_connection()
    
    # Print setup instructions
    print("\n🎉 Installation Complete!")
    print("=" * 50)
    
    print("📋 Setup Instructions:")
    print("1. Ensure robot is connected via Ethernet cable")
    print("2. Set your PC IP to 192.168.123.51/24")
    print("3. Robot should be accessible at 192.168.123.18")
    
    if not robot_connected:
        print("\n⚠️  Robot Connection:")
        print("   Robot is not currently responding")
        print("   Complete network setup before using MCP server")
    
    print(f"\n🔧 Configuration File:")
    print(f"   {config_file}")
    print("   Restart Claude Desktop to load the new MCP server")
    
    print("\n🎮 Available Commands in Claude:")
    print("   - capture_image: Take a photo with robot camera")
    print("   - get_robot_pose: Get current robot position/orientation")
    print("   - robot_stand_up: Make robot stand up")
    print("   - robot_move: Move robot with specified speeds")
    print("   - robot_stop: Stop robot movement")
    print("   - get_robot_status: Get comprehensive robot status")
    
    print("\n🚀 Example Usage in Claude:")
    print('   "Can you take a picture with the robot camera?"')
    print('   "Make the robot stand up and move forward slowly"')
    print('   "What is the robot\'s current position?"')
    print('   "Stop the robot and show me its status"')
    
    print(f"\n📁 Server Files:")
    print(f"   Main server: {os.path.abspath('mcp_robot_server_minimal.py')}")
    print(f"   Config: {os.path.abspath('mcp_server_config.json')}")
    print(f"   This installer: {os.path.abspath(__file__)}")
    
    return 0

if __name__ == "__main__":
    sys.exit(main())