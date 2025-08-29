#!/usr/bin/env python3
"""
Installation script for Discrete Movement Unitree Go2 MCP Server

This installs the MCP server that uses discrete movements like robot_controller_simple.py
"""

import json
import os
import sys
from pathlib import Path

def create_mcp_settings():
    """Create MCP server settings for Claude Desktop"""
    
    # Get current directory
    current_dir = os.getcwd()
    server_path = os.path.join(current_dir, "mcp_robot_server_discrete.py")
    
    # MCP server configuration for Claude Desktop
    mcp_config = {
        "mcpServers": {
            "unitree-go2-robot-discrete": {
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
    
    existing_config["mcpServers"]["unitree-go2-robot-discrete"] = mcp_config["mcpServers"]["unitree-go2-robot-discrete"]
    
    # Write updated config
    with open(config_file, 'w') as f:
        json.dump(existing_config, f, indent=2)
    
    print(f"✅ Discrete movement MCP server configuration added to: {config_file}")
    return config_file

def main():
    print("🚀 Unitree Go2 Discrete Movement MCP Server Installation")
    print("=" * 60)
    print("This installs the MCP server with discrete movement commands")
    print("(like robot_controller_simple.py)")
    
    # Check if we're in the right directory
    if not os.path.exists("mcp_robot_server_discrete.py"):
        print("❌ mcp_robot_server_discrete.py not found in current directory")
        print("   Please run this script from the directory containing the MCP server")
        return 1
    
    # Create MCP configuration
    print("\nSetting up discrete movement MCP server configuration...")
    config_file = create_mcp_settings()
    
    print("\n🎉 Installation Complete!")
    print("=" * 50)
    
    print(f"\n🔧 Configuration File:")
    print(f"   {config_file}")
    print("   Restart Claude Desktop to load the new MCP server")
    
    print("\n🎮 Available Discrete Movement Commands:")
    print("   - capture_image: Take a photo with robot camera")
    print("   - get_robot_pose: Get current robot position/orientation")
    print("   - robot_stand: Make robot stand up")
    print("   - robot_down: Make robot lie down")
    print("   - move_forward: Move forward for 0.5s (like 'w')")
    print("   - move_backward: Move backward for 0.5s (like 's')")
    print("   - move_left: Move left for 0.5s (like 'a')")
    print("   - move_right: Move right for 0.5s (like 'd')")
    print("   - turn_left: Turn left for 0.5s (like 'q')")
    print("   - turn_right: Turn right for 0.5s (like 'e')")
    print("   - robot_stop: Stop all movement")
    print("   - set_speed_mode: Change speed (slow/normal/fast/turbo)")
    
    print("\n🚀 Example Usage in Claude:")
    print('   "Take a picture and stand the robot up"')
    print('   "Move the robot forward 3 times, then turn left"')
    print('   "Set speed to fast mode and move right"')
    print('   "What is the robot\'s current position after moving?"')
    
    print(f"\n📁 Server Files:")
    print(f"   Main server: {os.path.abspath('mcp_robot_server_discrete.py')}")
    print(f"   This installer: {os.path.abspath(__file__)}")
    
    print("\n⚠️  Remember:")
    print("   - Robot must be connected via Ethernet (192.168.123.18)")
    print("   - Each movement command lasts exactly 0.5 seconds")
    print("   - Use robot_stand before any movement commands")
    
    return 0

if __name__ == "__main__":
    sys.exit(main())