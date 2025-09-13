#!/usr/bin/env python3
"""
Try to remotely trigger ROS2 launch via DDS commands
"""

import time
import sys
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
from unitree_sdk2py.idl.default import std_msgs_msg_dds__String_

def try_remote_ros2_launch(interface):
    """Try various DDS commands to activate LiDAR remotely"""
    
    print(f"🔧 Attempting remote LiDAR activation on interface: {interface}")
    
    # Initialize channel factory
    ChannelFactoryInitialize(0, interface)
    
    # Try various command topics and messages
    commands_to_try = [
        # Standard LiDAR commands
        ("rt/utlidar/switch", "ON"),
        ("rt/utlidar/control", "START"),
        ("rt/utlidar/command", "ACTIVATE"),
        
        # ROS2 launch attempts
        ("rt/ros2/launch", "go2_livox system.launch.py"),
        ("rt/system/launch", "lidar"),
        ("rt/go2/lidar/launch", "start"),
        
        # System commands
        ("rt/system/command", "ros2 launch go2_livox system.launch.py"),
        ("rt/exec/command", "ros2 launch go2_livox system.launch.py"),
        
        # Alternative activation
        ("rt/utlidar/mode", "ACTIVE"),
        ("rt/utlidar/state", "ENABLE"),
    ]
    
    publishers = {}
    
    for topic, command in commands_to_try:
        try:
            print(f"📤 Trying: {topic} -> '{command}'")
            
            # Create publisher if not exists
            if topic not in publishers:
                publishers[topic] = ChannelPublisher(topic, String_)
                publishers[topic].Init()
            
            # Send command
            cmd = std_msgs_msg_dds__String_()
            cmd.data = command
            
            # Send multiple times
            for i in range(3):
                publishers[topic].Write(cmd)
                time.sleep(0.1)
                
            print(f"✅ Command sent successfully")
            
        except Exception as e:
            print(f"❌ Failed: {e}")
    
    print(f"\n⏱️ Waiting 5 seconds for robot to process commands...")
    time.sleep(5)
    print(f"✅ Remote activation attempts completed")

def main():
    interface = sys.argv[1] if len(sys.argv) > 1 else "en8"
    try_remote_ros2_launch(interface)

if __name__ == "__main__":
    main()