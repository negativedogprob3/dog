#!/usr/bin/env python3
"""
Simple LiDAR switch controller for Go2
"""

import time
import sys
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
from unitree_sdk2py.idl.default import std_msgs_msg_dds__String_

def control_lidar(interface, command):
    """Turn LiDAR ON or OFF"""
    
    print(f"🔧 Controlling Go2 LiDAR on interface: {interface}")
    print(f"📤 Sending command: {command}")
    
    # Initialize channel factory
    ChannelFactoryInitialize(0, interface)
    
    # Create publisher for LiDAR switch
    publisher = ChannelPublisher("rt/utlidar/switch", String_)
    publisher.Init()
    
    # Create and send command
    cmd = std_msgs_msg_dds__String_()
    cmd.data = command
    
    # Send command multiple times to ensure it's received
    for i in range(5):
        publisher.Write(cmd)
        print(f"📤 Command sent ({i+1}/5)")
        time.sleep(0.2)
    
    print(f"✅ LiDAR {command} command sent successfully")
    print("⏱️ Waiting a moment for LiDAR to respond...")
    time.sleep(3)

def main():
    if len(sys.argv) < 3:
        print("Usage: python3 lidar_switch.py <interface> <ON|OFF>")
        print("Example: python3 lidar_switch.py en8 ON")
        return
        
    interface = sys.argv[1]
    command = sys.argv[2].upper()
    
    if command not in ["ON", "OFF"]:
        print("❌ Command must be ON or OFF")
        return
    
    control_lidar(interface, command)

if __name__ == "__main__":
    main()