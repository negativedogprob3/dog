#!/usr/bin/env python3
"""
Quick LiDAR status checker for Go2 robot
"""

import time
import sys
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LidarState_

class LidarStatusChecker:
    def __init__(self):
        self.latest_state = None
        self.received_data = False
        
    def state_callback(self, msg: LidarState_):
        """Callback for LiDAR state updates"""
        self.latest_state = msg
        self.received_data = True
        
        print(f"📡 LiDAR Status Received:")
        print(f"  Firmware: {msg.firmware_version}")
        print(f"  Software: {msg.software_version}")
        print(f"  Rotation Speed: {msg.sys_rotation_speed:.1f} RPM")
        print(f"  Cloud Frequency: {msg.cloud_frequency:.1f} Hz")
        print(f"  Cloud Size: {msg.cloud_size} points")
        print(f"  Packet Loss: {msg.cloud_packet_loss_rate:.2f}%")
        print(f"  Error State: {msg.error_state}")
        
        if msg.error_state != 0:
            print(f"⚠️ LiDAR Error State: {msg.error_state}")
        else:
            print("✅ LiDAR appears to be working normally")
            
        if msg.cloud_frequency > 0:
            print("✅ LiDAR is actively scanning")
        else:
            print("❌ LiDAR scanning frequency is 0 - may be OFF")

def main():
    interface = sys.argv[1] if len(sys.argv) > 1 else "eth0"
    
    print(f"🔍 Checking Go2 LiDAR status on interface: {interface}")
    
    # Initialize channel factory
    ChannelFactoryInitialize(0, interface)
    
    # Setup LiDAR state subscriber
    checker = LidarStatusChecker()
    subscriber = ChannelSubscriber("rt/utlidar/state", LidarState_)
    subscriber.Init(checker.state_callback, 10)
    
    print("📡 Listening for LiDAR state messages...")
    print("⏱️ Waiting 10 seconds for data...")
    
    # Wait for data
    start_time = time.time()
    timeout = 10.0
    
    while time.time() - start_time < timeout:
        if checker.received_data:
            break
        time.sleep(0.1)
    
    if not checker.received_data:
        print("❌ No LiDAR state data received!")
        print("   - Check if robot is connected")
        print("   - Check if LiDAR is powered on")
        print("   - Verify network interface is correct")
        return False
    else:
        print("✅ LiDAR state data received successfully")
        return True

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)