#!/usr/bin/env python3
"""
Try different DDS domain IDs - maybe robot uses non-default domain
"""

import time
import sys
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LidarState_

def test_domain(domain_id, interface):
    """Test a specific DDS domain"""
    
    print(f"🔍 Testing DDS Domain {domain_id} on {interface}")
    
    try:
        # Initialize with different domain
        ChannelFactoryInitialize(domain_id, interface)
        
        # Quick test for any data
        received_data = False
        
        def callback(msg):
            nonlocal received_data
            received_data = True
            print(f"🎯 FOUND DATA on Domain {domain_id}!")
            print(f"  LiDAR RPM: {msg.sys_rotation_speed}")
            print(f"  Cloud Freq: {msg.cloud_frequency} Hz")
        
        subscriber = ChannelSubscriber("rt/utlidar/state", LidarState_)
        subscriber.Init(callback, 10)
        
        # Wait 3 seconds for this domain
        for i in range(30):
            if received_data:
                return True
            time.sleep(0.1)
            
        return False
        
    except Exception as e:
        print(f"❌ Domain {domain_id} failed: {e}")
        return False

def main():
    interface = sys.argv[1] if len(sys.argv) > 1 else "en8"
    
    print("🔍 Trying different DDS domains (robot might use non-zero domain)")
    
    # Try domains 0-10
    for domain in range(11):
        if test_domain(domain, interface):
            print(f"✅ SUCCESS! Robot is on DDS Domain {domain}")
            return domain
            
    print("❌ No active domains found")
    return None

if __name__ == "__main__":
    main()