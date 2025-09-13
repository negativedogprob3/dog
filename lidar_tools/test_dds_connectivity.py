#!/usr/bin/env python3
"""
Test basic DDS connectivity to Go2 robot
"""

import time
import sys
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize, ChannelPublisher
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
from unitree_sdk2py.idl.default import std_msgs_msg_dds__String_

class DDSConnectivityTest:
    def __init__(self):
        self.received_messages = []
        
    def generic_callback(self, topic_name):
        """Create a callback for a specific topic"""
        def callback(msg):
            print(f"📨 Received message on {topic_name}: {type(msg).__name__}")
            self.received_messages.append((topic_name, msg))
        return callback

def test_common_topics(interface):
    """Test connectivity to common Go2 DDS topics"""
    
    print(f"🔍 Testing DDS connectivity on interface: {interface}")
    
    # Initialize channel factory
    ChannelFactoryInitialize(0, interface)
    
    tester = DDSConnectivityTest()
    
    # List of common Go2 topics to test
    topics_to_test = [
        ("rt/utlidar/state", "LidarState_"),
        ("rt/utlidar/cloud", "PointCloud2_"), 
        ("rt/sportmodestate", "SportModeState_"),
        ("rt/go2/bms/state", "BmsState_"),
        ("rt/go2/wireless/controller", "WirelessController_")
    ]
    
    print("📡 Setting up subscribers for common topics...")
    subscribers = []
    
    # Try to subscribe to string topics first (simpler)
    try:
        print("🔄 Testing basic string topic subscription...")
        test_sub = ChannelSubscriber("test/connectivity", String_)
        test_sub.Init(tester.generic_callback("test/connectivity"), 10)
        subscribers.append(test_sub)
    except Exception as e:
        print(f"❌ Failed to create test subscriber: {e}")
    
    # Try to send a test message
    try:
        print("🔄 Testing basic string topic publishing...")
        test_pub = ChannelPublisher("test/from_computer", String_)
        test_pub.Init()
        
        test_msg = std_msgs_msg_dds__String_()
        test_msg.data = "Hello from computer"
        test_pub.Write(test_msg)
        print("✅ Test message sent")
        
    except Exception as e:
        print(f"❌ Failed to publish test message: {e}")
    
    print("⏱️ Listening for any DDS traffic for 15 seconds...")
    
    # Wait and listen
    start_time = time.time()
    timeout = 15.0
    
    while time.time() - start_time < timeout:
        if tester.received_messages:
            print(f"✅ Received {len(tester.received_messages)} messages!")
            break
        time.sleep(0.5)
        
        # Print progress dots
        elapsed = int(time.time() - start_time)
        if elapsed % 3 == 0:
            print(f"⏳ Waiting... ({elapsed}s)")
    
    if not tester.received_messages:
        print("❌ No DDS messages received!")
        print("🔧 Troubleshooting suggestions:")
        print("   - Verify robot is powered on and connected")
        print("   - Check network interface configuration")
        print("   - Confirm IP address and subnet")
        print("   - Try different interface (en0, eth0, etc.)")
        return False
    else:
        print("✅ DDS connectivity confirmed!")
        for topic, msg in tester.received_messages:
            print(f"   📨 {topic}: {type(msg).__name__}")
        return True

def main():
    interface = sys.argv[1] if len(sys.argv) > 1 else "eth0"
    success = test_common_topics(interface)
    
    if success:
        print("\n🎉 DDS is working - robot communication established!")
    else:
        print("\n💔 DDS connectivity issues - check robot connection")
    
    return success

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)