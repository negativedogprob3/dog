#!/usr/bin/env python3
"""
Discover active DDS topics on the Go2 robot
"""

import time
import sys
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_

class TopicDiscovery:
    def __init__(self):
        self.discovered_topics = set()
        
    def generic_callback(self, topic_name):
        def callback(msg):
            if topic_name not in self.discovered_topics:
                print(f"🎯 FOUND ACTIVE TOPIC: {topic_name} ({type(msg).__name__})")
                self.discovered_topics.add(topic_name)
        return callback

def test_topic_variations():
    """Test various topic name patterns"""
    
    interface = sys.argv[1] if len(sys.argv) > 1 else "en8"
    print(f"🔍 Discovering DDS topics on interface: {interface}")
    
    # Initialize channel factory
    ChannelFactoryInitialize(0, interface)
    
    discovery = TopicDiscovery()
    
    # Try many possible LiDAR topic names
    possible_topics = [
        # LiDAR state topics
        "rt/utlidar/state",
        "utlidar/state", 
        "lidar/state",
        "/lidar_state",
        "go2/lidar/state",
        
        # LiDAR data topics
        "rt/utlidar/cloud",
        "utlidar/cloud",
        "lidar/cloud",
        "lidar/pointcloud", 
        "/scan",
        "/lidar_scan",
        "go2/lidar/cloud",
        "sensor_msgs/PointCloud2",
        
        # Generic robot topics
        "rt/go2/state",
        "go2/state",
        "robot/state",
        "rt/sportmodestate",
        "sportmode/state",
        
        # Other common topics
        "rt/go2/bms/state",
        "bms/state",
        "battery/state",
        "rt/go2/imu",
        "imu/data"
    ]
    
    print(f"🔍 Testing {len(possible_topics)} possible topic names...")
    
    subscribers = []
    
    # Try to subscribe to string topics (most compatible)
    for topic in possible_topics:
        try:
            sub = ChannelSubscriber(topic, String_)
            sub.Init(discovery.generic_callback(topic), 10)
            subscribers.append(sub)
            print(f"✅ Subscribed to: {topic}")
        except Exception as e:
            print(f"❌ Failed {topic}: {str(e)[:50]}")
            
    print(f"\n⏱️ Listening for 20 seconds to discover active topics...")
    
    start_time = time.time()
    timeout = 20.0
    
    while time.time() - start_time < timeout:
        elapsed = int(time.time() - start_time)
        if elapsed % 5 == 0 and elapsed > 0:
            print(f"⏳ Still listening... ({elapsed}s) - Found {len(discovery.discovered_topics)} topics")
        time.sleep(0.5)
    
    print(f"\n📊 DISCOVERY RESULTS:")
    if discovery.discovered_topics:
        print(f"✅ Found {len(discovery.discovered_topics)} active topics:")
        for topic in sorted(discovery.discovered_topics):
            print(f"   📡 {topic}")
    else:
        print("❌ No active topics discovered")
        print("💡 This suggests:")
        print("   - Robot may not be publishing DDS data")
        print("   - Different DDS domain/configuration needed") 
        print("   - LiDAR may need manual activation on robot")
        print("   - Network/firewall issues")

if __name__ == "__main__":
    test_topic_variations()