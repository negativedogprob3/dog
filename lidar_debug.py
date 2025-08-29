#!/usr/bin/env python3

import time
import sys
import threading
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
from unitree_sdk2py.idl.default import std_msgs_msg_dds__String_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LidarState_, LowState_, SportModeState_
from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_

class LidarDebugger:
    def __init__(self):
        # Create publisher for lidar switch control
        self.lidar_switch_publisher = ChannelPublisher("rt/utlidar/switch", String_)
        self.lidar_switch_publisher.Init()
        
        # Track received messages
        self.message_counts = {}
        self.last_messages = {}
        self.active_subscribers = []
        
        # All possible topics and message types to try
        self.test_topics = [
            # Lidar-specific topics
            ("rt/utlidar/state", LidarState_),
            ("rt/utlidar/data", LidarState_),
            ("rt/utlidar/status", LidarState_),
            ("rt/utlidar/info", LidarState_),
            ("rt/utlidar", LidarState_),
            ("rt/lidar/state", LidarState_),
            ("rt/lidar/data", LidarState_),
            ("rt/lidar/status", LidarState_),
            ("rt/lidar/info", LidarState_),
            ("rt/lidar", LidarState_),
            
            # Point cloud topics
            ("rt/utlidar/pointcloud", PointCloud2_),
            ("rt/utlidar/pointcloud2", PointCloud2_),
            ("rt/utlidar/cloud", PointCloud2_),
            ("rt/utlidar/points", PointCloud2_),
            ("rt/lidar/pointcloud", PointCloud2_),
            ("rt/lidar/pointcloud2", PointCloud2_),
            ("rt/lidar/cloud", PointCloud2_),
            ("rt/lidar/points", PointCloud2_),
            ("rt/pointcloud", PointCloud2_),
            ("rt/pointcloud2", PointCloud2_),
            ("rt/cloud", PointCloud2_),
            ("rt/scan", PointCloud2_),
            
            # Other sensor topics that might contain lidar data
            ("rt/sensors/lidar", LidarState_),
            ("rt/sensors/utlidar", LidarState_),
            ("rt/sensor/lidar", LidarState_),
            ("rt/sensor/utlidar", LidarState_),
            
            # Common robot state topics (might contain lidar info)
            ("rt/lowstate", LowState_),
            ("rt/sportmodestate", SportModeState_),
            
            # Generic data topics
            ("rt/data/lidar", LidarState_),
            ("rt/data/utlidar", LidarState_),
            ("rt/state/lidar", LidarState_),
            ("rt/state/utlidar", LidarState_),
        ]
        
    def generic_handler(self, topic_name):
        """Create a generic message handler for any topic"""
        def handler(msg):
            if topic_name not in self.message_counts:
                self.message_counts[topic_name] = 0
            self.message_counts[topic_name] += 1
            self.last_messages[topic_name] = time.time()
            
            # Print first message details for each topic
            if self.message_counts[topic_name] == 1:
                print(f"\n🎉 FIRST MESSAGE RECEIVED from {topic_name}!")
                print(f"Message type: {type(msg).__name__}")
                self.print_message_details(msg, topic_name)
        
        return handler
    
    def print_message_details(self, msg, topic_name):
        """Print details about the received message"""
        try:
            if hasattr(msg, '__dict__'):
                print(f"Message attributes for {topic_name}:")
                for attr, value in msg.__dict__.items():
                    if isinstance(value, (int, float, str, bool)):
                        print(f"  {attr}: {value}")
                    elif isinstance(value, list) and len(value) < 10:
                        print(f"  {attr}: {value}")
                    elif isinstance(value, list):
                        print(f"  {attr}: [list with {len(value)} items]")
                    else:
                        print(f"  {attr}: {type(value).__name__}")
        except Exception as e:
            print(f"Error printing message details: {e}")
    
    def setup_all_subscribers(self):
        """Try to subscribe to all possible topics"""
        print("🔍 Testing all possible lidar-related topics...")
        print("=" * 60)
        
        successful_subscriptions = 0
        
        for topic, msg_type in self.test_topics:
            try:
                print(f"📡 Trying: {topic} with {msg_type.__name__}")
                
                subscriber = ChannelSubscriber(topic, msg_type)
                handler = self.generic_handler(topic)
                subscriber.Init(handler, 10)
                
                self.active_subscribers.append((topic, subscriber))
                successful_subscriptions += 1
                print(f"   ✅ SUCCESS - Subscribed to {topic}")
                
            except Exception as e:
                print(f"   ❌ FAILED - {topic}: {str(e)[:80]}...")
                
        print("=" * 60)
        print(f"📊 Successfully subscribed to {successful_subscriptions} topics")
        
        if successful_subscriptions == 0:
            print("⚠️  WARNING: No topics could be subscribed to!")
            print("   This could mean:")
            print("   - Robot is not connected")
            print("   - Network interface is wrong")
            print("   - DDS communication is not working")
        else:
            print(f"🎯 Monitoring {successful_subscriptions} topics for data...")
            
    def enable_lidar(self):
        """Turn on the lidar"""
        print("🔄 Enabling lidar...")
        switch_msg = std_msgs_msg_dds__String_()
        switch_msg.data = "ON"
        self.lidar_switch_publisher.Write(switch_msg)
        print("📤 Lidar enable command sent")
        time.sleep(2)  # Give lidar time to start
        
    def disable_lidar(self):
        """Turn off the lidar"""
        print("🔄 Disabling lidar...")
        switch_msg = std_msgs_msg_dds__String_()
        switch_msg.data = "OFF"
        self.lidar_switch_publisher.Write(switch_msg)
        print("📤 Lidar disable command sent")
    
    def print_status(self):
        """Print current status of all subscriptions"""
        if not self.message_counts:
            print("📭 No messages received on any topic yet...")
            return
            
        print(f"\n📈 MESSAGE STATISTICS:")
        print("-" * 50)
        current_time = time.time()
        
        for topic, count in self.message_counts.items():
            last_msg_time = self.last_messages.get(topic, 0)
            time_since_last = current_time - last_msg_time
            
            status = "🟢 ACTIVE" if time_since_last < 5 else "🟡 IDLE" if time_since_last < 30 else "🔴 SILENT"
            
            print(f"{status} {topic}: {count} messages (last: {time_since_last:.1f}s ago)")

def main():
    print("🚀 UNITREE GO2 LIDAR DEBUG TOOL")
    print("=" * 60)
    print("This tool will:")
    print("1. Try to subscribe to ALL possible lidar topics")
    print("2. Enable the lidar")
    print("3. Monitor for any incoming data")
    print("4. Print detailed debug information")
    print("=" * 60)
    
    print("⚠️  WARNING: Please ensure there are no obstacles around the robot!")
    input("Press Enter to continue...")
    
    # Initialize channel factory with network interface if provided
    if len(sys.argv) > 1:
        interface = sys.argv[1]
        print(f"🌐 Using network interface: {interface}")
        ChannelFactoryInitialize(0, interface)
    else:
        print("🌐 Using default network interface")
        ChannelFactoryInitialize(0)
    
    # Create debugger
    debugger = LidarDebugger()
    
    try:
        # Set up all possible subscribers
        debugger.setup_all_subscribers()
        
        # Enable lidar
        debugger.enable_lidar()
        
        print("\n⏰ Starting 30-second monitoring period...")
        print("   (Press Ctrl+C to stop early)")
        
        # Monitor for 30 seconds with status updates every 5 seconds
        for i in range(6):  # 6 * 5 = 30 seconds
            time.sleep(5)
            print(f"\n⏱️  Time elapsed: {(i+1)*5} seconds")
            debugger.print_status()
            
        print(f"\n🏁 Monitoring complete!")
        debugger.print_status()
        
        if not debugger.message_counts:
            print("\n🔍 TROUBLESHOOTING SUGGESTIONS:")
            print("1. Check robot connection (ping 192.168.123.18)")
            print("2. Verify network interface (try different interface name)")
            print("3. Ensure robot is powered on and ready")
            print("4. Check if lidar hardware is actually present on this robot")
            print("5. Try running other SDK examples first")
            
    except KeyboardInterrupt:
        print("\n⏹️  Monitoring stopped by user")
        debugger.print_status()
    except Exception as e:
        print(f"\n❌ Error during monitoring: {e}")
    finally:
        # Always disable lidar on exit
        debugger.disable_lidar()
        print("🛑 Debug session ended")

if __name__ == "__main__":
    main()