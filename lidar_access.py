#!/usr/bin/env python3

import time
import sys
import struct
import numpy as np
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
from unitree_sdk2py.idl.default import std_msgs_msg_dds__String_
from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_

class LidarAccess:
    def __init__(self):
        # Create publisher for lidar switch control
        self.lidar_switch_publisher = ChannelPublisher("rt/utlidar/switch", String_)
        self.lidar_switch_publisher.Init()
        
        # Create subscriber for lidar point cloud data
        self.lidar_subscriber = ChannelSubscriber("rt/utlidar/cloud", PointCloud2_)
        self.lidar_subscriber.Init(self.pointcloud_handler, 10)
        
        self.point_cloud_count = 0
        self.last_print_time = time.time()
        self.total_points = 0
        
    def pointcloud_handler(self, msg: PointCloud2_):
        """Handler for incoming point cloud messages"""
        self.point_cloud_count += 1
        current_time = time.time()
        
        # Parse point cloud data every few messages
        if current_time - self.last_print_time >= 2.0:
            points = self.parse_pointcloud(msg)
            self.total_points += len(points)
            
            print(f"\n=== Lidar Point Cloud Data (Scan #{self.point_cloud_count}) ===")
            print(f"Timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}")
            print(f"Frame ID: {msg.header.frame_id}")
            print(f"Points in this scan: {len(points):,}")
            print(f"Total points received: {self.total_points:,}")
            print(f"Data rate: ~{self.point_cloud_count / (current_time - self.last_print_time + 1):.1f} Hz")
            
            if len(points) > 0:
                self.analyze_points(points)
                
            print("-" * 60)
            self.last_print_time = current_time
            
    def parse_pointcloud(self, msg: PointCloud2_):
        """Parse binary point cloud data into readable points"""
        points = []
        
        try:
            # Each point is 32 bytes according to point_step
            point_step = msg.point_step
            data = bytes(msg.data)
            
            for i in range(msg.width):
                offset = i * point_step
                if offset + point_step <= len(data):
                    # Unpack point data: x, y, z (float32), skip padding, intensity (float32), ring (uint16), time (float32)
                    point_data = struct.unpack_from('<fff4xf H 2x f', data, offset)
                    x, y, z, intensity, ring, timestamp = point_data
                    
                    points.append({
                        'x': x, 'y': y, 'z': z,
                        'intensity': intensity,
                        'ring': ring,
                        'time': timestamp
                    })
                    
        except Exception as e:
            print(f"Error parsing point cloud: {e}")
            
        return points
        
    def analyze_points(self, points):
        """Analyze and display point cloud statistics"""
        if not points:
            return
            
        # Extract coordinates
        x_coords = [p['x'] for p in points]
        y_coords = [p['y'] for p in points]
        z_coords = [p['z'] for p in points]
        intensities = [p['intensity'] for p in points]
        rings = [p['ring'] for p in points]
        
        # Calculate ranges (distance from origin)
        ranges = [np.sqrt(x**2 + y**2 + z**2) for x, y, z in zip(x_coords, y_coords, z_coords)]
        
        print(f"Point Cloud Analysis:")
        print(f"  X range: {min(x_coords):.2f} to {max(x_coords):.2f} meters")
        print(f"  Y range: {min(y_coords):.2f} to {max(y_coords):.2f} meters") 
        print(f"  Z range: {min(z_coords):.2f} to {max(z_coords):.2f} meters")
        print(f"  Distance range: {min(ranges):.2f} to {max(ranges):.2f} meters")
        print(f"  Intensity range: {min(intensities):.1f} to {max(intensities):.1f}")
        print(f"  Rings: {min(rings)} to {max(rings)} (total: {len(set(rings))} rings)")
        
        # Show some sample points
        print(f"Sample points (first 5):")
        for i, point in enumerate(points[:5]):
            dist = np.sqrt(point['x']**2 + point['y']**2 + point['z']**2)
            print(f"  Point {i+1}: x={point['x']:.2f}, y={point['y']:.2f}, z={point['z']:.2f}, "
                  f"dist={dist:.2f}m, intensity={point['intensity']:.1f}, ring={point['ring']}")
    
    def enable_lidar(self):
        """Turn on the lidar"""
        print("Enabling lidar...")
        switch_msg = std_msgs_msg_dds__String_()
        switch_msg.data = "ON"
        self.lidar_switch_publisher.Write(switch_msg)
        time.sleep(2)  # Give lidar time to start
        
    def disable_lidar(self):
        """Turn off the lidar"""
        print("Disabling lidar...")
        switch_msg = std_msgs_msg_dds__String_()
        switch_msg.data = "OFF"
        self.lidar_switch_publisher.Write(switch_msg)

def main():
    print("Initializing Go2 lidar connection...")
    print("Note: To connect to a Go2 robot:")
    print("  1. Connect via Ethernet cable") 
    print("  2. Set PC IP to 192.168.123.51/24")
    print("  3. Robot should be at 192.168.123.18")
    print("  4. Usage: python3 lidar_access.py [network_interface]")
    print("")
    
    print("WARNING: Please ensure there are no obstacles around the robot while running this example.")
    input("Press Enter to continue...")
    
    # Initialize channel factory with network interface if provided
    if len(sys.argv) > 1:
        interface = sys.argv[1]
        print(f"Using network interface: {interface}")
        ChannelFactoryInitialize(0, interface)
    else:
        print("Using default network interface")
        ChannelFactoryInitialize(0)
    
    # Create lidar access instance
    lidar = LidarAccess()
    
    try:
        # Enable lidar
        lidar.enable_lidar()
        
        print("Lidar enabled. Monitoring point cloud data from rt/utlidar/cloud...")
        print("Press Ctrl+C to stop and disable lidar.")
        
        # Monitor lidar data
        while True:
            time.sleep(0.1)  # Small sleep to prevent busy waiting
            
    except KeyboardInterrupt:
        print("\nReceived interrupt signal. Shutting down...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Always disable lidar on exit
        lidar.disable_lidar()
        print("Lidar disabled. Connection closed.")

if __name__ == "__main__":
    main()