#!/usr/bin/env python3

import time
import sys
import struct
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import threading
from collections import deque
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
from unitree_sdk2py.idl.default import std_msgs_msg_dds__String_
from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_

class LidarVisualizer:
    def __init__(self):
        # Create publisher for lidar switch control
        self.lidar_switch_publisher = ChannelPublisher("rt/utlidar/switch", String_)
        self.lidar_switch_publisher.Init()
        
        # Create subscriber for lidar point cloud data
        self.lidar_subscriber = ChannelSubscriber("rt/utlidar/cloud", PointCloud2_)
        self.lidar_subscriber.Init(self.pointcloud_handler, 10)
        
        # Data storage for visualization
        self.latest_points = None
        self.point_cloud_count = 0
        self.data_lock = threading.Lock()
        
        # Visualization settings
        self.max_range = 15.0  # Max range to display (meters)
        self.subsample_factor = 5  # Show every Nth point for performance
        self.color_by_intensity = True
        
        # Setup matplotlib
        self.setup_plot()
        
    def setup_plot(self):
        """Setup the 3D matplotlib figure"""
        plt.ion()  # Interactive mode
        self.fig = plt.figure(figsize=(12, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Set labels and title
        self.ax.set_xlabel('X (meters)')
        self.ax.set_ylabel('Y (meters)')
        self.ax.set_zlabel('Z (meters)')
        self.ax.set_title('Go2 Lidar Point Cloud Visualization')
        
        # Set equal aspect ratio and limits
        limit = self.max_range
        self.ax.set_xlim([-limit, limit])
        self.ax.set_ylim([-limit, limit])
        self.ax.set_zlim([-2, 3])
        
        # Add grid and improve appearance
        self.ax.grid(True, alpha=0.3)
        self.ax.view_init(elev=20, azim=45)  # Nice viewing angle
        
    def pointcloud_handler(self, msg: PointCloud2_):
        """Handler for incoming point cloud messages"""
        points = self.parse_pointcloud(msg)
        
        with self.data_lock:
            self.latest_points = points
            self.point_cloud_count += 1
            
    def parse_pointcloud(self, msg: PointCloud2_):
        """Parse binary point cloud data into readable points"""
        points = []
        
        try:
            point_step = msg.point_step
            data = bytes(msg.data)
            
            # Sample every Nth point for performance
            for i in range(0, msg.width, self.subsample_factor):
                offset = i * point_step
                if offset + point_step <= len(data):
                    # Unpack point data: x, y, z (float32), skip padding, intensity (float32), ring (uint16), time (float32)
                    point_data = struct.unpack_from('<fff4xf H 2x f', data, offset)
                    x, y, z, intensity, ring, timestamp = point_data
                    
                    # Filter out points that are too far or invalid
                    distance = np.sqrt(x**2 + y**2 + z**2)
                    if 0.1 < distance < self.max_range and not (np.isnan(x) or np.isnan(y) or np.isnan(z)):
                        points.append({
                            'x': x, 'y': y, 'z': z,
                            'intensity': intensity,
                            'ring': ring,
                            'distance': distance
                        })
                    
        except Exception as e:
            print(f"Error parsing point cloud: {e}")
            
        return points
        
    def update_plot(self):
        """Update the 3D plot with latest point cloud data"""
        with self.data_lock:
            if self.latest_points is None:
                return
            points = self.latest_points.copy()
            
        if not points:
            return
            
        # Clear previous plot
        self.ax.clear()
        
        # Re-setup plot properties
        self.ax.set_xlabel('X (meters)')
        self.ax.set_ylabel('Y (meters)')
        self.ax.set_zlabel('Z (meters)')
        self.ax.set_title(f'Go2 Lidar Point Cloud (Scan #{self.point_cloud_count}, {len(points)} points)')
        
        limit = self.max_range
        self.ax.set_xlim([-limit, limit])
        self.ax.set_ylim([-limit, limit])
        self.ax.set_zlim([-2, 3])
        self.ax.grid(True, alpha=0.3)
        
        # Extract coordinates
        x_coords = [p['x'] for p in points]
        y_coords = [p['y'] for p in points]
        z_coords = [p['z'] for p in points]
        
        # Color points
        if self.color_by_intensity:
            intensities = [p['intensity'] for p in points]
            # Normalize intensities for color mapping
            if max(intensities) > min(intensities):
                normalized_intensities = [(i - min(intensities)) / (max(intensities) - min(intensities)) 
                                        for i in intensities]
            else:
                normalized_intensities = [0.5] * len(intensities)
            
            scatter = self.ax.scatter(x_coords, y_coords, z_coords, 
                                    c=normalized_intensities, cmap='viridis', 
                                    s=1, alpha=0.6)
            
            # Add colorbar if not present
            if not hasattr(self, 'colorbar'):
                self.colorbar = plt.colorbar(scatter, ax=self.ax, shrink=0.5)
                self.colorbar.set_label('Intensity')
        else:
            # Color by distance
            distances = [p['distance'] for p in points]
            scatter = self.ax.scatter(x_coords, y_coords, z_coords, 
                                    c=distances, cmap='plasma', 
                                    s=1, alpha=0.6)
            
            if not hasattr(self, 'colorbar'):
                self.colorbar = plt.colorbar(scatter, ax=self.ax, shrink=0.5)
                self.colorbar.set_label('Distance (m)')
        
        # Add origin marker (robot position)
        self.ax.scatter([0], [0], [0], c='red', s=100, marker='^', alpha=1.0, label='Robot')
        
        # Add coordinate frame arrows
        arrow_length = 2.0
        self.ax.quiver(0, 0, 0, arrow_length, 0, 0, color='red', alpha=0.7, arrow_length_ratio=0.1)
        self.ax.quiver(0, 0, 0, 0, arrow_length, 0, color='green', alpha=0.7, arrow_length_ratio=0.1)
        self.ax.quiver(0, 0, 0, 0, 0, arrow_length, color='blue', alpha=0.7, arrow_length_ratio=0.1)
        
        # Add text annotations
        self.ax.text(arrow_length, 0, 0, 'X', color='red')
        self.ax.text(0, arrow_length, 0, 'Y', color='green')
        self.ax.text(0, 0, arrow_length, 'Z', color='blue')
        
        # Refresh plot
        plt.draw()
        plt.pause(0.01)
    
    def enable_lidar(self):
        """Turn on the lidar"""
        print("Enabling lidar...")
        switch_msg = std_msgs_msg_dds__String_()
        switch_msg.data = "ON"
        self.lidar_switch_publisher.Write(switch_msg)
        time.sleep(2)
        
    def disable_lidar(self):
        """Turn off the lidar"""
        print("Disabling lidar...")
        switch_msg = std_msgs_msg_dds__String_()
        switch_msg.data = "OFF"
        self.lidar_switch_publisher.Write(switch_msg)
        
    def run_visualization(self):
        """Main visualization loop"""
        print("Starting 3D visualization...")
        print("Controls:")
        print("  - Mouse: Rotate view")
        print("  - Scroll: Zoom")
        print("  - Close window or Ctrl+C to stop")
        
        try:
            while plt.get_fignums():  # While plot window is open
                self.update_plot()
                time.sleep(0.1)  # Update at ~10Hz
                
        except KeyboardInterrupt:
            print("\nVisualization stopped by user")
        except Exception as e:
            print(f"Visualization error: {e}")

def main():
    print("🚀 Go2 Lidar 3D Visualizer")
    print("=" * 50)
    print("This will show real-time 3D lidar data from your Go2 robot.")
    print("Make sure you have matplotlib installed: python3 -m pip install matplotlib")
    print("")
    
    print("Note: To connect to a Go2 robot:")
    print("  1. Connect via Ethernet cable")
    print("  2. Set PC IP to 192.168.123.51/24")
    print("  3. Robot should be at 192.168.123.18")
    print("  4. Usage: python3 lidar_visualize.py [network_interface]")
    print("")
    
    print("WARNING: Please ensure there are no obstacles around the robot!")
    input("Press Enter to continue...")
    
    # Initialize channel factory
    if len(sys.argv) > 1:
        interface = sys.argv[1]
        print(f"Using network interface: {interface}")
        ChannelFactoryInitialize(0, interface)
    else:
        print("Using default network interface")
        ChannelFactoryInitialize(0)
    
    # Create visualizer
    visualizer = LidarVisualizer()
    
    try:
        # Enable lidar
        visualizer.enable_lidar()
        
        print("Lidar enabled. Starting visualization...")
        print("Waiting for point cloud data...")
        
        # Wait for first data
        while visualizer.latest_points is None:
            time.sleep(0.1)
            
        print(f"✅ Received first point cloud! Starting 3D visualization...")
        
        # Run visualization
        visualizer.run_visualization()
        
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Cleanup
        visualizer.disable_lidar()
        plt.close('all')
        print("Lidar disabled. Visualization closed.")

if __name__ == "__main__":
    main()