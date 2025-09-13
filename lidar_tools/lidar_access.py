#!/usr/bin/env python3
"""
Unitree Go2 LiDAR Access and Visualization

A comprehensive script for accessing and visualizing LiDAR data from the Unitree Go2 robot.
Provides real-time point cloud visualization, data recording, and analysis capabilities.

Features:
- Real-time LiDAR point cloud visualization
- LiDAR state monitoring (rotation speed, error states, etc.)
- Point cloud data recording to file
- 3D visualization with matplotlib
- Distance measurements and obstacle detection
- LiDAR control (ON/OFF switching)

Usage:
    python3 lidar_access.py [--interface eth0] [--record] [--visualize]
"""

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber, ChannelPublisher
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LidarState_
from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
from unitree_sdk2py.idl.default import std_msgs_msg_dds__String_

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
import sys
import argparse
import struct
from datetime import datetime
from typing import Optional, List, Tuple
import threading
import queue

class LidarDataProcessor:
    """Process and analyze LiDAR point cloud data"""
    
    def __init__(self):
        self.current_cloud = None
        self.cloud_count = 0
        self.recording = False
        self.recorded_clouds = []
        
    def decode_point_cloud(self, cloud_msg: PointCloud2_) -> np.ndarray:
        """Decode PointCloud2 message to numpy array"""
        try:
            # Extract point cloud parameters
            width = cloud_msg.width
            height = cloud_msg.height
            point_step = cloud_msg.point_step
            row_step = cloud_msg.row_step
            data = bytes(cloud_msg.data)
            
            # Typical LiDAR point format: x, y, z, intensity (16 bytes per point)
            points_data = []
            
            # Parse binary data based on fields
            for i in range(0, len(data), point_step):
                if i + point_step <= len(data):
                    # Unpack x, y, z, intensity (assuming 4 floats)
                    point_data = struct.unpack('<ffff', data[i:i+16])
                    x, y, z, intensity = point_data
                    
                    # Filter out invalid points
                    if not (np.isnan(x) or np.isnan(y) or np.isnan(z)):
                        points_data.append([x, y, z, intensity])
            
            return np.array(points_data) if points_data else np.empty((0, 4))
            
        except Exception as e:
            print(f"Error decoding point cloud: {e}")
            return np.empty((0, 4))
    
    def analyze_obstacles(self, points: np.ndarray, min_distance: float = 0.5) -> List[dict]:
        """Analyze point cloud for nearby obstacles"""
        if len(points) == 0:
            return []
        
        obstacles = []
        
        # Calculate distances from origin
        distances = np.sqrt(points[:, 0]**2 + points[:, 1]**2 + points[:, 2]**2)
        
        # Find points closer than minimum distance
        close_points = points[distances < min_distance]
        
        if len(close_points) > 0:
            # Group nearby points into clusters (simple clustering)
            for point in close_points[:10]:  # Limit to first 10 for performance
                obstacles.append({
                    'position': point[:3],
                    'distance': np.sqrt(np.sum(point[:3]**2)),
                    'intensity': point[3] if len(point) > 3 else 0
                })
        
        return obstacles
    
    def get_statistics(self, points: np.ndarray) -> dict:
        """Calculate point cloud statistics"""
        if len(points) == 0:
            return {
                'total_points': 0,
                'valid_points': 0,
                'min_distance': 0,
                'max_distance': 0,
                'mean_distance': 0
            }
        
        distances = np.sqrt(points[:, 0]**2 + points[:, 1]**2 + points[:, 2]**2)
        
        return {
            'total_points': len(points),
            'valid_points': np.sum(~np.isnan(distances)),
            'min_distance': np.min(distances),
            'max_distance': np.max(distances),
            'mean_distance': np.mean(distances),
            'std_distance': np.std(distances)
        }

class LidarController:
    """Control LiDAR operations (ON/OFF, settings)"""
    
    def __init__(self):
        self.switch_publisher = ChannelPublisher("rt/utlidar/switch", String_)
        self.switch_publisher.Init()
        self.switch_cmd = std_msgs_msg_dds__String_()
    
    def turn_on(self):
        """Turn LiDAR ON"""
        self.switch_cmd.data = "ON"
        self.switch_publisher.Write(self.switch_cmd)
        print("🟢 LiDAR turned ON")
    
    def turn_off(self):
        """Turn LiDAR OFF"""
        self.switch_cmd.data = "OFF"
        self.switch_publisher.Write(self.switch_cmd)
        print("🔴 LiDAR turned OFF")

class LidarVisualizer:
    """Real-time 3D visualization of LiDAR data"""
    
    def __init__(self):
        self.fig = None
        self.ax = None
        self.scatter = None
        self.setup_plot()
    
    def setup_plot(self):
        """Setup matplotlib 3D plot"""
        self.fig = plt.figure(figsize=(12, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')  
        self.ax.set_zlabel('Z (m)')
        self.ax.set_title('Go2 LiDAR Point Cloud')
        
        # Set reasonable limits
        self.ax.set_xlim([-10, 10])
        self.ax.set_ylim([-10, 10])
        self.ax.set_zlim([-2, 5])
        
        plt.ion()  # Interactive mode
        plt.show(block=False)
    
    def update_plot(self, points: np.ndarray):
        """Update 3D plot with new point cloud data"""
        if len(points) == 0:
            return
        
        # Clear previous plot
        self.ax.clear()
        
        # Plot points colored by intensity or Z value
        if points.shape[1] >= 4:
            # Color by intensity
            colors = points[:, 3]
            self.scatter = self.ax.scatter(points[:, 0], points[:, 1], points[:, 2], 
                                         c=colors, cmap='viridis', s=1, alpha=0.6)
        else:
            # Color by height (Z)
            colors = points[:, 2]
            self.scatter = self.ax.scatter(points[:, 0], points[:, 1], points[:, 2], 
                                         c=colors, cmap='viridis', s=1, alpha=0.6)
        
        # Reset labels and limits
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_zlabel('Z (m)')
        self.ax.set_title(f'Go2 LiDAR Point Cloud ({len(points)} points)')
        
        self.ax.set_xlim([-10, 10])
        self.ax.set_ylim([-10, 10])
        self.ax.set_zlim([-2, 5])
        
        # Add robot position indicator
        self.ax.scatter([0], [0], [0], c='red', s=100, marker='o', label='Robot')
        self.ax.legend()
        
        plt.draw()
        plt.pause(0.01)

class Go2LidarAccess:
    """Main class for Go2 LiDAR access and processing"""
    
    def __init__(self, interface: str = "eth0", enable_visualization: bool = False, enable_recording: bool = False):
        self.interface = interface
        self.enable_visualization = enable_visualization
        self.enable_recording = enable_recording
        
        # Initialize components
        self.processor = LidarDataProcessor()
        self.controller = None  # Initialize after channel factory
        self.visualizer = LidarVisualizer() if enable_visualization else None
        
        # Channel subscribers
        self.lidar_state_subscriber = None
        self.pointcloud_subscriber = None
        
        # Data storage
        self.latest_lidar_state = None
        self.latest_pointcloud = None
        
        # Threading
        self.running = False
        self.data_queue = queue.Queue()
        
    def initialize_channels(self):
        """Initialize DDS channels"""
        print(f"Initializing LiDAR channels on interface: {self.interface}")
        
        # Initialize channel factory
        ChannelFactoryInitialize(0, self.interface)
        
        # Initialize controller after channel factory
        self.controller = LidarController()
        
        # Setup LiDAR state subscriber
        self.lidar_state_subscriber = ChannelSubscriber("rt/utlidar/state", LidarState_)
        self.lidar_state_subscriber.Init(self.lidar_state_callback, 10)
        
        # Setup point cloud subscriber (common ROS2 topic names)
        try:
            self.pointcloud_subscriber = ChannelSubscriber("rt/utlidar/cloud", PointCloud2_)
            self.pointcloud_subscriber.Init(self.pointcloud_callback, 10)
        except:
            # Try alternative topic names
            try:
                self.pointcloud_subscriber = ChannelSubscriber("lidar/pointcloud", PointCloud2_)
                self.pointcloud_subscriber.Init(self.pointcloud_callback, 10)
            except:
                print("⚠️ Could not subscribe to point cloud topic")
        
        print("✅ LiDAR channels initialized")
    
    def lidar_state_callback(self, msg: LidarState_):
        """Callback for LiDAR state updates"""
        self.latest_lidar_state = msg
        
        # Print state information periodically
        if hasattr(self, 'last_state_print'):
            if time.time() - self.last_state_print > 2.0:  # Every 2 seconds
                self.print_lidar_state(msg)
                self.last_state_print = time.time()
        else:
            self.print_lidar_state(msg)
            self.last_state_print = time.time()
    
    def pointcloud_callback(self, msg: PointCloud2_):
        """Callback for point cloud updates"""
        self.latest_pointcloud = msg
        
        # Process point cloud data
        points = self.processor.decode_point_cloud(msg)
        self.processor.current_cloud = points
        self.processor.cloud_count += 1
        
        # Add to queue for processing
        self.data_queue.put(('pointcloud', points))
        
        if self.enable_recording:
            self.processor.recorded_clouds.append({
                'timestamp': time.time(),
                'points': points
            })
    
    def print_lidar_state(self, state: LidarState_):
        """Print LiDAR state information"""
        print(f"\n📡 LiDAR State Update:")
        print(f"  Firmware: {state.firmware_version}")
        print(f"  Rotation Speed: {state.sys_rotation_speed:.1f} RPM")
        print(f"  Cloud Frequency: {state.cloud_frequency:.1f} Hz")
        print(f"  Cloud Size: {state.cloud_size} points")
        print(f"  Packet Loss: {state.cloud_packet_loss_rate:.2f}%")
        print(f"  Error State: {state.error_state}")
        
        if state.error_state != 0:
            print(f"⚠️ LiDAR Error State: {state.error_state}")
    
    def process_data_queue(self):
        """Process queued data for visualization and analysis"""
        while self.running:
            try:
                data_type, data = self.data_queue.get(timeout=1.0)
                
                if data_type == 'pointcloud' and len(data) > 0:
                    # Update visualization
                    if self.visualizer:
                        self.visualizer.update_plot(data)
                    
                    # Analyze obstacles
                    obstacles = self.processor.analyze_obstacles(data)
                    if obstacles:
                        print(f"⚠️ {len(obstacles)} obstacles detected nearby!")
                        for i, obs in enumerate(obstacles[:3]):  # Show first 3
                            print(f"  Obstacle {i+1}: {obs['distance']:.2f}m away")
                    
                    # Print statistics occasionally
                    if self.processor.cloud_count % 30 == 0:  # Every 30 clouds
                        stats = self.processor.get_statistics(data)
                        print(f"📊 Cloud stats: {stats['total_points']} points, "
                              f"range: {stats['min_distance']:.1f}-{stats['max_distance']:.1f}m")
                
            except queue.Empty:
                continue
            except Exception as e:
                print(f"Error processing data: {e}")
    
    def run(self):
        """Main execution loop"""
        print("🚀 Starting Go2 LiDAR Access")
        print("Note: To connect to Go2 LiDAR:")
        print("  1. Ensure robot is powered on and connected")
        print("  2. LiDAR should be enabled on the robot")
        print("  3. Network interface should be configured properly")
        print()
        
        try:
            # Initialize channels
            self.initialize_channels()
            
            # Turn on LiDAR
            if self.controller:
                print("🔄 Turning on LiDAR...")
                self.controller.turn_on()
                time.sleep(2)
            
            # Start data processing thread
            self.running = True
            processing_thread = threading.Thread(target=self.process_data_queue)
            processing_thread.daemon = True
            processing_thread.start()
            
            print("✅ LiDAR access started successfully")
            print("Press Ctrl+C to stop...")
            
            # Main loop
            start_time = time.time()
            while True:
                time.sleep(0.1)
                
                # Print status every 10 seconds
                if time.time() - start_time > 10:
                    print(f"📈 Status: {self.processor.cloud_count} clouds received")
                    start_time = time.time()
                    
        except KeyboardInterrupt:
            print("\n🛑 Stopping LiDAR access...")
            
        except Exception as e:
            print(f"❌ Error in LiDAR access: {e}")
            
        finally:
            # Cleanup
            self.running = False
            if self.controller:
                print("🔴 Turning off LiDAR...")
                self.controller.turn_off()
            
            if self.enable_recording and self.processor.recorded_clouds:
                self.save_recorded_data()
            
            if self.visualizer:
                plt.close('all')
            
            print("✅ LiDAR access stopped")
    
    def save_recorded_data(self):
        """Save recorded point cloud data to file"""
        filename = f"lidar_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.npy"
        np.save(filename, self.processor.recorded_clouds)
        print(f"💾 Saved {len(self.processor.recorded_clouds)} point clouds to {filename}")

def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(description="Unitree Go2 LiDAR Access")
    parser.add_argument("--interface", default="eth0", help="Network interface (default: eth0)")
    parser.add_argument("--visualize", action="store_true", help="Enable 3D visualization")
    parser.add_argument("--record", action="store_true", help="Record point cloud data")
    parser.add_argument("--control", choices=['on', 'off'], help="Control LiDAR state and exit")
    
    args = parser.parse_args()
    
    # Simple control mode
    if args.control:
        print(f"🔄 Initializing LiDAR control...")
        ChannelFactoryInitialize(0, args.interface)
        controller = LidarController()
        
        if args.control == 'on':
            controller.turn_on()
        else:
            controller.turn_off()
        return
    
    # Full access mode
    print("📡 Unitree Go2 LiDAR Access")
    print("=" * 40)
    print(f"Interface: {args.interface}")
    print(f"Visualization: {'Enabled' if args.visualize else 'Disabled'}")
    print(f"Recording: {'Enabled' if args.record else 'Disabled'}")
    print("=" * 40)
    
    lidar_access = Go2LidarAccess(
        interface=args.interface,
        enable_visualization=args.visualize,
        enable_recording=args.record
    )
    
    lidar_access.run()

if __name__ == "__main__":
    main()