#!/usr/bin/env python3

import time
import sys
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from collections import deque
import threading
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.nav_msgs.msg.dds_ import Odometry_
from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import PoseStamped_

class RobotPositionVisualizer:
    def __init__(self):
        # Robot state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.data_lock = threading.Lock()
        self.has_data = False
        
        # Path tracking (keep ALL positions - no limit!)
        self.robot_path = []  # Use regular list to keep all path points
        self.last_path_point = None  # To avoid duplicate consecutive points
        
        # Visualization settings
        self.robot_size = 0.3  # meters
        self.update_rate = 10  # Hz
        
        # Setup matplotlib
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlabel('X Position (meters)')
        self.ax.set_ylabel('Y Position (meters)')
        self.ax.set_title('Go2 Robot Position & Orientation (Live)')
        
        # Initialize plot elements
        self.robot_patch = None
        self.path_line = None
        self.arrow_patch = None
        self.info_text = None
        
        # Subscribers for odometry data
        self.setup_subscribers()
        
    def setup_subscribers(self):
        """Setup subscribers for odometry topics"""
        try:
            # Try ROBOTODOM first (robot pose from lidar)
            self.pose_subscriber = ChannelSubscriber("rt/utlidar/robot_pose", PoseStamped_)
            self.pose_subscriber.Init(self.pose_handler, 10)
            print("✅ Subscribed to rt/utlidar/robot_pose (PoseStamped)")
        except Exception as e:
            print(f"⚠️  Failed to subscribe to robot_pose: {e}")
            
        try:
            # Try SLAM_ODOMETRY (full odometry with velocity)
            self.odom_subscriber = ChannelSubscriber("rt/lio_sam_ros2/mapping/odometry", Odometry_)
            self.odom_subscriber.Init(self.odom_handler, 10)
            print("✅ Subscribed to rt/lio_sam_ros2/mapping/odometry (Odometry)")
        except Exception as e:
            print(f"⚠️  Failed to subscribe to slam odometry: {e}")
            
    def quaternion_to_yaw(self, qx, qy, qz, qw):
        """Convert quaternion to yaw angle (rotation around Z axis)"""
        # Convert quaternion to yaw angle
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw
        
    def pose_handler(self, msg: PoseStamped_):
        """Handle PoseStamped messages from robot_pose topic"""
        with self.data_lock:
            self.robot_x = msg.pose.position.x
            self.robot_y = msg.pose.position.y
            self.robot_yaw = self.quaternion_to_yaw(
                msg.pose.orientation.x,
                msg.pose.orientation.y, 
                msg.pose.orientation.z,
                msg.pose.orientation.w
            )
            # Only add point if robot has moved significantly (avoid cluttering)
            current_point = (self.robot_x, self.robot_y)
            if (self.last_path_point is None or 
                abs(current_point[0] - self.last_path_point[0]) > 0.01 or
                abs(current_point[1] - self.last_path_point[1]) > 0.01):
                self.robot_path.append(current_point)
                self.last_path_point = current_point
            self.has_data = True
            
    def odom_handler(self, msg: Odometry_):
        """Handle Odometry messages from SLAM topic"""
        with self.data_lock:
            self.robot_x = msg.pose.pose.position.x
            self.robot_y = msg.pose.pose.position.y
            self.robot_yaw = self.quaternion_to_yaw(
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z, 
                msg.pose.pose.orientation.w
            )
            # Only add point if robot has moved significantly (avoid cluttering)
            current_point = (self.robot_x, self.robot_y)
            if (self.last_path_point is None or 
                abs(current_point[0] - self.last_path_point[0]) > 0.01 or
                abs(current_point[1] - self.last_path_point[1]) > 0.01):
                self.robot_path.append(current_point)
                self.last_path_point = current_point
            self.has_data = True
            
    def draw_robot(self, x, y, yaw):
        """Draw robot as a rectangle with orientation arrow"""
        # Remove old robot if exists
        if self.robot_patch:
            self.robot_patch.remove()
        if self.arrow_patch:
            self.arrow_patch.remove()
            
        # Robot body (rectangle)
        width = self.robot_size
        height = self.robot_size * 0.6
        
        # Create rectangle centered at robot position
        rect = patches.Rectangle(
            (x - width/2, y - height/2), width, height,
            angle=math.degrees(yaw), 
            facecolor='blue', edgecolor='darkblue', alpha=0.8
        )
        self.robot_patch = self.ax.add_patch(rect)
        
        # Direction arrow
        arrow_length = self.robot_size * 0.8
        dx = arrow_length * math.cos(yaw)
        dy = arrow_length * math.sin(yaw)
        
        self.arrow_patch = self.ax.arrow(
            x, y, dx, dy,
            head_width=0.1, head_length=0.08,
            fc='red', ec='red', alpha=0.9
        )
        
    def draw_path(self):
        """Draw robot path as a persistent line"""
        if len(self.robot_path) < 2:
            return
            
        # DON'T remove old path - keep it persistent!
        # Only update if we don't have a path line yet, or need to redraw completely
        if self.path_line is None:
            # Extract x, y coordinates
            path_x = [pos[0] for pos in self.robot_path]
            path_y = [pos[1] for pos in self.robot_path]
            
            # Draw complete path 
            self.path_line = self.ax.plot(path_x, path_y, 'g-', alpha=0.7, linewidth=2, label='Robot Path')[0]
        else:
            # Update existing line with new data points
            path_x = [pos[0] for pos in self.robot_path]
            path_y = [pos[1] for pos in self.robot_path]
            self.path_line.set_data(path_x, path_y)
        
    def update_info_text(self, x, y, yaw):
        """Update information text display"""
        if self.info_text:
            self.info_text.remove()
            
        info_str = f"Position: ({x:.3f}, {y:.3f})\nYaw: {math.degrees(yaw):.1f}°\nPath Points: {len(self.robot_path)}\n📍 PERSISTENT PATH - Never disappears!"
        
        self.info_text = self.ax.text(
            0.02, 0.98, info_str,
            transform=self.ax.transAxes,
            verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='white', alpha=0.8),
            fontfamily='monospace'
        )
        
    def update_plot_limits(self, x, y):
        """Update plot limits to follow robot with some padding"""
        padding = 2.0  # meters
        
        # Get current limits
        xlim = self.ax.get_xlim()
        ylim = self.ax.get_ylim()
        
        # Check if robot is near edges, expand if needed
        if x - padding < xlim[0] or x + padding > xlim[1]:
            self.ax.set_xlim(x - 5, x + 5)
            
        if y - padding < ylim[0] or y + padding > ylim[1]:
            self.ax.set_ylim(y - 5, y + 5)
        
    def update_visualization(self):
        """Update the visualization with current robot state"""
        if not self.has_data:
            return
            
        with self.data_lock:
            x, y, yaw = self.robot_x, self.robot_y, self.robot_yaw
            
        # Update plot limits to follow robot
        self.update_plot_limits(x, y)
        
        # Draw robot at current position
        self.draw_robot(x, y, yaw)
        
        # Draw path trail
        self.draw_path()
        
        # Update info text
        self.update_info_text(x, y, yaw)
        
        # Refresh display
        plt.draw()
        plt.pause(0.001)  # Small pause to allow GUI update
        
    def run_visualization(self):
        """Main visualization loop"""
        print("🎯 Starting robot position visualization...")
        print("   Waiting for odometry data...")
        
        # Wait for first data
        while not self.has_data:
            time.sleep(0.1)
            
        print("✅ Received first odometry data! Starting visualization...")
        
        try:
            while plt.get_fignums():  # While plot window is open
                self.update_visualization()
                time.sleep(1.0 / self.update_rate)  # Control update rate
                
        except KeyboardInterrupt:
            print("\n⏹️  Visualization stopped by user")
        except Exception as e:
            print(f"\n❌ Visualization error: {e}")

def main():
    print("🚀 Go2 Robot Position Visualizer")
    print("=" * 50)
    print("This shows real-time 2D position and orientation of your Go2 robot.")
    print("")
    print("Features:")
    print("  - Live position tracking")
    print("  - Orientation arrow (red)")
    print("  - Path trail (green)")
    print("  - Auto-following camera")
    print("")
    
    print("Connection Setup:")
    print("  1. Connect via Ethernet cable")
    print("  2. Set PC IP to 192.168.123.51/24")
    print("  3. Robot should be at 192.168.123.18")
    print("  4. Usage: python3 robot_position_viz.py [network_interface]")
    print("")
    
    input("Press Enter to start visualization...")
    
    # Initialize channel factory
    if len(sys.argv) > 1:
        interface = sys.argv[1]
        print(f"🌐 Using network interface: {interface}")
        ChannelFactoryInitialize(0, interface)
    else:
        print("🌐 Using default network interface")
        ChannelFactoryInitialize(0)
    
    # Create and run visualizer
    visualizer = RobotPositionVisualizer()
    visualizer.run_visualization()

if __name__ == "__main__":
    main()