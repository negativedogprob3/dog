#!/usr/bin/env python3

import time
import sys
import math
import numpy as np
import cv2
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import threading
from collections import deque
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.go2.video.video_client import VideoClient
from unitree_sdk2py.idl.nav_msgs.msg.dds_ import Odometry_
from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import PoseStamped_

class RobotBallTracker:
    def __init__(self):
        # Robot state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.data_lock = threading.Lock()
        self.has_odom_data = False
        
        # Robot path tracking (persistent - never disappears)
        self.robot_path = []  # Keep ALL robot positions
        self.last_path_point = None  # To avoid duplicate consecutive points
        
        # Ball state
        self.current_ball_position = None  # Current ball world coordinates
        self.ball_detected = False  # Whether ball is currently visible
        
        # Camera parameters (estimated for Go2)
        self.camera_fov_horizontal = 70.0  # degrees
        self.camera_height = 0.3  # meters above ground
        self.camera_tilt = 0.0  # degrees (0 = horizontal)
        
        # Ball detection parameters
        self.ball_diameter = 0.5  # meters (50cm diameter ball)
        
        # Specific green ball HSV color range for RGB(90, 180, 120)
        # Convert target RGB(90, 180, 120) to HSV for reference
        target_rgb = np.uint8([[[90, 180, 120]]])
        target_hsv = cv2.cvtColor(target_rgb, cv2.COLOR_RGB2HSV)[0][0]
        print(f"Target RGB(90,180,120) = HSV({target_hsv[0]}, {target_hsv[1]}, {target_hsv[2]})")
        
        # Narrow HSV range around the target color (less sensitive)
        h_tolerance = 10  # Hue tolerance
        s_tolerance = 50  # Saturation tolerance  
        v_tolerance = 50  # Value tolerance
        
        self.hsv_lower = np.array([
            max(0, target_hsv[0] - h_tolerance),
            max(0, target_hsv[1] - s_tolerance), 
            max(0, target_hsv[2] - v_tolerance)
        ])
        self.hsv_upper = np.array([
            min(179, target_hsv[0] + h_tolerance),
            min(255, target_hsv[1] + s_tolerance),
            min(255, target_hsv[2] + v_tolerance)
        ])
        print(f"HSV range: {self.hsv_lower} to {self.hsv_upper}")
        
        # Visualization settings
        self.robot_size = 0.3  # meters
        self.update_rate = 10  # Hz
        
        # Setup matplotlib - single plot combining both
        plt.ion()
        self.fig, (self.ax_map, self.ax_camera) = plt.subplots(1, 2, figsize=(16, 8))
        self.setup_plots()
        
        # Initialize plot elements
        self.robot_patch = None
        self.robot_path_line = None
        self.arrow_patch = None
        self.info_text = None
        
        # Camera client
        self.video_client = VideoClient()
        self.video_client.SetTimeout(3.0)
        self.video_client.Init()
        
        # Setup subscribers
        self.setup_subscribers()
        
    def setup_plots(self):
        """Setup matplotlib plots"""
        # Map plot (left side)
        self.ax_map.set_aspect('equal')
        self.ax_map.grid(True, alpha=0.3)
        self.ax_map.set_xlabel('X Position (meters)')
        self.ax_map.set_ylabel('Y Position (meters)')
        self.ax_map.set_title('Robot Path + Ball Tracking')
        
        # Camera plot (right side)
        self.ax_camera.set_title('Camera View - Green Ball Detection')
        self.ax_camera.axis('off')
        
    def setup_subscribers(self):
        """Setup subscribers for robot odometry"""
        try:
            self.pose_subscriber = ChannelSubscriber("rt/utlidar/robot_pose", PoseStamped_)
            self.pose_subscriber.Init(self.pose_handler, 10)
            print("✅ Subscribed to robot pose")
        except Exception as e:
            print(f"⚠️  Failed to subscribe to robot_pose: {e}")
            
        try:
            self.odom_subscriber = ChannelSubscriber("rt/lio_sam_ros2/mapping/odometry", Odometry_)
            self.odom_subscriber.Init(self.odom_handler, 10)
            print("✅ Subscribed to SLAM odometry")
        except Exception as e:
            print(f"⚠️  Failed to subscribe to SLAM odometry: {e}")
            
    def quaternion_to_yaw(self, qx, qy, qz, qw):
        """Convert quaternion to yaw angle"""
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)
        
    def pose_handler(self, msg: PoseStamped_):
        """Handle robot pose updates"""
        with self.data_lock:
            self.robot_x = msg.pose.position.x
            self.robot_y = msg.pose.position.y
            self.robot_yaw = self.quaternion_to_yaw(
                msg.pose.orientation.x, msg.pose.orientation.y,
                msg.pose.orientation.z, msg.pose.orientation.w
            )
            self.update_robot_path()
            self.has_odom_data = True
            
    def odom_handler(self, msg: Odometry_):
        """Handle SLAM odometry updates"""
        with self.data_lock:
            self.robot_x = msg.pose.pose.position.x
            self.robot_y = msg.pose.pose.position.y
            self.robot_yaw = self.quaternion_to_yaw(
                msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
            )
            self.update_robot_path()
            self.has_odom_data = True
            
    def update_robot_path(self):
        """Update robot path with current position"""
        # Only add point if robot has moved significantly (avoid cluttering)
        current_point = (self.robot_x, self.robot_y)
        if (self.last_path_point is None or 
            abs(current_point[0] - self.last_path_point[0]) > 0.01 or
            abs(current_point[1] - self.last_path_point[1]) > 0.01):
            self.robot_path.append(current_point)
            self.last_path_point = current_point
            
    def detect_green_ball(self, frame):
        """Detect green ball in camera frame"""
        if frame is None:
            return None, None
            
        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Create mask for green color
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        
        # More aggressive morphological operations to reduce noise
        kernel_small = np.ones((3, 3), np.uint8)
        kernel_large = np.ones((7, 7), np.uint8)
        
        # Remove small noise
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_small)
        # Fill small holes
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_large)
        # Additional erosion to be more strict
        mask = cv2.erode(mask, kernel_small, iterations=1)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return frame, None
            
        # Find largest contour (assuming it's the ball)
        largest_contour = max(contours, key=cv2.contourArea)
        
        # Less sensitive detection - larger minimum area for 50cm ball
        area = cv2.contourArea(largest_contour)
        if area < 300:  # Increased from 100 - must be substantial
            return frame, None
            
        # Get bounding circle
        (x, y), radius = cv2.minEnclosingCircle(largest_contour)
        center = (int(x), int(y))
        radius = int(radius)
        
        # Stricter radius requirement for less sensitivity
        if radius < 20:  # Increased from 10 - ball must appear reasonably large
            return frame, None
            
        # Check aspect ratio to ensure circular shape
        rect = cv2.boundingRect(largest_contour)
        rect_width, rect_height = rect[2], rect[3]
        if rect_width > 0 and rect_height > 0:
            aspect_ratio = max(rect_width, rect_height) / min(rect_width, rect_height)
            if aspect_ratio > 1.3:  # Must be nearly circular
                return frame, None
                
        # Check circularity (how round the shape is)
        perimeter = cv2.arcLength(largest_contour, True)
        if perimeter == 0:
            return frame, None
        circularity = 4 * math.pi * area / (perimeter * perimeter)
        if circularity < 0.6:  # Must be reasonably circular (perfect circle = 1.0)
            return frame, None
            
        # Estimate ball size in real world and check if it's ~50cm
        # Using pinhole camera model to estimate distance and size
        focal_length_pixels = frame.shape[1] / (2 * math.tan(math.radians(self.camera_fov_horizontal / 2)))
        pixel_diameter = radius * 2
        
        if pixel_diameter > 0:
            # Estimate distance to ball
            estimated_distance = (self.ball_diameter * focal_length_pixels) / pixel_diameter
            estimated_distance = max(0.5, min(estimated_distance, 10.0))  # Clamp to reasonable range
            
            # Calculate what the ball diameter should be at this distance
            expected_pixel_diameter = (self.ball_diameter * focal_length_pixels) / estimated_distance
            size_ratio = pixel_diameter / expected_pixel_diameter
            
            # Ball should appear the right size for a 50cm ball at estimated distance
            if size_ratio < 0.7 or size_ratio > 1.5:  # Allow some tolerance
                return frame, None
                
        # Draw detection on frame with additional info
        cv2.circle(frame, center, radius, (0, 255, 255), 2)  # Yellow circle
        cv2.circle(frame, center, 2, (0, 0, 255), -1)        # Red center dot
        cv2.putText(frame, f'Ball: d={pixel_diameter:.0f}px c={circularity:.2f}', 
                   (center[0]-70, center[1]-radius-10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(frame, f'~{estimated_distance:.1f}m', 
                   (center[0]-20, center[1]+radius+20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        return frame, {'center': center, 'radius': radius, 'circularity': circularity, 'distance': estimated_distance}
        
    def pixel_to_world_coordinates(self, pixel_x, pixel_y, frame_shape, radius_pixels):
        """Convert pixel coordinates to world coordinates using ball size"""
        height, width = frame_shape[:2]
        
        # Convert pixel to normalized image coordinates (-1 to 1)
        norm_x = (pixel_x - width/2) / (width/2)
        
        # Calculate angle from camera center
        angle_horizontal = norm_x * (self.camera_fov_horizontal / 2) * (math.pi / 180)
        
        # Estimate distance based on known ball diameter (50cm) and pixel radius
        focal_length_pixels = width / (2 * math.tan(math.radians(self.camera_fov_horizontal / 2)))
        
        # Distance estimation: distance = (real_diameter * focal_length) / pixel_diameter
        pixel_diameter = radius_pixels * 2
        if pixel_diameter > 0:
            estimated_distance = (self.ball_diameter * focal_length_pixels) / pixel_diameter
            # Clamp distance to reasonable range
            estimated_distance = max(0.5, min(estimated_distance, 10.0))
        else:
            estimated_distance = 2.0  # Default fallback
        
        # Convert to robot-relative coordinates
        rel_x = estimated_distance * math.cos(angle_horizontal)
        rel_y = estimated_distance * math.sin(angle_horizontal)
        
        # Convert to world coordinates using robot pose
        with self.data_lock:
            world_x = self.robot_x + rel_x * math.cos(self.robot_yaw) - rel_y * math.sin(self.robot_yaw)
            world_y = self.robot_y + rel_x * math.sin(self.robot_yaw) + rel_y * math.cos(self.robot_yaw)
            
        return world_x, world_y
        
    def update_ball_position(self, world_x, world_y):
        """Update current ball position in real-time"""
        self.current_ball_position = (world_x, world_y)
        self.ball_detected = True
        print(f"🟢 Ball at: ({world_x:.2f}, {world_y:.2f})")
        
    def draw_robot_path(self):
        """Draw robot path as a persistent line"""
        if len(self.robot_path) < 2:
            return
            
        # DON'T remove old path - keep it persistent!
        if self.robot_path_line is None:
            # Extract x, y coordinates
            path_x = [pos[0] for pos in self.robot_path]
            path_y = [pos[1] for pos in self.robot_path]
            
            # Draw complete path 
            self.robot_path_line = self.ax_map.plot(path_x, path_y, 'b-', alpha=0.5, linewidth=1, label='Robot Path')[0]
        else:
            # Update existing line with new data points
            path_x = [pos[0] for pos in self.robot_path]
            path_y = [pos[1] for pos in self.robot_path]
            self.robot_path_line.set_data(path_x, path_y)
            
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
        self.robot_patch = self.ax_map.add_patch(rect)
        
        # Direction arrow
        arrow_length = self.robot_size * 0.8
        dx = arrow_length * math.cos(yaw)
        dy = arrow_length * math.sin(yaw)
        
        self.arrow_patch = self.ax_map.arrow(
            x, y, dx, dy,
            head_width=0.1, head_length=0.08,
            fc='red', ec='red', alpha=0.9
        )
        
    def update_plot_limits(self, x, y):
        """Update plot limits to follow robot with some padding"""
        padding = 3.0  # meters
        
        # Get current limits
        xlim = self.ax_map.get_xlim()
        ylim = self.ax_map.get_ylim()
        
        # Check if robot is near edges, expand if needed
        if x - padding < xlim[0] or x + padding > xlim[1]:
            self.ax_map.set_xlim(x - 5, x + 5)
            
        if y - padding < ylim[0] or y + padding > ylim[1]:
            self.ax_map.set_ylim(y - 5, y + 5)
            
    def update_info_text(self, x, y, yaw):
        """Update information text display"""
        if self.info_text:
            self.info_text.remove()
            
        ball_info = ""
        if self.current_ball_position:
            bx, by = self.current_ball_position
            distance_to_ball = math.sqrt((bx - x)**2 + (by - y)**2)
            ball_info = f"\nBall: ({bx:.2f}, {by:.2f})\nDistance: {distance_to_ball:.2f}m"
        
        info_str = f"Robot: ({x:.2f}, {y:.2f})\nYaw: {math.degrees(yaw):.1f}°\nPath Points: {len(self.robot_path)}{ball_info}"
        
        self.info_text = self.ax_map.text(
            0.02, 0.98, info_str,
            transform=self.ax_map.transAxes,
            verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='white', alpha=0.8),
            fontfamily='monospace'
        )
        
    def update_visualization(self, frame, ball_detection):
        """Update both camera view and map visualization"""
        if not self.has_odom_data:
            return
            
        # Update camera view
        self.ax_camera.clear()
        if frame is not None:
            # Convert BGR to RGB for matplotlib
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            self.ax_camera.imshow(frame_rgb)
        self.ax_camera.set_title('Camera View - Green Ball Detection')
        self.ax_camera.axis('off')
        
        # Update map view
        with self.data_lock:
            x, y, yaw = self.robot_x, self.robot_y, self.robot_yaw
            
        # Update plot limits to follow robot
        self.update_plot_limits(x, y)
        
        # Clear map but keep path data
        self.ax_map.clear()
        self.ax_map.set_aspect('equal')
        self.ax_map.grid(True, alpha=0.3)
        self.ax_map.set_xlabel('X Position (meters)')
        self.ax_map.set_ylabel('Y Position (meters)')
        self.ax_map.set_title('Robot Path + Ball Tracking (Live)')
        
        # Draw persistent robot path
        self.robot_path_line = None  # Reset so it gets redrawn
        self.draw_robot_path()
        
        # Draw current robot position
        self.draw_robot(x, y, yaw)
        
        # Draw current ball if detected
        if ball_detection and self.current_ball_position:
            ball_x, ball_y = self.current_ball_position
            circle = plt.Circle((ball_x, ball_y), self.ball_diameter/2, 
                              color='green', alpha=0.7, label='Ball (Live)')
            self.ax_map.add_patch(circle)
            self.ax_map.text(ball_x, ball_y + 0.3, 'BALL', ha='center', va='bottom', 
                           fontweight='bold', color='darkgreen')
            
        # Update info text
        self.update_info_text(x, y, yaw)
        
        # Add legend
        handles, labels = self.ax_map.get_legend_handles_labels()
        if handles:
            self.ax_map.legend()
        
        # Refresh display
        plt.draw()
        plt.pause(0.001)
        
    def run(self):
        """Main loop combining robot tracking and ball detection"""
        print("🚀 Starting Robot + Ball Tracker...")
        print("Features:")
        print("  - Persistent robot path (never disappears)")
        print("  - Live ball detection and localization")
        print("  - Real-time camera + map visualization")
        print("Press Ctrl+C to stop")
        
        # Wait for odometry data
        while not self.has_odom_data:
            print("⏳ Waiting for robot position data...")
            time.sleep(1)
            
        print("✅ Robot position data available")
        
        try:
            while plt.get_fignums():  # While plot window is open
                # Get camera frame
                code, data = self.video_client.GetImageSample()
                
                if code == 0:
                    # Convert to OpenCV image
                    image_data = np.frombuffer(bytes(data), dtype=np.uint8)
                    frame = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
                    
                    if frame is not None:
                        # Detect green ball
                        annotated_frame, ball_detection = self.detect_green_ball(frame)
                        
                        # If ball detected, calculate world coordinates
                        if ball_detection:
                            world_x, world_y = self.pixel_to_world_coordinates(
                                ball_detection['center'][0], 
                                ball_detection['center'][1],
                                frame.shape,
                                ball_detection['radius']
                            )
                            self.update_ball_position(world_x, world_y)
                        else:
                            # No ball detected in current frame
                            self.ball_detected = False
                        
                        # Update visualization
                        self.update_visualization(annotated_frame, ball_detection)
                else:
                    print(f"⚠️  Camera error: {code}")
                    # Still update visualization without camera data
                    self.update_visualization(None, None)
                    
                time.sleep(1.0 / self.update_rate)  # Control update rate
                
        except KeyboardInterrupt:
            print("\n⏹️  Robot + Ball tracking stopped")
        except Exception as e:
            print(f"❌ Error: {e}")
        finally:
            cv2.destroyAllWindows()
            print(f"🚀 Session complete!")
            print(f"   Robot path points: {len(self.robot_path)}")
            if self.current_ball_position:
                x, y = self.current_ball_position
                print(f"   Last ball position: ({x:.2f}, {y:.2f})")
            else:
                print("   No ball detected at end")

def main():
    print("🚀 Go2 Robot + Ball Tracker")
    print("=" * 60)
    print("Combined robot path tracking and ball localization")
    print("")
    print("Features:")
    print("  - Robot path trail (blue line - never disappears)")
    print("  - Current robot position (blue rectangle + red arrow)")
    print("  - Live ball detection (green circle when visible)")
    print("  - Real-time camera feed with detection overlay")
    print("  - Distance to ball calculation")
    print("")
    
    print("Setup:")
    print("  1. Connect to Go2 robot")
    print("  2. Place 50cm green ball in environment")  
    print("  3. Usage: python3 robot_ball_tracker.py [network_interface]")
    print("")
    
    input("Press Enter to start tracking...")
    
    # Initialize channel factory
    if len(sys.argv) > 1:
        interface = sys.argv[1]
        print(f"🌐 Using network interface: {interface}")
        ChannelFactoryInitialize(0, interface)
    else:
        print("🌐 Using default network interface")
        ChannelFactoryInitialize(0)
    
    # Create and run tracker
    tracker = RobotBallTracker()
    tracker.run()

if __name__ == "__main__":
    main()