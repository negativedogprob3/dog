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
from ultralytics import YOLO
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.go2.video.video_client import VideoClient
from unitree_sdk2py.idl.nav_msgs.msg.dds_ import Odometry_
from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import PoseStamped_

class BallLocalizer:
    def __init__(self):
        # Robot state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.data_lock = threading.Lock()
        self.has_odom_data = False
        
        # Camera parameters (estimated for Go2)
        self.camera_fov_horizontal = 70.0  # degrees
        self.camera_height = 0.3  # meters above ground
        self.camera_tilt = 0.0  # degrees (0 = horizontal)
        
        # Ball detection parameters
        self.ball_diameter = 0.5  # meters (50cm diameter ball)
        self.current_ball_position = None  # Current ball world coordinates (only one ball)
        self.ball_detected = False  # Whether ball is currently visible
        
        # Green ball HSV color range
        self.hsv_lower = np.array([35, 50, 50])   # Lower HSV bound for green (updated from ball_detection.py)
        self.hsv_upper = np.array([85, 255, 255]) # Upper HSV bound for green (updated from ball_detection.py)
        
        # YOLO model for object detection
        print("Loading YOLOv8 nano model...")
        self.yolo_model = YOLO('yolov8n.pt')
        
        # Visualization
        plt.ion()
        self.fig, (self.ax_map, self.ax_camera) = plt.subplots(1, 2, figsize=(15, 6))
        self.setup_plots()
        
        # Camera client
        self.video_client = VideoClient()
        self.video_client.SetTimeout(3.0)
        self.video_client.Init()
        
        # Setup subscribers
        self.setup_subscribers()
        
    def setup_plots(self):
        """Setup matplotlib plots"""
        # Map plot
        self.ax_map.set_aspect('equal')
        self.ax_map.grid(True, alpha=0.3)
        self.ax_map.set_xlabel('X Position (meters)')
        self.ax_map.set_ylabel('Y Position (meters)')
        self.ax_map.set_title('Ball Localization Map')
        
        # Camera plot
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
            self.has_odom_data = True
            
    def detect_green_ball(self, frame):
        """Detect green ball using YOLO + HSV dual detection system"""
        if frame is None:
            return None, None
            
        ball_candidates = []
        
        # YOLO Detection
        try:
            results = self.yolo_model(frame, conf=0.15, imgsz=800, verbose=False, classes=[32])  # class 32 = sports ball
            
            for result in results:
                if result.boxes is not None:
                    for box in result.boxes:
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        confidence = float(box.conf[0])
                        
                        # Check if detected object is green
                        roi = frame[y1:y2, x1:x2]
                        if roi.size > 0:
                            hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                            green_mask_roi = cv2.inRange(hsv_roi, self.hsv_lower, self.hsv_upper)
                            green_ratio = cv2.countNonZero(green_mask_roi) / roi.size
                            
                            if green_ratio > 0.2:  # At least 20% green
                                area = (x2 - x1) * (y2 - y1)
                                score = confidence * green_ratio * min(area / 10000, 1.0)
                                ball_candidates.append({
                                    'bbox': (x1, y1, x2, y2),
                                    'confidence': confidence,
                                    'green_ratio': green_ratio,
                                    'score': score,
                                    'source': 'yolo'
                                })
        except Exception as e:
            print(f"⚠️  YOLO detection error: {e}")
        
        # HSV Color Detection (fallback/additional detection)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        green_mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        
        # Apply morphological operations
        kernel = np.ones((5, 5), np.uint8)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel)
        
        contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 500:  # Minimum area threshold
                x, y, w, h = cv2.boundingRect(contour)
                aspect_ratio = w / h
                
                if 0.5 <= aspect_ratio <= 2.0:  # Reasonable aspect ratio for ball
                    roi = frame[y:y+h, x:x+w]
                    if roi.size > 0:
                        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                        green_mask_roi = cv2.inRange(hsv_roi, self.hsv_lower, self.hsv_upper)
                        green_ratio = cv2.countNonZero(green_mask_roi) / roi.size
                        
                        if green_ratio > 0.4:  # At least 40% green for HSV-only detection
                            score = green_ratio * min(area / 10000, 1.0) * 0.5  # Lower weight than YOLO
                            ball_candidates.append({
                                'bbox': (x, y, x+w, y+h),
                                'green_ratio': green_ratio,
                                'score': score,
                                'source': 'color'
                            })
        
        # Select best candidate
        if not ball_candidates:
            return frame, None
            
        best_ball = max(ball_candidates, key=lambda x: x['score'])
        x1, y1, x2, y2 = best_ball['bbox']
        
        # Calculate center and radius for compatibility with existing code
        center_x, center_y = (x1 + x2) // 2, (y1 + y2) // 2
        center = (center_x, center_y)
        
        # Estimate radius from bounding box
        radius = min(x2 - x1, y2 - y1) // 2
        
        # Draw detection on frame
        color = (0, 255, 0) if best_ball['source'] == 'yolo' else (0, 255, 255)  # Green for YOLO, Yellow for HSV
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3)
        cv2.circle(frame, center, 5, (255, 0, 255), -1)  # Magenta center dot
        
        # Add label based on detection method
        if best_ball['source'] == 'yolo':
            label = f"Ball (YOLO): {best_ball['confidence']:.2f}"
        else:
            label = f"Ball (HSV): {best_ball['green_ratio']:.1%}"
        cv2.putText(frame, label, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        return frame, {'center': center, 'radius': radius, 'source': best_ball['source'], 'score': best_ball['score']}
        
    def pixel_to_world_coordinates(self, pixel_x, pixel_y, frame_shape, radius_pixels):
        """Convert pixel coordinates to world coordinates using ball size"""
        height, width = frame_shape[:2]
        
        # Convert pixel to normalized image coordinates (-1 to 1)
        norm_x = (pixel_x - width/2) / (width/2)
        norm_y = (pixel_y - height/2) / (height/2)
        
        # Calculate angle from camera center
        angle_horizontal = norm_x * (self.camera_fov_horizontal / 2) * (math.pi / 180)
        
        # Estimate distance based on known ball diameter (50cm) and pixel radius
        # Using simple pinhole camera model: distance = (real_size * focal_length) / pixel_size
        # Focal length estimation based on FOV: f = width / (2 * tan(FOV/2))
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
        rel_y = -estimated_distance * math.sin(angle_horizontal)  # Flip Y to fix left/right
        
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
        
    def update_visualization(self, frame, ball_detection):
        """Update both camera view and map visualization"""
        # Update camera view
        self.ax_camera.clear()
        if frame is not None:
            # Convert BGR to RGB for matplotlib
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            self.ax_camera.imshow(frame_rgb)
        self.ax_camera.set_title('Camera View - Green Ball Detection')
        self.ax_camera.axis('off')
        
        # Update map view
        self.ax_map.clear()
        self.ax_map.set_aspect('equal')
        self.ax_map.grid(True, alpha=0.3)
        self.ax_map.set_xlabel('X Position (meters)')
        self.ax_map.set_ylabel('Y Position (meters)')
        
        # Draw robot
        with self.data_lock:
            robot_x, robot_y, robot_yaw = self.robot_x, self.robot_y, self.robot_yaw
            
        # Robot as arrow showing orientation
        arrow_length = 0.5
        dx = arrow_length * math.cos(robot_yaw)
        dy = arrow_length * math.sin(robot_yaw)
        self.ax_map.arrow(robot_x, robot_y, dx, dy, head_width=0.2, head_length=0.2,
                         fc='blue', ec='blue', alpha=0.8, label='Robot')
        
        # Draw ball position (current or last known)
        if self.current_ball_position:
            ball_x, ball_y = self.current_ball_position
            
            if ball_detection:
                # Currently visible - solid green
                circle = plt.Circle((ball_x, ball_y), self.ball_diameter/2, 
                                  color='green', alpha=0.7, label='Ball (Live)')
                label_text = 'BALL'
                label_color = 'darkgreen'
                title = 'Ball Localization - Live Tracking'
            else:
                # Last known position - faded green with dashed border
                circle = plt.Circle((ball_x, ball_y), self.ball_diameter/2, 
                                  color='lightgreen', alpha=0.5, linestyle='--', 
                                  fill=True, linewidth=2, label='Ball (Last Known)')
                label_text = 'LAST\nKNOWN'
                label_color = 'gray'
                title = 'Ball Localization - Tracking Last Position'
                
            self.ax_map.add_patch(circle)
            self.ax_map.text(ball_x, ball_y + 0.3, label_text, ha='center', va='bottom', 
                           fontweight='bold', color=label_color, fontsize=9)
            self.ax_map.set_title(title)
            self.ax_map.legend()
        else:
            # No ball ever detected
            self.ax_map.set_title('Ball Localization - No Ball Detected')
            
        # Set fixed axis limits centered on origin (0,0)
        axis_range = 5.0  # meters in each direction
        self.ax_map.set_xlim(-axis_range, axis_range)
        self.ax_map.set_ylim(-axis_range, axis_range)
            
        plt.draw()
        plt.pause(0.001)
        
    def run(self):
        """Main loop"""
        print("🟢 Starting ball localization...")
        print("Looking for large green balls in camera view")
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
                            # No ball detected in current frame - keep last position
                            self.ball_detected = False
                            # Don't clear current_ball_position - keep showing last known location
                        
                        # Update visualization
                        self.update_visualization(annotated_frame, ball_detection)
                else:
                    print(f"⚠️  Camera error: {code}")
                    
                time.sleep(0.1)  # 10Hz update rate
                
        except KeyboardInterrupt:
            print("\n⏹️  Ball localization stopped")
        except Exception as e:
            print(f"❌ Error: {e}")
        finally:
            cv2.destroyAllWindows()
            if self.current_ball_position:
                x, y = self.current_ball_position
                print(f"🟢 Session complete. Last ball position: ({x:.2f}, {y:.2f})")
            else:
                print("🟢 Session complete. No ball detected at end.")

def main():
    print("🟢 Go2 Ball Localizer")
    print("=" * 50)
    print("This detects large green balls and localizes them in world coordinates.")
    print("")
    print("Features:")
    print("  - Real-time green ball detection")
    print("  - World coordinate mapping")
    print("  - Persistent ball position tracking")
    print("  - Live camera + map visualization")
    print("")
    
    print("Setup:")
    print("  1. Connect to Go2 robot")
    print("  2. Place large green ball(s) in view")  
    print("  3. Usage: python3 ball_localizer.py [network_interface]")
    print("")
    
    input("Press Enter to start ball localization...")
    
    # Initialize channel factory
    if len(sys.argv) > 1:
        interface = sys.argv[1]
        print(f"🌐 Using network interface: {interface}")
        ChannelFactoryInitialize(0, interface)
    else:
        print("🌐 Using default network interface")
        ChannelFactoryInitialize(0)
    
    # Create and run localizer
    localizer = BallLocalizer()
    localizer.run()

if __name__ == "__main__":
    main()