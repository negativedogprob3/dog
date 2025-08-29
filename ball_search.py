#!/usr/bin/env python3

import time
import sys
import math
import numpy as np
import cv2
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import threading
from enum import Enum
from ultralytics import YOLO
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.go2.sport.sport_client import SportClient
from unitree_sdk2py.go2.video.video_client import VideoClient
from unitree_sdk2py.idl.nav_msgs.msg.dds_ import Odometry_
from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import PoseStamped_

class SearchState(Enum):
    INITIALIZING = "initializing"
    STANDING_UP = "standing_up"
    SEARCHING = "searching"
    BALL_FOUND = "ball_found"
    STOPPED = "stopped"

class BallSearchRobot:
    def __init__(self):
        # Robot state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.data_lock = threading.Lock()
        self.has_odom_data = False
        
        # Search state
        self.search_state = SearchState.INITIALIZING
        self.is_running = True
        self.ball_found = False
        self.ball_position = None
        
        # Robot control
        self.sport_client = SportClient()
        self.sport_client.SetTimeout(3.0)
        self.sport_client.Init()
        self.is_standing = False
        
        # Movement parameters
        self.turn_speed = 1.5  # rad/s - moderate turning speed (back to original)
        
        # Camera parameters
        self.camera_fov_horizontal = 70.0  # degrees
        self.camera_height = 0.3  # meters above ground
        self.ball_diameter = 0.5  # meters (50cm diameter ball)
        
        # Ball detection - Green ball HSV color range
        self.hsv_lower = np.array([35, 50, 50])   # Lower HSV bound for green
        self.hsv_upper = np.array([85, 255, 255]) # Upper HSV bound for green
        
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
        
        # Search parameters
        self.search_start_time = 0
        self.max_search_time = 60.0  # Max 60 seconds of searching

    def setup_plots(self):
        """Setup matplotlib plots"""
        # Map plot
        self.ax_map.set_aspect('equal')
        self.ax_map.grid(True, alpha=0.3)
        self.ax_map.set_xlabel('X Position (meters)')
        self.ax_map.set_ylabel('Y Position (meters)')
        self.ax_map.set_title('Ball Search - Robot Position')
        
        # Camera plot
        self.ax_camera.set_title('Camera View - Ball Detection')
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
        
        # Calculate center and radius for compatibility
        center_x, center_y = (x1 + x2) // 2, (y1 + y2) // 2
        center = (center_x, center_y)
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
        
        # Calculate angle from camera center
        angle_horizontal = norm_x * (self.camera_fov_horizontal / 2) * (math.pi / 180)
        
        # Estimate distance based on known ball diameter and pixel radius
        focal_length_pixels = width / (2 * math.tan(math.radians(self.camera_fov_horizontal / 2)))
        pixel_diameter = radius_pixels * 2
        
        if pixel_diameter > 0:
            estimated_distance = (self.ball_diameter * focal_length_pixels) / pixel_diameter
            estimated_distance = max(0.5, min(estimated_distance, 10.0))
        else:
            estimated_distance = 2.0
        
        # Convert to robot-relative coordinates
        rel_x = estimated_distance * math.cos(angle_horizontal)
        rel_y = -estimated_distance * math.sin(angle_horizontal)
        
        # Convert to world coordinates using robot pose
        with self.data_lock:
            world_x = self.robot_x + rel_x * math.cos(self.robot_yaw) - rel_y * math.sin(self.robot_yaw)
            world_y = self.robot_y + rel_x * math.sin(self.robot_yaw) + rel_y * math.cos(self.robot_yaw)
            
        return world_x, world_y, estimated_distance

    def stand_up(self):
        """Make robot stand up"""
        if not self.is_standing:
            print("🚀 Standing up...")
            ret = self.sport_client.StandUp()
            if ret == 0:
                self.is_standing = True
                print("✅ Robot is standing")
                return True
            else:
                print(f"❌ Stand up failed: {ret}")
                return False
        return True

    def stop_movement(self):
        """Stop all robot movement"""
        ret = self.sport_client.StopMove()
        if ret != 0:
            print(f"⚠️  Stop movement failed: {ret}")

    def turn_left(self):
        """Turn robot left (counter-clockwise) - same as 'q' in robot_controller_simple.py"""
        if self.is_standing:
            # Send exact same command as pressing 'q' - positive vyaw for left turn
            ret = self.sport_client.Move(0.0, 0.0, self.turn_speed)
            if ret != 0:
                print(f"⚠️  Turn command failed: {ret}")

    def emergency_stop(self):
        """Emergency stop and damp robot"""
        self.stop_movement()
        self.sport_client.Damp()
        self.is_running = False
        print("🛑 Emergency stop activated")

    def update_visualization(self, frame, ball_detection):
        """Update both camera view and map visualization"""
        # Update camera view
        self.ax_camera.clear()
        if frame is not None:
            # Convert BGR to RGB for matplotlib
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            self.ax_camera.imshow(frame_rgb)
        
        # Show search state in camera title
        if self.search_state == SearchState.SEARCHING:
            camera_title = 'Camera View - Searching for Ball...'
        elif self.search_state == SearchState.BALL_FOUND:
            camera_title = 'Camera View - BALL FOUND!'
        else:
            camera_title = 'Camera View - Ball Detection'
        self.ax_camera.set_title(camera_title)
        self.ax_camera.axis('off')
        
        # Update map view
        self.ax_map.clear()
        self.ax_map.set_aspect('equal')
        self.ax_map.grid(True, alpha=0.3)
        self.ax_map.set_xlabel('X Position (meters)')
        self.ax_map.set_ylabel('Y Position (meters)')
        
        # Get robot position
        with self.data_lock:
            robot_x, robot_y, robot_yaw = self.robot_x, self.robot_y, self.robot_yaw
            
        # Draw robot as arrow showing orientation
        arrow_length = 0.5
        dx = arrow_length * math.cos(robot_yaw)
        dy = arrow_length * math.sin(robot_yaw)
        self.ax_map.arrow(robot_x, robot_y, dx, dy, head_width=0.2, head_length=0.2,
                         fc='blue', ec='blue', alpha=0.8, label='Robot')
        
        # Show search status in map title
        if self.search_state == SearchState.SEARCHING:
            elapsed = time.time() - self.search_start_time
            map_title = f'Ball Search - Turning Left ({elapsed:.1f}s)'
        elif self.search_state == SearchState.BALL_FOUND:
            map_title = 'Ball Search - BALL FOUND!'
        else:
            map_title = 'Ball Search - Robot Position'
        self.ax_map.set_title(map_title)
        
        # Draw ball position if found
        if self.ball_found and self.ball_position:
            ball_x, ball_y = self.ball_position
            circle = plt.Circle((ball_x, ball_y), self.ball_diameter/2, 
                              color='green', alpha=0.7, label='Ball')
            self.ax_map.add_patch(circle)
            self.ax_map.text(ball_x, ball_y + 0.3, 'BALL', ha='center', va='bottom', 
                           fontweight='bold', color='darkgreen', fontsize=9)
            
            # Show distance to ball
            distance = math.sqrt((ball_x - robot_x)**2 + (ball_y - robot_y)**2)
            self.ax_map.text(robot_x, robot_y - 0.5, f'Distance: {distance:.1f}m', 
                           ha='center', va='top', fontweight='bold', color='blue')
        
        # Set axis limits centered on robot
        axis_range = 3.0  # meters in each direction
        self.ax_map.set_xlim(robot_x - axis_range, robot_x + axis_range)
        self.ax_map.set_ylim(robot_y - axis_range, robot_y + axis_range)
        
        # Add legend if we have items
        handles, labels = self.ax_map.get_legend_handles_labels()
        if handles:
            self.ax_map.legend()
            
        plt.draw()
        plt.pause(0.001)

    def search_for_ball(self):
        """Main search loop - turn until ball is found"""
        print("🔍 Starting ball search...")
        
        # Wait for odometry data
        while not self.has_odom_data and self.is_running:
            print("⏳ Waiting for robot position data...")
            time.sleep(1)
        
        if not self.is_running:
            return
            
        print("✅ Robot position data available")
        
        # Stand up first
        self.search_state = SearchState.STANDING_UP
        if not self.stand_up():
            print("❌ Failed to stand up, aborting search")
            return
            
        time.sleep(2)  # Give robot time to stabilize
        
        # Start searching
        self.search_state = SearchState.SEARCHING
        self.search_start_time = time.time()
        self.last_turn_time = 0  # Track when we last sent turn command
        print("🔄 Searching for ball... (turning left)")
        
        # Show initial visualization
        self.update_visualization(None, None)
        
        try:
            while self.is_running and not self.ball_found and plt.get_fignums():
                # Check if we've been searching too long
                if time.time() - self.search_start_time > self.max_search_time:
                    print("⏰ Search timeout - no ball found")
                    break
                
                # Get camera frame
                code, data = self.video_client.GetImageSample()
                
                if code == 0:
                    # Convert to OpenCV image
                    image_data = np.frombuffer(bytes(data), dtype=np.uint8)
                    frame = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
                    
                    if frame is not None:
                        # Detect green ball
                        annotated_frame, ball_detection = self.detect_green_ball(frame)
                        
                        # Update visualization
                        self.update_visualization(annotated_frame, ball_detection)
                        
                        # If ball detected, stop searching
                        if ball_detection:
                            world_x, world_y, distance = self.pixel_to_world_coordinates(
                                ball_detection['center'][0], 
                                ball_detection['center'][1],
                                frame.shape,
                                ball_detection['radius']
                            )
                            
                            self.ball_position = (world_x, world_y)
                            self.ball_found = True
                            self.search_state = SearchState.BALL_FOUND
                            
                            print(f"🎯 BALL FOUND!")
                            print(f"   Position: ({world_x:.2f}, {world_y:.2f}) meters")
                            print(f"   Distance: {distance:.2f} meters")
                            print(f"   Detection: {ball_detection['source'].upper()}")
                            
                            # Stop turning
                            self.stop_movement()
                            break
                
                else:
                    print(f"⚠️  Camera error: {code}")
                    # Still update visualization without camera data
                    self.update_visualization(None, None)
                
                # Send turn command continuously for 0.5 seconds, then pause
                current_time = time.time()
                if self.search_state == SearchState.SEARCHING:
                    if current_time - self.last_turn_time < 0.5:
                        # During the 0.5 second turn duration, keep sending turn commands
                        self.turn_left()  # Same as pressing 'q' in robot_controller_simple.py
                    elif current_time - self.last_turn_time >= 1.0:
                        # After 1 second total (0.5s turning + 0.5s pause), start new turn
                        self.last_turn_time = current_time
                        print("↺ Turning left to search...")
                    
                time.sleep(0.5) 
                
        except KeyboardInterrupt:
            print("\n⏹️  Search interrupted by user")
        except Exception as e:
            print(f"❌ Search error: {e}")
        finally:
            # Stop movement and clean up
            self.stop_movement()
            self.search_state = SearchState.STOPPED
            plt.close('all')
            
            if self.ball_found:
                print(f"\n🟢 Search complete - Ball found at ({self.ball_position[0]:.2f}, {self.ball_position[1]:.2f})")
            else:
                print("\n🔴 Search complete - No ball found")

    def run(self):
        """Main execution method"""
        try:
            print("🎮 Ball Search Robot")
            print("=" * 50)
            print("This robot will turn in place until it finds a green ball.")
            print("")
            print("Press ENTER to start search, or Ctrl+C to abort")
            
            # Start the search
            self.search_for_ball()
            
        except KeyboardInterrupt:
            print("\n🛑 Aborted by user")
            self.emergency_stop()
        except Exception as e:
            print(f"\n❌ Error: {e}")
            self.emergency_stop()
        finally:
            # Ensure robot is stopped
            if self.is_standing:
                self.sport_client.Damp()
            plt.close('all')
            print("🔌 Ball search robot disconnected")

def main():
    print("🎯 Go2 Ball Search Robot")
    print("=" * 50)
    print("Autonomous ball detection with robot rotation")
    print("")
    print("Setup Instructions:")
    print("  1. Connect to Go2 robot via ethernet")
    print("  2. Set PC IP to 192.168.123.51/24") 
    print("  3. Robot should be at 192.168.123.18")
    print("  4. Place a large green ball somewhere visible")
    print("  5. Usage: python3 ball_search.py [network_interface]")
    print("")
    print("Behavior:")
    print("  - Robot will stand up automatically")
    print("  - Robot will turn right until ball is detected")
    print("  - Robot will stop when ball is found")
    print("  - Live camera view shows detection status")
    print("")
    
    print("⚠️  WARNING: Ensure clear space around robot!")
    input("Press Enter to continue...")
    
    # Initialize channel factory
    if len(sys.argv) > 1:
        interface = sys.argv[1]
        print(f"🌐 Using network interface: {interface}")
        ChannelFactoryInitialize(0, interface)
    else:
        print("🌐 Using default network interface")
        ChannelFactoryInitialize(0)
    
    # Create and run search robot
    search_robot = BallSearchRobot()
    search_robot.run()

if __name__ == "__main__":
    main()