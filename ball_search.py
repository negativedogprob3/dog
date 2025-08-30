#!/usr/bin/env python3
# NOTE: This script now includes full ball retrieval behavior!
# The robot will push the ball back to its starting position.

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
    APPROACHING = "approaching"
    POSITIONING = "positioning"  # Moving to behind ball
    ALIGNING = "aligning"        # Aligning to push direction
    PUSHING = "pushing"          # Pushing ball to start
    STOPPED = "stopped"

class BallSearchRobot:
    def __init__(self):
        # Robot state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.data_lock = threading.Lock()
        self.has_odom_data = False
        
        # Starting position (where to return the ball)
        self.start_x = None
        self.start_y = None
        self.start_recorded = False
        
        # Target positions for navigation
        self.target_x = None
        self.target_y = None
        self.behind_ball_x = None
        self.behind_ball_y = None
        
        # Search state
        self.search_state = SearchState.INITIALIZING
        self.is_running = True
        self.ball_found = False
        self.ball_position = None  # Current/live ball position
        
        # Ball position estimation (persists even when not visible)
        self.estimated_ball_x = None
        self.estimated_ball_y = None
        self.last_ball_seen_time = None
        self.ball_confidence = 0.0  # Confidence in estimated position (0-1)
        
        # Robot control
        self.sport_client = SportClient()
        self.sport_client.SetTimeout(3.0)
        self.sport_client.Init()
        self.is_standing = False
        
        # Movement parameters
        self.turn_speed = 0.8  # rad/s - slower turning speed for better detection
        self.walk_speed = 0.4  # m/s - slower forward walking speed
        self.approach_min_distance = 0.8  # meters - stop farther to avoid pushing ball
        
        # Camera parameters
        self.camera_fov_horizontal = 70.0  # degrees
        self.camera_height = 0.3  # meters above ground
        self.ball_diameter = 0.5  # meters (50cm diameter ball)
        
        # Ball detection - Green ball HSV color range (relaxed for better detection)
        self.hsv_lower = np.array([30, 40, 40])   # More relaxed lower HSV bound for green
        self.hsv_upper = np.array([90, 255, 255]) # Wider upper HSV bound for green
        
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
        
        # Push monitoring for realignment
        self.last_realign_time = 0
        self.realign_interval = 3.0  # Check alignment every 3 seconds
        self.push_attempts = 0
        self.max_push_attempts = 10  # Prevent infinite repositioning loops

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
            # Record starting position on first update
            if not self.start_recorded:
                self.start_x = self.robot_x
                self.start_y = self.robot_y
                self.start_recorded = True
                print(f"📍 Starting position recorded: ({self.start_x:.2f}, {self.start_y:.2f})")
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
            # Record starting position on first update
            if not self.start_recorded:
                self.start_x = self.robot_x
                self.start_y = self.robot_y
                self.start_recorded = True
                print(f"📍 Starting position recorded: ({self.start_x:.2f}, {self.start_y:.2f})")
            self.has_odom_data = True

    def detect_green_ball(self, frame):
        """Detect green ball using YOLO + HSV dual detection system"""
        if frame is None:
            return None, None
            
        ball_candidates = []
        
        # YOLO Detection
        try:
            results = self.yolo_model(frame, conf=0.10, imgsz=800, verbose=False, classes=[32])  # Lower confidence for better detection
            
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
                            
                            # Check if this is a large/close object
                            frame_area = frame.shape[0] * frame.shape[1]
                            box_area = (x2 - x1) * (y2 - y1)
                            is_large_yolo = box_area > frame_area * 0.05
                            
                            # Relax green requirement for close objects
                            min_green_ratio_yolo = 0.1 if is_large_yolo else 0.15
                            
                            if green_ratio > min_green_ratio_yolo:  # More relaxed for close objects
                                area = (x2 - x1) * (y2 - y1)
                                # Don't penalize large objects
                                area_factor = min(area / 10000, 2.0) if is_large_yolo else min(area / 10000, 1.0)
                                score = confidence * green_ratio * area_factor
                                ball_candidates.append({
                                    'bbox': (x1, y1, x2, y2),
                                    'confidence': confidence,
                                    'green_ratio': green_ratio,
                                    'score': score,
                                    'source': 'yolo',
                                    'is_large': is_large_yolo
                                })
        except Exception as e:
            print(f"⚠️  YOLO detection error: {e}")
        
        # HSV Color Detection (fallback/additional detection)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        green_mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        
        # Apply gentler morphological operations to preserve more detections
        kernel = np.ones((3, 3), np.uint8)  # Smaller kernel (was 5x5)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel)
        
        contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 200:  # Further relaxed: even smaller minimum area for close detection
                x, y, w, h = cv2.boundingRect(contour)
                aspect_ratio = w / h
                
                # Check if this might be a close-up ball (very large)
                frame_area = frame.shape[0] * frame.shape[1]
                is_large_object = area > frame_area * 0.05  # Object takes >5% of frame
                
                # More relaxed aspect ratio for large/close objects
                if is_large_object:
                    aspect_ok = 0.3 <= aspect_ratio <= 3.0  # Very relaxed for close balls
                else:
                    aspect_ok = 0.4 <= aspect_ratio <= 2.5  # Normal range
                
                if aspect_ok:
                    roi = frame[y:y+h, x:x+w]
                    if roi.size > 0:
                        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                        green_mask_roi = cv2.inRange(hsv_roi, self.hsv_lower, self.hsv_upper)
                        green_ratio = cv2.countNonZero(green_mask_roi) / roi.size
                        
                        # More relaxed green ratio for large/close objects
                        min_green_ratio = 0.2 if is_large_object else 0.3
                        
                        if green_ratio > min_green_ratio:
                            # Don't penalize large objects in scoring
                            area_score = min(area / 10000, 2.0) if is_large_object else min(area / 10000, 1.0)
                            score = green_ratio * area_score * 0.5
                            ball_candidates.append({
                                'bbox': (x, y, x+w, y+h),
                                'green_ratio': green_ratio,
                                'score': score,
                                'source': 'color',
                                'is_large': is_large_object
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
            # Debug: confirm command was sent
            # print(f"DEBUG: Sent Move(0, 0, {self.turn_speed})")
    
    def move_forward(self):
        """Move robot forward toward the ball"""
        if self.is_standing:
            ret = self.sport_client.Move(self.walk_speed, 0.0, 0.0)
            if ret != 0:
                print(f"⚠️  Forward movement failed: {ret}")
    
    def turn_toward_ball(self, angle_to_ball):
        """Turn robot toward ball based on angle"""
        if self.is_standing:
            # Turn speed proportional to angle (slower when close to correct heading)
            turn_rate = min(self.turn_speed, abs(angle_to_ball) * 2.0)
            if angle_to_ball > 0:
                turn_rate = -turn_rate  # Turn right if ball is to the right
            ret = self.sport_client.Move(0.0, 0.0, turn_rate)
            if ret != 0:
                print(f"⚠️  Turn toward ball failed: {ret}")

    def emergency_stop(self):
        """Emergency stop and damp robot"""
        self.stop_movement()
        self.sport_client.Damp()
        self.is_running = False
        print("🛑 Emergency stop activated")
    
    def update_ball_estimation(self, world_x, world_y):
        """Update estimated ball position with new detection"""
        current_time = time.time()
        
        # If this is first detection or significant movement
        if self.estimated_ball_x is None or self.estimated_ball_y is None:
            self.estimated_ball_x = world_x
            self.estimated_ball_y = world_y
            self.ball_confidence = 1.0
        else:
            # Smooth update using weighted average
            # More weight to new measurement if high confidence
            alpha = 0.7  # Weight for new measurement
            self.estimated_ball_x = alpha * world_x + (1 - alpha) * self.estimated_ball_x
            self.estimated_ball_y = alpha * world_y + (1 - alpha) * self.estimated_ball_y
            self.ball_confidence = min(1.0, self.ball_confidence + 0.2)  # Increase confidence
        
        self.last_ball_seen_time = current_time
        
    def decay_ball_confidence(self):
        """Decay confidence in ball position estimate over time"""
        if self.last_ball_seen_time is None:
            return
            
        current_time = time.time()
        time_since_seen = current_time - self.last_ball_seen_time
        
        # Decay confidence based on time (lose 10% confidence per second)
        decay_rate = 0.1
        self.ball_confidence = max(0.0, self.ball_confidence - decay_rate * time_since_seen * 0.02)  # 0.02 for 50Hz update
    
    def calculate_behind_ball_position(self, ball_x, ball_y):
        """Calculate position behind ball (opposite from starting position)"""
        if self.start_x is None or self.start_y is None:
            return None, None
            
        # Vector from ball to start
        to_start_x = self.start_x - ball_x
        to_start_y = self.start_y - ball_y
        distance_to_start = math.sqrt(to_start_x**2 + to_start_y**2)
        
        if distance_to_start < 0.01:  # Ball is already at start
            return ball_x, ball_y
            
        # Normalize vector
        norm_x = to_start_x / distance_to_start
        norm_y = to_start_y / distance_to_start
        
        # Position 0.7m behind ball (opposite from start)
        behind_x = ball_x - norm_x * 0.7
        behind_y = ball_y - norm_y * 0.7
        
        return behind_x, behind_y
    
    def is_on_correct_side_of_ball(self, ball_x, ball_y):
        """Check if robot is on the correct side of ball (behind it relative to goal)"""
        if self.start_x is None or self.start_y is None:
            return False
            
        with self.data_lock:
            # Vector from ball to goal
            ball_to_goal_x = self.start_x - ball_x
            ball_to_goal_y = self.start_y - ball_y
            
            # Vector from ball to robot
            ball_to_robot_x = self.robot_x - ball_x
            ball_to_robot_y = self.robot_y - ball_y
            
            # Dot product to check if robot is on opposite side from goal
            # If negative, robot is behind ball (correct side)
            # If positive, robot is between ball and goal (wrong side)
            dot_product = (ball_to_goal_x * ball_to_robot_x + 
                          ball_to_goal_y * ball_to_robot_y)
            
            return dot_product < 0  # True if robot is behind ball
    
    def calculate_angle_to_position(self, target_x, target_y):
        """Calculate angle from robot to target position"""
        with self.data_lock:
            dx = target_x - self.robot_x
            dy = target_y - self.robot_y
            target_angle = math.atan2(dy, dx)
            
            # Calculate angle difference
            angle_diff = target_angle - self.robot_yaw
            
            # Normalize to [-pi, pi]
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi
                
            return angle_diff
    
    def move_backward(self):
        """Move robot backward"""
        if self.is_standing:
            ret = self.sport_client.Move(-self.walk_speed * 0.5, 0.0, 0.0)  # Slower backward
            if ret != 0:
                print(f"⚠️  Backward movement failed: {ret}")
    
    def strafe_right(self):
        """Move robot sideways to the right"""
        if self.is_standing:
            ret = self.sport_client.Move(0.0, -self.walk_speed * 0.5, 0.0)
            if ret != 0:
                print(f"⚠️  Strafe right failed: {ret}")

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
        elif self.search_state == SearchState.APPROACHING:
            camera_title = 'Camera View - APPROACHING BALL!'
        elif self.search_state == SearchState.POSITIONING:
            camera_title = 'Camera View - Moving Behind Ball'
        elif self.search_state == SearchState.ALIGNING:
            camera_title = 'Camera View - Aligning for Push'
        elif self.search_state == SearchState.PUSHING:
            camera_title = 'Camera View - PUSHING BALL!'
        elif self.search_state == SearchState.STOPPED:
            camera_title = 'Camera View - MISSION COMPLETE!'
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
            map_title = f'Ball Retrieval - Searching ({elapsed:.1f}s)'
        elif self.search_state == SearchState.APPROACHING:
            if self.ball_position:
                distance = math.sqrt((self.ball_position[0] - robot_x)**2 + (self.ball_position[1] - robot_y)**2)
                map_title = f'Ball Retrieval - Approaching Ball ({distance:.2f}m)'
            else:
                map_title = 'Ball Retrieval - APPROACHING'
        elif self.search_state == SearchState.POSITIONING:
            if self.behind_ball_x is not None:
                dist = math.sqrt((self.behind_ball_x - robot_x)**2 + (self.behind_ball_y - robot_y)**2)
                map_title = f'Ball Retrieval - Moving Behind Ball ({dist:.2f}m)'
            else:
                map_title = 'Ball Retrieval - POSITIONING'
        elif self.search_state == SearchState.ALIGNING:
            map_title = 'Ball Retrieval - Aligning for Push'
        elif self.search_state == SearchState.PUSHING:
            if self.ball_position and self.start_x is not None:
                ball_x, ball_y = self.ball_position
                dist_to_goal = math.sqrt((ball_x - self.start_x)**2 + (ball_y - self.start_y)**2)
                map_title = f'Ball Retrieval - PUSHING! (Goal: {dist_to_goal:.2f}m)'
            else:
                map_title = 'Ball Retrieval - PUSHING'
        elif self.search_state == SearchState.STOPPED:
            map_title = '🎉 MISSION COMPLETE!'
        else:
            map_title = 'Ball Retrieval System'
        self.ax_map.set_title(map_title)
        
        # Draw starting position (goal)
        if self.start_x is not None and self.start_y is not None:
            # Goal marker
            goal_circle = plt.Circle((self.start_x, self.start_y), 0.2, 
                                    color='gold', alpha=0.5, linestyle='--', 
                                    linewidth=2, fill=False, label='Start/Goal')
            self.ax_map.add_patch(goal_circle)
            self.ax_map.text(self.start_x, self.start_y - 0.3, 'GOAL', 
                           ha='center', va='top', fontweight='bold', color='goldenrod', fontsize=8)
        
        # Draw target position behind ball
        if self.behind_ball_x is not None and self.behind_ball_y is not None and self.search_state == SearchState.POSITIONING:
            target_marker = plt.Circle((self.behind_ball_x, self.behind_ball_y), 0.15,
                                      color='red', alpha=0.3, label='Target Position')
            self.ax_map.add_patch(target_marker)
            self.ax_map.plot(self.behind_ball_x, self.behind_ball_y, 'rx', markersize=10)
        
        # Draw estimated ball position (if we have one)
        if self.estimated_ball_x is not None and self.estimated_ball_y is not None and self.ball_confidence > 0.1:
            # Draw estimated position with transparency based on confidence
            est_circle = plt.Circle((self.estimated_ball_x, self.estimated_ball_y), 
                                   self.ball_diameter/2, 
                                   color='lightgreen', alpha=self.ball_confidence * 0.3,
                                   linestyle='--', linewidth=2, fill=False,
                                   label=f'Est. Ball (conf={self.ball_confidence:.1f})')
            self.ax_map.add_patch(est_circle)
            
            # Show "EST" text only if not currently detected
            if not self.ball_found or self.ball_position is None:
                self.ax_map.text(self.estimated_ball_x, self.estimated_ball_y - 0.35, 
                               f'EST\n{self.ball_confidence:.1f}', 
                               ha='center', va='top', fontweight='bold', 
                               color='gray', fontsize=7, alpha=0.7)
        
        # Draw current ball position if detected
        if self.ball_found and self.ball_position:
            ball_x, ball_y = self.ball_position
            circle = plt.Circle((ball_x, ball_y), self.ball_diameter/2, 
                              color='green', alpha=0.7, label='Ball (Live)')
            self.ax_map.add_patch(circle)
            self.ax_map.text(ball_x, ball_y + 0.3, 'BALL', ha='center', va='bottom', 
                           fontweight='bold', color='darkgreen', fontsize=9)
            
            # Draw line from ball to goal during pushing
            if self.search_state in [SearchState.POSITIONING, SearchState.ALIGNING, SearchState.PUSHING]:
                if self.start_x is not None:
                    self.ax_map.plot([ball_x, self.start_x], [ball_y, self.start_y], 
                                   'g--', alpha=0.3, linewidth=1)
            
            # Show distance to ball only during approach
            if self.search_state == SearchState.APPROACHING:
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
            
        print("⏳ Waiting for robot to stabilize...")
        time.sleep(4)  # Give robot plenty of time to stabilize
        
        # Start searching
        self.search_state = SearchState.SEARCHING
        self.search_start_time = time.time()
        self.last_turn_time = 0  # Track when we last sent turn command
        print("🔄 Searching for ball... (turning left)")
        
        # Show initial visualization
        self.update_visualization(None, None)
        
        try:
            while self.is_running and self.search_state != SearchState.STOPPED and plt.get_fignums():
                # Check if we've been searching too long
                if time.time() - self.search_start_time > self.max_search_time:
                    print("⏰ Search timeout - no ball found")
                    break
                
                # Initialize ball detection for this iteration
                ball_detection = None
                annotated_frame = None
                frame = None
                
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
                        
                        # If ball detected, transition to approaching
                        if ball_detection:
                            world_x, world_y, distance = self.pixel_to_world_coordinates(
                                ball_detection['center'][0], 
                                ball_detection['center'][1],
                                frame.shape,
                                ball_detection['radius']
                            )
                            
                            self.ball_position = (world_x, world_y)
                            self.ball_found = True
                            
                            # Update ball position estimation
                            self.update_ball_estimation(world_x, world_y)
                            
                            if self.search_state == SearchState.SEARCHING:
                                print(f"🎯 BALL FOUND!")
                                print(f"   Position: ({world_x:.2f}, {world_y:.2f}) meters")
                                print(f"   Estimated: ({self.estimated_ball_x:.2f}, {self.estimated_ball_y:.2f}) conf={self.ball_confidence:.2f}")
                                print(f"   Distance: {distance:.2f} meters")
                                print(f"   Detection: {ball_detection['source'].upper()}")
                                self.search_state = SearchState.APPROACHING
                                self.stop_movement()  # Stop turning before approaching
                            
                            # Start calculating behind position early (at 1.5m) for smooth approach
                            if self.search_state == SearchState.APPROACHING:
                                if distance <= 1.5 and self.behind_ball_x is None:
                                    # Early calculation of behind position for planning
                                    print(f"📐 Pre-calculating behind position at {distance:.2f}m...")
                                    ball_x_for_calc = self.estimated_ball_x if self.estimated_ball_x is not None else world_x
                                    ball_y_for_calc = self.estimated_ball_y if self.estimated_ball_y is not None else world_y
                                    self.behind_ball_x, self.behind_ball_y = self.calculate_behind_ball_position(ball_x_for_calc, ball_y_for_calc)
                                    if self.behind_ball_x is not None:
                                        print(f"📍 Target behind position: ({self.behind_ball_x:.2f}, {self.behind_ball_y:.2f})")
                                
                                # Check if we're on correct side before transitioning
                                ball_x_check = self.estimated_ball_x if self.estimated_ball_x is not None else world_x
                                ball_y_check = self.estimated_ball_y if self.estimated_ball_y is not None else world_y
                                on_correct_side = self.is_on_correct_side_of_ball(ball_x_check, ball_y_check)
                                
                                # Only transition to positioning when close AND on correct side
                                if distance <= self.approach_min_distance and on_correct_side:
                                    print(f"✅ Ready to position! Distance: {distance:.2f}m, correct side: {on_correct_side}")
                                    
                                    # Recalculate for final positioning if needed
                                    if self.behind_ball_x is None:
                                        ball_x_for_calc = self.estimated_ball_x if self.estimated_ball_x is not None else world_x
                                        ball_y_for_calc = self.estimated_ball_y if self.estimated_ball_y is not None else world_y
                                        self.behind_ball_x, self.behind_ball_y = self.calculate_behind_ball_position(ball_x_for_calc, ball_y_for_calc)
                                    
                                    if self.behind_ball_x is not None:
                                        print(f"🎯 Moving to final position behind ball")
                                        self.search_state = SearchState.POSITIONING
                                        self.stop_movement()
                                    else:
                                        print("⚠️ Could not calculate behind position")
                                        self.search_state = SearchState.STOPPED
                                        break
                                elif distance <= self.approach_min_distance and not on_correct_side:
                                    # Too close but on wrong side - need to back off and arc around
                                    print(f"⚠️ Too close on wrong side! Need to reposition...")
                                    # Continue in APPROACHING state to handle arcing
                
                else:
                    print(f"⚠️  Camera error: {code}")
                    # Still update visualization without camera data
                    self.update_visualization(None, None)
                
                # Update ball confidence decay if not currently seeing ball
                if not ball_detection:
                    self.decay_ball_confidence()
                
                # Handle different states
                current_time = time.time()
                
                if self.search_state == SearchState.SEARCHING:
                    # Search pattern: turn for 0.7s, pause for 1.5s (slower rotation, more deliberate)
                    if current_time - self.last_turn_time < 0.7:
                        # During the 0.7 second turn duration, keep sending turn commands rapidly
                        self.turn_left()  # Same as pressing 'q' in robot_controller_simple.py
                    elif current_time - self.last_turn_time >= 2.2:
                        # After 2.2 seconds total (0.7s turning + 1.5s pause), start new turn
                        self.last_turn_time = current_time
                        print("↺ Turning left to search...")
                        self.stop_movement()  # Explicitly stop during pause
                    elif current_time - self.last_turn_time >= 0.7:
                        # During pause period (0.7-2.2s), ensure we stopped
                        if current_time - self.last_turn_time < 0.75:  # Only stop once
                            self.stop_movement()
                            
                elif self.search_state == SearchState.APPROACHING:
                    # Approach the ball strategically to get behind it
                    if ball_detection and frame is not None:
                        # Get current distance to ball
                        world_x, world_y, distance = self.pixel_to_world_coordinates(
                            ball_detection['center'][0], 
                            ball_detection['center'][1],
                            frame.shape,
                            ball_detection['radius']
                        )
                        
                        # Update ball estimation for position checking
                        ball_x = self.estimated_ball_x if self.estimated_ball_x is not None else world_x
                        ball_y = self.estimated_ball_y if self.estimated_ball_y is not None else world_y
                        
                        # Check if we're on the correct side of the ball
                        on_correct_side = self.is_on_correct_side_of_ball(ball_x, ball_y)
                        
                        # Calculate angle to ball from center of camera
                        frame_width = frame.shape[1]
                        ball_center_x = ball_detection['center'][0]
                        normalized_x = (ball_center_x - frame_width/2) / (frame_width/2)
                        angle_to_ball = normalized_x * (self.camera_fov_horizontal/2) * (math.pi/180)
                        
                        # Decision logic based on position relative to ball
                        if not on_correct_side and distance < 2.0:
                            # We're on wrong side (between ball and goal) - need to arc around
                            print(f"\n⚠️ On wrong side of ball! Arcing around at {distance:.2f}m...")
                            
                            # Calculate arc waypoint to the side of the ball
                            # Get perpendicular vector to ball-goal line
                            if self.start_x is not None:
                                ball_to_goal_x = self.start_x - ball_x
                                ball_to_goal_y = self.start_y - ball_y
                                dist_to_goal = math.sqrt(ball_to_goal_x**2 + ball_to_goal_y**2)
                                
                                if dist_to_goal > 0.01:
                                    # Normalize and get perpendicular
                                    norm_x = ball_to_goal_x / dist_to_goal
                                    norm_y = ball_to_goal_y / dist_to_goal
                                    
                                    # Perpendicular vector (90 degrees)
                                    perp_x = -norm_y
                                    perp_y = norm_x
                                    
                                    # Choose side based on current robot position
                                    with self.data_lock:
                                        cross_product = (ball_x - self.robot_x) * perp_y - (ball_y - self.robot_y) * perp_x
                                    
                                    if cross_product < 0:
                                        perp_x = -perp_x
                                        perp_y = -perp_y
                                    
                                    # Arc waypoint 1.5m to the side and slightly behind
                                    arc_x = ball_x + perp_x * 1.2 - norm_x * 0.5
                                    arc_y = ball_y + perp_y * 1.2 - norm_y * 0.5
                                    
                                    # Move toward arc waypoint
                                    angle_to_arc = self.calculate_angle_to_position(arc_x, arc_y)
                                    
                                    if abs(angle_to_arc) > 0.2:
                                        # Turn toward arc point
                                        turn_rate = min(self.turn_speed * 0.6, abs(angle_to_arc) * 2.0)
                                        if angle_to_arc < 0:
                                            turn_rate = -turn_rate
                                        ret = self.sport_client.Move(self.walk_speed * 0.3, 0.0, turn_rate)
                                        print(f"🔄 Arcing around ball (angle: {math.degrees(angle_to_arc):.1f}°)", end='\r')
                                    else:
                                        # Move toward arc point
                                        ret = self.sport_client.Move(self.walk_speed * 0.5, 0.0, 0.0)
                                        print(f"➡️ Moving to side of ball", end='\r')
                        
                        # If we're on correct side or far enough, use normal approach
                        elif on_correct_side and 0.8 < distance <= 1.5 and self.behind_ball_x is not None:
                            # We're behind the ball - can approach more directly to final position
                            angle_to_behind = self.calculate_angle_to_position(self.behind_ball_x, self.behind_ball_y)
                            
                            if abs(angle_to_behind) > 0.15:
                                # Turn toward behind position
                                turn_rate = min(self.turn_speed * 0.5, abs(angle_to_behind) * 1.5)
                                if angle_to_behind < 0:
                                    turn_rate = -turn_rate
                                ret = self.sport_client.Move(self.walk_speed * 0.4, 0.0, turn_rate)
                                print(f"✅ Approaching from behind (dist: {distance:.2f}m)", end='\r')
                            else:
                                # Move forward to behind position
                                self.move_forward()
                                print(f"✅ Moving to position (dist: {distance:.2f}m)", end='\r')
                        
                        # Far away - just approach the ball area
                        elif distance > 1.5:
                            # Standard approach when far
                            if abs(angle_to_ball) < 0.1:
                                self.move_forward()
                                print(f"🚶 Approaching ball area (dist: {distance:.2f}m)", end='\r')
                            else:
                                self.turn_toward_ball(angle_to_ball)
                                print(f"🔄 Turning to ball (angle: {math.degrees(angle_to_ball):.1f}°)", end='\r')
                        
                        # Default careful approach
                        else:
                            # Very slow careful approach
                            if abs(angle_to_ball) < 0.1:
                                ret = self.sport_client.Move(self.walk_speed * 0.2, 0.0, 0.0)
                                print(f"🐌 Careful approach (dist: {distance:.2f}m)", end='\r')
                            else:
                                turn_rate = angle_to_ball * -1.0
                                ret = self.sport_client.Move(0.0, 0.0, turn_rate)
                                print(f"🔄 Aligning carefully", end='\r')
                    else:
                        # Lost sight of ball, go back to searching
                        print("\n⚠️ Lost sight of ball! Resuming search...")
                        self.search_state = SearchState.SEARCHING
                        self.last_turn_time = current_time
                        self.ball_found = False  # Reset so we can find it again
                        self.behind_ball_x = None  # Reset behind position
                        self.behind_ball_y = None
                        
                elif self.search_state == SearchState.POSITIONING:
                    # Move to position behind ball
                    if self.behind_ball_x is not None and self.behind_ball_y is not None:
                        with self.data_lock:
                            dist_to_target = math.sqrt((self.behind_ball_x - self.robot_x)**2 + 
                                                      (self.behind_ball_y - self.robot_y)**2)
                        
                        if dist_to_target > 0.3:  # Not at target yet
                            # Calculate angle to behind position
                            angle_to_target = self.calculate_angle_to_position(self.behind_ball_x, self.behind_ball_y)
                            
                            # Turn toward target first
                            if abs(angle_to_target) > 0.2:  # Need to turn
                                turn_rate = min(self.turn_speed, abs(angle_to_target) * 2.0)
                                if angle_to_target < 0:
                                    turn_rate = -turn_rate
                                ret = self.sport_client.Move(0.0, 0.0, turn_rate)
                                print(f"🔄 Turning to behind position (angle: {math.degrees(angle_to_target):.1f}°)", end='\r')
                            else:
                                # Move forward toward target
                                self.move_forward()
                                print(f"🚶 Moving to behind position (distance: {dist_to_target:.2f}m)", end='\r')
                        else:
                            # Reached behind position, now align for pushing
                            print(f"\n✅ Reached position behind ball")
                            self.search_state = SearchState.ALIGNING
                            self.stop_movement()
                            # Note: If this was a realignment, push_attempts has already been incremented
                            
                elif self.search_state == SearchState.ALIGNING:
                    # Align to face the starting position
                    if self.start_x is not None and self.start_y is not None:
                        angle_to_start = self.calculate_angle_to_position(self.start_x, self.start_y)
                        
                        if abs(angle_to_start) > 0.1:  # Need to align
                            turn_rate = min(self.turn_speed * 0.5, abs(angle_to_start))  # Slower, precise turn
                            if angle_to_start < 0:
                                turn_rate = -turn_rate
                            ret = self.sport_client.Move(0.0, 0.0, turn_rate)
                            print(f"🎯 Aligning to push direction (angle: {math.degrees(angle_to_start):.1f}°)", end='\r')
                        else:
                            # Aligned, start pushing
                            print(f"\n✅ Aligned! Starting to push ball to start position")
                            self.search_state = SearchState.PUSHING
                            self.last_realign_time = current_time  # Reset realignment timer
                            
                elif self.search_state == SearchState.PUSHING:
                    # Push the ball toward starting position
                    if ball_detection and frame is not None:
                        # Keep ball centered while pushing
                        frame_width = frame.shape[1]
                        ball_center_x = ball_detection['center'][0]
                        normalized_x = (ball_center_x - frame_width/2) / (frame_width/2)
                        angle_to_ball = normalized_x * (self.camera_fov_horizontal/2) * (math.pi/180)
                        
                        # Check distance to start
                        if self.ball_position and self.start_x is not None:
                            ball_x, ball_y = self.ball_position
                            dist_to_start = math.sqrt((ball_x - self.start_x)**2 + (ball_y - self.start_y)**2)
                            
                            if dist_to_start < 0.3:  # Ball reached start position
                                print(f"\n🎉 SUCCESS! Ball pushed to starting position!")
                                self.search_state = SearchState.STOPPED
                                self.stop_movement()
                                break
                            
                            # Check if it's time to realign (every 3 seconds)
                            if current_time - self.last_realign_time > self.realign_interval:
                                # Update ball estimation for smoother tracking
                                self.update_ball_estimation(ball_x, ball_y)
                                
                                # Recalculate ideal position behind ball (use estimated for stability)
                                est_ball_x = self.estimated_ball_x if self.estimated_ball_x is not None else ball_x
                                est_ball_y = self.estimated_ball_y if self.estimated_ball_y is not None else ball_y
                                new_behind_x, new_behind_y = self.calculate_behind_ball_position(est_ball_x, est_ball_y)
                                
                                if new_behind_x is not None:
                                    # Check if robot has drifted from ideal position
                                    with self.data_lock:
                                        dist_from_ideal = math.sqrt((self.robot_x - new_behind_x)**2 + 
                                                                   (self.robot_y - new_behind_y)**2)
                                    
                                    # Also check angle to goal
                                    angle_to_goal = self.calculate_angle_to_position(self.start_x, self.start_y)
                                    
                                    # Check if ball is significantly off course
                                    # Calculate angle between ball-to-goal vector and robot-to-ball vector
                                    ideal_vec_x = self.start_x - ball_x
                                    ideal_vec_y = self.start_y - ball_y
                                    actual_vec_x = ball_x - self.robot_x
                                    actual_vec_y = ball_y - self.robot_y
                                    
                                    # Normalize vectors and compute angle
                                    ideal_mag = math.sqrt(ideal_vec_x**2 + ideal_vec_y**2)
                                    actual_mag = math.sqrt(actual_vec_x**2 + actual_vec_y**2)
                                    
                                    angle_deviation = 0
                                    if ideal_mag > 0.01 and actual_mag > 0.01:
                                        dot_product = ideal_vec_x * actual_vec_x + ideal_vec_y * actual_vec_y
                                        cos_angle = dot_product / (ideal_mag * actual_mag)
                                        cos_angle = max(-1.0, min(1.0, cos_angle))  # Clamp to [-1, 1]
                                        angle_deviation = math.acos(cos_angle)
                                    
                                    # Decide if realignment is needed
                                    if dist_from_ideal > 0.5 or abs(angle_to_goal) > 0.3 or angle_deviation > math.radians(30):
                                        # Need to reposition
                                        print(f"\n🔄 Realigning behind ball (drift: {dist_from_ideal:.2f}m, angle: {math.degrees(angle_deviation):.1f}°)")
                                        self.behind_ball_x = new_behind_x
                                        self.behind_ball_y = new_behind_y
                                        self.search_state = SearchState.POSITIONING
                                        self.push_attempts += 1
                                        
                                        if self.push_attempts > self.max_push_attempts:
                                            print("\n⚠️ Too many realignment attempts, stopping")
                                            self.search_state = SearchState.STOPPED
                                            break
                                    else:
                                        # Good alignment, continue pushing
                                        self.last_realign_time = current_time
                            
                            # Normal pushing behavior
                            if self.search_state == SearchState.PUSHING:  # Still in pushing state
                                if abs(angle_to_ball) < 0.15:  # Ball is centered
                                    ret = self.sport_client.Move(self.walk_speed * 0.25, 0.0, 0.0)  # Very slow push (0.1 m/s)
                                    print(f"⚽ Pushing ball (distance to goal: {dist_to_start:.2f}m, attempts: {self.push_attempts})", end='\r')
                                else:
                                    # Adjust heading to keep ball centered
                                    turn_rate = angle_to_ball * -1.5  # Slower turn adjustment
                                    ret = self.sport_client.Move(self.walk_speed * 0.15, 0.0, turn_rate)  # Even slower when adjusting
                                    print(f"⚽ Adjusting push angle", end='\r')
                    else:
                        # Lost ball while pushing
                        print("\n⚠️ Lost ball while pushing! Searching...")
                        self.search_state = SearchState.SEARCHING
                        self.last_turn_time = current_time
                        self.ball_found = False
                    
                time.sleep(0.02)  # 50Hz - MUST be fast to send commands continuously! 
                
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
            print("🎮 Ball Retrieval Mission")
            print("=" * 50)
            print("Robot will find the green ball and push it back to starting position.")
            print("")
            print("Press ENTER to start mission, or Ctrl+C to abort")
            
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
    print("🎯 Go2 Ball Retrieval System")
    print("=" * 50)
    print("Autonomous ball retrieval - Push ball back to starting position")
    print("")
    print("Setup Instructions:")
    print("  1. Connect to Go2 robot via ethernet")
    print("  2. Set PC IP to 192.168.123.51/24") 
    print("  3. Robot should be at 192.168.123.18")
    print("  4. Place a large green ball somewhere in the area")
    print("  5. Usage: python3 ball_search.py [network_interface]")
    print("")
    print("Mission Sequence:")
    print("  1. Record starting position as goal")
    print("  2. Search for green ball by turning")
    print("  3. Approach ball when found")
    print("  4. Navigate to position behind ball")
    print("  5. Align toward starting position")
    print("  6. Push ball back to start")
    print("")
    print("Visualization:")
    print("  - Gold circle: Starting position (goal)")
    print("  - Green circle: Ball location")
    print("  - Red X: Target position behind ball")
    print("  - Blue arrow: Robot position and heading")
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