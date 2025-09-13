#!/usr/bin/env python3
"""
Ball Tracker Demo - Integration with Go2 Robot Server

This demo script integrates the ball tracking system with the existing Go2 robot server
to provide real-time ball detection and position tracking with robot control capabilities.

Usage:
    python3 ball_tracker_demo.py [--mock-robot] [--save-data]

Features:
- Real-time ball detection using robot's camera
- 3D position estimation in world coordinates  
- Live visualization with robot and ball positions
- Integration with robot control system
- Data logging and export capabilities
"""

import sys
import os
import argparse
import time
import json
import cv2
import numpy as np
from typing import Optional, Dict

# Add parent directory to path to import robot controller
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    from mike.server import Go2RobotController, DogServer
    ROBOT_AVAILABLE = True
    print("✅ Robot controller imported successfully")
except ImportError as e:
    ROBOT_AVAILABLE = False
    print(f"⚠️  Robot controller not available: {e}")

from ball_tracker import BallTracker, BallDetection, RobotPose, Position3D

class BallTrackingDemo:
    """Complete demo system for ball tracking with robot integration"""
    
    def __init__(self, use_mock_robot: bool = False, save_data: bool = False):
        self.use_mock_robot = use_mock_robot
        self.save_data = save_data
        
        # Initialize robot controller
        self.robot_controller = None
        if ROBOT_AVAILABLE and not use_mock_robot:
            try:
                self.robot_controller = Go2RobotController()
                print("🤖 Attempting to connect to real robot...")
                if self.robot_controller.connect():
                    print("✅ Connected to Go2 robot")
                else:
                    print("⚠️  Could not connect to robot, using mock mode")
                    self.robot_controller = None
            except Exception as e:
                print(f"⚠️  Robot connection failed: {e}, using mock mode")
                self.robot_controller = None
        else:
            print("🤖 Using mock robot mode")
        
        # Initialize ball tracker
        self.tracker = BallTracker(self.robot_controller)
        
        # Data logging
        self.logged_data = []
        self.log_interval = 1.0  # Log every 1 second
        self.last_log_time = 0
        
        # Camera setup
        self.camera_source = None
        self.setup_camera()
        
        # Display settings
        self.show_debug_info = True
        self.show_detection_mask = False
        
    def setup_camera(self):
        """Setup camera source - robot camera or fallback"""
        if self.robot_controller and hasattr(self.robot_controller, 'get_camera_frame'):
            print("📹 Using robot camera")
            self.camera_source = "robot"
        else:
            print("📹 Attempting to use system camera as fallback")
            try:
                cap = cv2.VideoCapture(0)
                if cap.isOpened():
                    self.camera_source = cap
                    print("✅ System camera connected")
                else:
                    self.camera_source = "mock"
                    print("⚠️  No camera available, using mock frames")
            except:
                self.camera_source = "mock"
                print("⚠️  Camera setup failed, using mock frames")
    
    def get_frame(self) -> Optional[np.ndarray]:
        """Get current camera frame from available source"""
        if self.camera_source == "robot" and self.robot_controller:
            return self.robot_controller.get_camera_frame()
        elif isinstance(self.camera_source, cv2.VideoCapture):
            ret, frame = self.camera_source.read()
            return frame if ret else None
        elif self.camera_source == "mock":
            # Create mock frame with moving ball for demo
            frame = np.zeros((720, 1280, 3), dtype=np.uint8)
            
            # Add some background pattern
            cv2.rectangle(frame, (0, 600), (1280, 720), (20, 20, 20), -1)  # Ground
            cv2.rectangle(frame, (0, 0), (1280, 200), (100, 150, 200), -1)  # Sky
            
            # Add moving green ball
            t = time.time()
            ball_x = int(640 + 200 * np.sin(t * 0.5))  # Move left-right
            ball_y = int(400 + 50 * np.sin(t * 0.8))   # Move up-down slightly
            cv2.circle(frame, (ball_x, ball_y), 40, (0, 255, 0), -1)  # Green ball
            
            # Add some noise for realism
            noise = np.random.randint(0, 20, frame.shape, dtype=np.uint8)
            frame = cv2.add(frame, noise)
            
            return frame
        else:
            return None
    
    def create_debug_overlay(self, frame: np.ndarray, detection: BallDetection, 
                           robot_pose: Optional[RobotPose] = None) -> np.ndarray:
        """Create enhanced debug overlay with tracking information"""
        overlay = frame.copy()
        
        # Add semi-transparent info panel
        panel_height = 200
        panel = np.zeros((panel_height, frame.shape[1], 3), dtype=np.uint8)
        panel[:, :] = (0, 0, 0)  # Black background
        
        # Info text
        y_offset = 25
        line_height = 25
        
        # Robot information
        if robot_pose:
            robot_text = f"🤖 Robot Position: ({robot_pose.position.x:.2f}, {robot_pose.position.y:.2f}, {robot_pose.position.z:.2f})"
            cv2.putText(panel, robot_text, (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            y_offset += line_height
            
            yaw_deg = np.degrees(robot_pose.orientation[2])
            orientation_text = f"🧭 Robot Orientation: {yaw_deg:.1f}° (yaw)"
            cv2.putText(panel, orientation_text, (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            y_offset += line_height
        
        # Ball detection information
        if detection.found:
            ball_text = f"⚽ Ball Detected: Confidence {detection.confidence:.2f}, Radius {detection.radius:.1f}px"
            cv2.putText(panel, ball_text, (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
            y_offset += line_height
            
            if detection.pixel_position:
                pixel_text = f"📷 Pixel Position: ({detection.pixel_position[0]}, {detection.pixel_position[1]})"
                cv2.putText(panel, pixel_text, (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
                y_offset += line_height
            
            if detection.world_position:
                world_text = f"🌍 World Position: ({detection.world_position.x:.2f}, {detection.world_position.y:.2f}, {detection.world_position.z:.2f})"
                cv2.putText(panel, world_text, (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
                y_offset += line_height
                
                # Calculate distance from robot
                if robot_pose:
                    dx = detection.world_position.x - robot_pose.position.x
                    dy = detection.world_position.y - robot_pose.position.y
                    distance = np.sqrt(dx*dx + dy*dy)
                    distance_text = f"📏 Distance to Ball: {distance:.2f}m"
                    cv2.putText(panel, distance_text, (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
                    y_offset += line_height
        else:
            no_ball_text = "❌ No Ball Detected"
            cv2.putText(panel, no_ball_text, (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)
            y_offset += line_height
        
        # Add panel to frame
        overlay[frame.shape[0]-panel_height:, :] = panel
        
        # Add transparency
        alpha = 0.8
        result = cv2.addWeighted(overlay, alpha, frame, 1-alpha, 0)
        
        return result
    
    def create_detection_mask_view(self, frame: np.ndarray) -> np.ndarray:
        """Create visualization showing the color detection mask"""
        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Apply color mask using current tracker settings
        mask = cv2.inRange(hsv, self.tracker.detector.hsv_lower, self.tracker.detector.hsv_upper)
        
        # Convert mask to 3-channel for display
        mask_colored = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        
        # Create side-by-side view
        combined = np.hstack([frame, mask_colored])
        
        # Add labels
        cv2.putText(combined, "Original", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(combined, "Detection Mask", (frame.shape[1] + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        return combined
    
    def log_tracking_data(self, detection: BallDetection, robot_pose: Optional[RobotPose]):
        """Log tracking data for analysis"""
        if not self.save_data:
            return
            
        current_time = time.time()
        if current_time - self.last_log_time < self.log_interval:
            return
            
        # Prepare log entry
        log_entry = {
            "timestamp": current_time,
            "robot_pose": {
                "position": {
                    "x": robot_pose.position.x if robot_pose else 0,
                    "y": robot_pose.position.y if robot_pose else 0, 
                    "z": robot_pose.position.z if robot_pose else 0
                },
                "orientation": list(robot_pose.orientation) if robot_pose else [0, 0, 0]
            } if robot_pose else None,
            "ball_detection": {
                "found": detection.found,
                "confidence": detection.confidence,
                "pixel_position": list(detection.pixel_position) if detection.pixel_position else None,
                "world_position": {
                    "x": detection.world_position.x,
                    "y": detection.world_position.y,
                    "z": detection.world_position.z
                } if detection.world_position else None,
                "radius": detection.radius
            }
        }
        
        self.logged_data.append(log_entry)
        self.last_log_time = current_time
        
        # Save periodically (every 50 entries)
        if len(self.logged_data) % 50 == 0:
            self.save_logged_data()
    
    def save_logged_data(self):
        """Save logged data to file"""
        if not self.logged_data:
            return
            
        filename = f"ball_tracking_log_{int(time.time())}.json"
        filepath = os.path.join("object_detection", filename)
        
        try:
            with open(filepath, 'w') as f:
                json.dump(self.logged_data, f, indent=2)
            print(f"📊 Saved {len(self.logged_data)} tracking data points to {filepath}")
        except Exception as e:
            print(f"❌ Error saving log data: {e}")
    
    def run_demo(self):
        """Run the main demo loop"""
        print("\n🎯 Ball Tracking Demo Starting...")
        print("=" * 50)
        print("Controls:")
        print("  q - Quit")
        print("  c - Change ball color")
        print("  d - Toggle debug info")
        print("  m - Toggle mask view") 
        print("  r - Reset tracking")
        print("  s - Save current data")
        print("  SPACE - Pause/Resume")
        print("=" * 50)
        
        frame_count = 0
        paused = False
        fps_timer = time.time()
        fps_counter = 0
        
        try:
            while True:
                if not paused:
                    # Get camera frame
                    frame = self.get_frame()
                    if frame is None:
                        print("❌ Could not get camera frame")
                        break
                    
                    # Process frame for ball tracking
                    detection = self.tracker.track_frame(frame)
                    robot_pose = self.tracker.get_robot_pose()
                    
                    # Log data if enabled
                    self.log_tracking_data(detection, robot_pose)
                    
                    # Create visualization
                    if self.show_detection_mask:
                        display_frame = self.create_detection_mask_view(frame)
                    else:
                        display_frame = self.tracker.draw_detection(frame, detection, robot_pose)
                        
                        # Add debug overlay if enabled
                        if self.show_debug_info:
                            display_frame = self.create_debug_overlay(display_frame, detection, robot_pose)
                    
                    # Calculate FPS
                    fps_counter += 1
                    if time.time() - fps_timer >= 1.0:
                        fps = fps_counter / (time.time() - fps_timer)
                        fps_counter = 0
                        fps_timer = time.time()
                        
                        # Update window title with FPS
                        cv2.setWindowProperty('Ball Tracking Demo', cv2.WND_PROP_TOPMOST, 0)
                    
                    # Add FPS counter to frame
                    cv2.putText(display_frame, f"FPS: {fps:.1f}", (display_frame.shape[1] - 120, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                    
                    # Add pause indicator
                    if paused:
                        cv2.putText(display_frame, "PAUSED", (display_frame.shape[1]//2 - 80, 60),
                                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 3)
                    
                    frame_count += 1
                else:
                    # Just get the last frame for display when paused
                    frame = self.get_frame()
                    if frame is not None:
                        display_frame = frame.copy()
                        cv2.putText(display_frame, "PAUSED - Press SPACE to resume", 
                                   (display_frame.shape[1]//2 - 200, display_frame.shape[0]//2),
                                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                
                # Display frame
                cv2.imshow('Ball Tracking Demo', display_frame)
                
                # Handle keyboard input
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    print("👋 Quitting demo...")
                    break
                elif key == ord('c'):
                    # Cycle through ball colors
                    colors = list(self.tracker.detector.color_ranges.keys())
                    current_idx = colors.index(self.tracker.detector.current_color)
                    next_idx = (current_idx + 1) % len(colors)
                    self.tracker.detector.set_color(colors[next_idx])
                    print(f"🎨 Switched to tracking {colors[next_idx]} ball")
                elif key == ord('d'):
                    self.show_debug_info = not self.show_debug_info
                    print(f"🐛 Debug info: {'ON' if self.show_debug_info else 'OFF'}")
                elif key == ord('m'):
                    self.show_detection_mask = not self.show_detection_mask
                    print(f"👁️  Mask view: {'ON' if self.show_detection_mask else 'OFF'}")
                elif key == ord('r'):
                    self.tracker.detection_history = []
                    print("🔄 Tracking history reset")
                elif key == ord('s'):
                    self.tracker.save_detection_data()
                    if self.save_data:
                        self.save_logged_data()
                elif key == ord(' '):  # Space bar
                    paused = not paused
                    print(f"⏸️  {'Paused' if paused else 'Resumed'}")
                
                # Print periodic status
                if frame_count % 60 == 0 and not paused:  # Every ~2 seconds at 30fps
                    summary = self.tracker.get_positions_summary()
                    print(f"📊 Frame {frame_count}: Ball detected: {summary['ball']['detected']}")
                    
                    if summary['ball']['detected'] and summary['ball']['position']:
                        ball_pos = summary['ball']['position']
                        robot_pos = summary['robot']['position']
                        print(f"   🤖 Robot: ({robot_pos['x']:.2f}, {robot_pos['y']:.2f})")
                        print(f"   ⚽ Ball:  ({ball_pos['x']:.2f}, {ball_pos['y']:.2f})")
                        
                        # Calculate distance
                        dx = ball_pos['x'] - robot_pos['x']
                        dy = ball_pos['y'] - robot_pos['y']
                        distance = np.sqrt(dx*dx + dy*dy)
                        print(f"   📏 Distance: {distance:.2f}m")
                
        except KeyboardInterrupt:
            print("\n⚠️  Demo interrupted by user")
        except Exception as e:
            print(f"❌ Demo error: {e}")
        finally:
            # Cleanup
            if isinstance(self.camera_source, cv2.VideoCapture):
                self.camera_source.release()
            cv2.destroyAllWindows()
            
            # Save final data
            if self.save_data and self.logged_data:
                self.save_logged_data()
                print(f"📊 Demo completed. Total frames processed: {frame_count}")
                print(f"📝 Total tracking data points: {len(self.logged_data)}")
        
        print("✅ Ball tracking demo completed successfully")

def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(description="Ball Tracking Demo with Go2 Robot")
    parser.add_argument("--mock-robot", action="store_true", 
                       help="Use mock robot data instead of real robot connection")
    parser.add_argument("--save-data", action="store_true",
                       help="Enable data logging and save tracking data")
    parser.add_argument("--no-gui", action="store_true",
                       help="Run without GUI (headless mode)")
    
    args = parser.parse_args()
    
    # Print startup info
    print("🎯 Ball Tracking Demo for Unitree Go2")
    print("=" * 50)
    print(f"🤖 Robot mode: {'Mock' if args.mock_robot else 'Real'}")
    print(f"📊 Data logging: {'Enabled' if args.save_data else 'Disabled'}")
    print(f"🖥️  GUI mode: {'Headless' if args.no_gui else 'Interactive'}")
    print("=" * 50)
    
    if args.no_gui:
        print("❌ Headless mode not yet implemented")
        return
    
    # Create and run demo
    demo = BallTrackingDemo(
        use_mock_robot=args.mock_robot,
        save_data=args.save_data
    )
    
    demo.run_demo()

if __name__ == "__main__":
    main()