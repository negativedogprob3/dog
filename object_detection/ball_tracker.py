#!/usr/bin/env python3
"""
Ball Tracker for Unitree Go2 Robot

This module provides real-time ball detection and 3D position tracking using the robot's camera
and position data. It combines computer vision for ball detection with coordinate transformations
to provide both ball and robot positions in world coordinates.

Key Features:
- Real-time ball detection using color-based filtering and contour detection
- 3D position estimation using camera intrinsics and robot pose
- Coordinate transformation from camera frame to world frame
- Real-time visualization of both ball and robot positions
- Integration with existing Go2 robot control system
"""

import cv2
import numpy as np
import time
import json
import threading
from typing import Optional, Tuple, Dict, List
from dataclasses import dataclass, asdict
import math

@dataclass
class Position3D:
    """3D position with timestamp"""
    x: float
    y: float
    z: float
    timestamp: float

@dataclass
class BallDetection:
    """Ball detection result"""
    found: bool
    pixel_position: Optional[Tuple[int, int]]  # (x, y) in pixels
    world_position: Optional[Position3D]  # 3D position in world coordinates
    confidence: float  # 0.0 to 1.0
    radius: float  # detected ball radius in pixels

@dataclass
class RobotPose:
    """Robot pose in world coordinates"""
    position: Position3D
    orientation: Tuple[float, float, float]  # (roll, pitch, yaw) in radians
    timestamp: float

class BallDetector:
    """Computer vision-based ball detector"""
    
    def __init__(self):
        # Ball detection parameters (optimized for green ball)
        self.hsv_lower = np.array([40, 100, 100])  # Green lower bound
        self.hsv_upper = np.array([80, 255, 255])  # Green upper bound
        
        # Alternative HSV ranges for different ball colors
        self.color_ranges = {
            'green': (np.array([40, 50, 50]), np.array([80, 255, 255])),  # More inclusive default
            'bright_green': (np.array([45, 150, 150]), np.array([75, 255, 255])),  # Brighter green
            'tennis_green': (np.array([35, 120, 120]), np.array([85, 255, 255])),  # Tennis ball green
            'low_contrast_green': (np.array([35, 30, 30]), np.array([85, 255, 255])),  # Low contrast
            'orange': (np.array([10, 100, 100]), np.array([25, 255, 255])),
            'yellow': (np.array([20, 100, 100]), np.array([30, 255, 255])),
            'blue': (np.array([100, 100, 100]), np.array([130, 255, 255])),
            'red': (np.array([160, 100, 100]), np.array([180, 255, 255]))
        }
        
        # Detection parameters (optimized for green ball visibility)
        self.min_radius = 8   # Slightly smaller minimum for better detection
        self.max_radius = 250  # Larger maximum for close-up detection
        self.min_contour_area = 200  # Lower threshold for better green detection
        
        # Current color mode - defaulting to green with better range
        self.current_color = 'green'
        self.hsv_lower, self.hsv_upper = self.color_ranges[self.current_color]
        
    def set_color(self, color: str):
        """Set the target ball color"""
        if color in self.color_ranges:
            self.current_color = color
            self.hsv_lower, self.hsv_upper = self.color_ranges[color]
            print(f"Ball detection color set to: {color}")
        else:
            print(f"Unknown color: {color}. Available colors: {list(self.color_ranges.keys())}")
    
    def detect_ball(self, frame: np.ndarray) -> BallDetection:
        """
        Detect ball in the given frame with improved close-range handling
        
        Args:
            frame: Input image frame (BGR format)
            
        Returns:
            BallDetection object with detection results
        """
        if frame is None:
            return BallDetection(False, None, None, 0.0, 0.0)
        
        try:
            # Convert to HSV for better color filtering
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # Apply color mask
            mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
            
            # IMPROVEMENT 1: Handle partial balls better
            # Add border padding to help detect balls at frame edges
            border_size = 50
            mask_padded = cv2.copyMakeBorder(mask, border_size, border_size, border_size, border_size, 
                                           cv2.BORDER_CONSTANT, value=0)
            
            # Morphological operations to clean up the mask
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            mask_padded = cv2.morphologyEx(mask_padded, cv2.MORPH_OPEN, kernel)
            mask_padded = cv2.morphologyEx(mask_padded, cv2.MORPH_CLOSE, kernel)
            
            # IMPROVEMENT 2: More aggressive morphological closing for close balls
            if cv2.countNonZero(mask_padded) > frame.shape[0] * frame.shape[1] * 0.1:  # Large green area
                large_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
                mask_padded = cv2.morphologyEx(mask_padded, cv2.MORPH_CLOSE, large_kernel)
            
            # Remove padding for contour detection, but keep coordinates adjusted
            mask_processed = mask_padded[border_size:-border_size, border_size:-border_size]
            
            # Find contours
            contours, _ = cv2.findContours(mask_processed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if not contours:
                return BallDetection(False, None, None, 0.0, 0.0)
            
            # Find the largest contour (assumed to be the ball)
            largest_contour = max(contours, key=cv2.contourArea)
            contour_area = cv2.contourArea(largest_contour)
            
            # IMPROVEMENT 3: Dynamic area threshold based on frame coverage
            frame_area = frame.shape[0] * frame.shape[1]
            coverage_ratio = contour_area / frame_area
            
            # Use adaptive minimum area - lower for large balls that might be partially visible
            adaptive_min_area = self.min_contour_area
            if coverage_ratio > 0.2:  # Large ball covering >20% of frame
                adaptive_min_area = self.min_contour_area * 0.3  # Lower threshold
            
            if contour_area < adaptive_min_area:
                return BallDetection(False, None, None, 0.0, 0.0)
            
            # Get bounding circle
            (x, y), radius = cv2.minEnclosingCircle(largest_contour)
            center = (int(x), int(y))
            radius = int(radius)
            
            # IMPROVEMENT 4: More lenient radius bounds for close balls
            adaptive_max_radius = self.max_radius
            if coverage_ratio > 0.3:  # Very close ball
                adaptive_max_radius = min(frame.shape[0], frame.shape[1]) // 2  # Up to half frame size
            
            if radius < self.min_radius or radius > adaptive_max_radius:
                return BallDetection(False, None, None, 0.0, 0.0)
            
            # IMPROVEMENT 5: Enhanced confidence calculation
            perimeter = cv2.arcLength(largest_contour, True)
            if perimeter == 0:
                circularity = 0
            else:
                circularity = 4 * np.pi * contour_area / (perimeter * perimeter)
            
            # Confidence factors
            circularity_score = min(1.0, circularity * 1.2)  # Boost circularity weight
            size_score = min(1.0, contour_area / 5000)  # Size appropriateness  
            coverage_score = min(1.0, coverage_ratio * 2)  # Reward larger detections
            
            # For very close balls (high coverage), be more lenient on circularity
            if coverage_ratio > 0.25:
                confidence = circularity_score * 0.4 + size_score * 0.3 + coverage_score * 0.3
            else:
                confidence = circularity_score * 0.7 + size_score * 0.2 + coverage_score * 0.1
            
            confidence = min(1.0, confidence)
            
            return BallDetection(
                found=True,
                pixel_position=center,
                world_position=None,  # Will be calculated later
                confidence=confidence,
                radius=float(radius)
            )
            
        except Exception as e:
            print(f"Ball detection error: {e}")
            return BallDetection(False, None, None, 0.0, 0.0)

class CameraCalibration:
    """Camera intrinsic parameters and calibration data"""
    
    def __init__(self):
        # Go2 camera intrinsic parameters (estimated - should be calibrated for accuracy)
        # These are typical values for a wide-angle camera
        self.image_width = 1280
        self.image_height = 720
        
        # Camera matrix (fx, fy are focal lengths, cx, cy are principal points)
        self.fx = 800.0  # Focal length X
        self.fy = 800.0  # Focal length Y  
        self.cx = 640.0  # Principal point X (image_width / 2)
        self.cy = 360.0  # Principal point Y (image_height / 2)
        
        self.camera_matrix = np.array([
            [self.fx, 0, self.cx],
            [0, self.fy, self.cy],
            [0, 0, 1]
        ])
        
        # Distortion coefficients (k1, k2, p1, p2, k3)
        self.dist_coeffs = np.array([0.1, -0.2, 0, 0, 0])
        
        # Camera pose relative to robot base (estimated)
        # Go2 camera is approximately 30cm forward and 25cm up from base center
        self.camera_offset = np.array([0.30, 0.0, 0.25])  # [x_forward, y_left, z_up]
        self.camera_tilt = -0.1  # Camera tilted down slightly (radians)
    
    def pixel_to_camera_ray(self, pixel_x: int, pixel_y: int) -> np.ndarray:
        """
        Convert pixel coordinates to a 3D ray in camera frame
        
        Args:
            pixel_x, pixel_y: Pixel coordinates in image
            
        Returns:
            Normalized ray direction in camera frame
        """
        # Convert pixel to normalized image coordinates
        x_norm = (pixel_x - self.cx) / self.fx
        y_norm = (pixel_y - self.cy) / self.fy
        
        # Create ray direction (assuming z=1)
        ray = np.array([x_norm, y_norm, 1.0])
        
        # Normalize the ray
        ray = ray / np.linalg.norm(ray)
        
        return ray

class CoordinateTransformer:
    """Handles coordinate transformations between camera, robot, and world frames"""
    
    def __init__(self, calibration: CameraCalibration):
        self.calibration = calibration
        
    def camera_to_robot_frame(self, camera_ray: np.ndarray, robot_pose: RobotPose, 
                             ground_height: float = 0.0) -> Optional[Position3D]:
        """
        Transform camera ray to world coordinates assuming ball is on the ground
        
        Args:
            camera_ray: Ray direction in camera frame
            robot_pose: Current robot pose in world frame
            ground_height: Height of the ground plane
            
        Returns:
            3D position in world coordinates, or None if no intersection
        """
        try:
            # Robot orientation (roll, pitch, yaw)
            roll, pitch, yaw = robot_pose.orientation
            
            # Create rotation matrix from robot frame to world frame
            R_world_robot = self._euler_to_rotation_matrix(roll, pitch, yaw)
            
            # Camera rotation relative to robot (accounting for camera tilt)
            camera_pitch = self.calibration.camera_tilt
            R_robot_camera = self._euler_to_rotation_matrix(0, camera_pitch, 0)
            
            # Combined rotation from camera to world
            R_world_camera = R_world_robot @ R_robot_camera
            
            # Transform ray to world frame
            world_ray = R_world_camera @ camera_ray
            
            # Camera position in world frame
            robot_pos = np.array([robot_pose.position.x, robot_pose.position.y, robot_pose.position.z])
            camera_offset_world = R_world_robot @ self.calibration.camera_offset
            camera_pos_world = robot_pos + camera_offset_world
            
            # Intersect ray with ground plane (z = ground_height)
            # Ray equation: P = camera_pos + t * world_ray
            # Ground plane: z = ground_height
            # Solve: camera_pos.z + t * world_ray.z = ground_height
            
            if abs(world_ray[2]) < 1e-6:  # Ray is parallel to ground
                return None
                
            t = (ground_height - camera_pos_world[2]) / world_ray[2]
            
            if t < 0:  # Ball is behind camera
                return None
                
            # Calculate intersection point
            intersection = camera_pos_world + t * world_ray
            
            return Position3D(
                x=float(intersection[0]),
                y=float(intersection[1]),
                z=ground_height,
                timestamp=time.time()
            )
            
        except Exception as e:
            print(f"Coordinate transformation error: {e}")
            return None
    
    def _euler_to_rotation_matrix(self, roll: float, pitch: float, yaw: float) -> np.ndarray:
        """Convert Euler angles to rotation matrix"""
        # Rotation around X (roll)
        R_x = np.array([
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)]
        ])
        
        # Rotation around Y (pitch)
        R_y = np.array([
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)]
        ])
        
        # Rotation around Z (yaw)
        R_z = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])
        
        # Combined rotation (Z * Y * X)
        return R_z @ R_y @ R_x

class BallTracker:
    """Main ball tracking system that combines detection and position estimation"""
    
    def __init__(self, robot_controller=None):
        self.detector = BallDetector()
        self.calibration = CameraCalibration()
        self.transformer = CoordinateTransformer(self.calibration)
        self.robot_controller = robot_controller
        
        # State tracking
        self.last_detection = None
        self.detection_history = []
        self.max_history = 10
        
        # Threading for continuous tracking
        self.tracking = False
        self.track_thread = None
        
    def get_robot_pose(self) -> Optional[RobotPose]:
        """Get current robot pose from robot controller"""
        if self.robot_controller is None:
            # Return mock pose for testing
            return RobotPose(
                position=Position3D(0.0, 0.0, 0.3, time.time()),  # 30cm high
                orientation=(0.0, 0.0, 0.0),  # No rotation
                timestamp=time.time()
            )
            
        try:
            robot_state = self.robot_controller.state
            
            # Convert robot state to our format
            robot_pose = RobotPose(
                position=Position3D(
                    x=robot_state.position["x"],
                    y=robot_state.position["y"], 
                    z=robot_state.position["z"],
                    timestamp=robot_state.timestamp
                ),
                orientation=(
                    math.radians(robot_state.orientation["roll"]),
                    math.radians(robot_state.orientation["pitch"]),
                    math.radians(robot_state.orientation["yaw"])
                ),
                timestamp=robot_state.timestamp
            )
            
            return robot_pose
            
        except Exception as e:
            print(f"Error getting robot pose: {e}")
            return None
    
    def track_frame(self, frame: np.ndarray) -> BallDetection:
        """
        Process a single frame and track the ball
        
        Args:
            frame: Input camera frame
            
        Returns:
            BallDetection with complete position information
        """
        # Detect ball in image
        detection = self.detector.detect_ball(frame)
        
        if not detection.found:
            self.last_detection = detection
            return detection
            
        # Get robot pose
        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            print("Warning: Could not get robot pose")
            self.last_detection = detection
            return detection
        
        # Convert pixel position to world coordinates
        pixel_x, pixel_y = detection.pixel_position
        camera_ray = self.calibration.pixel_to_camera_ray(pixel_x, pixel_y)
        world_position = self.transformer.camera_to_robot_frame(camera_ray, robot_pose)
        
        # Update detection with world position
        detection.world_position = world_position
        
        # Store in history
        self.detection_history.append(detection)
        if len(self.detection_history) > self.max_history:
            self.detection_history.pop(0)
        
        self.last_detection = detection
        return detection
    
    def draw_detection(self, frame: np.ndarray, detection: BallDetection, 
                      robot_pose: Optional[RobotPose] = None) -> np.ndarray:
        """
        Draw detection results on the frame
        
        Args:
            frame: Input image frame
            detection: Ball detection results
            robot_pose: Current robot pose (optional)
            
        Returns:
            Annotated frame
        """
        annotated_frame = frame.copy()
        
        if detection.found and detection.pixel_position:
            x, y = detection.pixel_position
            radius = int(detection.radius)
            
            # Draw detection circle
            color = (0, 255, 0) if detection.confidence > 0.5 else (0, 255, 255)
            cv2.circle(annotated_frame, (x, y), radius, color, 2)
            cv2.circle(annotated_frame, (x, y), 3, color, -1)
            
            # Draw confidence text
            conf_text = f"Confidence: {detection.confidence:.2f}"
            cv2.putText(annotated_frame, conf_text, (x - 50, y - radius - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            
            # Draw world position if available
            if detection.world_position:
                pos = detection.world_position
                pos_text = f"World: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})"
                cv2.putText(annotated_frame, pos_text, (x - 100, y + radius + 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        # Draw robot position info
        if robot_pose:
            robot_text = f"Robot: ({robot_pose.position.x:.2f}, {robot_pose.position.y:.2f})"
            cv2.putText(annotated_frame, robot_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            yaw_deg = math.degrees(robot_pose.orientation[2])
            orientation_text = f"Yaw: {yaw_deg:.1f}°"
            cv2.putText(annotated_frame, orientation_text, (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # Draw current color mode
        color_text = f"Tracking: {self.detector.current_color} ball"
        cv2.putText(annotated_frame, color_text, (10, annotated_frame.shape[0] - 50),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # Draw instructions
        instructions = "Keys: q=quit, c=change color, r=reset, s=save"
        cv2.putText(annotated_frame, instructions, (10, annotated_frame.shape[0] - 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        return annotated_frame
    
    def get_positions_summary(self) -> Dict:
        """Get current positions of both ball and robot"""
        robot_pose = self.get_robot_pose()
        
        summary = {
            "timestamp": time.time(),
            "robot": {
                "position": {
                    "x": robot_pose.position.x if robot_pose else 0.0,
                    "y": robot_pose.position.y if robot_pose else 0.0,
                    "z": robot_pose.position.z if robot_pose else 0.0
                },
                "orientation": {
                    "yaw_degrees": math.degrees(robot_pose.orientation[2]) if robot_pose else 0.0
                }
            },
            "ball": {
                "detected": self.last_detection.found if self.last_detection else False,
                "confidence": self.last_detection.confidence if self.last_detection else 0.0,
                "position": None
            }
        }
        
        if (self.last_detection and self.last_detection.found and 
            self.last_detection.world_position):
            pos = self.last_detection.world_position
            summary["ball"]["position"] = {
                "x": pos.x,
                "y": pos.y,
                "z": pos.z
            }
            
        return summary
    
    def save_detection_data(self, filename: str = None):
        """Save current detection data to file"""
        if filename is None:
            filename = f"ball_detection_{int(time.time())}.json"
            
        data = {
            "summary": self.get_positions_summary(),
            "detection_history": [asdict(d) for d in self.detection_history[-5:]]  # Last 5 detections
        }
        
        try:
            with open(filename, 'w') as f:
                json.dump(data, f, indent=2, default=str)
            print(f"Detection data saved to: {filename}")
        except Exception as e:
            print(f"Error saving data: {e}")

# Example usage and testing
if __name__ == "__main__":
    print("Ball Tracker Test - Press 'q' to quit")
    
    # Create tracker (without robot controller for testing)
    tracker = BallTracker()
    
    # Try to connect to camera (mock for testing)
    try:
        # This would normally connect to the Go2 camera
        # For testing, we'll use webcam (index 0) or video file
        cap = cv2.VideoCapture(0)
        
        if not cap.isOpened():
            print("Could not open camera. Testing with mock data.")
            cap = None
    except:
        cap = None
        print("Camera not available. Testing with mock data.")
    
    frame_count = 0
    
    while True:
        if cap is not None:
            ret, frame = cap.read()
            if not ret:
                print("Could not read frame")
                break
        else:
            # Create mock frame for testing
            frame = np.zeros((720, 1280, 3), dtype=np.uint8)
            cv2.circle(frame, (640, 360), 50, (0, 165, 255), -1)  # Orange circle
            
        # Process frame
        detection = tracker.track_frame(frame)
        robot_pose = tracker.get_robot_pose()
        
        # Draw results
        annotated_frame = tracker.draw_detection(frame, detection, robot_pose)
        
        # Show frame
        cv2.imshow('Ball Tracker', annotated_frame)
        
        # Handle key presses
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('c'):
            # Cycle through colors
            colors = list(tracker.detector.color_ranges.keys())
            current_idx = colors.index(tracker.detector.current_color)
            next_idx = (current_idx + 1) % len(colors)
            tracker.detector.set_color(colors[next_idx])
        elif key == ord('r'):
            # Reset detection history
            tracker.detection_history = []
            print("Detection history reset")
        elif key == ord('s'):
            # Save detection data
            tracker.save_detection_data()
        
        # Print status periodically
        frame_count += 1
        if frame_count % 30 == 0:
            summary = tracker.get_positions_summary()
            print(f"Frame {frame_count}: Ball detected: {summary['ball']['detected']}")
            if summary['ball']['detected']:
                ball_pos = summary['ball']['position']
                if ball_pos:
                    print(f"  Ball position: ({ball_pos['x']:.2f}, {ball_pos['y']:.2f}, {ball_pos['z']:.2f})")
    
    # Cleanup
    if cap is not None:
        cap.release()
    cv2.destroyAllWindows()
    print("Ball tracker test completed")