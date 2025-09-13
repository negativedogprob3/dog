#!/usr/bin/env python3
"""
Ball Distance Estimation from Single Picture

Estimates the distance to a ball using only visual cues from a single image.
Multiple methods are used for robust distance estimation.

Methods:
1. Ball size-based estimation (assuming known ball diameter)
2. Ground plane projection (using camera geometry)  
3. Visual perspective cues
4. Texture/detail analysis
"""

import cv2
import numpy as np
import math
from ball_tracker import BallDetector
from typing import Optional, Tuple, Dict

class BallDistanceEstimator:
    def __init__(self, real_ball_diameter_cm: float = 6.5):
        """
        Initialize distance estimator
        
        Args:
            real_ball_diameter_cm: Actual diameter of the ball in centimeters
                                  (default 6.5cm for standard tennis ball)
        """
        self.detector = BallDetector()
        self.real_ball_diameter = real_ball_diameter_cm  # cm
        
        # Camera parameters (estimated for Go2 camera)
        self.focal_length_px = 800  # Focal length in pixels (estimated)
        self.camera_height_cm = 25  # Camera height from ground (cm)
        self.camera_tilt_deg = -5   # Camera tilt angle (degrees, negative = down)
        
        # Image center
        self.image_center_x = 640  # 1280/2
        self.image_center_y = 360  # 720/2
        
    def estimate_distance_from_size(self, ball_radius_px: float) -> float:
        """
        Estimate distance based on apparent ball size
        
        Formula: distance = (real_diameter * focal_length) / (2 * apparent_radius_px)
        
        Args:
            ball_radius_px: Detected ball radius in pixels
            
        Returns:
            Estimated distance in centimeters
        """
        if ball_radius_px <= 0:
            return -1
            
        # Convert diameter to mm for precision
        real_diameter_mm = self.real_ball_diameter * 10
        
        # Distance calculation
        distance_mm = (real_diameter_mm * self.focal_length_px) / (2 * ball_radius_px)
        distance_cm = distance_mm / 10
        
        return distance_cm
    
    def estimate_distance_from_ground_projection(self, ball_center_y: int, ball_radius_px: float) -> float:
        """
        Estimate distance using ground plane projection
        
        Assumes ball is on the ground and uses camera geometry
        
        Args:
            ball_center_y: Y coordinate of ball center in image
            ball_radius_px: Ball radius for ground contact point
            
        Returns:
            Estimated distance in centimeters
        """
        # Calculate ground contact point (bottom of ball)
        ground_contact_y = ball_center_y + ball_radius_px
        
        # Convert camera tilt to radians
        tilt_rad = math.radians(self.camera_tilt_deg)
        
        # Pixel offset from image center
        y_offset_px = ground_contact_y - self.image_center_y
        
        # Convert to angle (assuming 60-degree vertical field of view)
        vertical_fov_rad = math.radians(60)
        angle_per_pixel = vertical_fov_rad / 720  # Image height
        
        # Angle to ground contact point
        ground_angle_rad = tilt_rad + (y_offset_px * angle_per_pixel)
        
        # Distance calculation using trigonometry
        if abs(ground_angle_rad) < 0.001:  # Avoid division by zero
            return 1000  # Very far
            
        distance_cm = self.camera_height_cm / math.tan(abs(ground_angle_rad))
        
        # Sanity check
        if distance_cm < 0 or distance_cm > 1000:
            return -1
            
        return distance_cm
    
    def estimate_distance_from_perspective(self, ball_center_y: int) -> float:
        """
        Estimate distance based on perspective (where ball appears vertically)
        
        Objects closer to horizon appear farther away
        
        Args:
            ball_center_y: Y coordinate of ball center
            
        Returns:
            Estimated distance in centimeters
        """
        # Horizon line estimation (accounting for camera tilt)
        horizon_y = self.image_center_y + (self.camera_tilt_deg * 10)  # Rough estimate
        
        # Distance from horizon
        horizon_offset = abs(ball_center_y - horizon_y)
        
        # Normalize by image height
        normalized_offset = horizon_offset / 720
        
        # Empirical mapping (closer to bottom = closer to camera)
        if ball_center_y > horizon_y:  # Below horizon = closer
            distance_cm = 50 + (normalized_offset * 200)  # 50-250cm range
        else:  # Above horizon = farther
            distance_cm = 100 + (normalized_offset * 500)  # 100-600cm range
            
        return distance_cm
    
    def analyze_ball_detail_level(self, frame: np.ndarray, ball_center: Tuple[int, int], 
                                 ball_radius: float) -> float:
        """
        Analyze detail level in ball region to estimate distance
        
        Closer balls have more detail/texture variation
        
        Args:
            frame: Input image
            ball_center: (x, y) center of detected ball
            ball_radius: Radius of ball in pixels
            
        Returns:
            Detail score (higher = more detail = closer)
        """
        x, y = ball_center
        radius = int(ball_radius)
        
        # Extract ball region
        x1 = max(0, x - radius)
        y1 = max(0, y - radius)
        x2 = min(frame.shape[1], x + radius)
        y2 = min(frame.shape[0], y + radius)
        
        ball_region = frame[y1:y2, x1:x2]
        
        if ball_region.size == 0:
            return 0
        
        # Convert to grayscale for analysis
        gray = cv2.cvtColor(ball_region, cv2.COLOR_BGR2GRAY)
        
        # Calculate texture measures
        # 1. Standard deviation of pixel values (texture variation)
        texture_std = np.std(gray)
        
        # 2. Laplacian variance (edge detail)
        laplacian_var = cv2.Laplacian(gray, cv2.CV_64F).var()
        
        # 3. Gradient magnitude (overall detail)
        grad_x = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
        grad_y = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
        gradient_mag = np.sqrt(grad_x**2 + grad_y**2)
        avg_gradient = np.mean(gradient_mag)
        
        # Combine measures
        detail_score = (texture_std * 0.4 + laplacian_var * 0.3 + avg_gradient * 0.3)
        
        return detail_score
    
    def estimate_distance_from_detail(self, detail_score: float, ball_radius: float) -> float:
        """
        Convert detail score to distance estimate
        
        Args:
            detail_score: Detail analysis score
            ball_radius: Ball radius in pixels (for context)
            
        Returns:
            Estimated distance in centimeters
        """
        # Normalize detail score by ball size (larger balls naturally have more detail)
        normalized_detail = detail_score / max(1, ball_radius / 50)
        
        # Empirical mapping (higher detail = closer)
        # These values would need calibration with real data
        if normalized_detail > 100:
            return 30  # Very close
        elif normalized_detail > 50:
            return 60  # Close
        elif normalized_detail > 25:
            return 120  # Medium
        elif normalized_detail > 10:
            return 250  # Far
        else:
            return 500  # Very far
    
    def estimate_distance(self, frame: np.ndarray, detection=None) -> Dict[str, float]:
        """
        Comprehensive distance estimation using multiple methods
        
        Args:
            frame: Input image
            detection: Ball detection result (if None, will detect automatically)
            
        Returns:
            Dictionary with distance estimates from different methods
        """
        if detection is None:
            detection = self.detector.detect_ball(frame)
        
        if not detection.found:
            return {"error": "No ball detected"}
        
        x, y = detection.pixel_position
        radius = detection.radius
        
        # Method 1: Size-based estimation
        size_distance = self.estimate_distance_from_size(radius)
        
        # Method 2: Ground projection estimation  
        ground_distance = self.estimate_distance_from_ground_projection(y, radius)
        
        # Method 3: Perspective estimation
        perspective_distance = self.estimate_distance_from_perspective(y)
        
        # Method 4: Detail analysis
        detail_score = self.analyze_ball_detail_level(frame, (x, y), radius)
        detail_distance = self.estimate_distance_from_detail(detail_score, radius)
        
        # Weighted average (size-based is most reliable for known objects)
        valid_estimates = []
        weights = []
        
        if size_distance > 0:
            valid_estimates.append(size_distance)
            weights.append(0.4)  # High weight for size method
            
        if ground_distance > 0:
            valid_estimates.append(ground_distance) 
            weights.append(0.3)  # Medium weight for geometry
            
        if perspective_distance > 0:
            valid_estimates.append(perspective_distance)
            weights.append(0.2)  # Lower weight for perspective
            
        if detail_distance > 0:
            valid_estimates.append(detail_distance)
            weights.append(0.1)  # Lowest weight for detail
        
        # Calculate weighted average
        if valid_estimates:
            final_distance = np.average(valid_estimates, weights=weights[:len(valid_estimates)])
        else:
            final_distance = -1
        
        return {
            "final_distance_cm": final_distance,
            "size_method_cm": size_distance,
            "ground_method_cm": ground_distance,
            "perspective_method_cm": perspective_distance,
            "detail_method_cm": detail_distance,
            "detail_score": detail_score,
            "ball_radius_px": radius,
            "confidence": detection.confidence
        }
    
    def calibrate_with_known_distances(self, calibration_data: list):
        """
        Calibrate estimator with known distance measurements
        
        Args:
            calibration_data: List of (image_path, actual_distance_cm) tuples
        """
        print("🔧 Calibrating distance estimator...")
        
        size_errors = []
        ground_errors = []
        
        for img_path, actual_distance in calibration_data:
            frame = cv2.imread(img_path)
            if frame is None:
                continue
                
            estimates = self.estimate_distance(frame)
            
            if "final_distance_cm" in estimates:
                size_est = estimates["size_method_cm"]
                ground_est = estimates["ground_method_cm"]
                
                if size_est > 0:
                    size_errors.append(abs(size_est - actual_distance))
                if ground_est > 0:
                    ground_errors.append(abs(ground_est - actual_distance))
        
        if size_errors:
            avg_size_error = np.mean(size_errors)
            print(f"   Size method average error: {avg_size_error:.1f}cm")
        
        if ground_errors:
            avg_ground_error = np.mean(ground_errors)
            print(f"   Ground method average error: {avg_ground_error:.1f}cm")

def test_distance_estimation():
    """Test distance estimation with sample images"""
    estimator = BallDistanceEstimator()
    
    print("🎯 Testing Ball Distance Estimation")
    print("=" * 40)
    
    # Test with different simulated ball sizes (representing different distances)
    test_cases = [
        (80, "Very close ball"),
        (50, "Close ball"), 
        (30, "Medium distance ball"),
        (20, "Far ball"),
        (10, "Very far ball")
    ]
    
    for radius, description in test_cases:
        # Create test frame with ball at center
        frame = np.zeros((720, 1280, 3), dtype=np.uint8)
        frame[:, :] = (40, 40, 40)
        center = (640, 360)
        cv2.circle(frame, center, radius, (0, 255, 0), -1)
        
        # Estimate distance
        estimates = estimator.estimate_distance(frame)
        
        print(f"\n{description} (r={radius}px):")
        if "final_distance_cm" in estimates:
            final_dist = estimates["final_distance_cm"]
            size_dist = estimates["size_method_cm"] 
            ground_dist = estimates["ground_method_cm"]
            
            print(f"  📏 Final estimate: {final_dist:.1f} cm")
            print(f"  📊 Size method:   {size_dist:.1f} cm")
            print(f"  🌍 Ground method: {ground_dist:.1f} cm")
            print(f"  👁️  Detail score:  {estimates['detail_score']:.1f}")
        else:
            print("  ❌ Estimation failed")

def main():
    """Test the distance estimation system"""
    print("📏 Ball Distance Estimation System")
    print("=" * 40)
    
    test_distance_estimation()
    
    print(f"\n💡 Usage:")
    print("estimator = BallDistanceEstimator()")
    print("frame = cv2.imread('your_ball_image.jpg')")
    print("distances = estimator.estimate_distance(frame)")
    print("print(f'Ball is {distances[\"final_distance_cm\"]:.1f} cm away')")

if __name__ == "__main__":
    main()