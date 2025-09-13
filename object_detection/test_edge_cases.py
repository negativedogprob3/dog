#!/usr/bin/env python3
"""
Test specific edge cases for close ball detection
"""

import cv2
import numpy as np
from ball_tracker import BallDetector

def test_edge_detection():
    print("🔍 Testing Edge Cases for Close Ball Detection")
    print("=" * 50)
    
    detector = BallDetector()
    
    # Test cases that specifically target edge issues
    test_cases = [
        ("center_full", (640, 360), 100, "Full ball in center"),
        ("right_edge", (1200, 360), 150, "Ball mostly off right edge"), 
        ("left_edge", (80, 360), 150, "Ball mostly off left edge"),
        ("top_edge", (640, 80), 150, "Ball mostly off top edge"),
        ("bottom_edge", (640, 640), 150, "Ball mostly off bottom edge"),
        ("corner", (1200, 640), 180, "Ball in corner"),
        ("huge_center", (640, 360), 350, "Huge ball filling frame"),
        ("low_contrast_edge", (1150, 360), 200, "Low contrast at edge"),
    ]
    
    print("Test Case                | Status | Conf   | Detected Radius | Notes")
    print("-" * 70)
    
    for name, (cx, cy), radius, description in test_cases:
        if "low_contrast" in name:
            frame = create_low_contrast_frame(cx, cy, radius)
        else:
            frame = create_test_frame(cx, cy, radius)
        
        detection = detector.detect_ball(frame)
        
        status = "✅" if detection.found else "❌"
        conf = f"{detection.confidence:.3f}" if detection.found else "0.000"
        det_radius = f"{detection.radius:.0f}" if detection.found else "N/A"
        
        # Calculate how much of ball is visible
        visible_area = calculate_visible_area(cx, cy, radius, frame.shape)
        visibility = f"{visible_area:.0%}"
        
        print(f"{description:24s} | {status:6s} | {conf:6s} | {det_radius:15s} | {visibility} visible")
        
        # Save debug image
        cv2.imwrite(f"debug_edge_{name}.jpg", frame)

def create_test_frame(cx, cy, radius):
    """Create test frame with green ball at specified position"""
    frame = np.zeros((720, 1280, 3), dtype=np.uint8)
    frame[:, :] = (40, 40, 40)  # Dark gray background
    cv2.circle(frame, (cx, cy), radius, (0, 255, 0), -1)
    return frame

def create_low_contrast_frame(cx, cy, radius):
    """Create low contrast test frame"""
    frame = np.zeros((720, 1280, 3), dtype=np.uint8)
    frame[:, :] = (0, 120, 0)  # Green-ish background
    cv2.circle(frame, (cx, cy), radius, (0, 180, 0), -1)  # Slightly brighter green
    return frame

def calculate_visible_area(cx, cy, radius, frame_shape):
    """Calculate what percentage of the ball is visible in frame"""
    height, width = frame_shape[:2]
    
    # Create mask of full circle
    full_mask = np.zeros((height + 2*radius, width + 2*radius), dtype=np.uint8)
    cv2.circle(full_mask, (cx + radius, cy + radius), radius, 255, -1)
    
    # Create mask of visible area
    visible_mask = np.zeros_like(full_mask)
    visible_mask[radius:radius+height, radius:radius+width] = 255
    
    # Calculate intersection
    intersection = cv2.bitwise_and(full_mask, visible_mask)
    
    full_area = cv2.countNonZero(full_mask)
    visible_area = cv2.countNonZero(intersection)
    
    return visible_area / full_area if full_area > 0 else 0

def test_color_variations():
    print(f"\n🎨 Testing Color Variations for Close Balls")
    print("-" * 45)
    
    detector = BallDetector()
    
    # Different green variations that might appear when ball is close
    green_variations = [
        ((0, 255, 0), "Pure green"),
        ((0, 200, 0), "Dark green"), 
        ((50, 255, 50), "Light green"),
        ((0, 255, 100), "Green-cyan"),
        ((20, 255, 20), "Slightly yellow-green"),
        ((0, 180, 0), "Dim green"),
        ((100, 255, 100), "Very light green"),
    ]
    
    for color, name in green_variations:
        frame = np.zeros((720, 1280, 3), dtype=np.uint8)
        frame[:, :] = (30, 30, 30)
        cv2.circle(frame, (640, 360), 150, color, -1)
        
        detection = detector.detect_ball(frame)
        status = "✅" if detection.found else "❌"
        conf = f"{detection.confidence:.3f}" if detection.found else "0.000"
        
        print(f"{name:20s} {color} | {status} | {conf}")

if __name__ == "__main__":
    test_edge_detection()
    test_color_variations()
    
    print(f"\n💡 Key Insights:")
    print("- Check visibility percentages for failed edge cases")
    print("- Low contrast cases need better HSV range handling")  
    print("- Very large balls may need different detection approach")