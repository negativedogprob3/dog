#!/usr/bin/env python3
"""
Test ball detection at different distances/sizes to identify close-range issues
"""

import cv2
import numpy as np
from ball_tracker import BallDetector
import time

def test_ball_sizes():
    print("🔍 Testing Ball Detection at Different Distances/Sizes")
    print("=" * 55)
    
    detector = BallDetector()
    print(f"Current detector limits: min_radius={detector.min_radius}, max_radius={detector.max_radius}")
    print(f"Min contour area: {detector.min_contour_area}")
    
    # Test different ball sizes (simulating distance)
    test_sizes = [
        (10, "Very far"),
        (20, "Far"), 
        (40, "Medium distance"),
        (60, "Close"),
        (80, "Very close"),
        (120, "Extremely close"),
        (180, "Super close"),
        (250, "Touching camera")
    ]
    
    frame_base = np.zeros((720, 1280, 3), dtype=np.uint8)
    frame_base[:, :] = (40, 40, 40)  # Dark background
    
    print(f"\n📊 Detection Results:")
    print("Size | Distance      | Detected | Confidence | Radius | Issues")
    print("-" * 65)
    
    for radius, distance in test_sizes:
        # Create frame with ball of specific size
        frame = frame_base.copy()
        center = (640, 360)  # Center of frame
        cv2.circle(frame, center, radius, (0, 255, 0), -1)  # Solid green ball
        
        # Test detection
        detection = detector.detect_ball(frame)
        
        # Analyze issues
        issues = []
        if radius < detector.min_radius:
            issues.append("too_small")
        if radius > detector.max_radius:
            issues.append("too_large")
        
        # Calculate expected contour area vs actual
        expected_area = np.pi * radius * radius
        if expected_area < detector.min_contour_area:
            issues.append("area_too_small")
        
        status = "✅" if detection.found else "❌"
        conf = f"{detection.confidence:.3f}" if detection.found else "0.000"
        det_radius = f"{detection.radius:.1f}" if detection.found else "N/A"
        issue_str = ", ".join(issues) if issues else "none"
        
        print(f"{radius:4d} | {distance:13s} | {status:8s} | {conf:10s} | {det_radius:6s} | {issue_str}")
        
        # Save problematic frames for inspection
        if not detection.found and radius >= 20:  # Should detect medium+ balls
            cv2.imwrite(f"debug_ball_size_{radius}.jpg", frame)
            print(f"    💾 Saved debug frame: debug_ball_size_{radius}.jpg")

def test_partial_balls():
    """Test balls that are partially cut off (common when very close)"""
    print(f"\n🔍 Testing Partially Visible Balls (Close Range)")
    print("-" * 50)
    
    detector = BallDetector()
    frame_base = np.zeros((720, 1280, 3), dtype=np.uint8)
    frame_base[:, :] = (40, 40, 40)
    
    # Test balls partially outside frame (simulating very close)
    test_cases = [
        ((640, 360), 60, "Center - full ball"),
        ((640, 60), 100, "Top edge - partial"),
        ((60, 360), 120, "Left edge - partial"), 
        ((1220, 360), 120, "Right edge - partial"),
        ((640, 660), 100, "Bottom edge - partial"),
        ((50, 50), 80, "Corner - quarter visible"),
    ]
    
    for (cx, cy), radius, description in test_cases:
        frame = frame_base.copy()
        cv2.circle(frame, (cx, cy), radius, (0, 255, 0), -1)
        
        detection = detector.detect_ball(frame)
        status = "✅" if detection.found else "❌"
        conf = f"{detection.confidence:.3f}" if detection.found else "0.000"
        
        print(f"{description:25s} | {status} | Conf: {conf}")
        
        if not detection.found:
            cv2.imwrite(f"debug_partial_{cx}_{cy}.jpg", frame)

def test_with_improved_settings():
    """Test with adjusted parameters for better close-range detection"""
    print(f"\n🔧 Testing with Improved Settings")
    print("-" * 40)
    
    # Create detector with better close-range settings
    detector = BallDetector()
    
    # Save original settings
    orig_min_radius = detector.min_radius
    orig_max_radius = detector.max_radius  
    orig_min_area = detector.min_contour_area
    
    # Adjust for better close-range detection
    detector.min_radius = 5      # Smaller minimum for distant balls
    detector.max_radius = 400    # Much larger maximum for close balls
    detector.min_contour_area = 100  # Lower area threshold
    
    print(f"Adjusted settings: min_radius={detector.min_radius}, max_radius={detector.max_radius}")
    print(f"Adjusted min_area: {detector.min_contour_area}")
    
    # Re-test problematic sizes
    test_sizes = [(180, "Super close"), (250, "Touching camera"), (320, "Overfilled")]
    
    frame_base = np.zeros((720, 1280, 3), dtype=np.uint8)
    frame_base[:, :] = (40, 40, 40)
    
    for radius, distance in test_sizes:
        frame = frame_base.copy()
        cv2.circle(frame, (640, 360), radius, (0, 255, 0), -1)
        
        detection = detector.detect_ball(frame)
        status = "✅" if detection.found else "❌"
        conf = f"{detection.confidence:.3f}" if detection.found else "0.000"
        det_radius = f"{detection.radius:.1f}" if detection.found else "N/A"
        
        print(f"{distance:15s} (r={radius:3d}) | {status} | Conf: {conf} | Det_R: {det_radius}")
    
    # Restore original settings
    detector.min_radius = orig_min_radius
    detector.max_radius = orig_max_radius
    detector.min_contour_area = orig_min_area

if __name__ == "__main__":
    test_ball_sizes()
    test_partial_balls()
    test_with_improved_settings()
    
    print(f"\n💡 Recommendations:")
    print("1. Increase max_radius for close-range detection")
    print("2. Decrease min_contour_area for better sensitivity")  
    print("3. Handle partial balls better with edge detection")
    print("4. Consider adaptive thresholds based on ball size")