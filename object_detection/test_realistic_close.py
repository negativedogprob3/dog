#!/usr/bin/env python3
"""
Test realistic close-range scenarios that might cause detection failures
"""

import cv2
import numpy as np
from ball_tracker import BallDetector

def test_realistic_issues():
    print("🔍 Testing Realistic Close-Range Issues")
    print("=" * 45)
    
    detector = BallDetector()
    
    # Test scenarios that commonly fail in real conditions
    test_scenarios = [
        ("perfect_close", create_perfect_close_ball),
        ("blurry_close", create_blurry_close_ball),
        ("partial_frame", create_partial_frame_ball),
        ("low_contrast", create_low_contrast_ball),
        ("oversaturated", create_oversaturated_ball),
        ("with_shadows", create_shadowed_ball),
        ("distorted_oval", create_distorted_ball),
    ]
    
    print("Scenario          | Status | Conf   | Issues")
    print("-" * 45)
    
    for name, create_func in test_scenarios:
        frame = create_func()
        detection = detector.detect_ball(frame)
        
        status = "✅" if detection.found else "❌"
        conf = f"{detection.confidence:.3f}" if detection.found else "0.000"
        
        # Analyze why it might fail
        issues = analyze_frame(frame, detection)
        issue_str = ", ".join(issues) if issues else "none"
        
        print(f"{name:17s} | {status:6s} | {conf:6s} | {issue_str}")
        
        # Save frame for inspection
        cv2.imwrite(f"debug_{name}.jpg", frame)
        
        # Also save HSV version for color analysis
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        cv2.imwrite(f"debug_{name}_hsv.jpg", hsv)

def create_perfect_close_ball():
    """Perfect large green ball"""
    frame = np.zeros((720, 1280, 3), dtype=np.uint8)
    frame[:, :] = (40, 40, 40)
    cv2.circle(frame, (640, 360), 200, (0, 255, 0), -1)
    return frame

def create_blurry_close_ball():
    """Blurred ball (common when camera is too close)"""
    frame = create_perfect_close_ball()
    # Apply strong blur
    frame = cv2.GaussianBlur(frame, (21, 21), 10)
    return frame

def create_partial_frame_ball():
    """Ball that extends beyond frame edges"""
    frame = np.zeros((720, 1280, 3), dtype=np.uint8)
    frame[:, :] = (40, 40, 40)
    # Ball center outside frame, only part visible
    cv2.circle(frame, (1100, 360), 300, (0, 255, 0), -1)
    return frame

def create_low_contrast_ball():
    """Green ball on similar background"""
    frame = np.zeros((720, 1280, 3), dtype=np.uint8)
    frame[:, :] = (0, 100, 0)  # Dark green background
    cv2.circle(frame, (640, 360), 180, (0, 200, 0), -1)  # Slightly brighter green
    return frame

def create_oversaturated_ball():
    """Very bright/oversaturated ball"""
    frame = np.zeros((720, 1280, 3), dtype=np.uint8)
    frame[:, :] = (20, 20, 20)
    cv2.circle(frame, (640, 360), 150, (100, 255, 100), -1)  # Very bright
    return frame

def create_shadowed_ball():
    """Ball with shadows/lighting variations"""
    frame = np.zeros((720, 1280, 3), dtype=np.uint8)
    frame[:, :] = (40, 40, 40)
    
    # Create ball with gradient (simulating 3D lighting)
    center = (640, 360)
    radius = 160
    
    # Draw multiple circles with varying intensities
    for r in range(radius, 0, -5):
        intensity = int(255 * (r / radius) * 0.8 + 50)
        cv2.circle(frame, center, r, (0, intensity, 0), -1)
    
    # Add shadow
    cv2.ellipse(frame, (680, 400), (60, 30), 45, 0, 360, (20, 80, 20), -1)
    
    return frame

def create_distorted_ball():
    """Oval/distorted ball (perspective distortion when close)"""
    frame = np.zeros((720, 1280, 3), dtype=np.uint8)
    frame[:, :] = (40, 40, 40)
    
    # Draw ellipse instead of circle
    cv2.ellipse(frame, (640, 360), (220, 120), 30, 0, 360, (0, 255, 0), -1)
    return frame

def analyze_frame(frame, detection):
    """Analyze frame to understand why detection might fail"""
    issues = []
    
    # Convert to HSV for analysis
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Check color distribution
    detector = BallDetector()
    mask = cv2.inRange(hsv, detector.hsv_lower, detector.hsv_upper)
    green_pixels = cv2.countNonZero(mask)
    total_pixels = frame.shape[0] * frame.shape[1]
    green_ratio = green_pixels / total_pixels
    
    if green_ratio < 0.01:
        issues.append("low_green_content")
    elif green_ratio > 0.5:
        issues.append("too_much_green")
    
    # Check for blur (using Laplacian variance)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    laplacian_var = cv2.Laplacian(gray, cv2.CV_64F).var()
    if laplacian_var < 100:
        issues.append("blurry")
    
    # Check contour complexity
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        perimeter = cv2.arcLength(largest_contour, True)
        
        if perimeter > 0:
            circularity = 4 * np.pi * area / (perimeter * perimeter)
            if circularity < 0.5:
                issues.append("non_circular")
    
    return issues

if __name__ == "__main__":
    test_realistic_issues()
    
    print(f"\n📁 Debug images saved with prefixes:")
    print("  debug_*.jpg - Original frames")
    print("  debug_*_hsv.jpg - HSV analysis")
    print(f"\n🔍 Check the debug images to see detection issues visually")