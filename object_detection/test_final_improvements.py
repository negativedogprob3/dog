#!/usr/bin/env python3
"""
Final test of improved close-range ball detection
"""

import cv2
import numpy as np
from ball_tracker import BallDetector

def main():
    print("🎯 Final Test: Improved Close-Range Ball Detection")
    print("=" * 55)
    
    detector = BallDetector()
    print(f"✅ Detector ready - {detector.current_color} mode")
    print(f"📊 HSV range: {detector.hsv_lower} to {detector.hsv_upper}")
    
    # Test different scenarios including close-range
    scenarios = [
        ("distant_ball", create_distant_ball, "Small distant ball"),
        ("medium_ball", create_medium_ball, "Medium distance ball"),
        ("close_ball", create_close_ball, "Close ball (large)"),
        ("very_close_ball", create_very_close_ball, "Very close ball (huge)"),
        ("partial_close", create_partial_close, "Partially visible close ball"),
        ("edge_ball", create_edge_ball, "Ball at frame edge"),
        ("overfilled", create_overfilled, "Overfilled frame (extremely close)"),
    ]
    
    print(f"\n📊 Detection Results:")
    print("Scenario           | Size | Status | Confidence | Issues")
    print("-" * 55)
    
    total_tests = 0
    successful = 0
    
    for scenario_name, create_func, description in scenarios:
        frame, expected_size = create_func()
        detection = detector.detect_ball(frame)
        
        total_tests += 1
        if detection.found:
            successful += 1
        
        status = "✅" if detection.found else "❌"
        conf = f"{detection.confidence:.3f}" if detection.found else "0.000"
        
        # Analyze potential issues
        issues = []
        if expected_size > 300:
            issues.append("huge")
        if "partial" in scenario_name:
            issues.append("partial")
        
        issue_str = ", ".join(issues) if issues else "none"
        
        print(f"{description:18s} | {expected_size:4d} | {status:6s} | {conf:10s} | {issue_str}")
        
        # Save frame for manual inspection
        cv2.imwrite(f"final_test_{scenario_name}.jpg", frame)
    
    # Test different green color modes
    print(f"\n🎨 Testing Different Green Modes:")
    print("Color Mode           | Status | Confidence")
    print("-" * 40)
    
    test_frame, _ = create_close_ball()
    
    for color_mode in ['green', 'bright_green', 'tennis_green', 'low_contrast_green']:
        detector.set_color(color_mode)
        detection = detector.detect_ball(test_frame)
        
        status = "✅" if detection.found else "❌"
        conf = f"{detection.confidence:.3f}" if detection.found else "0.000"
        print(f"{color_mode:20s} | {status:6s} | {conf}")
    
    # Summary
    success_rate = (successful / total_tests) * 100
    print(f"\n📈 Overall Results:")
    print(f"  Tests passed: {successful}/{total_tests} ({success_rate:.1f}%)")
    
    if success_rate >= 85:
        print("✅ Excellent! Close-range detection significantly improved")
    elif success_rate >= 70:
        print("🟡 Good improvement, some edge cases remain")
    else:
        print("⚠️  More improvements needed for close-range detection")
    
    # Print available color modes
    print(f"\n🎨 Available color modes:")
    for color in detector.color_ranges.keys():
        print(f"  - {color}")

def create_distant_ball():
    frame = np.zeros((720, 1280, 3), dtype=np.uint8)
    frame[:, :] = (40, 40, 40)
    cv2.circle(frame, (640, 360), 25, (0, 255, 0), -1)
    return frame, 25

def create_medium_ball():
    frame = np.zeros((720, 1280, 3), dtype=np.uint8)
    frame[:, :] = (40, 40, 40)
    cv2.circle(frame, (640, 360), 80, (0, 255, 0), -1)
    return frame, 80

def create_close_ball():
    frame = np.zeros((720, 1280, 3), dtype=np.uint8)
    frame[:, :] = (40, 40, 40)
    cv2.circle(frame, (640, 360), 180, (0, 255, 0), -1)
    return frame, 180

def create_very_close_ball():
    frame = np.zeros((720, 1280, 3), dtype=np.uint8)
    frame[:, :] = (40, 40, 40)
    cv2.circle(frame, (640, 360), 280, (0, 255, 0), -1)
    return frame, 280

def create_partial_close():
    frame = np.zeros((720, 1280, 3), dtype=np.uint8)
    frame[:, :] = (40, 40, 40)
    cv2.circle(frame, (1100, 360), 250, (0, 255, 0), -1)  # Partially off-screen
    return frame, 250

def create_edge_ball():
    frame = np.zeros((720, 1280, 3), dtype=np.uint8)
    frame[:, :] = (40, 40, 40)
    cv2.circle(frame, (50, 50), 120, (0, 255, 0), -1)  # Corner
    return frame, 120

def create_overfilled():
    frame = np.zeros((720, 1280, 3), dtype=np.uint8)
    frame[:, :] = (40, 40, 40)
    cv2.circle(frame, (640, 360), 450, (0, 255, 0), -1)  # Larger than frame
    return frame, 450

if __name__ == "__main__":
    main()