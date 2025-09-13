#!/usr/bin/env python3
"""
Debug Middle Distance Detection Issues

Analyzes why ball detection fails at middle distances but works at close/far ranges.
Common causes:
1. Confidence threshold too high/low for medium sizes
2. Morphological operations removing medium-sized features  
3. HSV ranges not optimized for medium lighting conditions
4. Contour area thresholds inappropriate for medium balls
"""

import cv2
import numpy as np
from ball_tracker import BallDetector
# import matplotlib.pyplot as plt  # Not needed for this analysis

def test_distance_ranges():
    """Test detection across different distance ranges"""
    print("🔍 Testing Ball Detection Across Distance Ranges")
    print("=" * 50)
    
    detector = BallDetector()
    
    # Simulate different distances with ball sizes
    distance_tests = [
        # (radius, distance_name, expected_real_distance_cm)
        (5, "Very Far", 500),
        (10, "Far", 300),
        (15, "Medium-Far", 200),
        (25, "Medium", 120),     # PROBLEM AREA
        (35, "Medium-Close", 80), # PROBLEM AREA  
        (50, "Close", 50),
        (80, "Very Close", 30),
        (120, "Extremely Close", 20)
    ]
    
    print("Distance Range      | Radius | Status | Confidence | Issues")
    print("-" * 60)
    
    failed_ranges = []
    
    for radius, distance_name, real_distance in distance_tests:
        # Create test frame
        frame = create_test_frame(radius)
        
        # Test detection
        detection = detector.detect_ball(frame)
        
        status = "✅" if detection.found else "❌"
        conf = f"{detection.confidence:.3f}" if detection.found else "0.000"
        
        # Identify potential issues
        issues = analyze_detection_issues(detector, frame, radius, detection)
        issue_str = ", ".join(issues) if issues else "none"
        
        print(f"{distance_name:18s} | {radius:6d} | {status:6s} | {conf:10s} | {issue_str}")
        
        if not detection.found:
            failed_ranges.append((radius, distance_name, real_distance))
    
    return failed_ranges

def analyze_detection_issues(detector, frame, expected_radius, detection):
    """Analyze why detection might fail for a specific case"""
    issues = []
    
    # Convert to HSV and apply mask
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, detector.hsv_lower, detector.hsv_upper)
    
    # Check color mask coverage
    green_pixels = cv2.countNonZero(mask)
    total_pixels = frame.shape[0] * frame.shape[1]
    coverage = green_pixels / total_pixels
    
    if coverage < 0.001:
        issues.append("no_green_detected")
    elif coverage > 0.8:
        issues.append("too_much_green")
    
    # Check morphological operations effect
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask_after_morph = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask_after_morph = cv2.morphologyEx(mask_after_morph, cv2.MORPH_CLOSE, kernel)
    
    pixels_before = cv2.countNonZero(mask)
    pixels_after = cv2.countNonZero(mask_after_morph)
    
    if pixels_before > 0 and pixels_after < pixels_before * 0.5:
        issues.append("morphology_removed_ball")
    
    # Check contour analysis
    contours, _ = cv2.findContours(mask_after_morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        
        # Check area threshold
        if area < detector.min_contour_area:
            issues.append(f"area_too_small_{area:.0f}<{detector.min_contour_area}")
        
        # Check radius bounds
        (x, y), radius = cv2.minEnclosingCircle(largest_contour)
        if radius < detector.min_radius:
            issues.append(f"radius_too_small_{radius:.1f}<{detector.min_radius}")
        elif radius > detector.max_radius:
            issues.append(f"radius_too_large_{radius:.1f}>{detector.max_radius}")
        
        # Check circularity
        perimeter = cv2.arcLength(largest_contour, True)
        if perimeter > 0:
            circularity = 4 * np.pi * area / (perimeter * perimeter)
            if circularity < 0.3:
                issues.append(f"not_circular_{circularity:.2f}")
    else:
        issues.append("no_contours_found")
    
    return issues

def create_test_frame(ball_radius, add_noise=False):
    """Create test frame with specified ball radius"""
    frame = np.zeros((720, 1280, 3), dtype=np.uint8)
    frame[:, :] = (40, 40, 40)  # Dark gray background
    
    center = (640, 360)
    cv2.circle(frame, center, ball_radius, (0, 255, 0), -1)
    
    if add_noise:
        # Add realistic noise/lighting variation
        noise = np.random.randint(-20, 20, frame.shape, dtype=np.int16)
        frame = np.clip(frame.astype(np.int16) + noise, 0, 255).astype(np.uint8)
    
    return frame

def analyze_middle_distance_specific():
    """Deep dive analysis of middle distance issues"""
    print(f"\n🔬 Deep Analysis: Middle Distance Detection Issues")
    print("=" * 55)
    
    detector = BallDetector()
    
    # Test problematic middle range
    middle_radii = [15, 20, 25, 30, 35, 40]  # Problem zone
    
    for radius in middle_radii:
        print(f"\n📊 Analyzing radius {radius}px (middle distance):")
        
        frame = create_test_frame(radius)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Step-by-step analysis
        print(f"   1. Color mask analysis:")
        mask_initial = cv2.inRange(hsv, detector.hsv_lower, detector.hsv_upper)
        green_pixels = cv2.countNonZero(mask_initial)
        print(f"      Green pixels found: {green_pixels}")
        
        # Morphological operations
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask_open = cv2.morphologyEx(mask_initial, cv2.MORPH_OPEN, kernel)
        mask_final = cv2.morphologyEx(mask_open, cv2.MORPH_CLOSE, kernel)
        
        pixels_after_open = cv2.countNonZero(mask_open)
        pixels_final = cv2.countNonZero(mask_final)
        
        print(f"   2. Morphology effects:")
        print(f"      After opening: {pixels_after_open} ({(pixels_after_open/green_pixels*100):.1f}% retained)")
        print(f"      After closing: {pixels_final} ({(pixels_final/green_pixels*100):.1f}% retained)")
        
        # Contour analysis
        contours, _ = cv2.findContours(mask_final, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        print(f"   3. Contour analysis:")
        if contours:
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)
            (x, y), detected_radius = cv2.minEnclosingCircle(largest)
            
            print(f"      Contour area: {area:.0f} (min required: {detector.min_contour_area})")
            print(f"      Detected radius: {detected_radius:.1f} (range: {detector.min_radius}-{detector.max_radius})")
            
            # Circularity
            perimeter = cv2.arcLength(largest, True)
            circularity = 4 * np.pi * area / (perimeter * perimeter) if perimeter > 0 else 0
            print(f"      Circularity: {circularity:.3f}")
            
            # Final decision
            area_ok = area >= detector.min_contour_area
            radius_ok = detector.min_radius <= detected_radius <= detector.max_radius
            
            print(f"   4. Decision factors:")
            print(f"      Area check: {'✅' if area_ok else '❌'}")
            print(f"      Radius check: {'✅' if radius_ok else '❌'}")
            
            if area_ok and radius_ok:
                print(f"      🤔 Should be detected but may fail on confidence")
            else:
                print(f"      ❌ Fails basic checks")
        else:
            print(f"      ❌ No contours found after morphology")

def suggest_improvements():
    """Suggest specific improvements for middle distance detection"""
    print(f"\n💡 Suggested Improvements for Middle Distance Detection:")
    print("=" * 60)
    
    print("1. 📏 Adjust Size Thresholds:")
    print("   - Lower min_contour_area for medium balls")
    print("   - Current: 200, try: 100-150 for better medium coverage")
    
    print("2. 🔧 Optimize Morphological Operations:")
    print("   - Use smaller kernel for MORPH_OPEN (3x3 instead of 5x5)")
    print("   - This prevents removing medium-sized features")
    
    print("3. 📊 Adaptive Thresholds:")
    print("   - Dynamic thresholds based on detected ball size")
    print("   - More lenient for medium-sized detections")
    
    print("4. 🎯 Confidence Calibration:")
    print("   - Lower confidence threshold for medium distances")
    print("   - Current: 0.535, try: 0.3-0.4 for medium range")
    
    print("5. 🌈 HSV Range Expansion:")
    print("   - Slightly broader HSV range for medium lighting")
    print("   - May help with intermediate lighting conditions")

def create_improved_detector():
    """Create detector with middle-distance improvements"""
    print(f"\n🔧 Creating Improved Detector for Middle Distance")
    print("=" * 50)
    
    from ball_tracker import BallDetector
    
    # Create improved detector
    detector = BallDetector()
    
    # Apply improvements
    detector.min_contour_area = 120  # Reduced from 200
    detector.min_radius = 6          # Slightly reduced
    
    # Test the improvements
    print("Testing improved detector:")
    
    middle_radii = [15, 20, 25, 30, 35, 40]
    improvements = 0
    
    for radius in middle_radii:
        frame = create_test_frame(radius)
        detection = detector.detect_ball(frame)
        
        status = "✅" if detection.found else "❌"
        conf = f"{detection.confidence:.3f}" if detection.found else "0.000"
        
        if detection.found:
            improvements += 1
            
        print(f"   Radius {radius:2d}px: {status} (conf: {conf})")
    
    print(f"\n📈 Improvement Results:")
    print(f"   Middle-distance detections: {improvements}/{len(middle_radii)}")
    print(f"   Success rate: {improvements/len(middle_radii)*100:.1f}%")
    
    return detector

if __name__ == "__main__":
    # Run diagnostic tests
    failed_ranges = test_distance_ranges()
    
    if failed_ranges:
        print(f"\n❌ Failed Detection Ranges:")
        for radius, name, distance in failed_ranges:
            print(f"   {name} (r={radius}px, ~{distance}cm)")
    
    # Deep analysis of middle distance issues
    analyze_middle_distance_specific()
    
    # Suggest improvements
    suggest_improvements()
    
    # Test improved detector
    create_improved_detector()
    
    print(f"\n🎯 Conclusion: Middle distance issues likely due to:")
    print("   1. Morphological operations too aggressive")  
    print("   2. Contour area thresholds too high")
    print("   3. Need adaptive parameters based on ball size")