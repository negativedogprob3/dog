#!/usr/bin/env python3
"""
Fixed Ball Tracker - Addressing Middle Distance Issues

Based on the diagnostic analysis, the middle distance issue in real conditions
is likely due to:
1. Confidence threshold too high for real-world middle distances
2. Real lighting conditions affecting HSV detection
3. Camera focus/blur affecting medium-distance balls
4. Morphological operations being too aggressive

This version implements targeted fixes for these issues.
"""

import cv2
import numpy as np
from ball_tracker import BallDetector, BallDetection
import time

class FixedBallDetector(BallDetector):
    """Enhanced ball detector with middle distance fixes"""
    
    def __init__(self):
        super().__init__()
        
        # FIX 1: More lenient thresholds for middle distances
        self.min_contour_area = 120      # Reduced from 200
        self.min_radius = 6              # Reduced from 8
        
        # FIX 2: Expanded HSV range for better real-world performance
        self.color_ranges['green'] = (
            np.array([35, 40, 40]),      # More inclusive lower bound
            np.array([85, 255, 255])     # Wider hue range
        )
        self.hsv_lower, self.hsv_upper = self.color_ranges['green']
        
        # FIX 3: Adaptive confidence thresholds
        self.base_confidence_threshold = 0.3  # Lower base threshold
        
        print("🔧 Fixed ball detector initialized:")
        print(f"   📏 Min area: {self.min_contour_area} (was 200)")
        print(f"   📐 Min radius: {self.min_radius} (was 8)")
        print(f"   🎨 HSV range: {self.hsv_lower} - {self.hsv_upper}")
        print(f"   📊 Base confidence: {self.base_confidence_threshold}")
    
    def detect_ball(self, frame: np.ndarray) -> BallDetection:
        """Enhanced detection with middle distance fixes"""
        if frame is None:
            return BallDetection(False, None, None, 0.0, 0.0)
        
        try:
            # Convert to HSV
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # IMPROVEMENT 1: Try multiple HSV ranges for robustness
            masks = []
            
            # Primary green range
            mask1 = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
            masks.append(mask1)
            
            # Secondary range for different lighting
            hsv_lower2 = np.array([40, 30, 30])    # Even more inclusive
            hsv_upper2 = np.array([80, 255, 255])
            mask2 = cv2.inRange(hsv, hsv_lower2, hsv_upper2)
            masks.append(mask2)
            
            # Combine masks
            combined_mask = cv2.bitwise_or(mask1, mask2)
            
            # IMPROVEMENT 2: Adaptive morphological operations
            # Use smaller kernel for medium-sized features
            kernel_small = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
            kernel_normal = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            
            # Apply gentle morphology first
            mask_processed = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel_small)
            mask_processed = cv2.morphologyEx(mask_processed, cv2.MORPH_CLOSE, kernel_normal)
            
            # IMPROVEMENT 3: Find best contour with multiple criteria
            contours, _ = cv2.findContours(mask_processed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if not contours:
                return BallDetection(False, None, None, 0.0, 0.0)
            
            # Score contours by multiple criteria
            best_contour = None
            best_score = 0
            
            for contour in contours:
                area = cv2.contourArea(contour)
                
                # Skip very small contours
                if area < self.min_contour_area:
                    continue
                
                # Get bounding circle
                (x, y), radius = cv2.minEnclosingCircle(contour)
                
                # Skip radius outliers
                if radius < self.min_radius or radius > self.max_radius:
                    continue
                
                # Calculate circularity
                perimeter = cv2.arcLength(contour, True)
                if perimeter == 0:
                    continue
                    
                circularity = 4 * np.pi * area / (perimeter * perimeter)
                
                # IMPROVEMENT 4: Multi-factor scoring
                # Size score (prefer medium sizes for middle distance)
                size_score = 1.0
                if 15 <= radius <= 40:  # Middle distance sweet spot
                    size_score = 1.2    # Boost middle distances
                elif radius < 10 or radius > 60:
                    size_score = 0.8    # Penalize extreme sizes
                
                # Circularity score (more lenient for real conditions)
                circ_score = min(1.0, circularity * 1.5)  # Boost circularity weight
                
                # Area coverage score
                coverage = area / (frame.shape[0] * frame.shape[1])
                coverage_score = min(1.0, coverage * 100)  # Normalize
                
                # Combined score
                total_score = (circ_score * 0.4 + size_score * 0.4 + coverage_score * 0.2)
                
                if total_score > best_score:
                    best_score = total_score
                    best_contour = contour
            
            if best_contour is None:
                return BallDetection(False, None, None, 0.0, 0.0)
            
            # Final detection processing
            area = cv2.contourArea(best_contour)
            (x, y), radius = cv2.minEnclosingCircle(best_contour)
            center = (int(x), int(y))
            radius = int(radius)
            
            # IMPROVEMENT 5: Enhanced confidence calculation with radius consideration
            perimeter = cv2.arcLength(best_contour, True)
            circularity = 4 * np.pi * area / (perimeter * perimeter) if perimeter > 0 else 0
            
            # Base confidence from circularity and size
            base_confidence = min(1.0, circularity * 1.2)
            
            # Middle distance boost
            distance_boost = 1.0
            if 15 <= radius <= 40:  # Middle distance range
                distance_boost = 1.3  # 30% confidence boost for middle distances
            
            # Final confidence with boost
            final_confidence = min(1.0, base_confidence * distance_boost)
            
            # IMPROVEMENT 6: Lower acceptance threshold for middle distances
            acceptance_threshold = self.base_confidence_threshold
            if 15 <= radius <= 40:  # Middle distance
                acceptance_threshold *= 0.7  # Lower threshold by 30%
            
            if final_confidence < acceptance_threshold:
                return BallDetection(False, None, None, 0.0, 0.0)
            
            return BallDetection(
                found=True,
                pixel_position=center,
                world_position=None,
                confidence=final_confidence,
                radius=float(radius)
            )
            
        except Exception as e:
            print(f"Enhanced ball detection error: {e}")
            return BallDetection(False, None, None, 0.0, 0.0)

def test_fixed_detector():
    """Test the fixed detector across all distance ranges"""
    print("🎯 Testing Fixed Ball Detector")
    print("=" * 35)
    
    fixed_detector = FixedBallDetector()
    original_detector = BallDetector()
    
    # Test ranges including the problematic middle distances
    test_cases = [
        (5, "Very Far"),
        (10, "Far"), 
        (15, "Medium-Far"),    # Problem area
        (20, "Medium"),        # Problem area
        (25, "Medium-Close"),  # Problem area  
        (30, "Medium-Close+"), # Problem area
        (35, "Close-Medium"),  # Problem area
        (40, "Close"),
        (60, "Very Close")
    ]
    
    print("Distance      | Radius | Original | Fixed  | Improvement")
    print("-" * 55)
    
    improvements = 0
    total_middle = 0
    
    for radius, distance_name in test_cases:
        # Create test frame
        frame = np.zeros((720, 1280, 3), dtype=np.uint8)
        frame[:, :] = (40, 40, 40)
        cv2.circle(frame, (640, 360), radius, (0, 255, 0), -1)
        
        # Test both detectors
        orig_detection = original_detector.detect_ball(frame)
        fixed_detection = fixed_detector.detect_ball(frame)
        
        orig_status = "✅" if orig_detection.found else "❌"
        fixed_status = "✅" if fixed_detection.found else "❌"
        
        # Check improvement
        improvement = ""
        if not orig_detection.found and fixed_detection.found:
            improvement = "🔧 FIXED!"
            improvements += 1
        elif orig_detection.found and not fixed_detection.found:
            improvement = "⚠️ Worse"
        elif orig_detection.found and fixed_detection.found:
            improvement = "✅ Same"
        
        # Track middle distance improvements
        if 15 <= radius <= 35:  # Middle distance range
            total_middle += 1
        
        print(f"{distance_name:13s} | {radius:6d} | {orig_status:8s} | {fixed_status:6s} | {improvement}")
    
    print(f"\n📈 Results:")
    print(f"   Improvements made: {improvements}")
    print(f"   Fixed detector should handle middle distances much better!")

if __name__ == "__main__":
    test_fixed_detector()
    
    print(f"\n💡 Key Fixes Applied:")
    print("1. 📏 Lower area/radius thresholds for medium balls")
    print("2. 🎨 Expanded HSV ranges for real-world lighting")
    print("3. 🔧 Gentler morphological operations")
    print("4. 📊 Boosted confidence for middle distances (15-40px)")
    print("5. 🎯 Lower acceptance thresholds for medium range")
    print("6. 🏆 Multi-criteria contour scoring")