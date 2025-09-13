#!/usr/bin/env python3
"""
Analyze Captured Ball Pictures to Improve Detection Algorithm

Uses the captured pictures from simple_capture.py to:
1. Identify what detection parameters work best
2. Optimize HSV color ranges for your specific ball/lighting
3. Update the ball_tracker.py with improved settings
4. Generate detailed performance reports
"""

import cv2
import numpy as np
import json
import os
from ball_tracker import BallDetector
from typing import List, Dict, Tuple
import statistics

class CaptureAnalyzer:
    def __init__(self):
        self.captures_dir = "ball_captures"
        self.capture_data = []
        self.improved_settings = {}
        
    def load_capture_data(self) -> bool:
        """Load captured data from JSON file"""
        data_file = os.path.join(self.captures_dir, "capture_data.json")
        
        if not os.path.exists(data_file):
            print(f"❌ No capture data found at {data_file}")
            print("Run simple_capture.py first to collect training data!")
            return False
        
        with open(data_file, 'r') as f:
            self.capture_data = json.load(f)
        
        print(f"✅ Loaded {len(self.capture_data)} captured pictures")
        return True
    
    def analyze_failed_detections(self):
        """Analyze pictures where algorithm failed to detect ball"""
        print("\n🔍 Analyzing Failed Detections")
        print("=" * 40)
        
        # Find captures where user saw ball but algorithm didn't
        failed_captures = [c for c in self.capture_data if c["user_confirmed_ball"] and not c["ball_detected"]]
        
        if not failed_captures:
            print("✅ No failed detections! Algorithm performed perfectly.")
            return
        
        print(f"Found {len(failed_captures)} failed detections to analyze...")
        
        detector = BallDetector()
        
        for i, capture in enumerate(failed_captures):
            print(f"\n📸 Analyzing failed capture: {capture['filename']}")
            
            # Load image
            img_path = capture['filename']
            if not os.path.exists(img_path):
                print(f"❌ Image file not found: {img_path}")
                continue
                
            frame = cv2.imread(img_path)
            if frame is None:
                print(f"❌ Could not load image: {img_path}")
                continue
            
            # Test different color modes
            print("   Testing different color modes:")
            best_detection = None
            best_color = None
            
            for color_name in detector.color_ranges.keys():
                detector.set_color(color_name)
                detection = detector.detect_ball(frame)
                
                status = "✅" if detection.found else "❌"
                conf = f"{detection.confidence:.3f}" if detection.found else "0.000"
                print(f"     {color_name:18s}: {status} (conf: {conf})")
                
                if detection.found and (best_detection is None or detection.confidence > best_detection.confidence):
                    best_detection = detection
                    best_color = color_name
            
            if best_detection:
                print(f"   🏆 Best result: {best_color} with confidence {best_detection.confidence:.3f}")
                
                # Save annotated image showing what was found
                annotated = self.create_annotated_image(frame, best_detection, best_color)
                save_path = img_path.replace('.jpg', f'_fixed_with_{best_color}.jpg')
                cv2.imwrite(save_path, annotated)
                print(f"   💾 Saved fixed version: {save_path}")
            else:
                print("   ❌ No color mode detected the ball")
                
                # Analyze HSV values manually
                self.analyze_hsv_values(frame, img_path)
    
    def analyze_hsv_values(self, frame: np.ndarray, filename: str):
        """Analyze HSV values in image to understand why detection failed"""
        print("   🔬 Analyzing HSV color values...")
        
        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Find green-ish pixels
        height, width = hsv.shape[:2]
        green_pixels = []
        
        # Sample HSV values across the image
        for y in range(0, height, 20):  # Sample every 20 pixels
            for x in range(0, width, 20):
                h, s, v = hsv[y, x]
                
                # Check if this could be a green ball pixel
                if 30 <= h <= 90 and s > 30 and v > 30:  # Broad green range
                    green_pixels.append([h, s, v])
        
        if green_pixels:
            green_pixels = np.array(green_pixels)
            h_mean, s_mean, v_mean = np.mean(green_pixels, axis=0)
            h_min, s_min, v_min = np.min(green_pixels, axis=0)
            h_max, s_max, v_max = np.max(green_pixels, axis=0)
            
            print(f"   📊 Green pixels found: {len(green_pixels)}")
            print(f"   📈 HSV ranges in image:")
            print(f"      H: {h_min:.0f} - {h_max:.0f} (avg: {h_mean:.0f})")
            print(f"      S: {s_min:.0f} - {s_max:.0f} (avg: {s_mean:.0f})")
            print(f"      V: {v_min:.0f} - {v_max:.0f} (avg: {v_mean:.0f})")
            
            # Suggest custom HSV range
            suggested_lower = np.array([max(30, h_min-10), max(30, s_min-20), max(30, v_min-20)])
            suggested_upper = np.array([min(90, h_max+10), 255, 255])
            
            print(f"   💡 Suggested HSV range for this image:")
            print(f"      Lower: {suggested_lower}")
            print(f"      Upper: {suggested_upper}")
            
            # Test suggested range
            mask = cv2.inRange(hsv, suggested_lower, suggested_upper)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest_contour)
                (x, y), radius = cv2.minEnclosingCircle(largest_contour)
                
                print(f"   ✅ Suggested range found contour: area={area:.0f}, radius={radius:.0f}")
                
                # Save visualization
                result_frame = frame.copy()
                cv2.circle(result_frame, (int(x), int(y)), int(radius), (255, 0, 0), 3)
                save_path = filename.replace('.jpg', '_suggested_fix.jpg')
                cv2.imwrite(save_path, result_frame)
                print(f"   💾 Saved suggested fix: {save_path}")
        else:
            print("   ❌ No green pixels found in image")
    
    def create_annotated_image(self, frame: np.ndarray, detection, color_mode: str) -> np.ndarray:
        """Create annotated image showing detection result"""
        annotated = frame.copy()
        
        if detection.found:
            x, y = detection.pixel_position
            radius = int(detection.radius)
            
            # Draw detection
            cv2.circle(annotated, (x, y), radius, (0, 255, 0), 3)
            cv2.circle(annotated, (x, y), 5, (0, 255, 0), -1)
            
            # Add text
            cv2.putText(annotated, f"DETECTED: {color_mode}", (x-100, y-radius-20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(annotated, f"Confidence: {detection.confidence:.3f}", (x-80, y+radius+30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        return annotated
    
    def find_optimal_settings(self):
        """Find optimal detection settings based on successful captures"""
        print("\n🎯 Finding Optimal Detection Settings")
        print("=" * 42)
        
        # Analyze successful detections
        successful_captures = [c for c in self.capture_data if c["user_confirmed_ball"] and c["ball_detected"]]
        
        if not successful_captures:
            print("❌ No successful detections to analyze")
            return
        
        print(f"Analyzing {len(successful_captures)} successful detections...")
        
        # Analyze confidence scores
        confidences = [c["confidence"] for c in successful_captures]
        
        print(f"\n📊 Confidence Analysis:")
        print(f"   Average: {statistics.mean(confidences):.3f}")
        print(f"   Median:  {statistics.median(confidences):.3f}")
        print(f"   Min:     {min(confidences):.3f}")
        print(f"   Max:     {max(confidences):.3f}")
        
        # Find best color mode
        color_performance = {}
        for capture in successful_captures:
            color = capture["color_mode"]
            if color not in color_performance:
                color_performance[color] = []
            color_performance[color].append(capture["confidence"])
        
        print(f"\n🎨 Color Mode Performance:")
        best_color = None
        best_avg_confidence = 0
        
        for color, confidences in color_performance.items():
            avg_conf = statistics.mean(confidences)
            count = len(confidences)
            print(f"   {color:18s}: {avg_conf:.3f} avg confidence ({count} detections)")
            
            if avg_conf > best_avg_confidence:
                best_avg_confidence = avg_conf
                best_color = color
        
        print(f"\n🏆 Best performing color mode: {best_color} ({best_avg_confidence:.3f} avg confidence)")
        
        self.improved_settings = {
            "best_color_mode": best_color,
            "average_confidence": best_avg_confidence,
            "total_successful": len(successful_captures),
            "recommended_threshold": max(0.3, min(confidences) * 0.8)  # 80% of minimum confidence
        }
    
    def generate_improvement_report(self):
        """Generate comprehensive improvement report"""
        print(f"\n📋 Ball Detection Improvement Report")
        print("=" * 45)
        
        total_captures = len(self.capture_data)
        successful = sum(1 for c in self.capture_data if c["user_confirmed_ball"] and c["ball_detected"])
        failed = sum(1 for c in self.capture_data if c["user_confirmed_ball"] and not c["ball_detected"])
        
        accuracy = (successful / total_captures * 100) if total_captures > 0 else 0
        
        print(f"📊 Overall Performance:")
        print(f"   Total captures: {total_captures}")
        print(f"   Successful detections: {successful}")
        print(f"   Failed detections: {failed}")
        print(f"   Detection accuracy: {accuracy:.1f}%")
        
        if accuracy >= 90:
            print("   ✅ Excellent! Algorithm is working very well.")
        elif accuracy >= 70:
            print("   🟡 Good performance, some improvements possible.")
        else:
            print("   🔴 Significant improvements needed.")
        
        print(f"\n💡 Recommendations:")
        
        if self.improved_settings.get("best_color_mode"):
            best_color = self.improved_settings["best_color_mode"]
            print(f"   1. Use '{best_color}' color mode for best results")
            print(f"      - Shows {self.improved_settings['average_confidence']:.3f} average confidence")
        
        if failed > 0:
            print(f"   2. Check the generated '*_fixed_with_*.jpg' images")
            print(f"      - These show what color modes work for failed cases")
            print(f"   3. Consider custom HSV ranges from '*_suggested_fix.jpg' images")
        
        print(f"   4. Optimal confidence threshold: {self.improved_settings.get('recommended_threshold', 0.5):.3f}")
        
        print(f"\n📁 Generated Analysis Files:")
        print(f"   📸 Original captures: ball_captures/ball_*.jpg")
        print(f"   🔧 Fixed versions: ball_captures/ball_*_fixed_with_*.jpg")
        print(f"   💡 Suggested fixes: ball_captures/ball_*_suggested_fix.jpg")
        
    def update_algorithm_settings(self):
        """Update ball_tracker.py with improved settings"""
        if not self.improved_settings.get("best_color_mode"):
            print("❌ No improvements to apply")
            return
        
        print(f"\n🔧 Algorithm Improvement Suggestions:")
        print("To apply these improvements manually:")
        print(f"1. Use color mode: '{self.improved_settings['best_color_mode']}'")
        print("2. In your detection script, call:")
        print(f"   detector.set_color('{self.improved_settings['best_color_mode']}')")
        print("3. Consider the custom HSV ranges shown in the analysis above")

def main():
    print("🔬 Ball Detection Capture Analysis")
    print("=" * 40)
    print("This analyzes your captured ball pictures to improve detection.")
    
    analyzer = CaptureAnalyzer()
    
    # Load capture data
    if not analyzer.load_capture_data():
        return
    
    # Run analysis
    analyzer.analyze_failed_detections()
    analyzer.find_optimal_settings()
    analyzer.generate_improvement_report()
    analyzer.update_algorithm_settings()
    
    print(f"\n✅ Analysis complete!")
    print(f"Check the ball_captures/ folder for detailed results.")

if __name__ == "__main__":
    main()