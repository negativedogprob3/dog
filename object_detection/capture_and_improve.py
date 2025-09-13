#!/usr/bin/env python3
"""
Capture Green Ball Training Data and Improve Algorithm

This script:
1. Captures pictures from the front camera 
2. Lets you label ball positions manually
3. Analyzes the real data to optimize detection parameters
4. Updates the algorithm based on real-world performance
"""

import cv2
import numpy as np
from ball_tracker import BallDetector
import json
import time
import os
from typing import List, Dict, Tuple

class BallDataCollector:
    """Collects training data from real camera captures"""
    
    def __init__(self):
        self.detector = BallDetector()
        self.training_data = []
        self.capture_count = 0
        
        # Create directory for training images
        os.makedirs("training_data", exist_ok=True)
        
    def setup_camera(self):
        """Setup camera connection"""
        print("🔄 Setting up camera connection...")
        
        # Try Go2 camera first
        pipeline = (
            "udpsrc address=230.1.1.1 port=1720 multicast-iface=en8 ! "
            "application/x-rtp, media=video, encoding-name=H264 ! "
            "rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! "
            "video/x-raw,width=1280,height=720,format=BGR ! appsink drop=1"
        )
        
        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        
        if not cap.isOpened():
            print("⚠️  Go2 camera not available, trying system camera...")
            cap = cv2.VideoCapture(0)
        
        if not cap.isOpened():
            print("❌ No camera available")
            return None
            
        print("✅ Camera connected successfully!")
        return cap
    
    def capture_training_session(self):
        """Main training data capture session"""
        print("📸 Ball Detection Training Data Capture")
        print("=" * 45)
        print("Instructions:")
        print("  SPACE - Capture image for training")
        print("  'c' - Change ball color mode")
        print("  'q' - Quit and analyze data")
        print("  Move the green ball to different positions/distances")
        print()
        
        cap = self.setup_camera()
        if cap is None:
            return
        
        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    print("❌ Could not read frame")
                    break
                
                # Run current detection
                detection = self.detector.detect_ball(frame)
                
                # Draw current detection
                display_frame = frame.copy()
                if detection.found:
                    x, y = detection.pixel_position
                    radius = int(detection.radius)
                    cv2.circle(display_frame, (x, y), radius, (0, 255, 0), 3)
                    cv2.circle(display_frame, (x, y), 5, (0, 255, 0), -1)
                    
                    # Show detection info
                    cv2.putText(display_frame, f"DETECTED: Conf={detection.confidence:.3f}", 
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                    cv2.putText(display_frame, f"Pos: ({x}, {y}) R: {radius}", 
                               (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                else:
                    cv2.putText(display_frame, "NO BALL DETECTED", 
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                
                # Show instructions
                cv2.putText(display_frame, f"Color: {self.detector.current_color} | SPACE=Capture | Q=Quit", 
                           (10, display_frame.shape[0] - 20), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                
                cv2.putText(display_frame, f"Captured: {self.capture_count} images", 
                           (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                
                cv2.imshow('Ball Detection Training - Capture Mode', display_frame)
                
                # Handle key presses
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord(' '):  # Space bar to capture
                    self.capture_training_image(frame, detection)
                elif key == ord('c'):
                    # Cycle through colors
                    colors = list(self.detector.color_ranges.keys())
                    current_idx = colors.index(self.detector.current_color)
                    next_idx = (current_idx + 1) % len(colors)
                    self.detector.set_color(colors[next_idx])
                    print(f"🎨 Switched to {self.detector.current_color} mode")
        
        finally:
            cap.release()
            cv2.destroyAllWindows()
    
    def capture_training_image(self, frame: np.ndarray, detection):
        """Capture and label a training image"""
        self.capture_count += 1
        timestamp = int(time.time())
        
        # Save original image
        filename = f"training_data/capture_{self.capture_count:03d}_{timestamp}.jpg"
        cv2.imwrite(filename, frame)
        
        # Create training data entry
        training_entry = {
            "filename": filename,
            "timestamp": timestamp,
            "capture_id": self.capture_count,
            "detector_color": self.detector.current_color,
            "hsv_range": {
                "lower": self.detector.hsv_lower.tolist(),
                "upper": self.detector.hsv_upper.tolist()
            },
            "automatic_detection": {
                "found": detection.found,
                "confidence": detection.confidence if detection.found else 0.0,
                "position": detection.pixel_position if detection.found else None,
                "radius": detection.radius if detection.found else 0.0
            }
        }
        
        print(f"📸 Captured image {self.capture_count}: {filename}")
        if detection.found:
            x, y = detection.pixel_position
            print(f"    🎯 Auto-detected ball at ({x}, {y}) with confidence {detection.confidence:.3f}")
        else:
            print(f"    ❌ No automatic detection - will need manual labeling")
        
        # Start manual labeling for this image
        manual_label = self.manual_label_image(frame.copy(), filename)
        training_entry["manual_label"] = manual_label
        
        self.training_data.append(training_entry)
        
        # Save training data after each capture
        self.save_training_data()
    
    def manual_label_image(self, frame: np.ndarray, filename: str) -> Dict:
        """Manually label ball position in image"""
        print(f"\n👆 Manual Labeling for {filename}")
        print("Click on the center of the green ball (if visible)")
        print("Press 'n' if no ball is visible, 's' to skip, 'r' to retry")
        
        ball_positions = []
        window_name = "Manual Labeling - Click Ball Center"
        
        def mouse_callback(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN:
                ball_positions.append((x, y))
                # Draw circle at clicked position
                cv2.circle(frame, (x, y), 30, (255, 0, 0), 3)  # Blue circle
                cv2.circle(frame, (x, y), 3, (255, 0, 0), -1)   # Blue center
                cv2.putText(frame, f"Ball {len(ball_positions)}", (x+35, y), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                cv2.imshow(window_name, frame)
                print(f"  ✅ Marked ball at ({x}, {y})")
        
        cv2.imshow(window_name, frame)
        cv2.setMouseCallback(window_name, mouse_callback)
        
        # Instructions on image
        instructions = [
            "Click on ball center(s)",
            "n = no ball visible",
            "s = skip/done", 
            "r = clear and retry"
        ]
        
        for i, instruction in enumerate(instructions):
            cv2.putText(frame, instruction, (10, 30 + i*25), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        cv2.imshow(window_name, frame)
        
        while True:
            key = cv2.waitKey(0) & 0xFF
            if key == ord('s'):  # Done
                break
            elif key == ord('n'):  # No ball
                ball_positions = []
                print("  📝 Marked as 'no ball visible'")
                break
            elif key == ord('r'):  # Retry
                ball_positions = []
                frame = cv2.imread(filename)  # Reload original
                cv2.imshow(window_name, frame)
                print("  🔄 Cleared markings, try again")
        
        cv2.destroyWindow(window_name)
        
        # Return manual label data
        manual_data = {
            "has_ball": len(ball_positions) > 0,
            "ball_positions": ball_positions,
            "num_balls": len(ball_positions)
        }
        
        return manual_data
    
    def save_training_data(self):
        """Save training data to JSON file"""
        with open("training_data/training_data.json", "w") as f:
            json.dump(self.training_data, f, indent=2)
    
    def analyze_and_improve(self):
        """Analyze training data and suggest improvements"""
        if not self.training_data:
            print("❌ No training data to analyze")
            return
            
        print(f"\n📊 Analyzing {len(self.training_data)} training samples...")
        print("=" * 50)
        
        # Statistics
        auto_correct = 0  # Automatic detection was correct
        auto_missed = 0   # Ball present but not detected
        auto_false = 0    # Detected ball where none exists
        position_errors = []  # Distance between auto and manual positions
        
        for entry in self.training_data:
            auto_det = entry["automatic_detection"]
            manual_lab = entry["manual_label"]
            
            has_ball_manual = manual_lab["has_ball"]
            has_ball_auto = auto_det["found"]
            
            if has_ball_manual and has_ball_auto:
                # Both detected ball - check position accuracy
                auto_pos = auto_det["position"]
                manual_positions = manual_lab["ball_positions"]
                
                if manual_positions and auto_pos:
                    # Find closest manual position
                    min_dist = float('inf')
                    for manual_pos in manual_positions:
                        dist = np.sqrt((auto_pos[0] - manual_pos[0])**2 + 
                                     (auto_pos[1] - manual_pos[1])**2)
                        min_dist = min(min_dist, dist)
                    
                    position_errors.append(min_dist)
                    
                    if min_dist < 50:  # Within 50 pixels = correct
                        auto_correct += 1
                    else:
                        auto_false += 1  # Wrong position
                else:
                    auto_false += 1
                    
            elif has_ball_manual and not has_ball_auto:
                auto_missed += 1
            elif not has_ball_manual and has_ball_auto:
                auto_false += 1
            elif not has_ball_manual and not has_ball_auto:
                auto_correct += 1  # Correctly detected no ball
        
        total = len(self.training_data)
        accuracy = (auto_correct / total * 100) if total > 0 else 0
        
        print(f"📈 Detection Accuracy: {accuracy:.1f}%")
        print(f"  ✅ Correct detections: {auto_correct}/{total}")
        print(f"  ❌ Missed balls: {auto_missed}")
        print(f"  🔴 False detections: {auto_false}")
        
        if position_errors:
            avg_error = np.mean(position_errors)
            max_error = np.max(position_errors)
            print(f"  📍 Position error: {avg_error:.1f}px avg, {max_error:.1f}px max")
        
        # Analyze failure cases
        self.analyze_failure_cases()
        
        # Generate improvement suggestions
        self.suggest_improvements()
    
    def analyze_failure_cases(self):
        """Analyze what types of cases cause failures"""
        print(f"\n🔍 Failure Case Analysis:")
        
        missed_cases = []
        false_cases = []
        
        for entry in self.training_data:
            auto_det = entry["automatic_detection"]
            manual_lab = entry["manual_label"]
            
            if manual_lab["has_ball"] and not auto_det["found"]:
                missed_cases.append(entry)
            elif not manual_lab["has_ball"] and auto_det["found"]:
                false_cases.append(entry)
        
        if missed_cases:
            print(f"  🎯 Missed detections: {len(missed_cases)} cases")
            colors_missed = {}
            for case in missed_cases:
                color = case["detector_color"]
                colors_missed[color] = colors_missed.get(color, 0) + 1
            print(f"     By color: {colors_missed}")
        
        if false_cases:
            print(f"  🔴 False detections: {len(false_cases)} cases")
            colors_false = {}
            for case in false_cases:
                color = case["detector_color"]
                colors_false[color] = colors_false.get(color, 0) + 1
            print(f"     By color: {colors_false}")
    
    def suggest_improvements(self):
        """Generate specific improvement suggestions"""
        print(f"\n💡 Improvement Suggestions:")
        
        # Analyze HSV ranges that worked vs didn't work
        successful_ranges = []
        failed_ranges = []
        
        for entry in self.training_data:
            auto_det = entry["automatic_detection"]
            manual_lab = entry["manual_label"]
            hsv_range = entry["hsv_range"]
            
            success = (manual_lab["has_ball"] and auto_det["found"]) or \
                     (not manual_lab["has_ball"] and not auto_det["found"])
            
            if success:
                successful_ranges.append(hsv_range)
            else:
                failed_ranges.append(hsv_range)
        
        if successful_ranges:
            print("  📊 Most successful HSV ranges:")
            for i, range_data in enumerate(successful_ranges[:3]):
                print(f"     {i+1}. Lower: {range_data['lower']}, Upper: {range_data['upper']}")
        
        print("  🔧 Recommended improvements:")
        print("     1. Test 'low_contrast_green' mode for missed detections")
        print("     2. Adjust HSV ranges based on successful cases")
        print("     3. Consider lighting conditions during capture")
        print("     4. Try different morphological parameters")

def main():
    """Main training data collection and analysis"""
    print("🎯 Green Ball Detection Training System")
    print("=" * 45)
    
    collector = BallDataCollector()
    
    print("This will help improve ball detection by:")
    print("1. 📸 Capturing real images from your camera")
    print("2. 👆 Manually labeling ball positions") 
    print("3. 📊 Analyzing detection accuracy")
    print("4. 💡 Suggesting algorithm improvements")
    print()
    
    print("🚀 Starting automatically...")
    time.sleep(1)
    
    # Capture training data
    collector.capture_training_session()
    
    # Analyze results
    if collector.training_data:
        collector.analyze_and_improve()
        print(f"\n✅ Training complete! Data saved to training_data/")
        print(f"   📁 Images: training_data/*.jpg")
        print(f"   📋 Labels: training_data/training_data.json")
    else:
        print("ℹ️  No training data captured")

if __name__ == "__main__":
    main()