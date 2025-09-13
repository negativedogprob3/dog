#!/usr/bin/env python3
"""
Simple Green Ball Picture Capture

Press SPACE to capture pictures when green ball is visible.
Press 'q' to quit and analyze the captured data to improve the algorithm.
"""

import cv2
import numpy as np
from ball_tracker import BallDetector
import json
import time
import os

class SimpleBallCapture:
    def __init__(self):
        self.detector = BallDetector()
        self.captures = []
        self.capture_count = 0
        
        # Create directory for captured images
        os.makedirs("ball_captures", exist_ok=True)
    
    def setup_camera(self):
        """Setup camera connection"""
        print("🔄 Connecting to camera...")
        
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
            
        print("✅ Camera connected!")
        return cap
    
    def run_capture_session(self):
        """Main capture session"""
        print("📸 Green Ball Picture Capture")
        print("=" * 35)
        print("Controls:")
        print("  SPACE - Capture picture (when ball is visible)")
        print("  'q' - Quit and analyze data")
        print("  'c' - Change ball color mode")
        print()
        print("Position your green ball at different distances and angles.")
        print("Press SPACE each time to capture training pictures!")
        
        cap = self.setup_camera()
        if cap is None:
            return False
        
        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    print("❌ Could not read frame")
                    break
                
                # Run detection on current frame
                detection = self.detector.detect_ball(frame)
                
                # Create display frame
                display_frame = frame.copy()
                
                # Draw current detection (if any)
                if detection.found:
                    x, y = detection.pixel_position
                    radius = int(detection.radius)
                    
                    # Green circle for detected ball
                    cv2.circle(display_frame, (x, y), radius, (0, 255, 0), 3)
                    cv2.circle(display_frame, (x, y), 5, (0, 255, 0), -1)
                    
                    # Detection info
                    cv2.putText(display_frame, f"BALL DETECTED", (x - 80, y - radius - 20),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                    cv2.putText(display_frame, f"Conf: {detection.confidence:.3f}", (x - 60, y + radius + 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                # Status display
                status_color = (0, 255, 0) if detection.found else (0, 0, 255)
                status_text = "READY TO CAPTURE" if detection.found else "NO BALL DETECTED"
                cv2.putText(display_frame, status_text, (10, 40),
                           cv2.FONT_HERSHEY_SIMPLEX, 1.0, status_color, 2)
                
                # Instructions
                cv2.putText(display_frame, f"Color Mode: {self.detector.current_color}", (10, 80),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(display_frame, f"Captures: {self.capture_count}", (10, 110),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(display_frame, "SPACE = Capture | Q = Quit | C = Change Color", 
                           (10, display_frame.shape[0] - 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                
                # Show frame
                cv2.imshow('Green Ball Capture - Press SPACE when ball visible', display_frame)
                
                # Handle key presses
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord(' '):  # SPACE to capture
                    self.capture_picture(frame, detection)
                elif key == ord('c'):  # Change color
                    self.change_color()
        
        finally:
            cap.release()
            cv2.destroyAllWindows()
            
        return len(self.captures) > 0
    
    def capture_picture(self, frame, detection):
        """Capture a picture with ball information"""
        self.capture_count += 1
        timestamp = int(time.time())
        
        # Save image
        filename = f"ball_captures/ball_{self.capture_count:03d}_{timestamp}.jpg"
        cv2.imwrite(filename, frame)
        
        # Record capture data
        capture_data = {
            "filename": filename,
            "capture_id": self.capture_count,
            "timestamp": timestamp,
            "ball_detected": detection.found,
            "confidence": detection.confidence if detection.found else 0.0,
            "position": detection.pixel_position if detection.found else None,
            "radius": detection.radius if detection.found else 0.0,
            "color_mode": self.detector.current_color,
            "user_confirmed_ball": True  # User pressed SPACE, so ball should be present
        }
        
        self.captures.append(capture_data)
        
        # Feedback
        print(f"📸 Captured #{self.capture_count}: {filename}")
        if detection.found:
            x, y = detection.pixel_position
            print(f"    ✅ Algorithm detected ball at ({x}, {y}) with {detection.confidence:.3f} confidence")
        else:
            print(f"    ❌ Algorithm missed the ball - this will help improve detection!")
        
        # Save data after each capture
        with open("ball_captures/capture_data.json", "w") as f:
            json.dump(self.captures, f, indent=2)
    
    def change_color(self):
        """Change ball color detection mode"""
        colors = list(self.detector.color_ranges.keys())
        current_idx = colors.index(self.detector.current_color)
        next_idx = (current_idx + 1) % len(colors)
        self.detector.set_color(colors[next_idx])
        print(f"🎨 Changed to {self.detector.current_color} mode")
    
    def analyze_results(self):
        """Analyze capture results and suggest improvements"""
        if not self.captures:
            print("📊 No captures to analyze")
            return
        
        print(f"\n📊 Analysis of {len(self.captures)} Captures")
        print("=" * 40)
        
        # Calculate detection accuracy
        detected = sum(1 for c in self.captures if c["ball_detected"])
        missed = len(self.captures) - detected
        accuracy = (detected / len(self.captures)) * 100
        
        print(f"Detection Accuracy: {accuracy:.1f}%")
        print(f"  ✅ Correctly detected: {detected}/{len(self.captures)}")
        print(f"  ❌ Missed balls: {missed}/{len(self.captures)}")
        
        if missed > 0:
            print(f"\n💡 Algorithm Improvements Needed:")
            print(f"   - {missed} cases where you saw a ball but algorithm didn't")
            print(f"   - Try different color modes for missed detections")
            print(f"   - Consider adjusting HSV ranges for your lighting conditions")
        
        # Analyze by color mode
        color_stats = {}
        for capture in self.captures:
            color = capture["color_mode"]
            if color not in color_stats:
                color_stats[color] = {"total": 0, "detected": 0}
            color_stats[color]["total"] += 1
            if capture["ball_detected"]:
                color_stats[color]["detected"] += 1
        
        print(f"\n🎨 Performance by Color Mode:")
        for color, stats in color_stats.items():
            accuracy = (stats["detected"] / stats["total"]) * 100
            print(f"   {color}: {stats['detected']}/{stats['total']} ({accuracy:.1f}%)")
        
        # Find best performing mode
        best_color = max(color_stats.keys(), 
                        key=lambda c: color_stats[c]["detected"] / color_stats[c]["total"])
        best_accuracy = (color_stats[best_color]["detected"] / color_stats[best_color]["total"]) * 100
        
        print(f"\n🏆 Best performing color mode: {best_color} ({best_accuracy:.1f}% accuracy)")
        
        # Confidence analysis
        confidences = [c["confidence"] for c in self.captures if c["ball_detected"]]
        if confidences:
            avg_conf = np.mean(confidences)
            min_conf = np.min(confidences)
            print(f"\n📈 Confidence Analysis:")
            print(f"   Average: {avg_conf:.3f}")
            print(f"   Minimum: {min_conf:.3f}")
            if min_conf < 0.5:
                print(f"   ⚠️  Some detections have low confidence - algorithm may need tuning")
        
        print(f"\n📁 All data saved to:")
        print(f"   Images: ball_captures/*.jpg")
        print(f"   Data: ball_captures/capture_data.json")

def main():
    print("🎯 Simple Green Ball Capture System")
    print("=" * 40)
    print("This will capture pictures of your green ball to improve detection.")
    print("Make sure you have your green ball ready!")
    print()
    
    capture_system = SimpleBallCapture()
    
    # Run capture session
    success = capture_system.run_capture_session()
    
    if success:
        # Analyze results
        capture_system.analyze_results()
    else:
        print("ℹ️  No pictures captured")

if __name__ == "__main__":
    main()