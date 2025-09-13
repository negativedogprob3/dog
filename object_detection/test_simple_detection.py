#!/usr/bin/env python3
"""
Quick test of simple ball detection - runs for 5 seconds then exits
"""

import cv2
import numpy as np
from ball_tracker import BallDetector
import time

def main():
    print("🎯 Testing Simple Ball Detection (5 second demo)")
    print("=" * 45)
    
    # Initialize ball detector
    detector = BallDetector()
    print(f"✅ Ball detector ready - tracking {detector.current_color} balls")
    
    # Use mock camera for quick demo
    print("🔄 Starting mock camera with moving green ball...")
    
    frame_count = 0
    detections = 0
    start_time = time.time()
    
    # Run for 5 seconds
    while time.time() - start_time < 5.0:
        # Create mock frame with moving green ball
        frame = np.zeros((720, 1280, 3), dtype=np.uint8)
        frame[:, :] = (30, 30, 30)  # Dark background
        
        # Add moving green ball
        t = time.time()
        ball_x = int(640 + 200 * np.sin(t * 2))
        ball_y = int(360 + 100 * np.cos(t * 1.5))
        cv2.circle(frame, (ball_x, ball_y), 45, (0, 255, 0), -1)  # Green ball
        
        frame_count += 1
        
        # Run detection
        detection = detector.detect_ball(frame)
        
        if detection.found:
            detections += 1
            x, y = detection.pixel_position
            print(f"Frame {frame_count}: 🎯 Ball detected at ({x}, {y}) - confidence {detection.confidence:.3f}")
        else:
            print(f"Frame {frame_count}: ❌ No ball detected")
        
        time.sleep(0.1)  # 10 FPS for demo
    
    # Final results
    elapsed = time.time() - start_time
    detection_rate = (detections / frame_count * 100) if frame_count > 0 else 0
    
    print(f"\n📊 Demo Results:")
    print(f"  Frames processed: {frame_count}")
    print(f"  Ball detections: {detections}")
    print(f"  Detection rate: {detection_rate:.1f}%")
    print(f"  Average FPS: {frame_count / elapsed:.1f}")
    
    if detection_rate > 80:
        print("✅ Ball detection working perfectly!")
    else:
        print("⚠️  Detection rate low - check algorithm")

if __name__ == "__main__":
    main()