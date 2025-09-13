#!/usr/bin/env python3
"""
Simple Ball Detection Script

Opens the Go2 front camera and runs green ball detection in real-time.
No complex features - just camera + ball detection.

Usage:
    python3 simple_ball_detection.py
"""

import cv2
import numpy as np
from ball_tracker import BallDetector
import time

def main():
    print("🎯 Simple Ball Detection")
    print("=" * 30)
    print("Opening camera and starting green ball detection...")
    print("Press 'q' to quit, 'c' to change ball color")
    
    # Initialize ball detector
    detector = BallDetector()
    print(f"✅ Ball detector ready - tracking {detector.current_color} balls")
    
    # Try to open camera
    # Method 1: Try Go2 robot camera via GStreamer
    pipeline = (
        "udpsrc address=230.1.1.1 port=1720 multicast-iface=en8 ! "
        "application/x-rtp, media=video, encoding-name=H264 ! "
        "rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! "
        "video/x-raw,width=1280,height=720,format=BGR ! appsink drop=1"
    )
    
    print("🔄 Trying to connect to Go2 camera...")
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    
    # Method 2: Fallback to system camera
    if not cap.isOpened():
        print("⚠️  Go2 camera not available, trying system camera...")
        cap = cv2.VideoCapture(0)
    
    # Method 3: Create mock camera for testing
    if not cap.isOpened():
        print("⚠️  No camera available, using mock camera for testing")
        cap = None
    
    if cap and cap.isOpened():
        print("✅ Camera connected successfully!")
    else:
        print("✅ Using mock camera mode for testing")
    
    # Main detection loop
    frame_count = 0
    start_time = time.time()
    detections = 0
    
    try:
        while True:
            # Get frame
            if cap and cap.isOpened():
                ret, frame = cap.read()
                if not ret:
                    print("❌ Could not read frame")
                    break
            else:
                # Create mock frame with moving green ball for demo
                frame = np.zeros((720, 1280, 3), dtype=np.uint8)
                frame[:, :] = (40, 40, 40)  # Dark gray background
                
                # Add moving green ball
                t = time.time()
                ball_x = int(640 + 300 * np.sin(t * 0.8))
                ball_y = int(360 + 100 * np.sin(t * 1.2))
                cv2.circle(frame, (ball_x, ball_y), 50, (0, 255, 0), -1)  # Green ball
            
            frame_count += 1
            
            # Run ball detection
            detection = detector.detect_ball(frame)
            
            # Draw results
            if detection.found:
                detections += 1
                x, y = detection.pixel_position
                radius = int(detection.radius)
                
                # Draw green circle around ball
                cv2.circle(frame, (x, y), radius, (0, 255, 0), 3)
                cv2.circle(frame, (x, y), 5, (0, 255, 0), -1)
                
                # Draw detection info
                cv2.putText(frame, f"BALL DETECTED", (x - 80, y - radius - 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                cv2.putText(frame, f"Conf: {detection.confidence:.2f}", (x - 60, y - radius - 45),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(frame, f"Pos: ({x}, {y})", (x - 50, y + radius + 25),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            
            # Add stats overlay
            elapsed = time.time() - start_time
            fps = frame_count / elapsed if elapsed > 0 else 0
            detection_rate = (detections / frame_count * 100) if frame_count > 0 else 0
            
            cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(frame, f"Frames: {frame_count}", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(frame, f"Detections: {detections} ({detection_rate:.1f}%)", (10, 90),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(frame, f"Color: {detector.current_color}", (10, 120),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Add instructions
            cv2.putText(frame, "Press 'q' to quit, 'c' to change color", (10, frame.shape[0] - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # Show frame
            cv2.imshow('Simple Ball Detection', frame)
            
            # Handle key presses
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("👋 Quitting...")
                break
            elif key == ord('c'):
                # Cycle through ball colors
                colors = list(detector.color_ranges.keys())
                current_idx = colors.index(detector.current_color)
                next_idx = (current_idx + 1) % len(colors)
                detector.set_color(colors[next_idx])
                print(f"🎨 Switched to {detector.current_color} ball detection")
            
            # Print detection status every 2 seconds
            if frame_count % 60 == 0:  # Assuming ~30 FPS
                status = "🎯 DETECTED" if detection.found else "❌ No ball"
                print(f"Frame {frame_count}: {status} | Rate: {detection_rate:.1f}% | FPS: {fps:.1f}")
    
    except KeyboardInterrupt:
        print("\n⚠️  Interrupted by user")
    
    finally:
        # Cleanup
        if cap:
            cap.release()
        cv2.destroyAllWindows()
        
        # Print final stats
        elapsed_total = time.time() - start_time
        print(f"\n📊 Final Statistics:")
        print(f"  Total frames: {frame_count}")
        print(f"  Ball detections: {detections}")
        print(f"  Detection rate: {detection_rate:.1f}%")
        print(f"  Average FPS: {fps:.1f}")
        print(f"  Session duration: {elapsed_total:.1f}s")
        print("✅ Ball detection completed!")

if __name__ == "__main__":
    main()