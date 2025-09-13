#!/usr/bin/env python3
"""
Test Fixed Ball Detector with Real Camera

Tests the middle distance fixes with the actual camera to verify
that the improvements work in real conditions.
"""

import cv2
import numpy as np
from fixed_ball_tracker import FixedBallDetector
from ball_tracker import BallDetector
import time

def main():
    print("🔧 Testing Fixed Ball Detector with Real Camera")
    print("=" * 50)
    print("This will compare the original vs fixed detector")
    print("Focus on testing middle distance detection (15-40cm from camera)")
    print()
    print("Controls:")
    print("  'q' - Quit")
    print("  'o' - Toggle original detector")
    print("  'f' - Toggle fixed detector") 
    print("  'c' - Compare both side-by-side")
    print()
    
    # Initialize both detectors
    fixed_detector = FixedBallDetector()
    original_detector = BallDetector()
    
    # Camera setup
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
        return
    else:
        print("✅ Camera connected successfully!")
    
    # Display mode
    mode = 'compare'  # 'original', 'fixed', 'compare'
    frame_count = 0
    
    print(f"\n🎯 Starting camera test in COMPARE mode")
    print("Position your green ball at different distances and observe detection!")
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("❌ Could not read frame")
                break
            
            frame_count += 1
            
            # Test both detectors
            original_detection = original_detector.detect_ball(frame)
            fixed_detection = fixed_detector.detect_ball(frame)
            
            if mode == 'original':
                # Show only original
                display_frame = draw_detection(frame.copy(), original_detection, "ORIGINAL", (0, 0, 255))
                
            elif mode == 'fixed':
                # Show only fixed
                display_frame = draw_detection(frame.copy(), fixed_detection, "FIXED", (0, 255, 0))
                
            else:  # compare mode
                # Split screen comparison
                display_frame = create_comparison_view(frame, original_detection, fixed_detection)
            
            # Add status info
            cv2.putText(display_frame, f"Mode: {mode.upper()}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(display_frame, "Middle Distance Test - Focus on 15-40cm range", (10, display_frame.shape[0] - 50),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.putText(display_frame, "Q=Quit | O=Original | F=Fixed | C=Compare", (10, display_frame.shape[0] - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            
            # Show frame
            cv2.imshow('Fixed Ball Detector Test', display_frame)
            
            # Periodic status updates
            if frame_count % 60 == 0:  # Every 2 seconds at 30fps
                print(f"📊 Frame {frame_count} | Original: {original_detection.found} (conf: {original_detection.confidence:.3f}) | "
                      f"Fixed: {fixed_detection.found} (conf: {fixed_detection.confidence:.3f})")
                
                # Check for improvements
                if not original_detection.found and fixed_detection.found:
                    radius = fixed_detection.radius
                    print(f"   🔧 IMPROVEMENT: Fixed detector found ball (radius: {radius:.0f}px) where original failed!")
            
            # Handle key presses
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("👋 Shutting down...")
                break
            elif key == ord('o'):
                mode = 'original'
                print("📱 Switched to ORIGINAL detector view")
            elif key == ord('f'):
                mode = 'fixed'
                print("🔧 Switched to FIXED detector view")
            elif key == ord('c'):
                mode = 'compare'
                print("⚖️  Switched to COMPARE mode")
    
    except KeyboardInterrupt:
        print("\n⚠️  Interrupted by user")
    
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print("\n✅ Camera test completed!")

def draw_detection(frame, detection, label, color):
    """Draw detection results on frame"""
    if detection.found:
        x, y = detection.pixel_position
        radius = int(detection.radius)
        
        # Draw circle
        cv2.circle(frame, (x, y), radius, color, 3)
        cv2.circle(frame, (x, y), 5, color, -1)
        
        # Draw info
        cv2.putText(frame, f"{label} DETECTED", (x - 80, y - radius - 25),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        cv2.putText(frame, f"Conf: {detection.confidence:.3f}", (x - 60, y - radius - 5),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        cv2.putText(frame, f"R: {radius}px", (x - 30, y + radius + 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    else:
        cv2.putText(frame, f"{label}: NO DETECTION", (10, 70),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
    
    return frame

def create_comparison_view(frame, original_detection, fixed_detection):
    """Create split-screen comparison view"""
    height, width = frame.shape[:2]
    
    # Create comparison frame
    comparison_frame = np.zeros((height, width * 2, 3), dtype=np.uint8)
    
    # Left side - Original
    left_frame = draw_detection(frame.copy(), original_detection, "ORIGINAL", (0, 0, 255))
    comparison_frame[:, :width] = left_frame
    
    # Right side - Fixed
    right_frame = draw_detection(frame.copy(), fixed_detection, "FIXED", (0, 255, 0))
    comparison_frame[:, width:] = right_frame
    
    # Draw center divider
    cv2.line(comparison_frame, (width, 0), (width, height), (255, 255, 255), 2)
    
    # Add labels
    cv2.putText(comparison_frame, "ORIGINAL DETECTOR", (width//4, 100),
               cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    cv2.putText(comparison_frame, "FIXED DETECTOR", (width + width//4, 100),
               cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    
    # Detection status summary
    orig_status = "✅ FOUND" if original_detection.found else "❌ NOT FOUND"
    fixed_status = "✅ FOUND" if fixed_detection.found else "❌ NOT FOUND" 
    
    cv2.putText(comparison_frame, orig_status, (50, 140),
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    cv2.putText(comparison_frame, fixed_status, (width + 50, 140),
               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    
    return comparison_frame

if __name__ == "__main__":
    main()