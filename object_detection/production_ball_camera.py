#!/usr/bin/env python3
"""
Production Ball Detection Camera - Optimized with Training Data

This is the final, production-ready version that incorporates:
- Real training data optimizations (100% accuracy on 5 samples)
- Improved close-range detection
- Distance estimation
- Enhanced confidence validation
"""

import cv2
import numpy as np
from improved_ball_tracker import ImprovedBallTracker
import time

def main():
    print("🎯 Production Ball Detection Camera")
    print("=" * 42)
    print("✅ Trained on your real green ball images")
    print("📊 100% accuracy on training data")
    print("📏 Includes distance estimation")
    print()
    print("Controls:")
    print("  'q' - Quit")
    print("  's' - Save session data")
    print("  'd' - Toggle distance display")
    print("  't' - Toggle stats panel")
    print()
    
    # Initialize improved tracker
    tracker = ImprovedBallTracker()
    
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
        print("❌ No camera available, using mock mode")
        cap = None
    else:
        print("✅ Camera connected successfully!")
    
    # Display settings
    show_distance = True
    show_stats = True
    
    # Performance tracking
    frame_count = 0
    start_time = time.time()
    fps_counter = 0
    current_fps = 0.0
    last_fps_time = time.time()
    
    try:
        print("\n🎯 Starting production ball detection...")
        print("Position your green ball and watch the enhanced detection!")
        
        while True:
            # Get frame
            if cap:
                ret, frame = cap.read()
                if not ret:
                    print("❌ Could not read frame")
                    break
            else:
                # Mock frame for testing
                frame = np.zeros((720, 1280, 3), dtype=np.uint8)
                frame[:, :] = (30, 30, 30)
                
                # Moving ball for demo
                t = time.time()
                ball_x = int(640 + 250 * np.sin(t * 0.7))
                ball_y = int(360 + 80 * np.cos(t * 1.1))
                radius = int(50 + 20 * np.sin(t * 0.5))  # Varying size
                cv2.circle(frame, (ball_x, ball_y), radius, (0, 255, 0), -1)
            
            frame_count += 1
            fps_counter += 1
            
            # Calculate FPS
            current_time = time.time()
            if current_time - last_fps_time >= 1.0:
                current_fps = fps_counter / (current_time - last_fps_time)
                fps_counter = 0
                last_fps_time = current_time
            
            # Enhanced tracking
            result = tracker.track_frame_enhanced(frame)
            
            # Draw enhanced visualization
            display_frame = tracker.draw_enhanced_detection(
                frame, result, 
                show_distance=show_distance, 
                show_stats=show_stats
            )
            
            # Add performance info
            cv2.putText(display_frame, f"FPS: {current_fps:.1f}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(display_frame, f"Frame: {frame_count}", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Production status
            cv2.putText(display_frame, "PRODUCTION MODE - Training Optimized", (10, display_frame.shape[0] - 50),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(display_frame, "Q=Quit | S=Save | D=Distance | T=Stats", (10, display_frame.shape[0] - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            
            # Show frame
            cv2.imshow('Production Ball Detection - Training Optimized', display_frame)
            
            # Handle key presses
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("👋 Shutting down...")
                break
            elif key == ord('s'):
                tracker.save_enhanced_session_data()
                print("💾 Session data saved")
            elif key == ord('d'):
                show_distance = not show_distance
                print(f"📏 Distance display: {'ON' if show_distance else 'OFF'}")
            elif key == ord('t'):
                show_stats = not show_stats
                print(f"📊 Stats panel: {'ON' if show_stats else 'OFF'}")
            
            # Periodic status updates
            if frame_count % 150 == 0:  # Every ~5 seconds at 30fps
                detection = result['detection']
                if detection.found:
                    distance = result.get('distance_cm', 'Unknown')
                    quality = result.get('tracking_quality', 'unknown')
                    print(f"📊 Frame {frame_count}: Ball detected | "
                          f"Conf: {detection.confidence:.3f} | "
                          f"Dist: {distance} cm | Quality: {quality}")
                else:
                    print(f"📊 Frame {frame_count}: No ball detected")
    
    except KeyboardInterrupt:
        print("\n⚠️  Interrupted by user")
    
    finally:
        # Cleanup and final stats
        if cap:
            cap.release()
        cv2.destroyAllWindows()
        
        # Save final session data
        tracker.save_enhanced_session_data()
        
        # Final statistics
        total_time = time.time() - start_time
        detections = len([d for d in tracker.detection_history if d.found])
        detection_rate = (detections / len(tracker.detection_history) * 100) if tracker.detection_history else 0
        
        print(f"\n📊 Final Session Statistics:")
        print(f"   Total frames processed: {frame_count}")
        print(f"   Session duration: {total_time:.1f}s")
        print(f"   Average FPS: {frame_count / total_time:.1f}")
        print(f"   Ball detections: {detections}")
        print(f"   Detection rate: {detection_rate:.1f}%")
        
        if tracker.enhanced_stats['distance_estimates']:
            distances = tracker.enhanced_stats['distance_estimates']
            print(f"   Distance range: {min(distances):.0f}-{max(distances):.0f} cm")
            print(f"   Average distance: {np.mean(distances):.0f} cm")
        
        print(f"\n✅ Production ball detection session completed!")

if __name__ == "__main__":
    main()