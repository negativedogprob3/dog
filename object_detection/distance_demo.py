#!/usr/bin/env python3
"""
Ball Distance Estimation Demo

Captures pictures and estimates ball distance in real-time
"""

import cv2
from ball_distance_estimator import BallDistanceEstimator
from ball_tracker import BallDetector

def main():
    print("📏 Real-Time Ball Distance Estimation")
    print("=" * 42)
    print("Controls:")
    print("  SPACE - Capture and analyze distance")
    print("  'q' - Quit")
    print("  'c' - Change ball color")
    
    # Setup
    estimator = BallDistanceEstimator(real_ball_diameter_cm=6.5)  # Adjust for your ball
    detector = BallDetector()
    
    # Try camera connection
    cap = cv2.VideoCapture(0)  # Try system camera first
    
    if not cap.isOpened():
        print("❌ No camera available")
        return
    
    print("✅ Camera connected - show your green ball!")
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            
            # Detect ball
            detection = detector.detect_ball(frame)
            
            # Display frame
            display_frame = frame.copy()
            
            if detection.found:
                x, y = detection.pixel_position
                radius = int(detection.radius)
                
                # Draw detection
                cv2.circle(display_frame, (x, y), radius, (0, 255, 0), 3)
                cv2.circle(display_frame, (x, y), 5, (0, 255, 0), -1)
                
                # Quick distance estimate (size method)
                size_distance = estimator.estimate_distance_from_size(radius)
                
                # Show distance
                cv2.putText(display_frame, f"Distance: ~{size_distance:.0f} cm", 
                           (x - 80, y - radius - 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                cv2.putText(display_frame, f"Press SPACE for full analysis", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            else:
                cv2.putText(display_frame, "No ball detected", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            
            cv2.putText(display_frame, "Q=Quit | SPACE=Analyze | C=Color", 
                       (10, display_frame.shape[0] - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            cv2.imshow('Ball Distance Estimation', display_frame)
            
            # Handle keys
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord(' ') and detection.found:  # SPACE for full analysis
                analyze_distance(frame, estimator)
            elif key == ord('c'):
                colors = list(detector.color_ranges.keys())
                current_idx = colors.index(detector.current_color)
                next_idx = (current_idx + 1) % len(colors)
                detector.set_color(colors[next_idx])
                estimator.detector.set_color(colors[next_idx])
                print(f"🎨 Switched to {detector.current_color}")
    
    finally:
        cap.release()
        cv2.destroyAllWindows()

def analyze_distance(frame, estimator):
    """Perform comprehensive distance analysis"""
    print(f"\n📊 Full Distance Analysis:")
    print("-" * 30)
    
    distances = estimator.estimate_distance(frame)
    
    if "error" in distances:
        print(f"❌ {distances['error']}")
        return
    
    final_dist = distances["final_distance_cm"]
    print(f"🎯 Final Estimate: {final_dist:.1f} cm ({final_dist/100:.2f} meters)")
    print(f"")
    print(f"📊 Method Breakdown:")
    print(f"   Size-based:     {distances['size_method_cm']:.1f} cm")
    print(f"   Ground plane:   {distances['ground_method_cm']:.1f} cm") 
    print(f"   Perspective:    {distances['perspective_method_cm']:.1f} cm")
    print(f"   Detail level:   {distances['detail_method_cm']:.1f} cm")
    print(f"")
    print(f"📈 Detection Info:")
    print(f"   Ball radius:    {distances['ball_radius_px']:.1f} pixels")
    print(f"   Confidence:     {distances['confidence']:.3f}")
    print(f"   Detail score:   {distances['detail_score']:.1f}")

if __name__ == "__main__":
    main()