#!/usr/bin/env python3
import cv2
import numpy as np
import argparse

class GreenBallDetector:
    def __init__(self):
        # HSV color ranges for different green ball variants
        self.color_ranges = {
            'green': {
                'lower': np.array([40, 50, 50]),
                'upper': np.array([80, 255, 255])
            },
            'bright_green': {
                'lower': np.array([35, 100, 100]),
                'upper': np.array([85, 255, 255])
            },
            'tennis_green': {
                'lower': np.array([30, 80, 80]),
                'upper': np.array([90, 255, 255])
            }
        }
        
        # Morphological operations kernel
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        
    def detect_balls(self, frame):
        """
        Detect green balls in the given frame
        Returns list of (x, y, radius) tuples for detected balls
        """
        # Convert to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Create combined mask for all green variants
        combined_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        
        for color_name, color_range in self.color_ranges.items():
            mask = cv2.inRange(hsv, color_range['lower'], color_range['upper'])
            combined_mask = cv2.bitwise_or(combined_mask, mask)
        
        # Clean up the mask with morphological operations
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, self.kernel)
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, self.kernel)
        
        # Find contours
        contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        detected_balls = []
        
        for contour in contours:
            # Filter by minimum area
            area = cv2.contourArea(contour)
            if area < 100:  # Minimum area threshold
                continue
            
            # Fit circle to contour
            (x, y), radius = cv2.minEnclosingCircle(contour)
            
            # Filter by circularity and size
            if radius < 10 or radius > 200:  # Size constraints
                continue
                
            # Check circularity (area vs circle area ratio)
            circle_area = np.pi * radius * radius
            circularity = area / circle_area if circle_area > 0 else 0
            
            if circularity > 0.6:  # Circularity threshold
                detected_balls.append((int(x), int(y), int(radius)))
        
        return detected_balls, combined_mask
    
    def draw_detections(self, frame, balls):
        """
        Draw detection results on the frame
        """
        result_frame = frame.copy()
        
        for x, y, radius in balls:
            # Draw circle around detected ball
            cv2.circle(result_frame, (x, y), radius, (0, 255, 0), 2)
            # Draw center point
            cv2.circle(result_frame, (x, y), 5, (0, 0, 255), -1)
            # Add text label
            cv2.putText(result_frame, f'Ball ({x},{y})', (x-50, y-radius-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        return result_frame

def main():
    parser = argparse.ArgumentParser(description='Green Ball Detector')
    parser.add_argument('--camera', type=int, default=0, help='Camera index (default: 0)')
    parser.add_argument('--video', type=str, help='Path to video file (optional)')
    parser.add_argument('--show-mask', action='store_true', help='Show detection mask')
    args = parser.parse_args()
    
    detector = GreenBallDetector()
    
    # Initialize video capture
    if args.video:
        cap = cv2.VideoCapture(args.video)
    else:
        cap = cv2.VideoCapture(args.camera)
    
    if not cap.isOpened():
        print("Error: Could not open video source")
        return
    
    print("Green Ball Detector started. Press 'q' to quit, 's' to save screenshot")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("End of video or failed to read frame")
            break
        
        # Detect balls
        balls, mask = detector.detect_balls(frame)
        
        # Draw results
        result_frame = detector.draw_detections(frame, balls)
        
        # Add detection count
        cv2.putText(result_frame, f'Balls detected: {len(balls)}', (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # Show results
        cv2.imshow('Green Ball Detection', result_frame)
        
        if args.show_mask:
            cv2.imshow('Detection Mask', mask)
        
        # Handle keyboard input
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            cv2.imwrite('ball_detection_result.jpg', result_frame)
            print("Screenshot saved as ball_detection_result.jpg")
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()