#!/usr/bin/env python3
import cv2
from ultralytics import YOLO
import numpy as np

def detect_green_ball():
    print("Loading YOLOv8 nano model...")
    model = YOLO('yolov8n.pt')
    
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open camera")
        return
    
    print("Starting green ball detection... Press 'q' to quit")
    
    lower_green = np.array([35, 50, 50])
    upper_green = np.array([85, 255, 255])
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        results = model(frame, conf=0.15, imgsz=800, verbose=False, classes=[32])
        
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        green_mask = cv2.inRange(hsv_frame, lower_green, upper_green)
        contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        ball_candidates = []
        
        for result in results:
            if result.boxes is not None:
                for box in result.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    confidence = float(box.conf[0])
                    
                    roi = frame[y1:y2, x1:x2]
                    if roi.size > 0:
                        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                        green_mask_roi = cv2.inRange(hsv_roi, lower_green, upper_green)
                        green_ratio = cv2.countNonZero(green_mask_roi) / roi.size
                        
                        if green_ratio > 0.2:
                            area = (x2 - x1) * (y2 - y1)
                            score = confidence * green_ratio * min(area / 10000, 1.0)
                            ball_candidates.append({
                                'bbox': (x1, y1, x2, y2),
                                'confidence': confidence,
                                'green_ratio': green_ratio,
                                'score': score,
                                'source': 'yolo'
                            })
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 500:
                x, y, w, h = cv2.boundingRect(contour)
                aspect_ratio = w / h
                
                if 0.5 <= aspect_ratio <= 2.0:
                    roi = frame[y:y+h, x:x+w]
                    if roi.size > 0:
                        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                        green_mask_roi = cv2.inRange(hsv_roi, lower_green, upper_green)
                        green_ratio = cv2.countNonZero(green_mask_roi) / roi.size
                        
                        if green_ratio > 0.4:
                            score = green_ratio * min(area / 10000, 1.0) * 0.5
                            ball_candidates.append({
                                'bbox': (x, y, x+w, y+h),
                                'green_ratio': green_ratio,
                                'score': score,
                                'source': 'color'
                            })
        
        if ball_candidates:
            best_ball = max(ball_candidates, key=lambda x: x['score'])
            x1, y1, x2, y2 = best_ball['bbox']
            
            color = (0, 255, 0) if best_ball['source'] == 'yolo' else (0, 255, 255)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3)
            
            center_x, center_y = (x1 + x2) // 2, (y1 + y2) // 2
            cv2.circle(frame, (center_x, center_y), 5, (255, 0, 255), -1)
            
            if best_ball['source'] == 'yolo':
                label = f"Green Ball: {best_ball['confidence']:.2f}"
            else:
                label = f"Green Ball: {best_ball['green_ratio']:.1%}"
            cv2.putText(frame, label, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        cv2.imshow('Green Ball Detection', frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    detect_green_ball()