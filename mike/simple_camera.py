#!/usr/bin/env python3
"""
Simple Go2 Camera Viewer

A minimal script to quickly test camera access.
Usage: python3 simple_camera.py [interface_name]
"""

import cv2
import sys

def main():
    interface = sys.argv[1] if len(sys.argv) > 1 else "eth0"
    
    print(f"🐕 Connecting to Go2 camera via {interface}...")
    
    # GStreamer pipeline for Go2
    pipeline = (
        f"udpsrc address=230.1.1.1 port=1720 multicast-iface={interface} ! "
        "application/x-rtp, media=video, encoding-name=H264 ! "
        "rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! "
        "video/x-raw,width=1280,height=720,format=BGR ! appsink drop=1"
    )
    
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    
    if not cap.isOpened():
        print("❌ Failed to connect. Try:")
        print(f"  python3 {sys.argv[0]} enp2s0")
        print(f"  python3 {sys.argv[0]} wlan0")
        return
    
    print("✅ Connected! Press 'q' to quit")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
            
        cv2.imshow("Go2 Camera", frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()
    print("📹 Disconnected")

if __name__ == "__main__":
    main()