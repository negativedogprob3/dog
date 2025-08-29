#!/usr/bin/env python3
"""
Unitree Go2 Camera Access Script

This script provides multiple methods to access and display the front camera feed
from a Unitree Go2 robot. Choose the method that works best for your setup.

Requirements:
- opencv-python (pip install opencv-python)
- unitree_sdk2_python (for SDK method)
- Network connection to Go2 robot
"""

import cv2
import numpy as np
import sys
import time
import socket
from typing import Optional

class Go2CameraClient:
    """Client for accessing Unitree Go2 front camera"""
    
    def __init__(self):
        self.cap = None
        self.is_connected = False
    
    def connect_gstreamer(self, interface_name: str = "eth0") -> bool:
        """
        Connect using GStreamer pipeline (recommended for EDU version)
        
        Args:
            interface_name: Network interface name (e.g., "eth0", "enp2s0")
        """
        try:
            # GStreamer pipeline for Go2 camera
            pipeline = (
                f"udpsrc address=230.1.1.1 port=1720 multicast-iface={interface_name} ! "
                "application/x-rtp, media=video, encoding-name=H264 ! "
                "rtph264depay ! "
                "h264parse ! "
                "avdec_h264 ! "
                "videoconvert ! "
                "video/x-raw,width=1280,height=720,format=BGR ! "
                "appsink drop=1"
            )
            
            print(f"Connecting to Go2 camera via GStreamer on interface {interface_name}...")
            self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            
            if self.cap.isOpened():
                self.is_connected = True
                print("✓ Successfully connected to Go2 camera via GStreamer")
                return True
            else:
                print("✗ Failed to connect via GStreamer")
                return False
                
        except Exception as e:
            print(f"✗ GStreamer connection error: {e}")
            return False
    
    def connect_rtsp(self, robot_ip: str = "192.168.12.1") -> bool:
        """
        Connect using RTSP stream (alternative method)
        
        Args:
            robot_ip: IP address of the Go2 robot
        """
        try:
            rtsp_url = f"rtsp://{robot_ip}:8554/video"
            print(f"Connecting to Go2 camera via RTSP: {rtsp_url}")
            
            self.cap = cv2.VideoCapture(rtsp_url)
            
            if self.cap.isOpened():
                self.is_connected = True
                print("✓ Successfully connected to Go2 camera via RTSP")
                return True
            else:
                print("✗ Failed to connect via RTSP")
                return False
                
        except Exception as e:
            print(f"✗ RTSP connection error: {e}")
            return False
    
    def connect_webrtc(self, robot_ip: str = "192.168.12.1") -> bool:
        """
        Connect using WebRTC (for WiFi connection)
        Note: This is a simplified version - full WebRTC requires additional setup
        
        Args:
            robot_ip: IP address of the Go2 robot
        """
        try:
            # This would typically require aiortc or similar WebRTC library
            # For now, we'll try a direct HTTP stream approach
            stream_url = f"http://{robot_ip}:8080/stream"
            print(f"Connecting to Go2 camera via HTTP stream: {stream_url}")
            
            self.cap = cv2.VideoCapture(stream_url)
            
            if self.cap.isOpened():
                self.is_connected = True
                print("✓ Successfully connected to Go2 camera via HTTP stream")
                return True
            else:
                print("✗ Failed to connect via HTTP stream")
                return False
                
        except Exception as e:
            print(f"✗ HTTP stream connection error: {e}")
            return False
    
    def get_frame(self) -> Optional[np.ndarray]:
        """Get a single frame from the camera"""
        if not self.is_connected or self.cap is None:
            return None
            
        ret, frame = self.cap.read()
        return frame if ret else None
    
    def display_stream(self, window_name: str = "Go2 Camera Feed"):
        """Display live camera stream in a window"""
        if not self.is_connected:
            print("✗ Not connected to camera")
            return
        
        print("📹 Starting camera stream display...")
        print("Press 'q' to quit, 's' to save screenshot, 'f' to toggle fullscreen")
        
        fullscreen = False
        frame_count = 0
        start_time = time.time()
        
        while True:
            frame = self.get_frame()
            if frame is None:
                print("⚠️ No frame received")
                break
            
            # Add frame info overlay
            frame_count += 1
            elapsed = time.time() - start_time
            fps = frame_count / elapsed if elapsed > 0 else 0
            
            cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(frame, f"Frame: {frame_count}", (10, 70), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(frame, "Press 'q' to quit", (10, frame.shape[0] - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Display frame
            cv2.imshow(window_name, frame)
            
            # Handle key presses
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                filename = f"go2_screenshot_{int(time.time())}.jpg"
                cv2.imwrite(filename, frame)
                print(f"📸 Screenshot saved: {filename}")
            elif key == ord('f'):
                if fullscreen:
                    cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)
                    fullscreen = False
                else:
                    cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
                    fullscreen = True
        
        print("📹 Camera stream stopped")
        cv2.destroyAllWindows()
    
    def disconnect(self):
        """Clean up camera connection"""
        if self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()
        self.is_connected = False
        print("🔌 Camera disconnected")

def main():
    """Main function to run camera access"""
    print("🐕 Unitree Go2 Camera Access")
    print("=" * 40)
    
    camera = Go2CameraClient()
    
    # Try different connection methods
    connection_methods = [
        ("GStreamer (Ethernet)", lambda: camera.connect_gstreamer("eth0")),
        ("GStreamer (enp2s0)", lambda: camera.connect_gstreamer("enp2s0")), 
        ("RTSP Stream", lambda: camera.connect_rtsp("192.168.12.1")),
        ("HTTP Stream", lambda: camera.connect_webrtc("192.168.12.1")),
    ]
    
    connected = False
    for name, connect_func in connection_methods:
        print(f"\n🔄 Trying {name}...")
        if connect_func():
            connected = True
            break
    
    if not connected:
        print("\n❌ Could not connect to Go2 camera")
        print("\nTroubleshooting:")
        print("1. Ensure Go2 robot is powered on and connected")
        print("2. Check network interface name (try: ip link show)")
        print("3. Verify robot IP address (default: 192.168.12.1)")
        print("4. For Ethernet: ensure cable is connected")
        print("5. For WiFi: connect to robot's WiFi network")
        return
    
    try:
        # Display the camera stream
        camera.display_stream()
    except KeyboardInterrupt:
        print("\n⚠️ Interrupted by user")
    finally:
        camera.disconnect()

if __name__ == "__main__":
    main()