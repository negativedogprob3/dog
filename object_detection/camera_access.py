#!/usr/bin/env python3
"""
Unitree Go2 Camera Access Script with Ball Tracking

This script provides multiple methods to access and display the front camera feed
from a Unitree Go2 robot with integrated green ball detection and tracking.

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
from typing import Optional, Tuple
import os

# Import ball detection from object_detection module
try:
    from ball_tracker import BallDetector, BallDetection
    BALL_TRACKING_AVAILABLE = True
    print("✅ Ball tracking imported successfully")
except ImportError as e:
    BALL_TRACKING_AVAILABLE = False
    print(f"⚠️  Ball tracking not available: {e}")

# Try to import Unitree SDK for direct robot camera access
try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.go2.video.video_client import VideoClient
    SDK_AVAILABLE = True
    print("✅ Unitree SDK available")
except ImportError:
    SDK_AVAILABLE = False
    print("⚠️  Unitree SDK not available")

class Go2CameraClient:
    """Client for accessing Unitree Go2 front camera with ball tracking"""
    
    def __init__(self):
        self.cap = None
        self.video_client = None
        self.is_connected = False
        
        # Initialize ball detector if available
        self.ball_detector = None
        self.ball_tracking_enabled = False
        self.show_tracking = True
        self.tracking_stats = {
            'total_frames': 0,
            'balls_detected': 0,
            'last_detection_time': 0
        }
        
        if BALL_TRACKING_AVAILABLE:
            self.ball_detector = BallDetector()
            self.ball_tracking_enabled = True
            print("🎯 Ball tracking enabled (green ball detection)")
        else:
            print("⚠️  Ball tracking disabled - ball_tracker module not found")
    
    def connect_sdk(self, interface_name: str = "en0") -> bool:
        """
        Connect using Unitree SDK (recommended for real robot)
        
        Args:
            interface_name: Network interface name (e.g., "en0", "eth0")
        """
        if not SDK_AVAILABLE:
            print("✗ Unitree SDK not available")
            return False
            
        try:
            print(f"Connecting to Go2 camera via SDK on interface {interface_name}...")
            
            # Initialize channel factory
            ChannelFactoryInitialize(0, interface_name)
            
            # Create video client
            self.video_client = VideoClient()
            self.video_client.SetTimeout(3.0)
            self.video_client.Init()
            
            # Test connection with initial frame
            code, data = self.video_client.GetImageSample()
            if code == 0:
                self.is_connected = True
                print("✓ Successfully connected to Go2 camera via SDK")
                if self.ball_tracking_enabled:
                    print("🎯 Ball tracking ready - will detect green balls")
                return True
            else:
                print(f"✗ Failed to get initial frame, error code: {code}")
                return False
                
        except Exception as e:
            print(f"✗ SDK connection error: {e}")
            return False

    def connect_gstreamer(self, interface_name: str = "en0") -> bool:
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
                if self.ball_tracking_enabled:
                    print("🎯 Ball tracking ready - will detect green balls")
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
                if self.ball_tracking_enabled:
                    print("🎯 Ball tracking ready - will detect green balls")
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
                if self.ball_tracking_enabled:
                    print("🎯 Ball tracking ready - will detect green balls")
                return True
            else:
                print("✗ Failed to connect via HTTP stream")
                return False
                
        except Exception as e:
            print(f"✗ HTTP stream connection error: {e}")
            return False
    
    def get_frame(self) -> Optional[np.ndarray]:
        """Get a single frame from the camera"""
        if not self.is_connected:
            return None
        
        # Try SDK first if available
        if self.video_client is not None:
            try:
                code, data = self.video_client.GetImageSample()
                if code == 0:
                    # Convert to numpy image
                    image_data = np.frombuffer(bytes(data), dtype=np.uint8)
                    frame = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
                    return frame
                else:
                    return None
            except Exception as e:
                print(f"SDK frame error: {e}")
                return None
        
        # Fallback to OpenCV capture
        if self.cap is not None:
            ret, frame = self.cap.read()
            return frame if ret else None
            
        return None
    
    def detect_ball_in_frame(self, frame: np.ndarray) -> Optional[BallDetection]:
        """Detect ball in the given frame"""
        if not self.ball_tracking_enabled or self.ball_detector is None:
            return None
            
        return self.ball_detector.detect_ball(frame)
    
    def draw_ball_detection(self, frame: np.ndarray, detection: BallDetection) -> np.ndarray:
        """Draw ball detection results on frame"""
        if detection is None or not detection.found:
            return frame
            
        annotated_frame = frame.copy()
        
        if detection.pixel_position:
            x, y = detection.pixel_position
            radius = int(detection.radius)
            
            # Draw detection circle - green if high confidence, yellow if low
            color = (0, 255, 0) if detection.confidence > 0.6 else (0, 255, 255)
            cv2.circle(annotated_frame, (x, y), radius, color, 3)
            cv2.circle(annotated_frame, (x, y), 5, color, -1)
            
            # Draw crosshair at center
            cv2.line(annotated_frame, (x-15, y), (x+15, y), color, 2)
            cv2.line(annotated_frame, (x, y-15), (x, y+15), color, 2)
            
            # Draw confidence and position info
            conf_text = f"Ball: {detection.confidence:.2f}"
            cv2.putText(annotated_frame, conf_text, (x - 60, y - radius - 15),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            
            pos_text = f"({x}, {y})"
            cv2.putText(annotated_frame, pos_text, (x - 40, y + radius + 25),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            
            # Draw radius info
            radius_text = f"R: {radius}px"
            cv2.putText(annotated_frame, radius_text, (x - 35, y + radius + 45),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        
        return annotated_frame
    
    def draw_tracking_stats(self, frame: np.ndarray) -> np.ndarray:
        """Draw tracking statistics on frame"""
        if not self.ball_tracking_enabled:
            return frame
            
        annotated_frame = frame.copy()
        
        # Stats panel background
        panel_height = 120
        panel_width = 300
        overlay = annotated_frame.copy()
        cv2.rectangle(overlay, (10, 10), (panel_width, panel_height), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.7, annotated_frame, 0.3, 0, annotated_frame)
        
        # Stats text
        stats = self.tracking_stats
        detection_rate = (stats['balls_detected'] / max(stats['total_frames'], 1)) * 100
        
        y_offset = 35
        line_height = 25
        
        cv2.putText(annotated_frame, f"Ball Tracking Stats:", (15, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        y_offset += line_height
        
        cv2.putText(annotated_frame, f"Frames: {stats['total_frames']}", (15, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        y_offset += 20
        
        cv2.putText(annotated_frame, f"Detections: {stats['balls_detected']}", (15, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        y_offset += 20
        
        cv2.putText(annotated_frame, f"Rate: {detection_rate:.1f}%", (15, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0) if detection_rate > 20 else (0, 255, 255), 1)
        
        return annotated_frame
    
    def display_stream(self, window_name: str = "Go2 Camera Feed with Ball Tracking"):
        """Display live camera stream with ball tracking"""
        if not self.is_connected:
            print("✗ Not connected to camera")
            return
        
        print("📹 Starting camera stream with ball tracking...")
        print("Controls:")
        print("  'q' - Quit")
        print("  's' - Save screenshot")
        print("  'f' - Toggle fullscreen") 
        print("  't' - Toggle ball tracking display")
        print("  'c' - Change ball color")
        print("  'r' - Reset tracking stats")
        if self.ball_tracking_enabled:
            print(f"  Current ball color: {self.ball_detector.current_color}")
        
        fullscreen = False
        frame_count = 0
        start_time = time.time()
        last_fps_time = time.time()
        fps_counter = 0
        current_fps = 0
        
        while True:
            frame = self.get_frame()
            if frame is None:
                print("⚠️ No frame received")
                break
            
            # Update frame counter
            frame_count += 1
            fps_counter += 1
            self.tracking_stats['total_frames'] = frame_count
            
            # Calculate FPS
            current_time = time.time()
            if current_time - last_fps_time >= 1.0:
                current_fps = fps_counter / (current_time - last_fps_time)
                fps_counter = 0
                last_fps_time = current_time
            
            # Perform ball detection
            ball_detection = None
            if self.ball_tracking_enabled:
                ball_detection = self.detect_ball_in_frame(frame)
                if ball_detection and ball_detection.found:
                    self.tracking_stats['balls_detected'] += 1
                    self.tracking_stats['last_detection_time'] = current_time
            
            # Draw tracking overlay if enabled
            if self.show_tracking and ball_detection:
                frame = self.draw_ball_detection(frame, ball_detection)
            
            # Add frame info overlay
            elapsed = current_time - start_time
            
            cv2.putText(frame, f"FPS: {current_fps:.1f}", (frame.shape[1] - 150, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, f"Frame: {frame_count}", (frame.shape[1] - 150, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Draw tracking stats
            if self.show_tracking:
                frame = self.draw_tracking_stats(frame)
            
            # Add current color mode indicator
            if self.ball_tracking_enabled:
                color_text = f"Tracking: {self.ball_detector.current_color} ball"
                cv2.putText(frame, color_text, (10, frame.shape[0] - 50),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # Controls help
            cv2.putText(frame, "Press 'q' to quit, 't' to toggle tracking", (10, frame.shape[0] - 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
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
            elif key == ord('t'):
                self.show_tracking = not self.show_tracking
                print(f"🎯 Ball tracking display: {'ON' if self.show_tracking else 'OFF'}")
            elif key == ord('c') and self.ball_tracking_enabled:
                # Cycle through ball colors
                colors = list(self.ball_detector.color_ranges.keys())
                current_idx = colors.index(self.ball_detector.current_color)
                next_idx = (current_idx + 1) % len(colors)
                self.ball_detector.set_color(colors[next_idx])
                print(f"🎨 Switched to tracking {colors[next_idx]} ball")
            elif key == ord('r'):
                # Reset tracking stats
                self.tracking_stats = {
                    'total_frames': frame_count,
                    'balls_detected': 0,
                    'last_detection_time': 0
                }
                print("📊 Tracking statistics reset")
        
        print("📹 Camera stream stopped")
        
        # Print final statistics
        if self.ball_tracking_enabled:
            stats = self.tracking_stats
            elapsed_total = time.time() - start_time
            detection_rate = (stats['balls_detected'] / max(stats['total_frames'], 1)) * 100
            
            print("\n📊 Final Ball Tracking Statistics:")
            print(f"  Total frames processed: {stats['total_frames']}")
            print(f"  Ball detections: {stats['balls_detected']}")
            print(f"  Detection rate: {detection_rate:.1f}%")
            print(f"  Session duration: {elapsed_total:.1f}s")
            print(f"  Average FPS: {stats['total_frames'] / elapsed_total:.1f}")
        
        cv2.destroyAllWindows()
    
    def disconnect(self):
        """Clean up camera connection"""
        if self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()
        self.is_connected = False
        print("🔌 Camera disconnected")

def main():
    """Main function to run camera access with ball tracking"""
    print("🐕 Unitree Go2 Camera Access with Ball Tracking")
    print("=" * 50)
    
    camera = Go2CameraClient()
    
    # Try different connection methods
    connection_methods = [
        ("Unitree SDK (en8)", lambda: camera.connect_sdk("en8")),
        ("GStreamer (en8)", lambda: camera.connect_gstreamer("en8")),
        ("RTSP Stream (Ethernet)", lambda: camera.connect_rtsp("192.168.123.18")),
        ("HTTP Stream (Ethernet)", lambda: camera.connect_webrtc("192.168.123.18")),
        ("RTSP Stream (WiFi)", lambda: camera.connect_rtsp("192.168.12.1")),
        ("HTTP Stream (WiFi)", lambda: camera.connect_webrtc("192.168.12.1")),
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
        # Display the camera stream with ball tracking
        camera.display_stream()
    except KeyboardInterrupt:
        print("\n⚠️ Interrupted by user")
    finally:
        camera.disconnect()

if __name__ == "__main__":
    main()