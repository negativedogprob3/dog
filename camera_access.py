"""
python3 camera_access.py [network_interface (en8?)]
"""

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.video.video_client import VideoClient
import cv2
import numpy as np
import sys

def main():
    print("Initializing Go2 camera connection...")
    print("Note: To connect to a Go2 robot:")
    print("  1. Connect via Ethernet cable")
    print("  2. Set PC IP to 192.168.123.51/24")
    print("  3. Robot should be at 192.168.123.18")
    print("  4. Usage: python3 camera_access.py [network_interface]")
    print("")
    
    # Initialize channel factory with network interface if provided
    if len(sys.argv) > 1:
        interface = sys.argv[1]
        print(f"Using network interface: {interface}")
        ChannelFactoryInitialize(0, interface)
    else:
        print("Using default network interface")
        ChannelFactoryInitialize(0)
    
    # Create video client
    client = VideoClient()
    client.SetTimeout(3.0)
    client.Init()
    
    print("Camera initialized. Press 'q' to quit.")
    
    # Get initial image sample
    code, data = client.GetImageSample()
    
    # Capture and display frames
    while code == 0:
        try:
            # Get Image data from Go2 robot
            code, data = client.GetImageSample()
            
            if code == 0:
                # Convert to numpy image
                image_data = np.frombuffer(bytes(data), dtype=np.uint8)
                frame = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
                
                if frame is not None:
                    # Display frame
                    cv2.imshow('Go2 Camera', frame)
                    
                    # Check for quit key
                    if cv2.waitKey(20) & 0xFF == ord('q'):
                        break
                else:
                    print("Failed to decode frame")
                    
        except Exception as e:
            print(f"Error reading camera data: {e}")
            break
    
    if code != 0:
        print(f"Get image sample error. code: {code}")
    
    # Cleanup
    cv2.destroyAllWindows()
    print("Camera connection closed.")

if __name__ == "__main__":
    main()