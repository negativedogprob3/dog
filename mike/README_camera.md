# Go2 Camera Access

This directory contains Python code to access and display the Unitree Go2's front camera feed.

## Quick Start

1. **Install dependencies:**
   ```bash
   python3 -m pip install -r requirements.txt
   ```

2. **Connect to Go2:**
   - **Ethernet**: Connect ethernet cable between computer and Go2
   - **WiFi**: Connect to Go2's WiFi network (SSID: GO2_XXXXXX)

3. **Run the camera viewer:**
   ```bash
   python3 camera_access.py
   ```

## Connection Methods

The script tries multiple connection methods automatically:

### 1. GStreamer (Recommended for EDU)
- Uses UDP multicast stream on port 1720
- Requires network interface name (eth0, enp2s0, etc.)
- Best performance and reliability

### 2. RTSP Stream
- Standard RTSP protocol
- Default URL: `rtsp://192.168.12.1:8554/video`
- Good compatibility

### 3. HTTP Stream
- Fallback method via HTTP
- May have higher latency

## Controls

- **'q'**: Quit the application
- **'s'**: Save screenshot 
- **'f'**: Toggle fullscreen mode

## Troubleshooting

### Connection Issues
1. **Check network interface:**
   ```bash
   ip link show  # Linux
   ifconfig      # macOS
   ```

2. **Verify Go2 connection:**
   ```bash
   ping 192.168.12.1
   ```

3. **Check GStreamer installation:**
   ```bash
   gst-launch-1.0 --version
   ```

### Common Problems

- **"Failed to connect via GStreamer"**: Install GStreamer development packages
- **"No frame received"**: Check network connection and robot power
- **High latency**: Try ethernet connection instead of WiFi
- **Permission denied**: Run with appropriate network permissions

### Platform-Specific Notes

**Linux:**
- May need to install `python3-opencv` and `gstreamer1.0-plugins-*`
- Check firewall settings for UDP multicast

**macOS:** 
- Install GStreamer via Homebrew: `brew install gstreamer`
- May need to allow network access in Security preferences

**Windows:**
- Install GStreamer from official website
- Ensure Python can find GStreamer libraries

## Camera Specifications

- **Resolution**: 1280x720
- **Format**: H.264 encoded
- **Frame rate**: ~30 FPS
- **Field of view**: Wide angle front-facing camera
- **Protocol**: UDP multicast (GStreamer) or RTSP

## Integration with Robot Control

This camera access can be combined with robot control for Phase 2 of the experiment:

```python
# Example: Capture frame while controlling robot
camera = Go2CameraClient()
if camera.connect_gstreamer("eth0"):
    frame = camera.get_frame()
    # Process frame for ball detection
    # Send movement commands to robot
```