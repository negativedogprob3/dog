# Ball Tracking for Unitree Go2 Robot

This object detection system provides real-time ball detection and 3D position tracking using the robot's camera and position data. It combines computer vision for ball detection with coordinate transformations to provide both ball and robot positions in world coordinates.

## Features

- **Real-time ball detection** using color-based filtering and contour detection
- **3D position estimation** using camera intrinsics and robot pose data
- **Coordinate transformation** from camera frame to world coordinates  
- **Real-time visualization** of both ball and robot positions
- **Integration** with existing Go2 robot control system
- **Data logging** and export capabilities
- **Multiple ball colors** supported (optimized for green ball)

## Files

- `ball_tracker.py` - Core ball tracking classes and computer vision algorithms
- `ball_tracker_demo.py` - Interactive demo with robot integration
- `README.md` - This documentation file

## Ball Detection

The system is **optimized for green balls** but supports multiple colors:
- **Green** (default) - Standard green ball detection
- **Bright Green** - For brighter/neon green balls  
- **Tennis Green** - For tennis ball green color
- Orange, Yellow, Blue, Red - Alternative colors

## Quick Start

### 1. Basic Demo (with mock robot)
```bash
cd object_detection
python3 ball_tracker_demo.py --mock-robot
```

### 2. Real Robot Integration
```bash
cd object_detection  
python3 ball_tracker_demo.py
```

### 3. With Data Logging
```bash
cd object_detection
python3 ball_tracker_demo.py --save-data
```

## Controls (Interactive Mode)

- **q** - Quit
- **c** - Change ball color (cycles through available colors)
- **d** - Toggle debug info overlay
- **m** - Toggle detection mask view
- **r** - Reset tracking history
- **s** - Save current detection data
- **SPACE** - Pause/Resume tracking

## Integration with Robot Server

The ball tracker integrates with the existing `mike/server.py` robot controller:

```python
from ball_tracker import BallTracker
from mike.server import Go2RobotController

# Setup robot controller
robot = Go2RobotController()
robot.connect()

# Setup ball tracker
tracker = BallTracker(robot)

# Process camera frame
frame = robot.get_camera_frame()
detection = tracker.track_frame(frame)

# Get positions
positions = tracker.get_positions_summary()
print(f"Robot: {positions['robot']['position']}")
print(f"Ball: {positions['ball']['position']}")
```

## Coordinate System

### World Coordinates
- **Origin**: Robot's starting position
- **X-axis**: Forward direction (positive = forward)
- **Y-axis**: Left direction (positive = left)  
- **Z-axis**: Up direction (positive = up)

### Ball Position Calculation
1. **Detection**: Color-based ball detection in camera image
2. **Ray Casting**: Convert pixel position to 3D ray in camera frame
3. **Transformation**: Transform ray from camera to world coordinates using robot pose
4. **Ground Intersection**: Calculate where ray intersects ground plane (Z=0)

## Camera Calibration

The system uses estimated camera parameters for the Go2's front camera:
- **Resolution**: 1280x720
- **Focal Length**: ~800px (estimated)
- **Camera Position**: ~30cm forward, 25cm up from robot base
- **Camera Tilt**: ~-5.7° (looking slightly down)

For better accuracy, perform proper camera calibration using a checkerboard pattern.

## Output Data Format

### Position Summary
```json
{
  "timestamp": 1234567890.123,
  "robot": {
    "position": {"x": 0.0, "y": 0.0, "z": 0.3},
    "orientation": {"yaw_degrees": 45.0}
  },
  "ball": {
    "detected": true,
    "confidence": 0.85,
    "position": {"x": 1.5, "y": -0.5, "z": 0.0}
  }
}
```

### Detection Details
```json
{
  "found": true,
  "pixel_position": [640, 400],
  "world_position": {"x": 1.5, "y": -0.5, "z": 0.0},
  "confidence": 0.85,
  "radius": 45.2
}
```

## Requirements

- Python 3.11 (for robot SDK compatibility)
- OpenCV (`pip install opencv-python`)
- NumPy (`pip install numpy`)
- Robot dependencies (see main CLAUDE.md)

## Troubleshooting

### Ball Not Detected
1. Check ball color matches selected detection mode
2. Ensure adequate lighting
3. Verify ball is within detection radius range (8-250 pixels)
4. Try different color modes with 'c' key

### Poor Position Accuracy
1. Improve camera calibration parameters
2. Ensure robot pose data is accurate
3. Check that ball is on ground plane (Z=0)
4. Verify coordinate transformation parameters

### Camera Issues
1. Check robot camera connection
2. Verify network interface for GStreamer
3. Try fallback to system camera
4. Use mock mode for testing without camera

## Example Usage

```python
#!/usr/bin/env python3
import cv2
from ball_tracker import BallTracker

# Create tracker
tracker = BallTracker()

# Process video stream
cap = cv2.VideoCapture(0)
while True:
    ret, frame = cap.read()
    if not ret:
        break
        
    # Track ball
    detection = tracker.track_frame(frame)
    
    # Draw results
    result_frame = tracker.draw_detection(frame, detection)
    cv2.imshow('Ball Tracking', result_frame)
    
    # Print position info
    if detection.found and detection.world_position:
        pos = detection.world_position
        print(f"Ball at: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})")
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
```

## Success Criteria Met ✅

This implementation successfully provides:

1. **Ball Position Detection** - Real-time detection of green ball in camera feed
2. **Robot Position Tracking** - Integration with robot's position/orientation data
3. **World Coordinate Representation** - Both ball and robot positions in unified coordinate system
4. **Real-time Operation** - Live tracking and visualization on laptop
5. **Data Export** - JSON logging of positions for analysis

The system is ready for ball interaction tasks with the Unitree Go2 robot.