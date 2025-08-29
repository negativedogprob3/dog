Add things here that future versions of me should know about this codebase.

## Instructions
- Always open URLs for pull requests after creating or pushing them so the human can see them
- Always switch back to main and pull before starting new work

## Environment & Technical Notes
- **Python/pip**: Use `python3 -m pip` instead of `pip` - the system doesn't have `pip` in PATH
- **mike/ dependencies**: System has externally managed Python environment. For system-wide installs use `python3 -m pip install <package> --break-system-packages`. Key dependencies: websockets, opencv-python (cv2), numpy
- **Unitree Go2**: 
  - Hand controller L2 double-click disables obstacle avoidance for ball interaction
  - Camera access via GStreamer pipeline: `udpsrc address=230.1.1.1 port=1720`
  - Default robot IP: 192.168.12.1 (ethernet) or WiFi network GO2_XXXXXX
  - Multiple controller types: Handheld (bimanual) and Companion (side-following)

## Unitree SDK2 Python
- **Location**: `/Users/dogtrial1/unitree_sdk2_python/`
- **Key Components**:
  - **Examples**: Located in `example/` directory with robot-specific code (go2/, b2/, g1/, h1/)
  - **Go2 High-Level Control**: `example/go2/high_level/go2_sport_client.py` - Use SportClient for movement commands
  - **Go2 Low-Level Control**: `example/go2/low_level/go2_stand_example.py` - Direct motor control with CRC
  - **SDK Package**: `unitree_sdk2py/` contains the main SDK implementation
  
- **Robot Control Methods**:
  1. **High-Level (Sport Mode)**:
     - Uses `SportClient()` from `unitree_sdk2py.go2.sport.sport_client`
     - Commands: `StandUp()`, `StandDown()`, `Move(x, y, yaw)`, `StopMove()`, `Damp()`
     - Special actions: `BackFlip()`, `LeftFlip()`, `HandStand()`, `BalanceStand()`
     - Movement modes: `FreeWalk()`, `FreeBound()`, `FreeAvoid()`
  
  2. **Low-Level Control**:
     - Direct motor control via `LowCmd_` messages on `rt/lowcmd` channel
     - Requires CRC calculation and motor position/velocity/torque commands
     - Must use `MotionSwitcherClient` to release high-level mode first
  
  3. **Connection Setup**:
     - Uses DDS (Data Distribution Service) for communication
     - Initialize with `ChannelFactoryInitialize(0)` or with network interface
     - Robot IP: 192.168.123.18 (when connected via ethernet)
     - Ports: 8082 for sport mode, various channels for DDS communication

- **Video/Camera**: 
  - Front camera examples in `example/go2/front_camera/`
  - Uses OpenCV for image processing
  
- **Other Features**:
  - Obstacle avoidance control: `example/obstacles_avoid/`
  - Wireless controller input: `example/wireless_controller/`
  - Motion switcher for mode transitions: `example/motionSwitcher/`