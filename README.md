# Dog Project

A clean and organized repository for dog-related development.

## Experiment Challenge

The ultimate goal is to get the Unitree Go2 Edu quadruped robot to bring a beach ball back to you. The challenge is divided into three phases of increasing difficulty:

### Phase 1: Get the ball, easy
Navigate the robot to a beachball and have it fetch the ball back using out-of-the-box controllers (App or handheld controller).

### Phase 2: Get the ball, medium  
Establish programmatic communication and control without using mobile app/handheld controllers:
- Connect to robot's front-facing camera (capture images/video)
- Connect to robot's lidar system (stream lidar data)  
- Write a controller for high-level movement API
- Pilot robot to retrieve ball while streaming sensor data

### Phase 3: Get the ball, autonomously
Fully autonomous ball retrieval:
- Visualize robot trajectory during closed-loop traversal
- Implement object detection to locate ball position in world coordinates
- Generate autonomous strategy for ball retrieval

## Unitree Go2 Edu Technical Details

### Programming API & SDK
- Uses `unitree_sdk2` with Python interface via `unitree_sdk2_python`
- Compatible with ROS (Robot Operating System)
- Supports Python, C++, and block-coding
- SDK provides joint motor control, foot movements, and force control algorithms

### Computing Power
- EDU Base: 40 TOPS (NVIDIA Orin Nano) - suitable for LiDAR and basic mechatronics
- EDU Plus: 100 TOPS (NVIDIA Orin NX) - handles 4D LiDAR and spatial recognition

### Connectivity & Interfaces
- Expansion dock: 16V-60V power supply
- 1 USB3.0-Type A, 2 USB3.0-Type C ports
- 2 Gigabit Ethernet (RJ45), 1 100Gb Ethernet (GH1.25-4PIN)
- 4G/5G connectivity support
- Web interface control via IP 192.168.12.1

### Hand Controller Operation

The Go2 comes with two controller types: **Handheld Remote** (bimanual) and **Companion Remote** (side-following).

#### Handheld Remote Controller
- **Control Range**: 100m ultra-long distance with digital transmission
- **Dead Man's Switch**: L1 (slow speed) / R1 (high speed) - must be held for movement
- **Movement**: Left joystick (X/Y direction), Right joystick (yaw/rotation)  
- **Height Control**: Arrow keys up/down adjust body height
- **Position Control**: L2+R2 held together enables XY plane movement
- **Calibration**: Press F1+F3 simultaneously, then rotate both joysticks to full range until beeping stops

#### Companion Remote Controller  
- **M Button**: Double-press to enter accompanying mode (purple headlamp), single-press to exit
- **Auto-Follow Modes**: Double-press M twice for slow follow (1.5m/s), repeat for fast follow (3.0m/s)
- **Obstacle Avoidance**: Double-click **L2** to enable/disable in accompanying mode
- **Position Modes**: Double-press P to cycle through Down → Damped → Stand modes
- **Recovery**: Hold P for 1 second when robot is on its side to resume standing

#### Controller Setup & Safety Notes
- **First Use**: Bind controller via Unitree Go App → Settings → Remote Control Settings
- **Exclusive Control**: Cannot use App and remote simultaneously - exit side-following mode first
- **Obstacle Avoidance Limitations**: 
  - Only works for forward-facing obstacles (not 360°)
  - Minimum detection distance: 0.05m
  - **Disable when working with beach balls** or objects the robot needs to interact with

### Additional Features
- Voice UI client and audio interface support
- GPT-driven intelligence integration
- Comprehensive development platform for education and research
